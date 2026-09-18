/*----------------------------------------------------------------------*/
/* Hunt steps                                                           */
/*----------------------------------------------------------------------*/

/* H1 (single buffered build only).
 *
 * A send that expects a response, with a frame already reported by the
 * receiver and not yet processed: the completion and the good frame
 * arrive in one status word. Single buffered, dw1000_process_events()
 * writes DW1000_STATE_IDLE over DW1000_STATE_TX_W4R while the chip has
 * just put its own WAIT4RESP receiver up.
 *
 * What the step then shows is the consequence: _dw1000_tx_idle() asks
 * _dw1000_rx_up(), reads IDLE, issues no TRXOFF, and the next TXSTRT
 * goes into a listening receiver, where the chip drops it.
 */
static const char *
hunt_w4r_single_buffer_state(dw1000_t *dw, struct stub *s)
{
    const char *why = NULL;
    uint32_t    status;
    unsigned    tx_before, tx_after;
    unsigned    pmsc;
    int         i, rc;

    if (TIMING_DBLBUFF)
	return NULL;                    /* single buffered only */

    cfgp->rx_keep_on = 0;
    dw1000_rx_set_timeout(dw, 0);
    dw1000_rx_set_timeout_preamble(dw, 0);

    /* A frame for the stub to loop back later. */
    pthread_mutex_lock(&s->lock);
    s->deliver          = false;
    s->tx_done_delay_ms = 0;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0) {
	why = "the frame to loop back was refused";
	goto done;
    }
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done) {
	why = "the frame to loop back did not complete";
	goto done;
    }

    /* Receive it, and leave it unprocessed. */
    pthread_mutex_lock(&s->lock);
    s->deliver = true;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0) {
	why = "dw1000_rx_start did not start the reception";
	goto done;
    }
    if (!wait_line(IRQ_TIMEOUT_MS)) {
	why = "no interrupt for the frame to hold";
	goto done;
    }
    status = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
    if (!(status & DW1000_FLG_SYS_STATUS_RXFCG)) {
	why = REASON("RXFCG not in the status (0x%08" PRIx32 ")", status);
	goto done;
    }

    /* The response-expected send, beside that standing frame. */
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    dw1000_txrx_idle(dw);
    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE | DW1000_TX_RESPONSE_EXPECTED) != 0) {
	why = "the response-expected send beside the held frame was refused";
	goto done;
    }
    for (i = 0; i < 200 && !dw1000_tx_is_status_done(dw); i++)
	usleep(10000);
    if (!dw1000_tx_is_status_done(dw)) {
	why = "the response-expected send did not complete";
	goto done;
    }

    status = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
    if ((status & (DW1000_FLG_SYS_STATUS_RXFCG | DW1000_FLG_SYS_STATUS_TXFRS))
	!= (DW1000_FLG_SYS_STATUS_RXFCG | DW1000_FLG_SYS_STATUS_TXFRS)) {
	why = REASON("the status word does not carry both RXFCG and TXFRS"
		     " (0x%08" PRIx32 ")", status);
	goto done;
    }

    dw1000_process_events(dw);

    if (!evt.rx_ok || !evt.tx_done) {
	why = REASON("rx_ok=%d tx_done=%d after the pass",
		     (int)evt.rx_ok, (int)evt.tx_done);
	goto done;
    }

    pmsc = sys_state_pmsc(dw);
    printf("    H1: dw->state=%u (IDLE=%d RX=%d RX_W4R=%d) PMSC=%u"
	   " expecting_response=%d\n",
	   dw->state, DW1000_STATE_IDLE, DW1000_STATE_RX,
	   DW1000_STATE_RX_W4R, pmsc,
	   (int)dw1000_tx_is_expecting_response(dw));

    if (pmsc != 5) {
	why = REASON("PMSC reads %u, not 5: the chip did not put its own"
		     " WAIT4RESP receiver up, so there is nothing to catch",
		     pmsc);
	goto done;
    }
    if (dw->state != DW1000_STATE_IDLE) {
	why = REASON("dw->state is %u, not IDLE: the state is not lost after"
		     " all", dw->state);
	goto done;
    }

    /* The consequence: no TRXOFF before the next send, and the chip
     * drops a TXSTRT written into a listening receiver. */
    pthread_mutex_lock(&s->lock);
    tx_before = s->tx_count;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    rc = dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
			DW1000_TX_IMMEDIATE);
    usleep(200000);
    pthread_mutex_lock(&s->lock);
    tx_after = s->tx_count;
    pthread_mutex_unlock(&s->lock);

    printf("    H1: next send rc=%d, TX requests %u -> %u, tx_done=%d\n",
	   rc, tx_before, tx_after, (int)evt.tx_done);

    if (rc != 0) {
	why = REASON("the next send was refused (%d) rather than dropped", rc);
	goto done;
    }
    if (tx_after != tx_before)
	why = "the next send reached the medium: the chip did not drop it";

done:
    dw1000_txrx_off(dw);
    dw1000_process_events(dw);
    dw1000_txrx_off(dw);
    return why;
}


/* H2 (double buffered build only).
 *
 * dw1000_rx_start() on a receiver that already holds a frame the host
 * has not been told about yet. The transition table's cell is "RXENAB
 * again, RX"; _dw1000_rx_sync_dblbuff() sees ICRBP != HSRBP, reads it as
 * a misalignment to repair, and issues HRBPT, which hands the unread
 * frame back to the chip. rx_held guards only the read-out window
 * (inside rx_ok), not a frame reported by the chip and not yet reported
 * to the host.
 */
static const char *
hunt_rx_start_drops_queued_frame(dw1000_t *dw, struct stub *s)
{
    const char *why = NULL;
    uint32_t    before, after;
    int         rc;

    if (!TIMING_DBLBUFF)
	return NULL;                    /* double buffered only */

    cfgp->rx_keep_on = 0;
    dw1000_rx_set_timeout(dw, 0);
    dw1000_rx_set_timeout_preamble(dw, 0);

    pthread_mutex_lock(&s->lock);
    s->deliver          = false;
    s->tx_done_delay_ms = 0;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0) {
	why = "the frame to loop back was refused";
	goto done;
    }
    if (!wait_irq(dw, IRQ_TIMEOUT_MS) || !evt.tx_done) {
	why = "the frame to loop back did not complete";
	goto done;
    }

    pthread_mutex_lock(&s->lock);
    s->deliver = true;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    if (dw1000_rx_start(dw, DW1000_RX_IMMEDIATE) != 0) {
	why = "dw1000_rx_start did not start the reception";
	goto done;
    }
    if (!wait_line(IRQ_TIMEOUT_MS)) {
	why = "no interrupt for the received frame";
	goto done;
    }
    before = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
    if (!(before & DW1000_FLG_SYS_STATUS_RXFCG)) {
	why = REASON("RXFCG not in the status (0x%08" PRIx32 ")", before);
	goto done;
    }

    /* Nothing more to deliver: what happens next is the driver's. */
    pthread_mutex_lock(&s->lock);
    s->deliver = false;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    rc = dw1000_rx_start(dw, DW1000_RX_IMMEDIATE);
    after = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);

    printf("    H2: rx_start rc=%d, status 0x%08" PRIx32 " -> 0x%08" PRIx32
	   " (RXFCG %d -> %d, HSRBP %d -> %d, ICRBP %d -> %d)\n",
	   rc, before, after,
	   (before & DW1000_FLG_SYS_STATUS_RXFCG) != 0,
	   (after  & DW1000_FLG_SYS_STATUS_RXFCG) != 0,
	   (before & DW1000_FLG_SYS_STATUS_HSRBP) != 0,
	   (after  & DW1000_FLG_SYS_STATUS_HSRBP) != 0,
	   (before & DW1000_FLG_SYS_STATUS_ICRBP) != 0,
	   (after  & DW1000_FLG_SYS_STATUS_ICRBP) != 0);

    dw1000_process_events(dw);

    printf("    H2: rx_ok=%d after the pass\n", (int)evt.rx_ok);

    if (evt.rx_ok)
	why = "the frame survived the second dw1000_rx_start after all";

done:
    dw1000_txrx_off(dw);
    return why;
}


/* H3 (both builds).
 *
 * The four transmit flags of a completion that is consumed rather than
 * reported are never cleared: _dw1000_tx_release() books the state and
 * writes nothing, dw1000_tx_clear_status_done() writes only TXFRS, and
 * dw1000_tx_start() clears them for a delayed send alone, relying on
 * "automatically cleared at the next transmitter enable" (UM 7.2.17)
 * otherwise. A start the chip drops is never a transmitter enable, so
 * the dropped send inherits the previous send's flags.
 *
 * Half A: a standing TXFRS is consumed by a send the chip then drops,
 * and the next pass reports that stale TXFRS as the dropped send's
 * completion -- a tx_done for a frame that never left.
 *
 * Half B: a polling host acknowledges with dw1000_tx_clear_status_done(),
 * which leaves TXFRB, TXPRS and TXPHS standing; _dw1000_tx_dropped()
 * takes any of them as proof the send is real, so the drop is never
 * found out and every later send is refused DW1000_TX_ERR_BUSY.
 */
static const char *
hunt_stale_tx_flags(dw1000_t *dw, struct stub *s)
{
    const char *why = NULL;
    unsigned    tx_before, tx_after;
    uint32_t    status;
    int         i, rc;

    cfgp->rx_keep_on = 0;
    dw1000_rx_set_timeout(dw, 0);
    dw1000_rx_set_timeout_preamble(dw, 0);

    pthread_mutex_lock(&s->lock);
    s->deliver          = false;
    s->tx_done_delay_ms = 0;
    pthread_mutex_unlock(&s->lock);

    /*-- Half A ------------------------------------------------------*/
    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0) {
	why = "A: the first send was refused";
	goto done;
    }
    for (i = 0; i < 200 && !dw1000_tx_is_status_done(dw); i++)
	usleep(10000);
    if (!dw1000_tx_is_status_done(dw)) {
	why = "A: the first send did not complete";
	goto done;
    }
    /* Deliberately not processed: the completion stands. */

    dw1000_emulation_drop_next_start(emup);

    pthread_mutex_lock(&s->lock);
    tx_before = s->tx_count;
    pthread_mutex_unlock(&s->lock);

    evt_reset();
    rc = dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
			DW1000_TX_IMMEDIATE);
    if (rc != 0) {
	why = REASON("A: the second send was refused (%d)", rc);
	goto done;
    }
    usleep(100000);
    pthread_mutex_lock(&s->lock);
    tx_after = s->tx_count;
    pthread_mutex_unlock(&s->lock);
    if (tx_after != tx_before) {
	why = "A: the second send reached the medium; it was meant to be"
	      " dropped";
	goto done;
    }

    status = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
    dw1000_process_events(dw);
    printf("    H3A: after the dropped start, status 0x%08" PRIx32
	   " (TXFRS=%d), tx_done=%d tx_dropped=%d state=%u\n",
	   status, (status & DW1000_FLG_SYS_STATUS_TXFRS) != 0,
	   (int)evt.tx_done, (int)evt.tx_dropped, dw->state);
    if (!evt.tx_done) {
	why = "A: no phantom completion after all";
	goto done;
    }

    /*-- Half B ------------------------------------------------------*/
    dw1000_txrx_off(dw);

    evt_reset();
    if (dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
		       DW1000_TX_IMMEDIATE) != 0) {
	why = "B: the first send was refused";
	goto done;
    }
    for (i = 0; i < 200 && !dw1000_tx_is_status_done(dw); i++)
	usleep(10000);
    if (!dw1000_tx_is_status_done(dw)) {
	why = "B: the first send did not complete";
	goto done;
    }
    dw1000_tx_clear_status_done(dw);            /* the polling host's ack */

    status = _dw1000_reg_read32(dw, DW1000_REG_SYS_STATUS, DW1000_OFF_NONE);
    printf("    H3B: after clear_status_done, status 0x%08" PRIx32
	   " (TXFRB=%d TXPRS=%d TXPHS=%d TXFRS=%d)\n", status,
	   (status & DW1000_FLG_SYS_STATUS_TXFRB) != 0,
	   (status & DW1000_FLG_SYS_STATUS_TXPRS) != 0,
	   (status & DW1000_FLG_SYS_STATUS_TXPHS) != 0,
	   (status & DW1000_FLG_SYS_STATUS_TXFRS) != 0);

    dw1000_emulation_drop_next_start(emup);

    evt_reset();
    rc = dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
			DW1000_TX_IMMEDIATE);
    if (rc != 0) {
	why = REASON("B: the send to be dropped was refused (%d)", rc);
	goto done;
    }

    /* Well past the frame's airtime and the detector's millisecond, and
     * with the two sightings _dw1000_tx_dropped() asks for. */
    for (i = 0; i < 5; i++) {
	usleep(20000);
	dw1000_process_events(dw);
    }

    rc = dw1000_tx_send(dw, (uint8_t *)payload, PAYLOAD_LEN,
			DW1000_TX_IMMEDIATE);
    printf("    H3B: tx_dropped=%d state=%u, next send rc=%d"
	   " (DW1000_TX_ERR_BUSY=%d)\n",
	   (int)evt.tx_dropped, dw->state, rc, DW1000_TX_ERR_BUSY);

    if (evt.tx_dropped) {
	why = "B: the dropped send was found out after all";
	goto done;
    }
    if (rc != DW1000_TX_ERR_BUSY)
	why = REASON("B: the next send answered %d, not DW1000_TX_ERR_BUSY",
		     rc);

done:
    dw1000_txrx_off(dw);
    dw1000_process_events(dw);
    dw1000_txrx_off(dw);
    return why;
}
