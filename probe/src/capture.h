/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Private to probe/src: the capture's counters, owned by exchange.c (whose
 * dw1000_probe_*_capture() hooks are the only writers) and read by
 * solo.c's roles as well. Not under include/ because no application has
 * any business reading them: what a run counted is in its result.
 *
 * Every one is a monotonic count, never reset: a role takes a snapshot
 * when it starts and a difference when it ends. Read under the bus lock
 * (dw1000_probe_port_bus_lock()), as the writers run under it.
 */

#ifndef __DW1000_PROBE_CAPTURE_H__
#define __DW1000_PROBE_CAPTURE_H__

#include <stdint.h>

uint32_t _dw1000_probe_rx_count(void);         /* frames delivered      */
uint32_t _dw1000_probe_tx_count(void);         /* transmits completed   */
uint32_t _dw1000_probe_rx_error_count(void);   /* frames rejected       */

#endif
