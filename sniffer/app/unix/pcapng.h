/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PCAPNG__H
#define __PCAPNG__H

#include <stdint.h>

#include "capture.h"

/**
 * Open a pcapng capture file and write its Section Header Block and its
 * one Interface Description Block.
 *
 * @p path of "-" means the file is stdout; anything else is created (or
 * truncated if it already exists) and opened for writing. The interface
 * this writer describes is fixed: LINKTYPE_IEEE802_15_4_WITH_FCS (195),
 * because a captured frame is forwarded whole, CRC included (see the
 * note on the CRC in capture.h), and timestamps in nanoseconds, which is
 * the resolution @p pcapng_write() gives them.
 *
 * Only one file is open at a time: a second call before @p pcapng_close()
 * returns -EBUSY rather than leaking the first.
 *
 * @param[in] path     the file to write, or "-" for stdout
 * @param[in] snaplen  the interface's snap length, written into the
 *                      Interface Description Block; this writer does
 *                      not itself enforce it against a frame's length
 * @return 0 on success, a negative errno on failure
 */
int pcapng_open(const char *path, uint32_t snaplen);

/**
 * Write one captured frame as an Enhanced Packet Block.
 *
 * The timestamp comes from @p frame->wall; @p frame->length becomes the
 * block's captured length and @p frame->reported its original length,
 * which is larger exactly when the frame was truncated on capture (see
 * @p CAPTURE_F_TRUNCATED in capture.h). When @p frame->flags carries
 * @p CAPTURE_F_METADATA, a one-line rendering of @p frame->meta is
 * attached as the block's packet comment (option code 1); a frame
 * without that flag gets no option list at all.
 *
 * @p note, when it is neither NULL nor empty, is appended to that
 * comment after a bar. It is where a dissector's line goes: the radio
 * metadata comes first because it is always the same shape and always
 * true, and a dissector's reading of the bytes comes second. A frame
 * with neither metadata nor a note gets no option list at all.
 *
 * @param[in] frame  the frame to write; must not be NULL
 * @param[in] note   extra text for the packet comment, or NULL
 * @return 0 on success, a negative errno on failure
 */
int pcapng_write(const struct capture_frame *frame, const char *note);

/**
 * Flush and close the file opened by @p pcapng_open().
 *
 * stdout (from @p path "-") is flushed rather than closed: it is not
 * this module's to close, and whoever set the process up may still want
 * to write to it afterwards.
 *
 * @return 0 on success, a negative errno on failure
 */
int pcapng_close(void);

#endif

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
