/*
 * Copyright (c) 2024-2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RSVC_H__
#define __RSVC_H__

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

#define RSVC_DATA_MAXLEN		500

#define RSVC_NICKNAME_MAXLEN		62

#define RSVC_OK 			 0
#define RSVC_ERR_IO			-1
#define RSVC_ERR_SOCKET_PATH 		-2
#define RSVC_ERR_SOCKET 		-3
#define RSVC_ERR_DATA_TOO_BIG		-4
#define RSVC_ERR_THREAD			-5
#define RSVC_ERR_TIMEOUT		-6


/*
 * Assigned service type
 */
#define RSVC_OPEN   			0x0001
#define RSVC_CLOSE 			0x0002
#define RSVC_SEED_GET                   0x0003
//#define RSVC_TIME_DELAY		0x0101
//#define RSVC_TIME_GET			0x0102
//#define RSVC_MUTEX			0x0201
//#define RSVC_SEMAPHORE		0x0202
#define RSVC_UWB_OPEN			0x0301
#define RSVC_UWB_CLOSE			0x0302
#define RSVC_UWB_IO			0x0303


/* Forward declaration */
struct rsvc;


/**
 * Remote service (rsvc) context
 */
typedef struct rsvc rsvc_t;


/**
 * Interrupt handler
 */
typedef void (*rsvc_interrupt_handler_t)(rsvc_t *rsvc, uint16_t type,
					 void *data, size_t length, void *args);


/**
 * Open connection to the remote service.
 *
 * @param socket_path		server address (unix socket)
 * @param nickname		nickname (must be unique)
 * @param error			if not NULL, will hold error
 *
 * @return pointer to rsvc context or NULL in case of error
 */
rsvc_t *rsvc_open(char *socket_path, char *nickname, int *error);


/**
 * Close the connection with the remote service.
 *
 * @param rsvc		pointer to rsvc context
 */
void rsvc_close(rsvc_t *rsvc);


void _rsvc_cleanup(rsvc_t *rsvc);


/*
 * Perform a remote service call with extended parameters. 
 */
int rsvc_call_extended(rsvc_t *rsvc, uint16_t type,
		       void *in, size_t inlen,
		       void *out, size_t *outlen, uint8_t flags);


/**
 * Perform a remote service call. 
 *
 * @param rsvc		pointer to rsvc context
 * @param type		type of service
 * @param in		buffer for the request
 * @param inlen		length of the request
 * @param out		buffer for reply
 * @param outlen	length of the buffer/reply
 *
 * @return status of the call to the remote service
 */
static inline 
int rsvc_io(rsvc_t *rsvc, uint16_t type,
	      void *in, size_t inlen, void *out, size_t *outlen) {
    return rsvc_call_extended(rsvc, type, in, inlen, out, outlen, 0);
}


/**
 * Perform a remote service call. 
 *
 * @param rsvc		pointer to rsvc context
 * @param type		type of service
 * @param in		buffer for the request
 * @param inlen		length of the request
 *
 * @return status of the call to the remote service
 */
static inline
int rsvc_i(rsvc_t *rsvc, uint16_t type, void *in, size_t inlen) {
    return rsvc_call_extended(rsvc, type, in, inlen, NULL, NULL, 0);
}


/**
 * Perform a remote service call. 
 *
 * @param rsvc		pointer to rsvc context
 * @param type		type of service
 * @param out		buffer for reply
 * @param outlen	length of the buffer/reply
 *
 * @return status of the call to the remote service
 */
static inline
int rsvc_o(rsvc_t *rsvc, uint16_t type, void *out, size_t *outlen) {
    return rsvc_call_extended(rsvc, type, NULL, 0, out, outlen, 0);
}


/**
 * Set how long a call waits for its reply.
 *
 * A request that is never answered used to be an unbounded hang: the
 * server is another program, and one that does not know a service type,
 * or has stopped, leaves the caller blocked for ever. The wait is
 * bounded instead, and a call that runs out returns
 * @p RSVC_ERR_TIMEOUT.
 *
 * The bound is a diagnostic, not a schedule. A reply travels over a
 * local socket and arrives in microseconds; anything approaching the
 * default has already broken whatever real-time behaviour the caller
 * had, and the point of the timeout is that it says so instead of
 * stopping. Setting it tight enough to expire on a merely slow server
 * turns a working run into a broken one, so err long.
 *
 * Set it before any traffic; it is read without locking on every call.
 *
 * @param rsvc		pointer to rsvc context
 * @param ms		milliseconds, or 0 to wait for ever as before
 */
void rsvc_set_reply_timeout(rsvc_t *rsvc, unsigned ms);


/**
 * Register an interrupt handler to deal with unsolicited 
 * remote service packet for the specified type.
 *
 * @param rsvc		pointer to rsvc context
 * @param type		type of service
 * @param handler	interrupt handler
 * @param args		arguments for interrupt handler
 *
 * @return false if registration failed (due to a handler already registered)
 */
bool rsvc_register(rsvc_t *rsvc, uint16_t type,
		   rsvc_interrupt_handler_t handler, void *args);


/**
 *  Unregister the interrupt handler specified by type.
 *
 * @param rsvc		pointer to rsvc context
 * @param type		type of service
 *
 * @return false if no handler was registered for that service
 */
bool rsvc_unregister(rsvc_t *rsvc, uint16_t type);

#endif

