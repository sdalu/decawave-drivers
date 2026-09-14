/*
 * Copyright (c) 2024-2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Client for the remote service (rsvc) protocol: a datagram unix socket
 * to a server, with a reader thread.
 *
 * A call writes its request and blocks on a semaphore until the reader
 * thread matches a reply by sequence id, so several threads may call at
 * once. A frame the server sends unprompted carries the interrupt flag
 * and is handed to the handler registered for its service type.
 */

#if defined(__linux__)
#define _GNU_SOURCE
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/uio.h>
#include <stddef.h>
#include <assert.h>
#include <stdatomic.h>
#include <pthread.h>
#if defined(__FreeBSD__)
#include <pthread_np.h>
#endif
#include <semaphore.h>
#include <sys/queue.h>

#include "rsvc.h"

#if defined(__FreeBSD__)
#define THREAD_NAMELEN MAXCOMLEN
#elif defined(__linux__)
#define THREAD_NAMELEN 15
#else
#error "can't infer pthread name length"
#endif

#define RSVC_HDR_FLG_INCLUDE_NICKNAME		0x01
#define RSVC_HDR_FLG_INTERRUPT			0x02

#ifdef RSVC_WITH_DEBUG
#  define RSVC_DEBUG(fmt, ...) fprintf(stderr, "[RSVC] "fmt"\n", ##__VA_ARGS__)
#else
#  define RSVC_DEBUG(fmt, ...) do { } while (0)
#endif

#ifdef RSVC_WITH_DEBUG_LOCK
#ifndef RSVC_WITH_DEBUG
#error RSVC_WITH_DEBUG must be defined if RSVC_WITH_DEBUG_LOCK is used
#endif
#define RSVC_LOCK(rsvc)							\
    do {								\
	char tname[THREAD_NAMELEN+1] = "?failed?";			\
	pthread_getname_np(pthread_self(), tname, THREAD_NAMELEN);	\
	RSVC_DEBUG("rsvc lock (thread=%s, func=%s)",			\
		   tname[0] ? tname : "?unnamed?", __func__);		\
	rsvc_lock(rsvc);						\
    } while(0)

#define RSVC_UNLOCK(rsvc)						\
    do {								\
	char tname[THREAD_NAMELEN+1] = "?failed?";			\
	pthread_getname_np(pthread_self(), tname, THREAD_NAMELEN);	\
	RSVC_DEBUG("rsvc unlock (thread=%s, func=%s)",			\
		   tname[0] ? tname : "?unnamed?", __func__);		\
	rsvc_unlock(rsvc);						\
    } while(0)
#else
#define RSVC_LOCK(rsvc)   rsvc_lock(rsvc)
#define RSVC_UNLOCK(rsvc) rsvc_unlock(rsvc)
#endif


/*
 * Helper for setting error variable content
 */
#define RSVC_ERROR_SET(var, code)		\
    do {					\
	if (var)				\
	    *(var) = RSVC_ERR_##code;		\
    } while(0)


/*
 * Registered interrupt-like handlers.
 *
 * A flat table, scanned linearly: one handler is registered today (the
 * UWB channel), the lookup happens once per unsolicited frame, and a
 * tree here would only buy an unportable header.
 */
#define RSVC_INTERRUPTS_MAX	8

typedef struct rsvc_interrupt_descriptor {
    bool                         used;
    uint32_t                     type;
    rsvc_interrupt_handler_t     handler;
    void *                       args;
} rsvc_interrupt_descriptor_t;


/*
 * Structure for listing in-progress remote service operations
 */
typedef struct rsvc_inprogress {
    sem_t 	  sem;		// semaphore for blocking on reply
    uint16_t	  type;		// type of service requested
    uint64_t 	  id;		// id to match reply to request
    int8_t	  status;	// status of request (sucess, failure, ...)
    void 	 *data;		// buffer where to write reply data
    size_t 	 *datalen;	// size of the buffer
    TAILQ_ENTRY(rsvc_inprogress) entries; // chain of current in progress
} rsvc_inprogress_t;


/*
 * Context for the remote service
 */
struct rsvc {
    uint8_t connected   : 1;		// status of the connection to the
    uint8_t running     : 1;		//  remote service, intermediate states
    uint8_t initialized : 1;		//  are: connected, running, initialized
    
    uint64_t seq_id;			// global sequence id

    char *nickname;			// node nickname
    
    pthread_mutex_t mutex;		// mutex for driver access
    pthread_t thread;			// listening thread
    
    int fd;				// connection file descriptor
    struct sockaddr_un clt_addr;	// client address (our)
    struct sockaddr_un srv_addr;	// server address

    TAILQ_HEAD(, rsvc_inprogress) inprogress; // pending in-progress op.
    struct rsvc_interrupt_descriptor	// registered interrupts
                       interrupts[RSVC_INTERRUPTS_MAX];
};
 


/*
 * Header for input/output packet
 */
struct rsvc_inhdr {
    uint16_t type;			// type of service requested 
    uint64_t id;			// id of remote call 
    uint8_t  flags;
} __attribute__((packed));

struct rsvc_outhdr {
    uint16_t type;			// type of service requested
    uint64_t id;			// id of remote call
    int8_t   status;			// status of operation
    uint8_t  flags;
} __attribute__((packed));


/*
 * Structure for received data
 */
struct rsvc_data {
    uint8_t data[RSVC_DATA_MAXLEN];	// buffer to max data len
}  __attribute__((packed));



/*
 * Helpers for locking/unlocking context
 */
static inline void
rsvc_lock(rsvc_t *rsvc) {
    int rc = pthread_mutex_lock(&rsvc->mutex);
    assert(rc == 0);
}

static inline void
rsvc_unlock(rsvc_t *rsvc) {
    int rc = pthread_mutex_unlock(&rsvc->mutex);
    assert(rc == 0);
}


/* Caller holds the rsvc lock. */
static rsvc_interrupt_descriptor_t *
rsvc_interrupt_lookup(rsvc_t *rsvc, uint32_t type)
{
    for (size_t i = 0 ; i < RSVC_INTERRUPTS_MAX ; i++) {
	rsvc_interrupt_descriptor_t *idesc = &rsvc->interrupts[i];
	if (idesc->used && (idesc->type == type))
	    return idesc;
    }
    return NULL;
}


bool
rsvc_register(rsvc_t *rsvc,
	uint16_t type, rsvc_interrupt_handler_t handler, void *args) {
    bool ok = false;

    RSVC_LOCK(rsvc);

    if (rsvc_interrupt_lookup(rsvc, type) != NULL) {
	RSVC_DEBUG("register: handler already registered (0x%04x)", type);
    } else {
	for (size_t i = 0 ; i < RSVC_INTERRUPTS_MAX ; i++) {
	    rsvc_interrupt_descriptor_t *idesc = &rsvc->interrupts[i];
	    if (idesc->used)
		continue;
	    idesc->used    = true;
	    idesc->type    = type;
	    idesc->handler = handler;
	    idesc->args    = args;
	    ok = true;
	    break;
	}
	if (!ok)
	    RSVC_DEBUG("register: no free handler slot (0x%04x)", type);
    }

    RSVC_UNLOCK(rsvc);

    return ok;
}


bool
rsvc_unregister(rsvc_t *rsvc, uint16_t type)
{
    RSVC_LOCK(rsvc);

    rsvc_interrupt_descriptor_t *idesc = rsvc_interrupt_lookup(rsvc, type);

    if (idesc != NULL)
	idesc->used = false;

    RSVC_UNLOCK(rsvc);

    return idesc != NULL;
}



/**
 * Loop waiting for incomming packet,
 * and dispatching them to the waiting thread
 */
void *
rsvc_loop(void *args)
{
#if defined(__FreeBSD__)
    pthread_setname_np(pthread_self(), "rsvc");
#endif
    
    rsvc_t *rsvc = args;

    struct rsvc_outhdr hdr;  // packet header
    struct rsvc_data   data; // packet data
    
    struct iovec iov[2] = {
        { .iov_base = &hdr,  .iov_len = sizeof(hdr)  },
	{ .iov_base = &data, .iov_len = sizeof(data) },
    };
    
    while(1) {
	rsvc_inprogress_t *inprogress;
	ssize_t size, datasize;
	
	// Read packet 
	RSVC_DEBUG("loop: waiting for packet (read)");
    read_again:
	size = readv(rsvc->fd, iov, 2);
	if ((size < 0) && (errno == EINTR))
	    goto read_again;
	datasize = size - sizeof(hdr);
	
	// Force fixing problems now!
	assert(size     >= 0);		// -> fix socket communication
	assert(datasize >= 0);		// -> fix server implementation

	// Interruption-like ?
	if (hdr.flags & RSVC_HDR_FLG_INTERRUPT) {
	    RSVC_DEBUG("loop: processing interrupt-like handler"
		       " (0x%04x)", hdr.type);
	    RSVC_LOCK(rsvc);
	    rsvc_interrupt_descriptor_t *idesc =
		rsvc_interrupt_lookup(rsvc, hdr.type);
	    RSVC_UNLOCK(rsvc);
	    assert(idesc != NULL);
	    idesc->handler(rsvc, hdr.type, &data, datasize, idesc->args);

	// Reply to one of our request?
	} else {
	    RSVC_DEBUG("loop: processing in-progress operation"
		       " (%lu)", hdr.id);
	    // Lookup for an in progress operation
	    RSVC_LOCK(rsvc);
	    for (inprogress = TAILQ_FIRST(&rsvc->inprogress) ;
		 inprogress ;
		 inprogress = TAILQ_NEXT(inprogress, entries)) {
		if (inprogress->id == hdr.id) {
		    TAILQ_REMOVE(&rsvc->inprogress, inprogress, entries);
		    break;
		}
	    }
	    RSVC_UNLOCK(rsvc);
	    
	    // Sanity check
	    assert(inprogress != NULL);
	    assert((inprogress->datalen != NULL) || (inprogress->data == NULL));

	    // Transfer information to the in-progress structure
	    inprogress->status = hdr.status;

	    if (inprogress->data) {
		if (*inprogress->datalen > (size_t)datasize)
		    *inprogress->datalen = datasize;
		memcpy(inprogress->data, &data, *inprogress->datalen);
	    }
	    
	    // Unlock waiting thread
	    sem_post(&inprogress->sem);
	}

	RSVC_DEBUG("loop: remote service data processed");
    }
}



int
rsvc_call_extended(rsvc_t *rsvc, uint16_t type,
	  void *in, size_t inlen, void *out, size_t *outlen, uint8_t flags) {

    ssize_t size;
    int rc;
    
    // Sanity check
    assert((outlen != NULL) || (out == NULL));
    
    /* Prepare query/reply structures
     */
    struct rsvc_inhdr hdr = {
        .type  = type,
	.flags = flags,
    };

    bool   nick_included = flags & RSVC_HDR_FLG_INCLUDE_NICKNAME;
    char  *nickname      = nick_included ? rsvc->nickname : NULL;
    size_t nicklen       = nick_included ? strlen(nickname) + 1 : 0;
    
    struct iovec iov[3] = {
        { .iov_base = &hdr,     .iov_len = sizeof(hdr) },
	{ .iov_base = nickname, .iov_len = nicklen     },
	{ .iov_base = in,       .iov_len = inlen       },
    };

    /* Register and initialize in-progress structure
     */
    struct rsvc_inprogress inprogress = {
	.type    = type,
	.data    = out,
	.datalen = outlen,
    };

    rc = sem_init(&inprogress.sem, 0, 0);
    assert(rc == 0);
    RSVC_LOCK(rsvc);
    hdr.id = inprogress.id = rsvc->seq_id++;
    TAILQ_INSERT_TAIL(&rsvc->inprogress, &inprogress, entries);
    RSVC_UNLOCK(rsvc);
    
    /* Perform query
     */
 send_again:
    size = writev(rsvc->fd, iov, 3); // atomic for datagram socket
    if ((size < 0) && (errno == EINTR)) 
	goto send_again;
    assert(size >= 0);

    /* Waiting...
     */
 wait_again:
    if (sem_wait(&inprogress.sem) < 0) {
	assert(errno == EINTR);
	goto wait_again;
    }

    /* Process reply
     */
    // Remove in-progress from list
    RSVC_LOCK(rsvc);
    TAILQ_REMOVE(&rsvc->inprogress, &inprogress, entries);
    RSVC_UNLOCK(rsvc);

    // Properly destroy semaphore
    //  (before stack allocated data is removed)
    rc = sem_destroy(&inprogress.sem);
    assert(rc == 0);

    // Job's done
    return inprogress.status;
}



rsvc_t *
rsvc_open(char *socket_path, char *nickname, int *err)
{
    
    // Sanity check nickname
    if (strlen(nickname) > RSVC_NICKNAME_MAXLEN) {
	RSVC_DEBUG("open: nickname too long");
	RSVC_ERROR_SET(err, DATA_TOO_BIG);
	return NULL;
    }

    // Create instance
    rsvc_t *rsvc = calloc(1, sizeof(*rsvc));
    if (rsvc == NULL)
	goto failed;
    TAILQ_INIT(&rsvc->inprogress);
    rsvc->fd = -1;
    rsvc->nickname = nickname;
    
    
    /* Configure UNIX socket connection
     */
    int r = -1;

    // Server address
    memset(&rsvc->srv_addr, 0, sizeof(rsvc->srv_addr));
    rsvc->srv_addr.sun_family = AF_UNIX;
    r = snprintf(rsvc->srv_addr.sun_path, sizeof(rsvc->srv_addr.sun_path),
		 "%s", socket_path);
    if ((r <= 0) || ((size_t)r >= sizeof(rsvc->srv_addr.sun_path))) {
	RSVC_DEBUG("open: socket path too long (max=%zu)",
		   sizeof(rsvc->srv_addr.sun_path));
	RSVC_ERROR_SET(err, SOCKET_PATH);
	goto failed;
    }

    // Client address (necessary for bidirectional datagram)
    memset(&rsvc->clt_addr, 0, sizeof(rsvc->clt_addr));
    rsvc->clt_addr.sun_family = AF_UNIX;
    r = snprintf(rsvc->clt_addr.sun_path, sizeof(rsvc->clt_addr.sun_path),
	     "%s.%ld", socket_path, (long)getpid());
    if ((r <= 0) || ((size_t)r >= sizeof(rsvc->srv_addr.sun_path))) {
	RSVC_DEBUG("open: socket path too long (max=%zu)",
		   sizeof(rsvc->srv_addr.sun_path));
	RSVC_ERROR_SET(err, SOCKET_PATH);
	goto failed;
    }

    // Client socket
    if (((rsvc->fd = socket(AF_UNIX, SOCK_DGRAM, 0))    < 0) ||
	(bind(rsvc->fd,    (struct sockaddr*)&rsvc->clt_addr,
	                  sizeof(rsvc->clt_addr))       < 0) ||
	(connect(rsvc->fd, (struct sockaddr*)&rsvc->srv_addr,
		          sizeof(rsvc->srv_addr))       < 0)) {
	unlink(rsvc->clt_addr.sun_path);
	RSVC_DEBUG("open: socket/bind/connect failed");
	RSVC_ERROR_SET(err, SOCKET);
	goto failed;
    }

    // Mark as connected
    rsvc->connected = 1;

    
    /* Register internal management function and start thread
     */
    // Initialise mutex
    if (pthread_mutex_init(&rsvc->mutex, NULL) != 0) {
	RSVC_DEBUG("open: creating mutex failed");
	RSVC_ERROR_SET(err, THREAD);
	goto failed;
    }

    // Start reception loop
    if (pthread_create(&rsvc->thread, NULL, rsvc_loop, rsvc) != 0) {
	pthread_mutex_destroy(&rsvc->mutex);
	RSVC_DEBUG("open: creating thread failed");
	RSVC_ERROR_SET(err, THREAD);
	goto failed;
    }

    // Mark as running
    rsvc->running = 1;


    /* Register ourself to the server with an initialisation call
     */
    // Perform init call
    if (rsvc_call_extended(rsvc, RSVC_OPEN, NULL, 0, NULL, NULL,
			   RSVC_HDR_FLG_INCLUDE_NICKNAME) < 0) {
	RSVC_DEBUG("open: init remote call failed");
	RSVC_ERROR_SET(err, IO);
	goto failed;
    }

    // Mark as initialized
    rsvc->initialized = 1;


    /* Job's done
     */
    return rsvc;

    
    /* Failed
     */
 failed:
    if (rsvc) {
	rsvc_close(rsvc);
	free(rsvc);
    }
    return NULL;
}



void
rsvc_close(rsvc_t *rsvc)
{
    // Notify
    if (rsvc->initialized) {
	rsvc_call_extended(rsvc, RSVC_CLOSE, NULL, 0, NULL, NULL, 0);
	rsvc->initialized = 0;
    }

    // Stop thread
    if (rsvc->running) {
	pthread_cancel(rsvc->thread);
	pthread_mutex_destroy(&rsvc->mutex);
	rsvc->running = 0;
    }

    // Destroy socket
    if (rsvc->connected) {
	unlink(rsvc->clt_addr.sun_path);
	rsvc->connected = 0;
    }

    // Drop the registered handlers
    for (size_t i = 0 ; i < RSVC_INTERRUPTS_MAX ; i++)
	rsvc->interrupts[i].used = false;
}

void
_rsvc_cleanup(rsvc_t *rsvc)
{
    if (rsvc != NULL)
	unlink(rsvc->clt_addr.sun_path);
}


// Local Variables:
// mode: c
// c-basic-offset: 4
// End:
