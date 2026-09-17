/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * probe/include/dw1000/probe/port.h under Linux/Unix: a monotonic clock,
 * a pthread-backed latched binary semaphore for dw1000_probe_port_wait()
 * / dw1000_probe_port_wake(), and a line on stdout.
 *
 * Deliberately identical in shape to port/emulation/src/port.c, which is
 * already POSIX/pthread-based and already correct; this is that same
 * implementation for a real host rather than a simulated one. The two
 * are kept as separate translation units (rather than one shared file)
 * because they are separate PORTS in the manifest: an application
 * building for the emulation medium and one driving real hardware pick
 * one or the other at link time, the same way any two OSAL ports do,
 * even though nothing here differs from it today. A port that later
 * needs to depend on something Linux-specific (a HAT's GPIO edge count
 * for interrupt diagnostics, say) gets that without touching the
 * emulation port's build.
 *
 * "Interrupt context" here is the application's own SIGIO/GPIO-edge
 * handling of the DW1000's interrupt line calling
 * dw1000_probe_port_wake(); see probe/app/unix/main.c, which runs that
 * on an ordinary pthread rather than a real signal handler, so an
 * ordinary mutex is fine here.
 *
 * The condition variable is timed against CLOCK_MONOTONIC rather than
 * the default CLOCK_REALTIME, so that a step of the wall clock (NTP, a
 * user setting the time) cannot stretch or shorten a wait; that is also
 * why it needs pthread_condattr_setclock() rather than the static
 * PTHREAD_COND_INITIALIZER, and so a one-time init through pthread_once()
 * rather than a plain global.
 */

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>

#include <dw1000/probe/port.h>

/*===========================================================================*/
/* Time                                                                      */
/*===========================================================================*/

dw1000_probe_time_t
dw1000_probe_port_now(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (dw1000_probe_time_t)ts.tv_sec * 1000000u +
	   (dw1000_probe_time_t)(ts.tv_nsec / 1000);
}

void
dw1000_probe_port_sleep(uint32_t us)
{
    struct timespec req;

    req.tv_sec  = (time_t)(us / 1000000u);
    req.tv_nsec = (long)(us % 1000000u) * 1000L;

    /* nanosleep() may return early on a signal; resume with whatever it
     * left in req until the whole interval has passed.
     */
    while (nanosleep(&req, &req) != 0 && errno == EINTR)
	;
}

/*===========================================================================*/
/* Waiting for the radio                                                    */
/*===========================================================================*/

static pthread_mutex_t wait_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  wait_cond;
static bool            wait_pending;
static pthread_once_t  wait_once = PTHREAD_ONCE_INIT;

static void
wait_init(void)
{
    pthread_condattr_t attr;

    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&wait_cond, &attr);
    pthread_condattr_destroy(&attr);
}

bool
dw1000_probe_port_wait(uint32_t timeout_us)
{
    struct timespec deadline;
    bool woke;

    pthread_once(&wait_once, wait_init);

    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec  += (time_t)(timeout_us / 1000000u);
    deadline.tv_nsec += (long)(timeout_us % 1000000u) * 1000L;
    if (deadline.tv_nsec >= 1000000000L) {
	deadline.tv_sec  += 1;
	deadline.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&wait_lock);
    /* A wake-up already pending, including one that arrived before
     * this call was made, is consumed at once, timeout_us of 0
     * included: the loop condition is false and pthread_cond_timedwait()
     * is never entered. Otherwise it is entered with a deadline already
     * in the past, which returns ETIMEDOUT immediately: the "polls"
     * dw1000_probe_port_wait() promises for a zero timeout.
     */
    while (!wait_pending) {
	if (pthread_cond_timedwait(&wait_cond, &wait_lock, &deadline) != 0)
	    break;
    }
    woke = wait_pending;
    wait_pending = false;
    pthread_mutex_unlock(&wait_lock);

    return woke;
}

void
dw1000_probe_port_wake(void)
{
    pthread_once(&wait_once, wait_init);

    pthread_mutex_lock(&wait_lock);
    wait_pending = true;
    pthread_cond_signal(&wait_cond);
    pthread_mutex_unlock(&wait_lock);
}

/*===========================================================================*/
/* Output                                                                    */
/*===========================================================================*/

void
dw1000_probe_port_emit(const char *line)
{
    fputs(line, stdout);
    fputc('\n', stdout);
    fflush(stdout);
}

/*===========================================================================*/
/* The bus                                                                   */
/*===========================================================================*/

static pthread_mutex_t bus_lock = PTHREAD_MUTEX_INITIALIZER;

void
dw1000_probe_port_bus_lock(void)
{
    pthread_mutex_lock(&bus_lock);
}

void
dw1000_probe_port_bus_unlock(void)
{
    pthread_mutex_unlock(&bus_lock);
}
