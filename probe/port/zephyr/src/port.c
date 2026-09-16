/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma team, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    port.c
 * @brief   The probe's host glue for Zephyr.
 *
 * Time from the kernel's tick source, the latch from a k_sem, and a line
 * on the console. Nothing here knows a board: the interrupt line is the
 * application's to attach, and it calls dw1000_probe_port_wake() from its
 * handler.
 *
 * The latch is a k_sem with a limit of 1, which IS the semantics
 * <dw1000/probe/port.h> asks for and not an approximation of it: a give
 * before any take is remembered, a take consumes exactly one, and gives
 * beyond the limit are dropped rather than counted -- so wakes collapse.
 * k_sem_give() is safe from an interrupt handler, which is the context
 * the contract requires wake() to survive.
 */

#include <version.h>
#if (KERNEL_VERSION_NUMBER < 0x030100) || defined(CONFIG_LEGACY_INCLUDE_PATH)
#include <kernel.h>
#include <sys/printk.h>
#else
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#endif

#include <dw1000/probe/port.h>

/*===========================================================================*/
/* Time                                                                      */
/*===========================================================================*/

dw1000_probe_time_t
dw1000_probe_port_now(void)
{
    /* Ticks rather than k_uptime_get(): that one is milliseconds, and a
     * frame deadline is sub-millisecond. k_ticks_to_us_floor64() is the
     * conversion the kernel offers whatever CONFIG_SYS_CLOCK_TICKS_PER_SEC
     * happens to be, so no arithmetic here has to know the tick rate.
     */
    return (dw1000_probe_time_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

void
dw1000_probe_port_sleep(uint32_t us)
{
    k_usleep((int32_t)us);
}

/*===========================================================================*/
/* The latch                                                                 */
/*===========================================================================*/

/* Initial count 0, limit 1: nothing pending at boot, and never more than
 * one pending however many times wake() is called. */
static K_SEM_DEFINE(wake_sem, 0, 1);

bool
dw1000_probe_port_wait(uint32_t timeout_us)
{
    /* K_USEC(0) is K_NO_WAIT, so a zero timeout polls -- which is what
     * the contract promises -- and a pending give is still consumed. */
    return k_sem_take(&wake_sem, K_USEC(timeout_us)) == 0;
}

void
dw1000_probe_port_wake(void)
{
    k_sem_give(&wake_sem);
}

/*===========================================================================*/
/* Output                                                                    */
/*===========================================================================*/

void
dw1000_probe_port_emit(const char *line)
{
    printk("%s\n", line);
}

/*===========================================================================*/
/* The bus                                                                   */
/*===========================================================================*/

/* Not K_MUTEX_DEFINE'd as recursive, because the contract does not ask
 * for recursion and a caller that needs it has a structural problem
 * rather than a locking one. Zephyr's k_mutex does happen to allow the
 * owner to relock; nothing here relies on that. */
static K_MUTEX_DEFINE(bus_lock);

void
dw1000_probe_port_bus_lock(void)
{
    k_mutex_lock(&bus_lock, K_FOREVER);
}

void
dw1000_probe_port_bus_unlock(void)
{
    k_mutex_unlock(&bus_lock);
}
