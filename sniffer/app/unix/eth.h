/*
 * Copyright (c) 2019
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ETH__H
#define __ETH__H

#include <stdint.h>
#include <net/ethernet.h>
#include <net/if.h>

#define ETH_NAME_SIZE   IFNAMSIZ
#define ETH_HWADDR_SIZE ETHER_ADDR_LEN

/**
 * Check if an interface is valid for our usage in eth_init.
 *
 * @param[in]  ifname	name of the interface to validate
 * @param[out] errmsg	string describing the error
 * @return <0		an error occured when trying to get information
 *			about the interface
 * @return  0		the interface is not valid (loopback, wireless, ...)
 * @return  1		the interface is valid
 */
int eth_validate_interface(char *ifname, const char **errmsg);

/**
 * Retrieve the name of the first valid interface.
 *
 * @param[out] ifname (must be of size ETH_NAME_SIZE)
 * @return < 0 in case of error
 */
int eth_get_first_interface(char *ifname);

/**
 * Initialize network interface.
 *
 * @param[in] ifname	network interface name
 * @param[in] proto	ethernet prototype number
 * @param[out] hwaddr   mac address used by selected interface
 *                      must be NULL or of size ETH_HWADDR_SIZE
 * @return < 0		in case of error
 */
int eth_init(char *ifname, uint16_t proto, uint8_t *hwaddr);

/**
 * Send an ethernet packet, as a header followed by a payload.
 *
 * The two are kept apart rather than concatenated by the caller: the
 * header is wire.c's, built fresh for each frame, while the payload is
 * the frame still in the ring slot capture.c read it into, and sendmsg(2)
 * takes both as iovecs so neither is copied again on its way out.
 *
 * @param[in] dst	ethernet address of the remote host
 * @param[in] hdr	header to send ahead of the payload, or NULL
 * @param[in] hdrlen	size of @p hdr, 0 when there is none
 * @param[in] data	data to be send
 * @param[in] datalen	size of the data to send
 */
int eth_send(const uint8_t *dst,
	     const void *hdr, size_t hdrlen, const void *data, size_t datalen);

/**
 * Parse a string to an ethernet address.
 *
 * @param[in]  str	string representing an ethernet address
 * @param[out] addr	pointer to an ethernet address
 *			(must have ETH_HWADDR_SIZE bytes allocated)
 * @return < 0 in case of error
 */
int eth_parse_addr(const char *str, uint8_t *addr);

#endif
