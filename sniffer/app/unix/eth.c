/*
 * Copyright (c) 2019
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/ether.h>
#include <linux/wireless.h>
#include <linux/if_packet.h>
#include <arpa/inet.h>
#include <errno.h>
#include <assert.h>

#include "config.h"
#include "eth.h"

// https://gist.github.com/austinmarton/2862515

#if IFHWADDRLEN != ETHER_ADDR_LEN
#error miscmatch IFHWADDRLEN vs ETHER_ADDR_LEN
#endif

struct eth_ctx {
    char     ifname[IFNAMSIZ];		// Interface name
    int      ifindex;			// Inteface index
    uint8_t  hwaddr[ETHER_ADDR_LEN]; 	// Interface MAC address
    int      fd;			// File descriptor to raw socket
    uint16_t proto; 			// Protocol number to use
    			// /!\ length if  <= 0x05DC, proto if > 0x5DC
};

static struct eth_ctx eth = {
    .fd		= -1,
    .proto	=  0,
};


static int
_is_valid_interface(int fd, char *ifname, const char **errmsg)
{
    static const char *errmsg_failed         = "failed querying interface";
    static const char *errmsg_wireless       = "wireless interface";
    static const char *errmsg_loopback       = "loopback interface";
    static const char *errmsg_not_up_running = "interface not up and running";
    struct ifreq ifreq;
    
    /* Check if wireless interface */
    memset(&ifreq, 0, sizeof(ifreq));
    strncpy(ifreq.ifr_name, ifname, IFNAMSIZ);
    if (ioctl(fd, SIOCGIWNAME, &ifreq) >= 0) {
	if (errmsg) { *errmsg = errmsg_wireless; }
	return 0;
    }
    
    /* Check interface status and type */
    memset(&ifreq, 0, sizeof(ifreq));
    strncpy(ifreq.ifr_name, ifname, IFNAMSIZ);
    if (ioctl(fd, SIOCGIFFLAGS, &ifreq) < 0) {
	if (errmsg) { *errmsg = errmsg_failed; }
	WARN_ERRNO("failed to get information about %s", ifname);
	return -errno;
    } 
    if (!(ifreq.ifr_flags & IFF_UP      ) ||
	!(ifreq.ifr_flags & IFF_RUNNING )) {
	if (errmsg) { *errmsg = errmsg_not_up_running; }
	return 0;
    }
    if (ifreq.ifr_flags & IFF_LOOPBACK) {
	if (errmsg) { *errmsg = errmsg_loopback; }
	return 0;
    }

    /* Found one */
    return 1;
}    
   


int
eth_validate_interface(char *ifname, const char **errmsg)
{
    static const char *errmsg_failed = "failed querying interface";

    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) {
	if (errmsg) { *errmsg = errmsg_failed; }
	return -errno;
    }
    int rc = _is_valid_interface(fd, ifname, errmsg);
    close(fd);
    return rc;
}



int
eth_get_first_interface(char *ifname)
{
    int fd = -1;
    int rc = -ENOENT;
    struct if_nameindex *ifnameidx_list = NULL;

    /* Open a socket for performing ioctl */
    if ((fd = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
	rc = -errno;
	goto leave;
    }

    /* Iterate through interfaces to find an exceptable one */
    if ((ifnameidx_list = if_nameindex()) == NULL) {
	rc = -errno;
	goto leave;
    }
    for (struct if_nameindex *ifnameidx = ifnameidx_list        ;
	 ifnameidx->if_index != 0 || ifnameidx->if_name != NULL ;
	 ifnameidx++) {
	
	if (_is_valid_interface(fd, ifnameidx->if_name, NULL) > 0) {
	    strncpy(ifname, ifnameidx->if_name, IFNAMSIZ);
	    rc = 0;
	}
    }

 leave:
    if (fd             >= 0   ) close(fd);
    if (ifnameidx_list != NULL) if_freenameindex(ifnameidx_list);
    return rc;
}



int
eth_init(char *ifname, uint16_t proto, uint8_t *hwaddr)
{
    struct eth_ctx *ctx = &eth;

    /* Proto */
    if ((proto > 0) && (proto <= 0x05DC))
	return -EINVAL;
    ctx->proto = proto;
    
    /* Copy interface name */
    int ifname_len = strlen(ifname);
    if (ifname_len >= IFNAMSIZ)
	return -EINVAL;
    memcpy(&eth, ifname, ifname_len + 1);
    
    /* Open RAW socket to send on */
    if ((ctx->fd = socket(AF_PACKET, SOCK_RAW, htons(ctx->proto))) < 0) {
	WARN_ERRNO("failed to open raw socket for ethernet");
	return -errno;
    }

    /* Get the index of the interface to send on */
    struct ifreq if_idx = { 0 };
    strncpy(if_idx.ifr_name, ctx->ifname, IFNAMSIZ);
    if (ioctl(ctx->fd, SIOCGIFINDEX, &if_idx) < 0) {
	WARN_ERRNO("failed to get interface index");
	return -errno;
    }
    ctx->ifindex = if_idx.ifr_ifindex;
	
    /* Get the MAC address of the interface to send on */
    struct ifreq if_mac = { 0 };
    strncpy(if_mac.ifr_name, ctx->ifname, IFNAMSIZ);
    if (ioctl(ctx->fd, SIOCGIFHWADDR, &if_mac) < 0) {
	WARN_ERRNO("failed to get MAC address");
	return -1;
    }
    memcpy(ctx->hwaddr, if_mac.ifr_hwaddr.sa_data, IFHWADDRLEN);
    if (hwaddr) {
	memcpy(hwaddr, ctx->hwaddr, IFHWADDRLEN);
    }
    
    INFO("ethernet interface %s successfully intialized", ifname);
    return 0;
}



int
eth_send(const uint8_t *dstaddr, void *data, size_t datalen)
{
    struct eth_ctx *ctx = &eth;

    struct ether_header eh = { 
        .ether_type = htons(ctx->proto ? ctx->proto : datalen),
    };
    memcpy(eh.ether_shost, ctx->hwaddr, ETHER_ADDR_LEN);
    memcpy(eh.ether_dhost, dstaddr,     ETHER_ADDR_LEN);

    struct sockaddr_ll socket_address = {
        .sll_ifindex = ctx->ifindex,
	.sll_halen   = ETHER_ADDR_LEN,
    };
    memcpy(socket_address.sll_addr, dstaddr, ETHER_ADDR_LEN);
    
    struct iovec iovec[2] = {
	{ .iov_base = &eh,  .iov_len = sizeof(eh) },
	{ .iov_base = data, .iov_len = datalen    },
    };
    struct msghdr msg = {
        .msg_name     = &socket_address,
	.msg_namelen  = sizeof(socket_address),
	.msg_iov      = iovec,
        .msg_iovlen   = 2,
    };

    if (sendmsg(ctx->fd, &msg, 0) < 0) {
	WARN_ERRNO("failed to send ethernet frame");
	return -errno;
    }

    return 0;
}



int
eth_parse_addr(const char *str, uint8_t *addr)
{    
    unsigned int val[6];
    int scan_end;
    int s = sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x%n",
		   &val[0], &val[1], &val[2],
		   &val[3], &val[4], &val[5], &scan_end);
    if ((s != 6) || (str[scan_end] != '\0'))
	return -1;

    assert(ETH_HWADDR_SIZE >= 6);
    for (int i = 0 ; i < 6 ; i++)
	addr[i] = val[i];

    return 0;
}
   
