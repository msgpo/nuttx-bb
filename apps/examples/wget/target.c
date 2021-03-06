/****************************************************************************
 * examples/wget/target.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <unistd.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/net/uip/uip.h>

#include <apps/netutils/uiplib.h>
#include <apps/netutils/resolv.h>
#include <apps/netutils/webclient.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Configuation Checks ******************************************************/
/* BEWARE:
 * There are other configuration settings needed in netutitls/wget/wgetc.s,
 * but there are default values for those so we cannot check them here.
 */

#ifndef CONFIG_EXAMPLE_WGET_IPADDR
#  error "You must define CONFIG_EXAMPLE_WGET_IPADDR"
#endif

#ifndef CONFIG_EXAMPLE_WGET_DRIPADDR
#  error "You must define CONFIG_EXAMPLE_WGET_DRIPADDR"
#endif

#ifndef CONFIG_EXAMPLE_WGET_NETMASK
#  error "You must define CONFIG_EXAMPLE_WGET_NETMASK"
#endif

#ifndef CONFIG_NET
#  error "You must define CONFIG_NET"
#endif

#ifndef CONFIG_NET_TCP
#  error "You must define CONFIG_NET_TCP"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_iobuffer[512];

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: callback
 ****************************************************************************/

static void callback(FAR char **buffer, int offset, int datend,
                     FAR int *buflen, FAR void *arg)
{
  (void)write(1, &((*buffer)[offset]), datend - offset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * wget_main
 ****************************************************************************/

int wget_main(int argc, char *argv[])
{
  struct in_addr addr;
#if defined(CONFIG_EXAMPLE_WGET_NOMAC)
  uint8_t mac[IFHWADDRLEN];
#endif

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLE_WGET_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_WGET_IPADDR);
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_WGET_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_WGET_NETMASK);
  uip_setnetmask("eth0", &addr);

  /* Then start the server */
  
  wget(CONFIG_EXAMPLE_WGET_URL, g_iobuffer, 512, callback, NULL);
  return 0;
}
