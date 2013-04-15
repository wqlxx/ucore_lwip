#ifndef XV6_LWIP_ETHERNETIF_H_
#define XV6_LWIP_ETHERNETIF_H_
#include "lwip/netif.h"
err_t ethernetif_init(struct netif *);
void  ethernetif_input(struct netif *);

struct ethernetif {
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
  int (*send)(void *data, uint32_t len);
  int (*receive)(void *data, uint32_t len);
};

#endif // XV6_LWIP_ETHERNETIF_H_

