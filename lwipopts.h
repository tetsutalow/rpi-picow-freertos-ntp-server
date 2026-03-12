#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

#include "FreeRTOS.h"

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define NO_SYS                      0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (48 * 1024)
#define MEMP_NUM_TCP_PCB            16
#define MEMP_NUM_UDP_PCB            16
#define MEMP_NUM_SYS_TIMEOUT        20
#define PBUF_POOL_SIZE              32

#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DHCP                   1
#define LWIP_NETCONN                1
#define LWIP_SOCKET                 1
#define LWIP_STATS                  0

// 明示的に SNTP を無効化してポート 123 の衝突を防ぐ
#define SNTP_SERVER_ADDRESS         0
#define LWIP_SNTP                   0

#define TCP_MSS                     1460
#define TCP_WND                     (4 * TCP_MSS)
#define TCP_SND_BUF                 (4 * TCP_MSS)
#define TCP_SND_QUEUELEN            16

#define TCPIP_THREAD_PRIO           (configMAX_PRIORITIES - 1)
#define TCPIP_THREAD_STACKSIZE      4096
#define DEFAULT_TCP_RECVMBOX_SIZE   16
#define DEFAULT_UDP_RECVMBOX_SIZE   16
#define DEFAULT_ACCEPTMBOX_SIZE     16
#define TCPIP_MBOX_SIZE             16

#include <sys/time.h>
#include <errno.h>
#define LWIP_TIMEVAL_PRIVATE        0

#endif
