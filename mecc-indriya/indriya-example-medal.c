/** ------------------------------------------------------------------------
 * Department of Automatic Control,
 * KTH - Royal Institute of Technology,
 * School of Electrical Engineering
 * @address: Osquldasvag 10, SE-10044, STOCKHOLM, Sweden
 * @author: António Gonga < gonga@ee.kth.se>
 *
 * @date: May 15th, 2014
 * @filename: indriya-medal-example.c
 * @description: this example is used together with the medal RDC layer
 *               for testbed evaluations such as INDRIYA and TWIST*
 * NOTICE: This file is part of research I have been developing at KTH. You
 *         are welcome to modify it, AS LONG AS you leave this head notice
 *         and the author is properly acknowledged.
 * ------------------------------------------------------------------------*/

#include "./indriya-medalconst.h"
#include "./nd-dataqueue.h"

#include "contiki.h"
#include "net/rime.h"
#include "random.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static struct rime_sniffer pkt_rcv_sniffer;

//static void (* input_pkt)(void);
static void input_pkt();

RIME_SNIFFER(pkt_rcv_sniffer, input_pkt, NULL);

/*---------------------------------------------------------------------------*/
PROCESS(nd_process, "neigh disc example process");
AUTOSTART_PROCESSES(&nd_process);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
extern uint8_t get_rounds();
extern uint8_t get_fraction();
extern uint16_t get_discovery_time();
extern uint16_t get_mean_discovery_time();
extern node_info_t * get_network_table();
/*---------------------------------------------------------------------------*/
static void input_pkt()
{
   //PRINTF("packet rcvd");
   PRINTF(">:%d,%d,%u,%u,%u-%u\n", get_rounds(), rimeaddr_node_addr.u8[0],
          CHANNEL_POOL_SIZE, get_fraction(), get_discovery_time() , get_mean_discovery_time());
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(nd_process, ev, data)
{

  /** the process exits removing the sniffer created.*/
  //PROCESS_EXITHANDLER(rime_sniffer_remove(&pkt_rcv_sniffer);)

  PROCESS_BEGIN();

  /** Create a sniffer to receive incoming packets*/
  RIME_SNIFFER(pkt_rcv_sniffer, input_pkt, NULL);

  /** Tell RIME to add the sniffer*/
  rime_sniffer_add(&pkt_rcv_sniffer);

  while(1) {
    PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

