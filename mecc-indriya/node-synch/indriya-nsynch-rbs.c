/*
 *
 * \author
 *         Antonio Gonga <gonga@kth.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "random.h"

#include "indriya-nsynch-tb.h"

#include "net/packetbuf.h"
#include "net/netstack.h"
#include "net/rime.h"
#include "dev/watchdog.h"

#include "sys/rtimer.h"

#include "dev/leds.h"
#include "./cc2420.h"
#include "dev/button-sensor.h"

#include "dev/serial-line.h"
#include "node-id.h"

#include "./indriya-medalconst.h"
#include "./nd-dataqueue.h"

#include<string.h>

#define IN_TWIST 1
#define IN_INDRIYA 1
#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
/*nodes info storage related
 *
 */
/*typedef struct{
    uint8_t id;
    uint8_t status;
    uint16_t sum_time;
}node_exptime_t;*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

//static node_exptime_t node_queue[];
/*---------------------------------------------------------------------------*/
static uint8_t i3e154_channels [] ={11, 12, 13, 14, 15, 16, 17,
                                    18, 19, 20, 21, 22, 23, 24, 25, 26};
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
typedef rtimer_clock_t my_clock_t;
static volatile rtimer_clock_t callback_time;

static struct rtimer generic_timer;
/*---------------------------------------------------------------------------*/
static volatile uint16_t slot_counter     = 0;

static volatile uint8_t max_num_rounds = 0;

static volatile uint8_t synch_start_flag = 0;
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t seq_number = 0;
/*---------------------------------------------------------------------------*/

static my_clock_t ref_timer;

/*---------------------------------------------------------------------------*/
static volatile uint8_t synch_is_started = 0;
/*---------------------------------------------------------------------------*/
#define DELTA_0 ONE_MSEC
static volatile my_clock_t delta_n = 0;
/*---------------------------------------------------------------------------*/
/*a callback function that the radio driver calls when an interrupt is triggered
 *indicating the reception of a packet.
 */
//static void packet_input(rtimer_clock_t);

/*---------------------------------------------------------------------------*/
PROCESS(node_synch_process, "Node synch process");
PROCESS(node_serial_process, "Node serial process");
AUTOSTART_PROCESSES(&node_synch_process, &node_serial_process);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * @brief main_time_syncher
 * @param t
 * @param ptr
 * @return
 */
static char time_syncher(struct rtimer *t, void* ptr){
    int r;

    if (slot_counter <= 4){

        slot_counter++;

        packetbuf_clear();
        data_packet_t *pkt = (data_packet_t*)packetbuf_dataptr();

        packetbuf_set_datalen(DISCOVERY_PACKET_SIZE);

        pkt->ptype = SYNCH_PACKET;
        pkt->seqno = seq_number+1;
        pkt->rounds= max_num_rounds;

        cc2420_set_channel(26); //i3e154_channels []
        //cc2420_set_channel(i3e154_channels[0]);

        NETSTACK_RADIO.on();

        NETSTACK_RADIO.send(packetbuf_dataptr(), DISCOVERY_PACKET_SIZE);

        NETSTACK_RADIO.off();

        leds_toggle(LEDS_BLUE);
        leds_toggle(LEDS_GREEN);
        leds_toggle(LEDS_RED);

        callback_time = ref_timer + (slot_counter*TS);

        r =  rtimer_set(&generic_timer, callback_time, 1,
                          (void (*)(struct rtimer *, void *))time_syncher, NULL);
        if(r){
            PRINTF("error scheduler\n");
            return 0;
        }
    }else{
        //turn radio off
        //NETSTACK_RADIO.off();

        //reset resynch flag
        synch_is_started = 0;

        NETSTACK_RADIO.on();
    }

    return r;
}
/*---------------------------------------------------------------------------*/
static void packet_input(){
    int len = 0;

    //read data here..
    packetbuf_clear();
    len = NETSTACK_RADIO.read((void*)packetbuf_dataptr(), PACKETBUF_SIZE);

    if(len > 0){
        packetbuf_set_datalen(len);
    }else{
        //invalid packet..
        return;
    }

    //we know that we correctply received the packet..
    data_packet_t *pkt = (data_packet_t*)packetbuf_dataptr();

    if(pkt->ptype == SYNCH_PACKET){

        if(synch_is_started == 0){

            synch_is_started = 1;

            slot_counter = 0;

            //get the current sequence number..
            seq_number = pkt->seqno;

            //rebroadcast the message...
            ref_timer = RTIMER_NOW() + TS/2;//(2*TS)/3;

            int r =  rtimer_set(&generic_timer, ref_timer, 1,
                             (void (*)(struct rtimer *, void *))time_syncher, NULL);
            if(r){
                   PRINTF("error scheduler\n");
                   return;
            }
        }

    }    
    /**we no longer need the space, therefore, we free it*/
    //dataqueue_buffer_free(dq_in);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief send_packet
 * @param sent
 * @param ptr
 */
static void send_packet(mac_callback_t sent, void *ptr){
   //do nothing...
}
/*---------------------------------------------------------------------------*/
/**
 * @brief send_list
 * @param sent
 * @param ptr
 * @param buf_list
 */
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list){
    //do nothing...
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*Contiki RDC driver specific function*/
static int on(void){
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int off(int keep_radio_on){
    return NETSTACK_RADIO.off();
}
/*---------------------------------------------------------------------------*/
static unsigned short channel_check_interval(void){
  return 8;
}
/*---------------------------------------------------------------------------*/
static void init(void){
  on();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_synch_process, ev, data){

    PROCESS_BEGIN();

    static struct etimer et;
    //static uint8_t max_num_rounds = 0;
    //PRINTF("%u\n", RTIMER_SECOND);

    //cc2420_set_channel(i3e154_channels[0]);
    //uart1_tinyos_frames(1);

    while(1) {
        PROCESS_YIELD();

        /*etimer_set(&et, 5*CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et) && (synch_start_flag == 1));

        slot_counter = 0;
        ref_timer = RTIMER_NOW()+2;

        int r;

        r =  rtimer_set(&generic_timer, ref_timer, 1,
                        (void (*)(struct rtimer *, void *))time_syncher, NULL);
        if(r){
            PRINTF("error scheduler\n");
        }

        max_num_rounds++;

        //PRINTF("r%u\n", max_num_rounds);

        if((max_num_rounds % MAX_REPEAT) == 0){
            //max_num_rounds = 0;
            synch_start_flag = 0;
            PRINTF("reps %u\n", max_num_rounds);
            PROCESS_YIELD();
             //break;
        }*/
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_serial_process, ev, data){

    PROCESS_BEGIN();

    while(1) {
        PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);

        char *command = (char*)data;

        if(command != NULL){
            if(!strcmp(command, "start")){
                synch_start_flag = 1;
                max_num_rounds = 0;
            }

            if(!strcmp(command, "newround")){

                slot_counter = 0;

                ref_timer = RTIMER_NOW()+2;

                uint8_t r =  rtimer_set(&generic_timer, ref_timer, 1,
                                (void (*)(struct rtimer *, void *))time_syncher, NULL);
                if(r){
                    PRINTF("error scheduler\n");
                }
            }
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver indriya_nodesynch_driver = {
  "NSYNCH-Indriya-synch",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
