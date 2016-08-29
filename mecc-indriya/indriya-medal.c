/*
 * \filename: indriya-medal.c
 * \description: This is a  RDC layer protocol implementation
 *               of multichannel wireless neighbor discovery with additional
 *               epidemic dissemination mechanism.
 *
 * \author
 *         Antonio Gonga <gonga@kth.se>
 * \date
 *      May 20th, 2013, 01:40 AM
 *
 */
/** ------------------------------------------------------------------------
 * Department of Automatic Control,
 * KTH - Royal Institute of Technology,
 * School of Electrical Engineering
 * @address: Osquldasvag 10, SE-10044, STOCKHOLM, Sweden
 * @author: António Gonga < gonga@ee.kth.se>
 *
 * @lastModified: May 15th, 2014
 * @filename: indriya-medal.c
 * @description: this  is a RDC layer that implements multichannel neighbor
 *               discovery with epidemic information dissemination.
 *               This version is for testbed evaluations such as INDRIYA/TWIST*
 * NOTICE: This file is part of research I have been developing at KTH. You
 *         are welcome to modify it, AS LONG AS you leave this head notice
 *         and the author is properly acknowledged.
 * ----------------------------------------------------------------------------*/

#include "contiki-conf.h"
#include "dev/leds.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "lib/random.h"
/** MEDAL HEader FIle*/
#include "./indriya-medal.h"
/** */
#include "net/netstack.h"
#include "net/rime.h"
#include "sys/compower.h"
#include "sys/pt.h"
#include "sys/rtimer.h"


#include "dev/leds.h"
#include "./cc2420.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#include "node-id.h"

#include "./indriya-medalconst.h"
#include "./nd-dataqueue.h"

#include <string.h>

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SIZE_NETWORK 11
/*---------------------------------------------------------------------------*/
typedef struct{
    uint8_t channel;
    uint8_t num_nodes[SIZE_NETWORK];
    uint16_t prob_vec[SIZE_NETWORK];
}prob_table_t;
/*---------------------------------------------------------------------------*/
/** -------------------------------------------------------------------------
 *     (2*k + N - 1) - sqrt((2*k + N - 1)^2 - 4*k*N)
 * p = ---------------------------------------------------
 *                     2*N
 * where, k - number of channels
 *        N - clique size..
 ----------------------------------------------------------------------------*/
/** This lookup table stores values of specific network sizes, number of
 *  channels and their corresponding optimal transmission probabilities.
 *  Given a specific case, a node can look for the pretended value in this
 *  table.
 */
static prob_table_t lookup_tbl[] = {
    {1, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,13107,6554,4369,3277,2621,2185,1872,1638,1456,1311}},
    {2, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,20323,11685,8120,6210,5025,4219,3635,3193,2847,2568}},
    {3, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,24087,15575,11289,8812,7216,6105,5289,4664,4171,3772}},
    {4, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,26214,18488,13936,11102,9201,7847,6836,6053,5431,4923}},
    {5, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,27538,20681,16136,13107,10994,9450,8279,7362,6626,6023}},
    {6, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,28430,22356,17964,14857,12607,10923,9623,8594,7760,7072}},
    {7, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,29067,23659,19488,16384,14055,12272,10872,9750,8833,8070}},
    {8, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,29544,24693,20766,17716,15356,13506,12032,10835,9848,9021}},
    {9, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,29913,25528,21845,18881,16523,14635,13107,11852,10806,9925}},
    {10, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,30207,26214,22763,19904,17571,15668,14103,12803,11711,10784}},
    {11, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,30447,26786,23551,20804,18514,16612,15026,13694,12565,11599}},
    {12, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,30645,27269,24232,21600,19364,17476,15881,14528,13370,12373}},
    {13, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,30813,27683,24825,22307,20131,18268,16674,15307,14129,13107}},
    {14, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,30956,28039,25346,22937,20826,18994,17409,16037,14844,13803}},
    {15, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,31080,28351,25806,23502,21456,19661,18091,16720,15519,14464}},
    {16, {2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}, {32768,31188,28624,26214,24009,22030,20274,18724,17359,16155,15090}},
};
/*---------------------------------------------------------------------------*/
typedef rtimer_clock_t my_clock_t;
static volatile rtimer_clock_t callback_time;

static struct rtimer generic_timer;
/*---------------------------------------------------------------------------*/
/*static volatile uint16_t slot_increment_offset;
static volatile uint16_t discovery_time = 0;
static volatile uint16_t mean_disc_time = 0;
static volatile uint16_t max_disc_time = 0;
static uint8_t rounds_counter = 0;
*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t num_rounds_counter = 0;

static my_clock_t ref_timer;
/**
#if CONF_NETWORK_CLIQUE_SIZE != 0
static node_info_t neigh_table[CONF_NETWORK_CLIQUE_SIZE];
#else
static node_info_t neigh_table[MAX_NETWORK_SIZE];
#endif //CONF_NETWORK_CLIQUE_SIZE
*/
/*---------------------------------------------------------------------------*/
static volatile uint8_t radio_is_on = 0;
/*---------------------------------------------------------------------------*/
//static uint8_t num_neighbors = 0;


static volatile uint8_t nd_is_started = 0;

static volatile uint8_t nxt_channel_idx  = 0;
static volatile uint8_t clear_vars_val   = 0;

static volatile uint8_t packet_received_flag = 0;

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* the sender node_id: helps create links whose sender_id is the node that was 
 * behaving/acting as a transmitter
 */
static volatile uint8_t sender_nodeid = 0;
static volatile uint8_t all_neighbors_found_flag = 0;

/*---------------------------------------------------------------------------*/
static uint8_t i3e154_channels [] ={11, 12, 13, 14, 15, 16, 17,
                                    18, 19, 20, 21, 22, 23, 24, 25, 26};
/*---------------------------------------------------------------------------*/
PROCESS(node_discovery_process, "medal-disc process");
/*AUTOSTART_PROCESSES(&node_discovery_process);
*/
/*---------------------------------------------------------------------------*/
uint8_t get_rounds(){
    return rounds_counter;
}
/*---------------------------------------------------------------------------*/
uint16_t get_discovery_time(){
    return discovery_time;
}
/*---------------------------------------------------------------------------*/
uint16_t get_mean_discovery_time(){
    return (mean_disc_time/rounds_counter);
}
/*---------------------------------------------------------------------------*/
uint8_t get_fraction(){
    uint8_t k = 0, frac_neigh = 0;
    for (k = 0; k < num_neighbors; k++){
        if (neigh_table[k].node_id != 0){
            frac_neigh = frac_neigh + 1;
        }
    }
    return frac_neigh;
}
/**---------------------------------------------------------------------------*/
node_info_t * get_network_table(){
    return neigh_table;
}
/**---------------------------------------------------------------------------*/
static void on(void){
  if(radio_is_on == 0) {
    radio_is_on = 1;
    NETSTACK_RADIO.on();
  }
}
/*----------------------------------------------------------------------------*/
static void off(void){
  if(radio_is_on) {
    radio_is_on = 0;
    NETSTACK_RADIO.off();
  }
}
/*----------------------------------------------------------------------------*/
/**
 * @brief leds_set : display a counter into the Leds
 * @param count
 */
static void leds_set(uint8_t count){
    if(count & LEDS_GREEN){
        leds_toggle(LEDS_GREEN);
    }else{

        if(count & LEDS_YELLOW){
            leds_toggle(LEDS_YELLOW);
        }else{

            if(count & LEDS_RED){
                leds_toggle(LEDS_RED);
            }
        }
    }
}
/*---------------------------------------------------------------------------*/
/**
 * @brief nodeid_no_exist
 * @param nodeid
 * @return
 */
static uint8_t nodeid_exist(uint8_t nodeid){
    uint8_t k;
    for (k = 0; k < num_neighbors; k++){   
        if (neigh_table[k].node_id == nodeid){
            return 1;
        }
    }
 return 0;
}
/*---------------------------------------------------------------------------*/
/* add and update neighbors table upon reception of a packet from a neighbor..
 * it also performs hopcount filtering, by checking the length of each received
 * nodeid hop count..
 *----------------------------------------------------------------------------*/
/**
 * @brief add_neighbors: adds and updates the neighbors table
 * @param p
 */
static void add_neighbors(uint8_t *p){
    uint8_t j, k;
    for (k = 0; k < num_neighbors; k++){

        node_info_t *nif = (node_info_t*)(p + k*sizeof(node_info_t));
        uint8_t nodeid = nif->node_id;
        uint8_t hopcount = nif->hopcount;

        /*check if the received nodeid is not zero/null*/
        if(nodeid != 0){
            /**insert only when the nodeid is new, otherwise, do an update*/
            if(nodeid_exist(nodeid)){
                /*update current estimative..node already exists...*/
                for (j = 0; j < num_neighbors; j++){
                    node_info_t *nt = &neigh_table[j];
                    if(nt->node_id == nodeid && hopcount < nt->hopcount){
                        neigh_table[j].hopcount = hopcount;
                        break;
                    }
                }
            } //if==>nodeid_no_exist UPDATE....
            else{                
                for(j = 0; j < num_neighbors; j++){
                    if(neigh_table[j].node_id == 0 && hopcount <= MAX_HOPCOUNT){
                        neigh_table[j].node_id  = nodeid;
                        neigh_table[j].hopcount = hopcount;
                        break;
                    }
                } //end for ==>J
            }
        } //end of if==>nid != 0
    } //end of for ==>K
}
/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/**
 * @brief get_random_channel
 * @param num_channels: the maximum number of channels in use for that case
 * @param rand_num    : is the pseudo random number to extract the channelId
 * @return            : a random index from 0 to num_channels-1. The index
 *                      gives the channel to use for current time-slot
 */
static uint8_t random_int(uint8_t size_num){
    uint16_t rand_num = random_rand();

    uint16_t val = (65535 / size_num);
    if ( rand_num < val){
        return 0;
    }else{
        uint8_t k;
        for(k = 1; k < size_num; k++){
            if (rand_num >= k*val && rand_num <= (k+1)*val){
                return k;
            }
        }
    }
    return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief node_link_active
 * @param ploss
 * @return
 */
static uint8_t node_link_active(uint8_t ploss){
    uint16_t loss_p = (65535/100)*ploss;

    if (random_rand() > loss_p){
        return 1;
    }else{
        return 0;
    }
}
/*---------------------------------------------------------------------------*/
/**
 * @brief calc_trans_p : computes the transmission probability given
 *                       k-the number of channels, and n-the clique size..
 *                            ((2k + n - 1) - sqrt((2k + n - 1)^2 - 4kn))
 *                      p = -------------------------------------
 *                                        2n
 * @param k            : the number of channels
 * @param n            : the clique size
 * @return             : the computed transmission probability
 */
static uint16_t calc_trans_p(const uint8_t k, const uint8_t n){
    uint8_t i = 0;
    uint16_t a = 2*k + n - 1;
    uint16_t s = a*a - 4*k*n;
    uint16_t c = 2*n;

    uint16_t x1 = 0, xn = 100;

    for(i = 0; i < 10; i++){
        xn = (xn + s/xn)/2;

        if(i > 0 && x1 == xn){
            break;
        }else{
            x1 = xn;
        }
        //PRINTF("(%3u %3u)", xn, a);
    }

    uint16_t prob = (a - xn)*(65535/c);

    return  prob;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief get_tx_prob  : returns an optimal transmisison probability given the
 *                       network size and the number of channels in use.
 * @param num_channels : the maximum number of channels in use
 * @param nwk_size     : the current network size.
 * @return
 */
static uint16_t get_tx_prob(const uint8_t num_channels, const uint8_t nwk_size){
    uint8_t j;

    prob_table_t *prb = &lookup_tbl[(num_channels-1)%16];
    if(prb->channel == num_channels){
        for (j = 0; j < SIZE_NETWORK; j++){
            if (prb->num_nodes[j] == nwk_size){
                return prb->prob_vec[j];
            }
        }
    }

    return calc_trans_p(num_channels, nwk_size);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief all_neighbors_found: checks if all neighbors have been discovered
 * @param num_neighs         : current network size
 * @return
 */
static uint8_t all_neighbors_found(uint8_t num_neighs){
    uint8_t k;

    for(k = 0; k < num_neighs; k++){
        if (neigh_table[k].node_id == 0){
            return 0;
        }
    }

    return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief clear_vars
 */
static void clear_vars(){

    clear_vars_val           = 0;
    nd_is_started            = 0;
    slot_increment_offset    = 0;
    packet_received_flag     = 0;
    all_neighbors_found_flag = 0;

    //set channel back to RF_CHANNEL
    cc2420_set_channel(26);

    //turn radio on
    on();

    //clear network table to start a new round
    memset(neigh_table, 0, sizeof(neigh_table));

    //get first position of neighbor table.. sould always contain our info
    node_info_t *np  =  &neigh_table[0];

    //fill in our info into the network table
    np->node_id      =  (rimeaddr_node_addr.u8[0]  & 0xff);
    np->hopcount     =  0;

    //reset discovery time, to begin the next iteration
    discovery_time = 0;

    ///
    //watchdog_stop();
}
/*---------------------------------------------------------------------------*/
/**
 * @brief aloha_transmit: main transmit function that's invoked probabilisticly
 *                        to transmit a neighbor advert packet. Epidemic dissemination
 *                        is used here to speed up the neighbor discovery process
 *
 * @param t             : the real time timer pointer
 * @param ptr           : a pointer of data to the callback function
 * @return              : an integer without any meaningful value
 */
static char aloha_transmit(void){
    uint8_t k;

    rtimer_clock_t t_end = RTIMER_NOW() + 2*ONE_MSEC;

    while(RTIMER_CLOCK_LT(RTIMER_NOW(), t_end)){
        //delay = delay + 1;
    }

    /** we get a free buffer for transmission*/
    dataqueue_t *dq = dataqueue_get_freebuffer();

    if (dq != NULL){
        dataqueue_set_type(dq, QUEUE_PKT_OUT);
        data_packet_t *dp = (data_packet_t*)dataqueue_get_buffer_payload(dq);

        dp->ptype   = DATA_PACKET;
        dp->node_id = (rimeaddr_node_addr.u8[0]  & 0xff);

        /** fill the payload*/
        for (k = 0; k < num_neighbors; k++){

            node_info_t *nt = &neigh_table[k];

            if(nt->node_id != 0 && (nt->hopcount + 1 <= MAX_HOPCOUNT)){
                uint8_t dt_idx = k*sizeof(node_info_t);
                dp->data[dt_idx]     = nt->node_id;
                dp->data[dt_idx + 1] = nt->hopcount + 1;
            }
        }

        /** we tell the network driver to flush the packet*/
        NETSTACK_RADIO.send(dq->payload, DISCOVERY_PACKET_SIZE);

        /** we no longer need the space, therefore, we free it*/
        dataqueue_buffer_free(dq);

    }else{
        dataqueue_freeall ();
    }

    return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief auto_channel_hopping: it's a callback and a main process function
 *                              and it's triggered by the real time timer for
 *                              T-millisecs, where T, is the size of the time
 *                              slot. A node flips a coin and based on pre-stored
 *                              values, it decides if it should transmit or receive.
 *                              A random channel for every time-slot is also
 *                              generated.
 *                              After \tau-terminating time, the function exits
 * @param t                   : The real time timer pointer
 * @param ptr                 : a data pointer to the callback function
 * @return                    : an integer without any meaningful value yet...
 */
static char auto_channel_hopping(struct rtimer *t, void *ptr){   

    if(nd_is_started /*&& (slot_increment_offset < upper_bound_time)*/){

        int ret = 0;
        uint16_t prob = random_rand();

        uint16_t tx_prob = get_tx_prob(num_channels, num_neighbors);


        slot_increment_offset++;
        nxt_channel_idx =  random_int(num_channels);

        cc2420_set_channel(i3e154_channels[nxt_channel_idx]);

        //turn radio on
        on();

        if(prob < tx_prob){
            aloha_transmit();
        }else{
            //listen
        }

        callback_time   = (ref_timer + (slot_increment_offset*TS));

        ret = rtimer_set(&generic_timer, callback_time, 1,
                         (void (*)(struct rtimer *, void *))auto_channel_hopping, NULL);

        leds_toggle(LEDS_GREEN);

        if(ret){
            PRINTF("synchronization failed\n");
        }

        return 1;
    }
    else{
        //clear all state variables to comence a new round...
        /*if(clear_vars_val){
         clear_vars();
        }*/

        clear_vars();

        return 0;
    }

    return 0;
}
/*---------------------------------------------------------------------------*/
static void start_up_protocol(uint8_t seqno){

        //clear variable of the number of elapsed time slots
        slot_increment_offset = 0;

        //clear the flag indicating that all neighbors are found
        all_neighbors_found_flag = 0;

        /**clear neighbors list.. */
        memset(neigh_table, 0, sizeof(neigh_table));

        //set initial information for
        node_info_t *np  =  &neigh_table[0];
        np->node_id      =  (rimeaddr_node_addr.u8[0]  & 0xff);
        np->hopcount     =  0;

        //compute offset required to start the protocol
        uint8_t time_2_start = (MAX_SYNCH_PKTS  - seqno);

        if(time_2_start == 0){
           callback_time = RTIMER_NOW() + 4;
        }else{
            callback_time = RTIMER_NOW() + time_2_start*TS;
        }

        //set reference timer to the time we start our computation
        ref_timer = callback_time;

        //start the real-time timer.. used to obtain a high resolution timer
        uint8_t ret = rtimer_set(&generic_timer, callback_time, 1,
                                 (void (*)(struct rtimer *, void *))auto_channel_hopping, NULL);

        if(ret){
            PRINTF("error auto_channel\n");
        }

}
/*---------------------------------------------------------------------------*/
/**
 * @brief packet_input: this fucntion is called by the device radio driver
 *                      whenever a packet is received. Appropriate actions are
 *                      taken according to an algorithm to determine the exact
 *                      actions to perform..
 */
static void packet_input(void){

    int len = 0;

    dataqueue_t *dq_in = dataqueue_get_freebuffer();

    if(dq_in != NULL){
        dataqueue_set_type(dq_in, QUEUE_PKT_IN);

        //read data here..
        len = NETSTACK_RADIO.read((void*)dq_in->payload, PACKETBUF_SIZE);
        if(len > 0){
            dq_in->data_len = len;
        }else{
            //invalid packet..
            return;
        }
    }else{
        //no valid packet exist..clear all existing packets..
        dataqueue_freeall();
        //return for the next reading...
        return;
    }

    //we know that we correctply received the packet..
    data_packet_t *pkt = (data_packet_t*)dataqueue_get_buffer_payload(dq_in);

    if(pkt->ptype == SYNCH_PACKET){
        
        leds_set(pkt->seqno);

        //
        sender_nodeid  = pkt->node_id;

        if(nd_is_started == 0){

            //set a flag indicating a new round of the protocol
            nd_is_started = 1;

            //increment number of rounds..
            num_rounds_counter++;
            //rounds_counter = num_rounds_counter;

            ///turn the radio off
            ///off();

            //trigger another round of measurements
            start_up_protocol(pkt->seqno);
        }

        /**we no longer need the space, therefore, we free it*/
        dataqueue_buffer_free(dq_in);

        //break here since there is nothing else to do.
        return;
    }

    //we received a data packet.. process its content accordingly
    if(pkt->ptype == DATA_PACKET){

        /**signal that we have received a packet.. maybe redundant nowaddays..??*/
        packet_received_flag = 1;

        leds_toggle(LEDS_BLUE);
        leds_toggle(LEDS_YELLOW);
        leds_toggle(LEDS_RED);

        /**while we havent found all our neighbors, add possible neighbors to the table.
         */
        if(all_neighbors_found_flag == 0){

            add_neighbors(&pkt->data[0]);

            discovery_time = slot_increment_offset + 1;

            if(all_neighbors_found(num_neighbors)){

                all_neighbors_found_flag = 1;
                //discovery_time = slot_increment_offset + 1;

                mean_disc_time = mean_disc_time + discovery_time;

                if(discovery_time > max_disc_time){
                    max_disc_time = discovery_time;
                }

                //we only signal upper layers when the computation is finished
                NETSTACK_MAC.input();

                /*PRINTF(">:%d,%d,%u,%u,%u-%u\n", rounds_counter, node_id,
                       CHANNEL_POOL_SIZE, num_neighbors, discovery_time, mean_disc_time/rounds_counter);*/
            }

            //signal the upper layers that a packet have beed received
            //NETSTACK_MAC.input();

        } //end of <all_neighbors_found_flag>

        /** we no longer need the space, therefore, we free it*/
        dataqueue_buffer_free(dq_in);
    } //end of <pkt->ptype == DATA_PACKET>
    else{
        ///no valid packet exist..clear all existing packets..
        dataqueue_freeall();
    }
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
/**
 * @brief on
 * @return
 */
static int rd_on(void){
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
/**
 * @brief off
 * @param keep_radio_on
 * @return
 */
static int rd_off(int keep_radio_on){
    return NETSTACK_RADIO.off();
}
/*---------------------------------------------------------------------------*/
/**
 * @brief channel_check_interval
 * @return
 */
static unsigned short channel_check_interval(void){
  //return 8;
    return 16;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief init: initialization function, it's called by the network driver
 *              main function to start the RDC-Radio Duty Cycling layer
 */
static void init(void){

  memset(neigh_table, 0, sizeof(neigh_table));

  node_info_t *np  =  &neigh_table[0];
  np->node_id      =  (rimeaddr_node_addr.u8[0]  & 0xff);
  np->hopcount     =  0;

  on();

  /** initialize data queue*/
  dataqueue_init();

  /** start the node discovery process NOTE: ONLY USED FOR COOJA SIMULATIONS*/
#ifndef IN_INDRIYA
  process_start(&node_discovery_process, NULL);
#endif //IN_INDRIYA || IN_TWIST
}
/*---------------------------------------------------------------------------*/
/**
 * @brief PROCESS_THREAD
 */
PROCESS_THREAD(node_discovery_process, ev, data){

    PROCESS_BEGIN();

    /*PRINTF("node_id: %2u, netsize: %2u, num_ch: %2u, hopc: %u\n",
           node_id, num_neighbors, num_channels, HOPCOUNT_FILTER_NDISC);*/

    PRINTF("=======> NodeId: %d starting \n", rimeaddr_node_addr.u8[0]);

    while(1) {
        PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);

        char *command = (char*)data;

        if(command != NULL){
            if(!strcmp(command, "mean")){
                PRINTF("report,%u,%u,%u,%u,%u,%u,%u,E\n", rounds_counter, node_id, num_channels,
                       num_neighbors,  MAX_HOPCOUNT, mean_disc_time/rounds_counter, max_disc_time);
            }
            if(!strcmp(command, "nodeinfo")){
                if(node_id != 0){
                    PRINTF("nid:,%u,%u,E\n", node_id, rimeaddr_node_addr.u8[1]);
                }else{
                    PRINTF("rid:,%u,E\n", rimeaddr_node_addr.u8[1]);
                }
            }
            if(!strcmp(command, "allconv")){
                /*test_case related
                 *all nodes have converged..
                */
                cc2420_set_channel(26);
                nd_is_started = 0;
                clear_vars_val = 1;

            }
            if(!strcmp(command, "newround")){
                /*test_case related all nodes have converged..begin a new round*/
                //nd_is_started = 1;
                all_neighbors_found_flag = 0;

                //increment number of rounds.
                rounds_counter = rounds_counter + 1;
            }
        }
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
const struct rdc_driver indriya_medal_driver = {
    "MEDAL-Indriya-RDC v3.0",
    init,
    send_packet,
    send_list,
    packet_input,
    rd_on,
    rd_off,
    channel_check_interval,
};
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
