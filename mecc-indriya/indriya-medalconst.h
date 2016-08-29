/*
 *
 *CopyLeft gonga@kth.se 2013 :-)
 */

#ifndef MEDAL_H
#define MEDAL_H

#include "net/rime/rimeaddr.h"



#define ONE_KILO            (1024)
#define ONE_MSEC            (RTIMER_SECOND/ONE_KILO)
#define TS                  (15*ONE_MSEC)
#define TS_75P              (3*TS)/4
#define HALF_MSEC           (ONE_MSEC/2)
#define GUARD_PERIOD        ONE_MSEC
#define SWITCHING_PERIOD    (3*ONE_MSEC)
#define DATA_PERIOD         (5*ONE_MSEC)
#define FRAME_PERIOD        (16*TS)

#define CLOCK_DT(a,b)     ((signed short)((a)-(b)) < 0)

/*---------------------------------------------------------------------------*/
#define MAX_SYNCH_PKTS   10
#define MAX_REPEAT   100
/*----------------------------------------------------------------------------*/
typedef struct{
    uint8_t node_id;
    uint8_t hopcount;
}node_info_t;
/*----------------------------------------------------------------------------*/

#if CONF_NETWORK_CLIQUE_SIZE != 0
#define MAX_NETWORK_SIZE  (sizeof(node_info_t))*CONF_NETWORK_CLIQUE_SIZE
static uint8_t num_neighbors = CONF_NETWORK_CLIQUE_SIZE;
#else
static uint8_t num_neighbors = 2;
#define MAX_NETWORK_SIZE  90
#endif //CONF_NETWORK_CLIQUE_SIZE != 0

/*----------------------------------------------------------------------------*/
static volatile uint16_t upper_bound_time     = 120;
static volatile uint16_t slot_increment_offset= 0;
static volatile uint16_t discovery_time       = 0;
static volatile uint16_t mean_disc_time       = 0;
static volatile uint16_t max_disc_time        = 0;
static uint8_t rounds_counter                 = 0;
/*----------------------------------------------------------------------------*/
uint8_t get_rounds();
uint8_t get_fraction();
uint16_t get_discovery_time();
uint16_t get_mean_discovery_time();
node_info_t * get_network_table();
/*----------------------------------------------------------------------------*/
#if CONF_NETWORK_CLIQUE_SIZE != 0
static node_info_t neigh_table[CONF_NETWORK_CLIQUE_SIZE];
#else
static node_info_t neigh_table[MAX_NETWORK_SIZE];
#endif //CONF_NETWORK_CLIQUE_SIZE
/*----------------------------------------------------------------------------*/
typedef struct {
    uint8_t ptype;          //packet type
    uint8_t node_id;
    uint8_t seqno;         //data sequence number
    uint8_t rounds;
    uint8_t data[MAX_NETWORK_SIZE]; //payload
}data_packet_t;


#define DISCOVERY_PACKET_SIZE sizeof(data_packet_t) //packet

/*----------------------------------------------------------------------------*/
typedef enum{
    SYNCHER,
    DATA_NODE,
}node_type_t;
/*----------------------------------------------------------------------------*/
enum{
    NOTHING,
    SYNCHING,
    SEND_HOPPING,
    RESET
};
/*---------------------------------------------------------------------------*/
enum{
    SYNCH_PACKET,
    DATA_PACKET,
    PRINT_DATA
};

#if CONF_CHANNEL_POOL_SIZE != 0
#define CHANNEL_POOL_SIZE CONF_CHANNEL_POOL_SIZE
static  uint8_t num_channels = CHANNEL_POOL_SIZE;
#else   //CONF_CHANNEL_POOL_SIZE
#define CHANNEL_POOL_SIZE    1
static  uint8_t num_channels = CHANNEL_POOL_SIZE;
#endif //CONF_CHANNEL_POOL_SIZE

/*#if CONF_NETWORK_CLIQUE_SIZE != 0
//#define MAX_NETWORK_SIZE  (1+sizeof(node_info_t))*CONF_NETWORK_CLIQUE_SIZE
static uint8_t num_neighbors = CONF_NETWORK_CLIQUE_SIZE;
#else
static uint8_t num_neighbors = 2;
//#define MAX_NETWORK_SIZE  60
#endif //CONF_NETWORK_CLIQUE_SIZE != 0*/

/*#if CONF_NETWORK_CLIQUE_ADAPTIVE != 0
static uint8_t adapt_num_neighbors = CONF_NETWORK_CLIQUE_ADAPTIVE;
#else
static uint8_t adapt_num_neighbors = 2;
#endif //CONF_NETWORK_CLIQUE_ADAPTIVE
*/

#if HOPCOUNT_FILTER_NDISC != 0
    #if CHANNEL_POOL_SIZE == 1
        #define MAX_HOPCOUNT  1
    #else //CHANNEL_POOL_SIZE == 1
        #define MAX_HOPCOUNT HOPCOUNT_FILTER_NDISC
    #endif //CHANNEL_POOL_SIZE == 1
#else //HOPCOUNT_FILTER_NDISC != 0
    #define MAX_HOPCOUNT 15
#endif //CONF_MAX_HOPCOUNT != 0

#endif /* __MEDAL_H__ */
