
/**----------------------------------------------------------------------------* 
 * Automatic Control Lab
 *      School of Electrical Engineering
 *          KTH - The Royal Institute of Technology
 *-----------------------------------------------------------------------------*
 * @author:
 *        António Gonga <gonga@ee.kth.se>
 *        Osquldasväg 10, 6tr STOCKHOLM-Sweden
 *-----------------------------------------------------------------------------* 
 *\File: rdcqueue.c
 *Description: TSCH mac implementation for fixed infrastructure and mobile
 *             nodes. This version supports Ad Hoc communication between mobile
 *             sensor nodes.  
 *-----------------------------------------------------------------------------*
 *Date: Friday, Sep 5th, 2012 Time: 10:20:56
 *----------------------------------------------------------------------------*/
#include "contiki.h"
#include "net/rime/rimeaddr.h"

#include "nd-dataqueue.h"

#include <string.h>

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define DATA_QUEUE_SIZE 4

static dataqueue_t *data_queue[DATA_QUEUE_SIZE];
static dataqueue_t data_queue_bufs[DATA_QUEUE_SIZE];

/**----------------------------------------------------------------------------*/
void dataqueue_init()
{
  uint8_t i = 0;
  for(i = 0; i < DATA_QUEUE_SIZE; i++){          
      memset((void*)&data_queue_bufs[i], 0, sizeof(data_queue_bufs[i]));
      data_queue[i] = &data_queue_bufs[i];
  }  
}
/**----------------------------------------------------------------------------*/
/**
 * @brief dataqueue_packet_init
 * @param p
 */
void dataqueue_buffer_init(dataqueue_t *p)
{
    p->empty     = QUEUE_PKT_EMPY; //number of tries before discarding pkt
    p->data_len  = 0; //payload length in bytes
    //p->hdr_ptr   = &p->data[0]; //easy access data
    p->payload   = &p->data[0];
    p->hdr_ptr   = p->payload;
    memset((void*)&p->data[0], 0, PACKETBUF_SIZE);
}
/**----------------------------------------------------------------------------*/
/**
 * @brief dataqueue_get_freebuffer
 * @return
 */
dataqueue_t* dataqueue_get_freebuffer()
{
    uint8_t index = 0;
    for(index = 0; index < DATA_QUEUE_SIZE; index++){
        if(data_queue[index]->empty == QUEUE_PKT_EMPY){

            dataqueue_buffer_init(data_queue[index]);

            data_queue[index] = &data_queue_bufs[index];

            return data_queue[index];
        }
    }
    return NULL;
}
/**----------------------------------------------------------------------------*/
/**
 * @brief dataqueue_freepacket
 * @param p
 */
void dataqueue_buffer_free(dataqueue_t *p)
{
    uint8_t idx = 0;
    for(idx = 0; idx < DATA_QUEUE_SIZE; idx++){
        if(data_queue[idx] == p){
            dataqueue_buffer_init(data_queue[idx]);
        }
    }
}
/**----------------------------------------------------------------------------*/
/**
 * @brief dataqueue_freeall
 * @param p
 */
void dataqueue_freeall()
{
    uint8_t idx = 0;
    for(idx = 0; idx < DATA_QUEUE_SIZE; idx++){
        dataqueue_buffer_init(data_queue[idx]);
        data_queue[idx] = &data_queue_bufs[idx];
    }
}
/**----------------------------------------------------------------------------*/
/**
 * @brief dataqueue_get_payloadpacket
 * @param p
 * @return
 */
void*  dataqueue_get_buffer_payload(dataqueue_t *p)
{
    return (void*)p->payload;
}
/**----------------------------------------------------------------------------*/
/**
 * @brief dataqueue_get_data_packet
 * @param queue_type
 * @param tgtaddr
 * @return
 */
dataqueue_t* dataqueue_get_queuetype(uint8_t queue_type)
{
    uint8_t qidx = 0;
    
    for(qidx = 0; qidx < DATA_QUEUE_SIZE; qidx++){
        if(data_queue[qidx]->empty == queue_type){

            /** found packet we were looking for*/
            return data_queue[qidx];
        }
    }
    return NULL;
}
/**----------------------------------------------------------------------------*/
/**
 * @brief dataqueue_set_type
 * @param p
 * @param pkt_type
 */
void dataqueue_set_type(dataqueue_t *p, uint8_t pkt_type)
{
    p->empty = pkt_type;
}
/**----------------------------------------------------------------------------*/
/**
 * @brief dataqueue_is_full
 * @return
 */
uint8_t dataqueue_isfull()
{
    uint8_t npkts = 0, qidx = 0;

    for(qidx = 0; qidx < DATA_QUEUE_SIZE; qidx++){
        if (data_queue[qidx]->empty != QUEUE_PKT_EMPY){
            npkts++;
        }
    }
    
    if ((DATA_QUEUE_SIZE - npkts) == 0){
        return 1;
    }
    return  0;
}

