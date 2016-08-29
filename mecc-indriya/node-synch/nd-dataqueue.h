/**----------------------------------------------------------------------------* 
 * Automatic Control Lab
 *      School of Electrical Engineering
 *          KTH - The Royal Institute of Technology
 *-----------------------------------------------------------------------------*
 * @author:
 *        António Gonga <gonga@ee.kth.se>
 *        Osquldasväg 10, 6tr STOCKHOLM-Sweden
 *-----------------------------------------------------------------------------* 
 * \File: dataqueue.h
 *Description: TSCH mac implementation for fixed infrastructure and mobile
 *             nodes. This version supports Ad Hoc communication between mobile
 *             sensor nodes.  
 *-----------------------------------------------------------------------------*
 *Date: Friday, Sep 5th, 2012 Time: 10:21:56
 *----------------------------------------------------------------------------*/
#ifndef ND_DATAQUEUE_H
#define ND_DATAQUEUE_H

#include "contiki.h"
#include "net/packetbuf.h"
//#include "net/rime/rimeaddr.h"
/*----------------------------------------------------------------------------*/
typedef struct{
    uint8_t empty;
    uint8_t data_len;
    uint8_t *hdr_ptr;
    uint8_t *payload;
    uint8_t data[PACKETBUF_SIZE]; 
}dataqueue_t;
/*----------------------------------------------------------------------------*/
/**struct rdchash_table{   
    LIST_STRUCT(hash);
    struct memb *memb;
};*/

enum{
    QUEUE_PKT_EMPY,
    QUEUE_PKT_IN,
    QUEUE_PKT_OUT
};

void dataqueue_init();

void dataqueue_buffer_init(dataqueue_t *p);

dataqueue_t* dataqueue_get_freebuffer();

void dataqueue_buffer_free(dataqueue_t *p);

void dataqueue_freeall();

void*  dataqueue_get_buffer_payload(dataqueue_t *p);

dataqueue_t* dataqueue_get_queuetype(uint8_t queue_type);

void dataqueue_set_type(dataqueue_t *p, uint8_t pkt_type);

uint8_t dataqueue_isfull();

#endif //ND_DATAQUEUE
