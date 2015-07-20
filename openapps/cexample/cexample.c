/**
\brief An example CoAP application.
*/

#include "opendefs.h"
#include "cexample.h"
#include "opencoap.h"
#include "opentimers.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "openserial.h"
#include "openrandom.h"
#include "scheduler.h"
//#include "ADC_Channel.h"
#include "idmanager.h"
#include "IEEE802154E.h"
#include "sixtop.h"
#include <stdio.h>



//=========================== defines =========================================

/// info for traffic generation
#define  PAYLOADLEN           40
#define  CEXAMPLE_PERIOD      10000

const uint16_t cexample_timeout = 4000;
const uint8_t cexample_path0[] = "ex";

//=========================== variables =======================================

cexample_vars_t cexample_vars;

//=========================== prototypes ======================================

owerror_t cexample_receive(OpenQueueEntry_t* msg,
                    coap_header_iht*  coap_header,
                    coap_option_iht*  coap_options);
void    cexample_start(void);
void    cexample_timer_start(void);
void    cexample_timer_cb(void);
void    cexample_task_cb(void);
void    cexample_sendDone(OpenQueueEntry_t* msg, owerror_t error);

//=========================== public ==========================================

void cexample_init() {
   
   // prepare the resource descriptor for the /ex path
   cexample_vars.desc.path0len             = sizeof(cexample_path0)-1;
   cexample_vars.desc.path0val             = (uint8_t*)(&cexample_path0);
   cexample_vars.desc.path1len             = 0;
   cexample_vars.desc.path1val             = NULL;
   cexample_vars.desc.componentID          = COMPONENT_CEXAMPLE;
   cexample_vars.desc.callbackRx           = &cexample_receive;
   cexample_vars.desc.callbackSendDone     = &cexample_sendDone;
   cexample_vars.seqnum                    = openrandom_get16b();

#ifdef TRACK_ACTIVE
   //I am the owner of this track (8 bytes address)
   memcpy(&(cexample_vars.track.owner), idmanager_getMyID(ADDR_64B), sizeof(open_addr_t));
   memset(&(cexample_vars.track.owner.addr_64b),1,8);
   cexample_vars.track.instance            = (uint16_t)TRACK_BALANCING;
#else
   bzero(&(cexample_vars.track.owner), sizeof(open_addr_t));
   cexample_vars.track.instance            = (uint16_t)0;
#endif

   opencoap_register(&cexample_vars.desc);

   //starts to generate packets when I am synchronized
   uint64_t  next = openrandom_get16b();
   while (next > 5 * CEXAMPLE_PERIOD)
      next -= CEXAMPLE_PERIOD/3;


   cexample_vars.timerId    = opentimers_start(
         next,
         TIMER_ONESHOT,
         TIME_MS,
         cexample_timer_start);

}




//=========================== private =========================================

owerror_t cexample_receive(OpenQueueEntry_t* msg,
                      coap_header_iht* coap_header,
                      coap_option_iht* coap_options) {
   return E_FAIL;
}


//starts generating the packet only once I am synchronized
void cexample_timer_start(void){

   //next verification between CEXAMPLE_PERIOD and 2 * CEXAMPLE_PERIOD
   uint64_t  next = openrandom_get16b();
   while (next > 5 * CEXAMPLE_PERIOD)
      next -= CEXAMPLE_PERIOD;

   if (ieee154e_isSynch() == FALSE){
     cexample_vars.timerId    = opentimers_start(
            next,
            TIMER_ONESHOT,
            TIME_MS,
            cexample_timer_start);
   }
   else{
        cexample_vars.timerId    = opentimers_start(
            CEXAMPLE_PERIOD,
            TIMER_PERIODIC,TIME_MS,
            cexample_timer_cb);
   }
}

//timer fired, but we don't want to execute task in ISR mode
//instead, push task to scheduler with COAP priority, and let scheduler take care of it
void cexample_timer_cb(){
   scheduler_push_task(cexample_task_cb,TASKPRIO_COAP);
}

void cexample_task_cb() {
   OpenQueueEntry_t*    pkt;
   owerror_t            outcome;
   uint8_t              i;


   char                 msg[15];
   snprintf(msg, 15, "generation");
   openserial_printf(COMPONENT_CEXAMPLE, msg, strlen(msg));
   
   // don't run if not synch
   if (ieee154e_isSynch() == FALSE) return;
   
   // don't run on dagroot
   if (idmanager_getIsDAGroot()) {
      opentimers_stop(cexample_vars.timerId);
      return;
   }
   
   // create a CoAP RD packet
   pkt = openqueue_getFreePacketBuffer_with_timeout(COMPONENT_CEXAMPLE, cexample_timeout);
   if (pkt==NULL) {
      openserial_printError(
         COMPONENT_CEXAMPLE,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      openqueue_freePacketBuffer(pkt);
      return;
   }
   // take ownership over that packet
   pkt->creator                   = COMPONENT_CEXAMPLE;
   pkt->owner                     = COMPONENT_CEXAMPLE;

   // CoAP payload - seqnum are the first two bytes
   packetfunctions_reserveHeaderSize(pkt,PAYLOADLEN);
   pkt->payload[0]                = (cexample_vars.seqnum>>8)&0xff;
   pkt->payload[1]                = (cexample_vars.seqnum>>0)&0xff;
   (cexample_vars.seqnum)++;

   //garbage for the remaining bytes
   for (i=2;i<PAYLOADLEN;i++) {
       pkt->payload[i]             = i;
    }

   //coap packet
   packetfunctions_reserveHeaderSize(pkt,1);
   pkt->payload[0] = COAP_PAYLOAD_MARKER;
   
   // content-type option
   packetfunctions_reserveHeaderSize(pkt,2);
   pkt->payload[0]                = (COAP_OPTION_NUM_CONTENTFORMAT - COAP_OPTION_NUM_URIPATH) << 4
                                    | 1;
   pkt->payload[1]                = COAP_MEDTYPE_APPOCTETSTREAM;

   // location-path option
   packetfunctions_reserveHeaderSize(pkt, sizeof(cexample_path0)-1);
   memcpy(&pkt->payload[0],cexample_path0, sizeof(cexample_path0)-1);
   packetfunctions_reserveHeaderSize(pkt,1);
   pkt->payload[0]                = ((COAP_OPTION_NUM_URIPATH) << 4) | (sizeof(cexample_path0)-1);
   

   // metadata
   pkt->l2_track                  = cexample_vars.track;
   pkt->l4_destination_port       = WKP_UDP_COAP;
   pkt->l3_destinationAdd.type    = ADDR_128B;
   memcpy(&pkt->l3_destinationAdd.addr_128b[0], &ipAddr_unistra, 16);
   
   // send
   outcome = opencoap_send(
      pkt,
      COAP_TYPE_NON,
      COAP_CODE_REQ_PUT,
      1,
      &cexample_vars.desc
   );
   
   // avoid overflowing the queue if fails
   if (outcome==E_FAIL) {
      openqueue_freePacketBuffer(pkt);
   }
   
   //a frame was generated (seqnum was meanwhile incremented)
   openserial_statGen(cexample_vars.seqnum -1, cexample_vars.track);

   return;
}

void cexample_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}
