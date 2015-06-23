#include "opendefs.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "IEEE802154E.h"
#include "scheduler.h"
#include "opentimers.h"
#include "sixtop.h"
#include <stdio.h>

//=========================== variables =======================================

openqueue_vars_t openqueue_vars;

//=========================== prototypes ======================================

void openqueue_reset_entry(OpenQueueEntry_t* entry);

//to remove timeouts in the queue
void openqueue_timeout_timer_cb(void);
void openqueue_timeout_timer_fired(void);


//=========================== public ==========================================

//======= admin

/**
\brief Initialize this module.
*/
void openqueue_init() {
   uint8_t i;
   for (i=0;i<QUEUELENGTH;i++){
      openqueue_reset_entry(&(openqueue_vars.queue[i]));
   }

}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_queue() {
   debugOpenQueueEntry_t output;
   openqueue_vars.debugPrintRow         = (openqueue_vars.debugPrintRow+1)%QUEUELENGTH;

   output.row     = openqueue_vars.debugPrintRow;
   output.creator = openqueue_vars.queue[openqueue_vars.debugPrintRow].creator;
   output.owner   = openqueue_vars.queue[openqueue_vars.debugPrintRow].owner;

   //ASN to push (in ASN format)
   output.timeout.byte4 = openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[4];
   output.timeout.bytes0and1 =
         openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[0] +
         openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[1] * 256;
   output.timeout.bytes2and3 =
         openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[2] +
         openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[3] *256;
   output.trackInstance                  = \
       (uint16_t)openqueue_vars.queue[openqueue_vars.debugPrintRow].l2_track.instance;
    memcpy(
          &output.trackOwner,
          &(openqueue_vars.queue[openqueue_vars.debugPrintRow].l2_track.owner),
          sizeof(open_addr_t)
       );


   openserial_printStatus(
         STATUS_QUEUE,
         (uint8_t*)&output,
         sizeof(debugOpenQueueEntry_t));
   return TRUE;
}

//this represents an invalid timeout
bool openqueue_timeout_is_zero(timeout_t value){
   uint8_t  i;

   for (i=0; i<5; i++)
      if (value.byte[i] != 0)
      return FALSE;

   return(TRUE);
}

//is a >= b AND b != INVALID
bool openqueue_timeout_is_greater(timeout_t a, timeout_t b){
   uint8_t  i;

   //invalid timeout
   if(openqueue_timeout_is_zero(b))
      return(FALSE);

   //for each byte (byte 4 is the biggest)
   for(i=sizeof(timeout_t)-1; i>=0 && i<=sizeof(timeout_t)-1; i--)
      if (a.byte[i] > b.byte[i])
         return(TRUE);
      else if (a.byte[i] < b.byte[i])
         return(FALSE);

   //equal case
   return(TRUE);
}


//returns a - b (or 0 if b > a)
uint64_t openqueue_timeout_diff(timeout_t a, timeout_t b){
   uint8_t  i, max;
   bool     greater = FALSE;
   uint64_t diff = 0;

   max = sizeof(timeout_t) - 1;
   for(i=max ; i>=0 && i<=max ; i--){

      //returns 0 if a < b
      if (!greater && a.byte[i] < b.byte[i])
         return(0);

      // we have the first different byte (a > b)
      else if (a.byte[i] > b.byte[i])
         greater = TRUE;

      //computes the difference
      diff = diff * 256 + (a.byte[i] - b.byte[i]);
   }

   return(diff);
}


//remove the packets which are timeouted in the queue
void openqueue_timeout_drop(void){
   uint8_t     i;
   timeout_t   now;

   //initialization
   ieee154e_getAsn(now.byte);


   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {

      if (openqueue_vars.queue[i].owner != COMPONENT_NULL)
         if (!openqueue_timeout_is_zero(openqueue_vars.queue[i].timeout))
            if (openqueue_timeout_is_greater(now, openqueue_vars.queue[i].timeout)){

               openserial_statPktTimeout(&(openqueue_vars.queue[i]));

              /* openserial_printError(COMPONENT_OPENQUEUE, ERR_OPENQUEUE_TIMEOUT,
                     (errorparameter_t)openqueue_vars.queue[i].owner,
                     (errorparameter_t)openqueue_vars.queue[i].creator);
*/
               openqueue_reset_entry(&(openqueue_vars.queue[i]));
            }
   }


   ENABLE_INTERRUPTS();
}




//======= called by any component



/**
\brief Request a new (free) packet buffer.

Component throughout the protocol stack can call this function is they want to
get a new packet buffer to start creating a new packet.

\note Once a packet has been allocated, it is up to the creator of the packet
      to free it using the openqueue_freePacketBuffer() function.

\returns A pointer to the queue entry when it could be allocated, or NULL when
         it could not be allocated (buffer full or not synchronized).
*/
OpenQueueEntry_t* openqueue_getFreePacketBuffer(uint8_t creator) {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   
   // refuse to allocate if we're not in sync
   if (ieee154e_isSynch()==FALSE && creator > COMPONENT_IEEE802154E){
     ENABLE_INTERRUPTS();
     return NULL;
   }
   
   // if you get here, I will try to allocate a buffer for you
   
   // walk through queue and find free entry
   for (i=0;i<QUEUELENGTH;i++) {
      if (openqueue_vars.queue[i].owner==COMPONENT_NULL) {

         bzero(openqueue_vars.queue[i].timeout.byte, sizeof(timeout_t));
         openqueue_vars.queue[i].creator  = creator;
         openqueue_vars.queue[i].owner    = COMPONENT_OPENQUEUE;

         ENABLE_INTERRUPTS();

      /*   openserial_printCritical(COMPONENT_OPENQUEUE, ERR_GENERIC,
                                                  (errorparameter_t)123,
                                                  (errorparameter_t)i);
         openserial_printCritical(COMPONENT_OPENQUEUE, ERR_GENERIC,
                                                  (errorparameter_t)456,
                                                  (errorparameter_t)creator);

*/
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}



//======= called by any component

/**
\brief Request a new (free) packet buffer, specifying a timeout (in ms)

\note Once a packet has been allocated, it is up to the creator of the packet
      to free it using the openqueue_freePacketBuffer() function.

\returns A pointer to the queue entry when it could be allocated, or NULL when
         it could not be allocated (buffer full or not synchronized).
*/
OpenQueueEntry_t* openqueue_getFreePacketBuffer_with_timeout(uint8_t creator, const uint16_t duration_ms) {
   OpenQueueEntry_t* entry;
   timeout_t     now;
   uint8_t       remainder, i;
   uint64_t      diff;
   timeout_t     duration_asn;

   // a new entry in the queue
   entry = openqueue_getFreePacketBuffer(creator);


   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();

   //no packet is available
   if (entry == NULL){
      ENABLE_INTERRUPTS();
      return(NULL);
   }

   //*1000 since ms have to be converted in us
   //+1 to upper ceil the nb. of slots
   diff = ((uint64_t) duration_ms) / (TsSlotDuration * PORT_TICS_PER_MS / 1000) + 1;

   //offset in ASN format
   bzero(duration_asn.byte, sizeof(timeout_t));
   for(i=sizeof(timeout_t)-1; i>=0  && i<=sizeof(timeout_t)-1; i--){
      duration_asn.byte[i] = (uint8_t)(diff >> (8*i));
      diff -= (uint64_t)duration_asn.byte[4] << (8*i);
   }


   //translates the duration into an ASN
   ieee154e_getAsn(now.byte);
   remainder = 0;
   for(i=0; i<sizeof(timeout_t); i++){
      entry->timeout.byte[i] = duration_asn.byte[i] + now.byte[i] + remainder;
      if (entry->timeout.byte[i] < duration_asn.byte[i] && entry->timeout.byte[i] < now.byte[i])
         remainder = 1;
      else
         remainder = 0;
   }


   ENABLE_INTERRUPTS();
   return(entry);
}





/**
\brief Free a previously-allocated packet buffer.

\param pkt A pointer to the previously-allocated packet buffer.

\returns E_SUCCESS when the freeing was successful.
\returns E_FAIL when the module could not find the specified packet buffer.
*/
owerror_t openqueue_freePacketBuffer(OpenQueueEntry_t* pkt) {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {
      if (&openqueue_vars.queue[i]==pkt) {
         if (openqueue_vars.queue[i].owner==COMPONENT_NULL) {
            // log the error
            openserial_printCritical(COMPONENT_OPENQUEUE,ERR_FREEING_UNUSED,
                                  (errorparameter_t)0,
                                  (errorparameter_t)0);
         }
         openqueue_reset_entry(&(openqueue_vars.queue[i]));
         ENABLE_INTERRUPTS();
         return E_SUCCESS;
      }
   }
   // log the error
   openserial_printCritical(COMPONENT_OPENQUEUE,ERR_FREEING_ERROR,
                         (errorparameter_t)0,
                         (errorparameter_t)0);
   ENABLE_INTERRUPTS();
   return E_FAIL;
}

/**
\brief Free all the packet buffers created by a specific module.

\param creator The identifier of the component, taken in COMPONENT_*.
*/
void openqueue_removeAllCreatedBy(uint8_t creator) {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++){
      if (openqueue_vars.queue[i].creator == creator) {
         openqueue_reset_entry(&(openqueue_vars.queue[i]));
         openserial_printCritical(COMPONENT_OPENQUEUE, ERR_GENERIC,
                                        (errorparameter_t)i,
                                        (errorparameter_t)979);
         openserial_printCritical(COMPONENT_OPENQUEUE, ERR_GENERIC,
                                        (errorparameter_t)creator,
                                        (errorparameter_t)979);

      }
   }
   ENABLE_INTERRUPTS();
}

/**
\brief Free all the packet buffers owned by a specific module.

\param owner The identifier of the component, taken in COMPONENT_*.
*/
void openqueue_removeAllOwnedBy(uint8_t owner) {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++){
      if (openqueue_vars.queue[i].owner==owner) {
         openqueue_reset_entry(&(openqueue_vars.queue[i]));
      }
   }
   ENABLE_INTERRUPTS();
}

/**
\brief Count the number of packets in the queue with a specific track.

\param id of the track.
\returns the number of packets with track
*/
uint8_t openqueue_count_track(track_t track) {
   uint8_t i;
   uint8_t resVal = 0;

   for (i=0;i<QUEUELENGTH;i++){
      if(
            sixtop_track_equal(openqueue_vars.queue[i].l2_track, track)
            && openqueue_vars.queue[i].creator != COMPONENT_NULL
            )
         resVal++;
   }
   return(resVal);
}

//======= called by RES

OpenQueueEntry_t* openqueue_sixtopGetSentPacket() {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {
      if (openqueue_vars.queue[i].owner==COMPONENT_IEEE802154E_TO_SIXTOP &&
          openqueue_vars.queue[i].creator!=COMPONENT_IEEE802154E) {
         ENABLE_INTERRUPTS();
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}

OpenQueueEntry_t* openqueue_sixtopGetReceivedPacket() {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {
      if (openqueue_vars.queue[i].owner==COMPONENT_IEEE802154E_TO_SIXTOP &&
          openqueue_vars.queue[i].creator==COMPONENT_IEEE802154E) {
         ENABLE_INTERRUPTS();
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}

//======= called by IEEE80215E

OpenQueueEntry_t* openqueue_macGetDataPacket(open_addr_t* toNeighbor, track_t *track) {
   uint8_t  i;

   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   if (toNeighbor->type == ADDR_64B) {
      // a neighbor is specified, look for a packet unicast to that neighbor
      for (i=0;i<QUEUELENGTH;i++) {
         if (openqueue_vars.queue[i].owner==COMPONENT_SIXTOP_TO_IEEE802154E &&
            packetfunctions_sameAddress(toNeighbor, &openqueue_vars.queue[i].l2_nextORpreviousHop) &&
            (openqueue_vars.queue[i].l2_track.instance == track->instance) &&
            (packetfunctions_sameAddress(&(openqueue_vars.queue[i].l2_track.owner), &(track->owner)))
            ) {
            ENABLE_INTERRUPTS();

           /* openserial_printCritical(COMPONENT_OPENQUEUE, ERR_GENERIC,
                                         (errorparameter_t)i,
                                         (errorparameter_t)980);
*/
            return &openqueue_vars.queue[i];
         }
      }
   } else if (toNeighbor->type==ADDR_ANYCAST) {
      // anycast case: look for a packet which is either not created by RES
      // or an KA (created by RES, but not broadcast)
      // and MUST be a best effort packet (else, should use a specific track)
      for (i=0;i<QUEUELENGTH;i++) {
         if ((openqueue_vars.queue[i].l2_track.instance == TRACK_BESTEFFORT) &&
             (openqueue_vars.queue[i].owner == COMPONENT_SIXTOP_TO_IEEE802154E) &&
             (openqueue_vars.queue[i].creator != COMPONENT_SIXTOP ||
                (
                   openqueue_vars.queue[i].creator==COMPONENT_SIXTOP &&
                   packetfunctions_isBroadcastMulticast(&(openqueue_vars.queue[i].l2_nextORpreviousHop))==FALSE
                )
             )
            ) {
            ENABLE_INTERRUPTS();
/*
            openserial_printCritical(COMPONENT_OPENQUEUE, ERR_GENERIC,
                                         (errorparameter_t)i,
                                         (errorparameter_t)981);
*/
            return &openqueue_vars.queue[i];
         }
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}

OpenQueueEntry_t* openqueue_macGetAdvPacket() {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {
      if (openqueue_vars.queue[i].owner==COMPONENT_SIXTOP_TO_IEEE802154E &&
          openqueue_vars.queue[i].creator==COMPONENT_SIXTOP              &&
          packetfunctions_isBroadcastMulticast(&(openqueue_vars.queue[i].l2_nextORpreviousHop))) {
         ENABLE_INTERRUPTS();
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}

//returns the i^th packet of the queue (called by OTF to walk enough resource is allocated for the present queue)
OpenQueueEntry_t* openqueue_getPacket(uint8_t i) {
   return (&(openqueue_vars.queue[i]));
}



//=========================== private =========================================

void openqueue_reset_entry(OpenQueueEntry_t* entry) {
   //admin
   entry->creator                      = COMPONENT_NULL;
   entry->owner                        = COMPONENT_NULL;
   entry->payload                      = &(entry->packet[127]);
   entry->length                       = 0;
   //l4
   entry->l4_protocol                  = IANA_UNDEFINED;
   //l3
   entry->l3_destinationAdd.type       = ADDR_NONE;
   entry->l3_sourceAdd.type            = ADDR_NONE;
   //l2
   entry->l2_nextORpreviousHop.type    = ADDR_NONE;
   entry->l2_frameType                 = IEEE154_TYPE_UNDEFINED;
   entry->l2_retriesLeft               = 0;
   entry->l2_IEListPresent             = 0;
   bzero(&(entry->l2_track), sizeof(track_t));
   entry->l2_track.owner.type          = ADDR_NONE;
   entry->l2_track.instance            = TRACK_BESTEFFORT;
   bzero(entry->timeout.byte, sizeof(timeout_t));
}
