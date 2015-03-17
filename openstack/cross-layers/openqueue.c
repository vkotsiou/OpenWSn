#include "opendefs.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "IEEE802154E.h"

//=========================== variables =======================================

openqueue_vars_t openqueue_vars;

//=========================== prototypes ======================================

void openqueue_reset_entry(OpenQueueEntry_t* entry);
//void openqueue_timeout_flush(void);

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
   openqueue_vars.timeoutScheduled = FALSE;

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

 //  memcpy(&(output.timeout), &(openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout), sizeof(asn_t));


   //ASN to push (in ASN format)
   output.timeout.byte4 = openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[4];
   output.timeout.bytes0and1 =
         openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[0] +
         openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[1] * 256;
   output.timeout.bytes2and3 =
         openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[2] +
         openqueue_vars.queue[openqueue_vars.debugPrintRow].timeout.byte[3] *256;


   openserial_printStatus(
         STATUS_QUEUE,
         (uint8_t*)&output,
         sizeof(debugOpenQueueEntry_t));
   return TRUE;
}

//is a >= b
bool openqueue_timeout_is_greater(timeout_t a, timeout_t b){
   uint8_t  i;

   //for each byte (byte 4 is the biggest)
   for(i=4; i<=0; i--)
      if (a.byte[i] > b.byte[i])
         return(TRUE);
      else if (a.byte[i] < b.byte[i])
         return(FALSE);

   //equal case
   return(TRUE);
}

//this represents an invalid timeout
bool openqueue_timeout_is_zero(timeout_t value){
   uint8_t  i;

   for (i=0; i<5; i++)
      if (value.byte[i] != 0)
      return FALSE;

   return(TRUE);
}

//returns a - b (or 0 if a <= b)
uint64_t openqueue_timeout_diff(timeout_t a, timeout_t b){
   uint8_t  i;
   bool     greater = FALSE;
   uint64_t diff = 0;

   for(i=sizeof(timeout_t)-1; i>=0; i--){

      //returns 0 if a <= b
      if (!greater && a.byte[i] < b.byte[i])
         return(0);
      else if (a.byte[i] > b.byte[i])
         greater = TRUE;

      //else, computes the difference
      diff = diff * 256 + a.byte[i] - b.byte[i];

   }
   return(diff);
}


//remove the packets which are timeouted in the queue
void openqueue_timeout_flush(void){
   uint8_t     i;
   timeout_t   now;
   uint64_t    diff_oldest, tmp;

   //initialization
   ieee154e_getAsn(now.byte);
   //bzero(oldest.byte, sizeof(timeout_t));
   diff_oldest = 0;


   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   for (i=0;i<QUEUELENGTH;i++) {

      //this timeout must not be considered
      if(openqueue_timeout_is_zero(openqueue_vars.queue[i].timeout))
      {}

      //this timeout has elapsed
      else if (openqueue_timeout_is_greater(
            now,
            openqueue_vars.queue[i].timeout)) {
         openqueue_reset_entry(&(openqueue_vars.queue[i]));
      }

      //entry is in the future, and we must remember the oldest one
      else if (tmp = openqueue_timeout_diff(now, openqueue_vars.queue[i].timeout) < diff_oldest)
         diff_oldest = tmp;
         /*(openqueue_timeout_is_greater(
            oldest,
            openqueue_vars.queue[i].timeout))
         memcpy(&oldest, &(openqueue_vars.queue[i].timeout), 5);
         */

   }
   //next verification
   if (diff_oldest != 0)
      openqueue_vars.timeoutTimerId = opentimers_start(
            diff_oldest * TsSlotDuration,
            TIMER_ONESHOT,
            TIME_MS,
            openqueue_timeout_flush
      );
   else
      openqueue_vars.timeoutScheduled = FALSE;

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
         openqueue_vars.queue[i].creator=creator;
         openqueue_vars.queue[i].owner=COMPONENT_OPENQUEUE;
         bzero(&(openqueue_vars.queue[i].timeout),sizeof(timeout_t)); //by default, no timeout
         ENABLE_INTERRUPTS(); 
         return &openqueue_vars.queue[i];
      }
   }
   ENABLE_INTERRUPTS();
   return NULL;
}




//======= called by any component

/**
\brief Request a new (free) packet buffer, specifying a timeout

\note Once a packet has been allocated, it is up to the creator of the packet
      to free it using the openqueue_freePacketBuffer() function.

\returns A pointer to the queue entry when it could be allocated, or NULL when
         it could not be allocated (buffer full or not synchronized).
*/
OpenQueueEntry_t* openqueue_getFreePacketBuffer_with_timeout(uint8_t creator, const timeout_t timeout) {
   OpenQueueEntry_t* entry;
   timeout_t     now;
   timeout_t     res;
   uint8_t       remainder, i;
   uint64_t       diff;

   entry = openqueue_getFreePacketBuffer(creator);

   //no packet is available
   if (entry == NULL)
      return(NULL);

   //computes when this entry will become obsolete
   ieee154e_getAsn(now.byte);
   remainder = 0;
   for(i=0; i<sizeof(timeout_t); i++){
      res.byte[i] = timeout.byte[i] + now.byte[i] + remainder;
      if (res.byte[i] < timeout.byte[i] && res.byte[i] < now.byte[i])
         remainder = 1;
      else
         remainder = 0;
   }

   //saves the timeout in the new entry
   for(i=0;i<5;i++)
      entry->timeout.byte[i] = res.byte[i];


   if (openqueue_vars.timeoutScheduled)
      return(entry);

   //verification schedule
   diff = openqueue_timeout_diff(now, openqueue_vars.queue[i].timeout);
   openqueue_vars.timeoutTimerId = opentimers_start(
         diff * TsSlotDuration,
         TIMER_ONESHOT,
         TIME_MS,
         openqueue_timeout_flush
   );
   openqueue_vars.timeoutScheduled = TRUE;


   return(entry);
}





/**
\brief Free a previously-allocated packet buffer.

\param pkt A pointer to the previsouly-allocated packet buffer.

\returns E_SUCCESS when the freeing was succeful.
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
      if (openqueue_vars.queue[i].creator==creator) {
         openqueue_reset_entry(&(openqueue_vars.queue[i]));
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
\brief Count the number of packets in the queue with a specific trackId.

\param id of the trackId.
\returns the number of packets with trackId
*/
uint8_t openqueue_count_trackId(trackId_t id) {
   uint8_t i;
   uint8_t resVal = 0;

   for (i=0;i<QUEUELENGTH;i++){
      if(
            openqueue_vars.queue[i].l2_trackId == id
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

OpenQueueEntry_t* openqueue_macGetDataPacket(open_addr_t* toNeighbor) {
   uint8_t i;
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   if (toNeighbor->type==ADDR_64B) {
      // a neighbor is specified, look for a packet unicast to that neigbhbor
      for (i=0;i<QUEUELENGTH;i++) {
         if (openqueue_vars.queue[i].owner==COMPONENT_SIXTOP_TO_IEEE802154E &&
            packetfunctions_sameAddress(toNeighbor,&openqueue_vars.queue[i].l2_nextORpreviousHop)) {
            ENABLE_INTERRUPTS();
            return &openqueue_vars.queue[i];
         }
      }
   } else if (toNeighbor->type==ADDR_ANYCAST) {
      // anycast case: look for a packet which is either not created by RES
      // or an KA (created by RES, but not broadcast)
      for (i=0;i<QUEUELENGTH;i++) {
         if (openqueue_vars.queue[i].owner==COMPONENT_SIXTOP_TO_IEEE802154E &&
             ( openqueue_vars.queue[i].creator!=COMPONENT_SIXTOP ||
                (
                   openqueue_vars.queue[i].creator==COMPONENT_SIXTOP &&
                   packetfunctions_isBroadcastMulticast(&(openqueue_vars.queue[i].l2_nextORpreviousHop))==FALSE
                )
             )
            ) {
            ENABLE_INTERRUPTS();
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
   entry->l2_trackId                   = TRACK_BESTEFFORT;
}
