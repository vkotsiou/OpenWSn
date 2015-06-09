#include "opendefs.h"
#include "otf.h"
#include "neighbors.h"
#include "sixtop.h"
#include "scheduler.h"
#include "openqueue.h"
#include "schedule.h"
#include "openserial.h"


//=========================== variables =======================================

//=========================== prototypes ======================================

void     otf_addCell_task(void);
void     otf_removeCell_task(void);
void     otf_verif_openqueue(void);
uint8_t  otf_reserve_for(OpenQueueEntry_t* msg);


otf_vars_t  otf_vars;

//=========================== public ==========================================

void otf_init(void) {
}




//asks 6top to reserve a cell if we don't have enough for this packet
//returns the number of cells asked to 6top
uint8_t otf_reserve_agressive_for(OpenQueueEntry_t* msg){
   uint8_t nbCells_curr, nbCells_req;

   //when 6top will have finished, otf will ask for bandwidth for this packet (if reqquired)
    if (sixtop_getState() != SIX_IDLE)
       return(0);

   // track 0 -> only periodical, no sixtop requests
   if (msg->l2_track.instance == TRACK_BESTEFFORT)
      return(0);

   // requested and current allocations
   nbCells_curr   = schedule_getNbCellsWithTrack(msg->l2_track);
   nbCells_req    = openqueue_count_track(msg->l2_track);

   //the current allocation is correct
   if (nbCells_curr >= nbCells_req)
      return(0);

   //debug
   openserial_printError(
       COMPONENT_OTF,
       ERR_OTF_INSUFFICIENT,
       (errorparameter_t)(uint16_t)(msg->l2_track.instance),
       (errorparameter_t)nbCells_req
   );

   //ask 6top the required number of cells
   sixtop_addCells(&(msg->l2_nextORpreviousHop), nbCells_req - nbCells_curr, msg->l2_track);
   return(nbCells_req - nbCells_curr);

}


//aggressive allocation: walks in openqueue and verifies enough cells are schedules to empty the queue during the slotframe
void otf_update_agressive(void){
   uint8_t  i;
   OpenQueueEntry_t* msg;

   openserial_printError(
       COMPONENT_OTF,
       ERR_GENERIC,
       (errorparameter_t)123,
       (errorparameter_t)12
   );

   //only one request may be transmitted through sixtop.
   //This function will be called back when sixtop has finished its reservation later for the other messages in the queue
   for (i=0;i<QUEUELENGTH;i++){
      msg = openqueue_getPacket(i);

      if(msg->owner != COMPONENT_NULL)
          if(otf_reserve_agressive_for(msg) > 0)
             return;
  }
}


void otf_update_schedule(void){
   openserial_printError(
        COMPONENT_OTF,
        ERR_GENERIC,
        (errorparameter_t)123,
        (errorparameter_t)45
    );

#ifdef OTF_AGRESSIVE
   otf_update_agressive();
#endif
}

//a packet is pushed to the MAC layer -> OTF notification
void otf_notif_transmit(OpenQueueEntry_t* msg){
#ifdef OTF_AGRESSIVE
   otf_reserve_agressive_for(msg);
#endif
}




/****otf_notif_addedCell
 * Nothing to do when sixtop has removed / added some cells
 * i.e. We DON'T reserve a track for the whole path toward the sink. It is reserved hop-by-hop
 */
void otf_notif_addedCell(void) {
   //  scheduler_push_task(otf_addCell_task,TASKPRIO_OTF);
}

void otf_notif_removedCell(void) {
//   scheduler_push_task(otf_removeCell_task,TASKPRIO_OTF);
}




//=========================== private =========================================

