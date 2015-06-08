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


/****otf_notif_addedCell
 * Nothing to do when sixtop has removed / added some cells
 * i.e. We DON'T reserve a track for the whole path toward the sink. It is reserved hop-by-hop
 */
void otf_notif_addedCell(void) {
   //  scheduler_push_task(otf_addCell_task,TASKPRIO_OTF);

   //does another message require an oTF reservation?
   otf_verif_openqueue();
}

void otf_notif_removedCell(void) {
//   scheduler_push_task(otf_removeCell_task,TASKPRIO_OTF);
}

//to update the schedule (for on the fly re-scheduling)
void otf_update_schedule(void){

}


//a packet is pushed to the MAC layer -> OTF notification
void otf_notif_transmit(OpenQueueEntry_t* msg){

#ifdef OTF_AGRESSIVE
   otf_reserve_for(msg);
#endif
}


//verifies no packet in openqeueue requires a track
void otf_verif_openqueue(void){
   uint8_t  i;
   OpenQueueEntry_t* msg;

   //only one request may be transmitted through sixtop.
   //This function will be called back when sixtop has finished its reservation later for the other messages in the queue
   for (i=0;i<QUEUELENGTH;i++){

      msg = openqueue_getPacket(i);

      if(msg->creator != COMPONENT_NULL)
          if(otf_reserve_for(msg) > 0)
             return;
  }

}


//asks 6top to reserve a cell if we don't have enough for this packet
//returns the number of cells asked to 6top
uint8_t otf_reserve_for(OpenQueueEntry_t* msg){
   uint8_t nbCells_curr, nbCells_req;


     //error
     if (msg->l2_track.instance == TRACK_BESTEFFORT && msg->l2_track.owner.type != ADDR_NONE){
        openserial_printError(
                     COMPONENT_OTF,
                     ERR_BAD_TRACKID,
                     (errorparameter_t)(uint16_t)(msg->l2_track.owner.type),
                     (errorparameter_t)msg->l2_track.owner.addr_64b[7]
                  );
        return(0);
     }


     // track 0 -> only periodical, no sixtop requests
     if (msg->l2_track.instance == TRACK_BESTEFFORT)
        return(0);

     //when 6top will have finished, otf will ask for bandwidth for this packet (if reqquired)
        if (sixtop_getState() == SIX_IDLE)
           return(0);


     // requested and current allocations
     nbCells_curr   = schedule_getNbCellsWithTrack(msg->l2_track);
     nbCells_req    = openqueue_count_track(msg->l2_track);

     //correct allocation
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


//=========================== private =========================================

void otf_addCell_task(void) {
   open_addr_t          neighbor;
   bool                 foundNeighbor;
   track_t              track;

   track.owner.type   = ADDR_NONE;
   track.instance     = TRACK_BESTEFFORT;

   // get preferred parent
   foundNeighbor = neighbors_getPreferredParentEui64(&neighbor);
   if (foundNeighbor==FALSE) {
      return;
   }

   // call sixtop
   sixtop_addCells(
      &neighbor,
      1,
      track
   );
}

void otf_removeCell_task(void) {
   open_addr_t          neighbor;
   bool                 foundNeighbor;
   track_t              track;

   track.owner.type   = ADDR_NONE;        //no owner for the best effort track
   track.instance     = TRACK_BESTEFFORT;

   // get preferred parent
   foundNeighbor = neighbors_getPreferredParentEui64(&neighbor);
   if (foundNeighbor==FALSE) {
      return;
   }

   // call sixtop
   sixtop_removeCell(
      &neighbor,
      track
   );
}
