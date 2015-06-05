#include "opendefs.h"
#include "otf.h"
#include "neighbors.h"
#include "sixtop.h"
#include "scheduler.h"

//=========================== variables =======================================

//=========================== prototypes ======================================

void otf_addCell_task(void);
void otf_removeCell_task(void);

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
   uint8_t nbCells_curr, nbCells_req;


      //error
      if (msg->l2_track.instance == TRACK_BESTEFFORT && msg->l2_track.owner.type != ADDR_NONE){
         openserial_printError(
                      COMPONENT_OTF,
                      ERR_BAD_TRACKID,
                      (errorparameter_t)(uint16_t)(msg->l2_track.owner.type),
                      (errorparameter_t)msg->l2_track.owner.addr_64b
                   );
      }


      // track 0 -> only periodical
      if (msg->l2_track.instance == TRACK_BESTEFFORT)
         return;

      // requested and current allocations
      nbCells_curr   = schedule_getNbCellsWithTrack(msg->l2_track);
      nbCells_req    = openqueue_count_track(msg->l2_track);

      //correct allocation
      if (nbCells_curr >= nbCells_req)
         return;


      //ask 6top only if no other request is on-the-fly
      if (sixtop_getState() == SIX_IDLE){

         //debug
         openserial_printError(
              COMPONENT_OTF,
              ERR_OTF_INSUFFICIENT,
              (errorparameter_t)(uint16_t)(msg->l2_track.instance),
              (errorparameter_t)nbCells_req
           );


         sixtop_addCells(&(msg->l2_nextORpreviousHop), nbCells_req - nbCells_curr, msg->l2_track);

      }
      else{
         //TODO: when 6top has finished, should ask otf to verify the schedule is sufficient for the other tracks

         openserial_printError(
              COMPONENT_OTF,
              ERR_SIXTOP_WRONG_STATE,
              (errorparameter_t)sixtop_getState(),
              (errorparameter_t)SIX_IDLE
           );

      }
#endif
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
