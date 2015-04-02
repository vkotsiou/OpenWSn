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


/****
 * Nothing to do when sixtop has removed / added some cells
 * we (or our neighbor) already asked for that
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

      //trackid 0 -> only periodical
      if (msg->l2_trackId.owner == TRACK_BESTEFFORT && msg->l2_trackId.instance == TRACK_BESTEFFORT)
         return;


      // requested and current allocations
      nbCells_curr   = schedule_getNbCellsWithTrackId(msg->l2_trackId);
      nbCells_req    = openqueue_count_trackId(msg->l2_trackId);

      //correct allocation
      if (nbCells_curr >= nbCells_req)
         return;


      //ask 6top only if no other request is on-the-fly
      if (sixtop_getState() == SIX_IDLE){

         //debug
         openserial_printError(
              COMPONENT_OTF,
              ERR_OTF_INSUFFICIENT,
              (errorparameter_t)(uint16_t)(msg->l2_trackId.instance),
              (errorparameter_t)nbCells_req
           );


         sixtop_addCells(&(msg->l2_nextORpreviousHop), nbCells_req - nbCells_curr, msg->l2_trackId);

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
   trackId_t            trackId;

   trackId.owner        = TRACK_BESTEFFORT;
   trackId.instance     = TRACK_BESTEFFORT;

   // get preferred parent
   foundNeighbor = neighbors_getPreferredParentEui64(&neighbor);
   if (foundNeighbor==FALSE) {
      return;
   }

   // call sixtop
   sixtop_addCells(
      &neighbor,
      1,
      trackId
   );
}

void otf_removeCell_task(void) {
   open_addr_t          neighbor;
   bool                 foundNeighbor;
   trackId_t            trackId;

   trackId.owner        = TRACK_BESTEFFORT;
   trackId.instance     = TRACK_BESTEFFORT;

   // get preferred parent
   foundNeighbor = neighbors_getPreferredParentEui64(&neighbor);
   if (foundNeighbor==FALSE) {
      return;
   }

   // call sixtop
   sixtop_removeCell(
      &neighbor,
      trackId
   );
}
