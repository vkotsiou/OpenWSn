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
//   otf_vars. = ;
}

void otf_notif_addedCell(void) {
   scheduler_push_task(otf_addCell_task,TASKPRIO_OTF);
}

void otf_notif_removedCell(void) {
   scheduler_push_task(otf_removeCell_task,TASKPRIO_OTF);
}

//a packet is pushed to the MAC layer -> OTF notification
void otf_NotifTransmit(OpenQueueEntry_t* msg){
      uint8_t nbCells_curr, nbCells_req;

      nbCells_curr   = schedule_getNbCellsWithTrackId(msg->l2_trackId);
      nbCells_req    = openqueue_count_trackId(msg->l2_trackId);

      //everything is ok
      if (nbCells_curr >= nbCells_req)
         return;

      //debug
      openserial_printError(
         COMPONENT_OTF,
         ERR_OTF_INSUFFICIENT,
         (errorparameter_t)msg->l2_trackId,
         (errorparameter_t)nbCells_curr
      );

      sixtop_addCells(&(msg->l2_nextORpreviousHop), nbCells_req - nbCells_curr, msg->l2_trackId);
}

//=========================== private =========================================

void otf_addCell_task(void) {
   open_addr_t          neighbor;
   bool                 foundNeighbor;
   
   // get preferred parent
   foundNeighbor = neighbors_getPreferredParentEui64(&neighbor);
   if (foundNeighbor==FALSE) {
      return;
   }
   
   // call sixtop
   sixtop_addCells(
      &neighbor,
      1,
      TRACK_BESTEFFORT
   );
}

void otf_removeCell_task(void) {
   open_addr_t          neighbor;
   bool                 foundNeighbor;
   
   // get preferred parent
   foundNeighbor = neighbors_getPreferredParentEui64(&neighbor);
   if (foundNeighbor==FALSE) {
      return;
   }
   
   // call sixtop
   sixtop_removeCell(
      &neighbor,
      TRACK_BESTEFFORT
   );
}
