#include "opendefs.h"
#include "neighbors.h"
#include "openqueue.h"
#include "openrandom.h"
#include "packetfunctions.h"
#include "idmanager.h"
#include "openserial.h"
#include "IEEE802154E.h"
#include "otf.h"
#include "sixtop.h"
#include "schedule.h"
#include <stdio.h>


//#define _DEBUG_ICMPv6RPL_RANK_
//#define _DEBUG_NEIGHBORS_
//#define _DEBUG_NEIGHBORS_DETAIL_
#define _DEBUG_NEIGHBORS_DETAIL_PLUS

//=========================== variables =======================================

neighbors_vars_t neighbors_vars;

//=========================== prototypes ======================================

void registerNewNeighbor(
        open_addr_t* neighborID,
        int8_t       rssi,
        asn_t*       asnTimestamp,
        bool         joinPrioPresent,
        uint8_t      joinPrio
     );
bool isNeighbor(open_addr_t* neighbor);
void removeNeighbor(uint8_t neighborIndex);
bool isThisRowMatching(
        open_addr_t* address,
        uint8_t      rowNumber
     );

//=========================== public ==========================================

/**
\brief Initializes this module.
*/
void neighbors_init() {
   uint8_t i;
 
   // clear module variables
   memset(&neighbors_vars,0,sizeof(neighbors_vars_t));
   
   // set myDAGrank
   if (idmanager_getIsDAGroot()==TRUE) {
      neighbors_vars.myDAGrank=0;
   } else {
      neighbors_vars.myDAGrank=DEFAULTDAGRANK;
   }

   // init balance factor and bottleneck set
   for (i=0; i<MAX_NUM_BTNECKS; i++){
      neighbors_vars.traffic_ratios[i] = 0;
      memset(&(neighbors_vars.btnecks[i]),0,sizeof(btneck_t));
   }

   neighbors_vars.bootstrap_period = TRUE;

   // Init DAG root id 
   neighbors_vars.DAGroot_id = 250;
}

//===== getters

/**
\brief Retrieve this mote's current DAG rank.

\returns This mote's current DAG rank.
*/
dagrank_t neighbors_getMyDAGrank() {
   return neighbors_vars.myDAGrank;
}

/**
\brief Retrieve the number of neighbors this mote's currently knows of.

\returns The number of neighbors this mote's currently knows of.
*/
uint8_t neighbors_getNumNeighbors() {
   uint8_t i;
   uint8_t returnVal;
   
   returnVal=0;
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         returnVal++;
      }
   }
   return returnVal;
}

/**
\brief Retrieve the number of bottlenecks this mote's currently selected.

\returns The number of bottlenecks this mote's currently selected.
*/
uint8_t neighbors_getNumBtnecks() {
   uint8_t i;
   uint8_t returnVal;
   
   for (i=0; i<MAX_NUM_BTNECKS; i++) {
      if (neighbors_vars.btnecks[i].counter != 0){
         returnVal++;
      }
   }
   return returnVal;
}   

bool neighbors_getPreferredTrack(open_addr_t* addressToWrite){
   uint8_t i;
   bool track_found;
   uint8_t cursor;
   uint8_t rand;

   cursor = 0;
   track_found = FALSE;

   // get random number between 1 and 100
   rand = (uint8_t)((openrandom_get16b() % 99)+1);

   // get the corresponding track
   for (i=0; i<MAX_NUM_BTNECKS; i++) {
      if(neighbors_vars.btnecks[i].counter != 0){
         if (neighbors_vars.traffic_ratios[i] != 0){
            if (rand >= cursor && rand < cursor + neighbors_vars.traffic_ratios[i]){
               // copy address of preferred track
               memcpy(addressToWrite->addr_64b,&(neighbors_vars.btnecks[i].addr_64b),LENGTH_ADDR64b);
               addressToWrite->type = ADDR_64B;
               track_found = TRUE;         
               break;
            } 
            cursor += neighbors_vars.traffic_ratios[i];
         }
      }
   }
   return track_found;
}

 
/**
\brief Retrieve the address of the parent that:
   - Advertise the track
   - Have the smallest path ETX to that track
*/
bool neighbors_getPreferredTrackParent(open_addr_t track_owner, open_addr_t* addressToWrite) {
   uint8_t i,j;
   uint16_t min_etx;
   uint16_t link_etx;
   uint16_t total_etx;

   min_etx = 0xFFFF; // init to max value

   // get parents (in parent set) announcing given track and choose the best one (smaller ETX for a path)
   for (i=0; i<MAXNUMNEIGHBORS; i++) {
      if (neighbors_vars.neighbors[i].inParentSet==TRUE){
         // compute the ETX with the neighbor
         link_etx = (uint16_t)((float)neighbors_vars.neighbors[i].numTx+1
                              /(float)neighbors_vars.neighbors[i].numTxACK+1);

         // save minimal ETX
         for (j=0; j<MAX_NUM_BTNECKS; j++){
            if(neighbors_vars.neighbors[i].btnecks[j].counter != 0){
               // if neighbor announces the track
               if (packetfunctions_sameAddress64(&neighbors_vars.neighbors[i].btnecks[j].addr_64b,
                        &(track_owner.addr_64b))){
                  // add last hop ETX to btneck ETX
                  total_etx = link_etx + neighbors_vars.neighbors[i].btnecks[j].etx;

                  //save neighbor address if offered ETX is minimal
                  if (total_etx < min_etx){
                     memcpy(addressToWrite, &(neighbors_vars.neighbors[i].addr_64b), sizeof(open_addr_t));
                     min_etx = total_etx;
                     break; // only on btneck per neighbor can correspond to a track
                  }
               }
            }
         }
      }
   }

   if (min_etx != 0xFFFF)
      return TRUE;
   else
      return FALSE;
}

/**
\brief Retreive the advertised bottleneck with updated ETX values
**/
void neighbors_getAdvBtnecks(btneck_t * advBtnecks){
   uint8_t i,j,k,l;
   uint8_t sum_etx;     // ETX value of the path between btneck and parent
   uint16_t link_etx;   // ETX value of last hop
   uint8_t nb_etx;      // Number of ETX value per track
   uint16_t avg_etx;    // Average value of path ETX for a track
   bool btneck_found;
   uint8_t self_counter;

   btneck_found = FALSE;
   
   // copy selected bottleneck list
   for (l=0; l<MAX_NUM_BTNECKS;l++){
      memset(&advBtnecks[l],0, sizeof(btneck_t));
   }

   // get local counter
   self_counter = sixtop_getBtneckCounter();

   for (i=0; i<MAX_NUM_BTNECKS; i++){
      if (neighbors_vars.btnecks[i].counter != 0){
         sum_etx = 0;
         nb_etx = 0;

         // if at least one btneck is more constrained than I am => self announcement
         // or bottleneck is DAG root
         if (neighbors_vars.btnecks[i].counter > self_counter 
               || (neighbors_isDAGroot(&neighbors_vars.btnecks[i].addr_64b))){
            btneck_found = FALSE;
            break;
         }

         // for each parent get the btneck path ETX and add the last hop ETX
         for (j=0;j<MAXNUMNEIGHBORS;j++) {
            if (neighbors_vars.neighbors[j].inParentSet==TRUE){
               for (k=0; k<MAX_NUM_BTNECKS; k++){
                  if(neighbors_vars.neighbors[j].btnecks[k].counter != 0){
                     // if parent advertise the bottleneck, record the path ETX
                     if (packetfunctions_sameAddress64(&neighbors_vars.neighbors[j].btnecks[k].addr_64b,
                              &neighbors_vars.btnecks[i].addr_64b)){
                        link_etx = (uint16_t)((float)(neighbors_vars.neighbors[j].numTx +1)
                              / (float)(neighbors_vars.neighbors[j].numTxACK +1));
                        sum_etx += neighbors_vars.btnecks[i].etx + link_etx;
                        nb_etx++;
                        btneck_found = TRUE;
                     }
                  }
               }
            }
         }

         // if at least one parent was found
         if (nb_etx != 0){
            // copy local btneck
            memcpy(&advBtnecks[i],&neighbors_vars.btnecks[i], sizeof(btneck_t));

            // Calculate ETX average for this bottleneck
            avg_etx = (uint16_t)((float)sum_etx / (float)nb_etx);

            // update the returned value
            advBtnecks[i].etx = avg_etx;
         }
      }
   }

#ifdef _DEBUG_NEIGHBORS_DETAIL_PLU
   char strd[150];
   sprintf(strd, "NEIGH - Adv btnecks:");
   for(l=0; l< MAX_NUM_BTNECKS; l++){
      if (advBtnecks[l].counter != 0){
         strncat(strd, " --->", 150);
         openserial_ncat_uint8_t_hex(strd, (uint8_t)advBtnecks[l].addr_64b[6], 150);
         openserial_ncat_uint8_t_hex(strd, (uint8_t)advBtnecks[l].addr_64b[7], 150);
         strncat(strd, " ratio: ", 150);
         openserial_ncat_uint32_t(strd, (uint32_t)neighbors_vars.traffic_ratios[l], 150);
         strncat(strd, " etx: ", 150);
         openserial_ncat_uint32_t(strd, (uint32_t)advBtnecks[l].etx, 150);
         openserial_printf(COMPONENT_NEIGHBORS, strd, strlen(strd));
      }
   }
#endif

   if (btneck_found == FALSE){
      // empty local list
      memset(advBtnecks,0,sizeof(btneck_t)*MAX_NUM_BTNECKS);

      // self announcement
      memcpy(advBtnecks[0].addr_64b, idmanager_getMyID(ADDR_64B)->addr_64b, LENGTH_ADDR64b); 
      advBtnecks[0].counter = self_counter;
      advBtnecks[0].etx = 0;
   }

}

/**
\brief Retrieve my preferred parent's EUI64 address.

\param[out] addressToWrite Where to write the preferred parent's address to.
*/
bool neighbors_getPreferredParentEui64(open_addr_t* addressToWrite) {
   uint8_t   i;
   bool      foundPreferred;
   uint8_t   numNeighbors;
   dagrank_t minRankVal;
   uint8_t   minRankIdx;
   
   addressToWrite->type = ADDR_NONE;
   
   foundPreferred       = FALSE;
   numNeighbors         = 0;
   minRankVal           = MAXDAGRANK;
   minRankIdx           = MAXNUMNEIGHBORS+1;
   
   //===== step 1. Try to find preferred parent
   for (i=0; i<MAXNUMNEIGHBORS; i++) {
      if (neighbors_vars.neighbors[i].used==TRUE){
         if (neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
            memcpy(addressToWrite,&(neighbors_vars.neighbors[i].addr_64b),sizeof(open_addr_t));
            addressToWrite->type=ADDR_64B;
            foundPreferred=TRUE;
         }
         // identify neighbor with lowest rank
         if (neighbors_vars.neighbors[i].DAGrank < minRankVal) {
            minRankVal=neighbors_vars.neighbors[i].DAGrank;
            minRankIdx=i;
         }

         numNeighbors++;
      }
   }
   
   //===== step 2. (backup) Promote neighbor with min rank to preferred parent
   if (foundPreferred==FALSE && numNeighbors > 0){
      // promote neighbor
      neighbors_vars.neighbors[minRankIdx].parentPreference       = MAXPREFERENCE;
      neighbors_vars.neighbors[minRankIdx].stableNeighbor         = TRUE;
      neighbors_vars.neighbors[minRankIdx].switchStabilityCounter = 0;
      // return its address
      memcpy(addressToWrite,&(neighbors_vars.neighbors[minRankIdx].addr_64b),sizeof(open_addr_t));
      addressToWrite->type=ADDR_64B;
      foundPreferred=TRUE;         
   }
   
   return foundPreferred;
}

/**
\brief Find neighbor to which to send KA.

This function iterates through the neighbor table and identifies the neighbor
we need to send a KA to, if any. This neighbor satisfies the following
conditions:
- it is one of our preferred parents
- we haven't heard it for over kaPeriod

\param[in] kaPeriod The maximum number of slots I'm allowed not to have heard
   it.

\returns A pointer to the neighbor's address, or NULL if no KA is needed.
*/
open_addr_t* neighbors_getKANeighbor(uint16_t kaPeriod) {
   uint8_t         i;
   uint16_t        timeSinceHeard;
   open_addr_t*    addrPreferred;
   open_addr_t*    addrOther;
   
   // initialize
   addrPreferred = NULL;
   addrOther     = NULL;
   
   // scan through the neighbor table, and populate addrPreferred and addrOther
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==1) {
         timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[i].asn);
         if (timeSinceHeard>kaPeriod) {
            // this neighbor needs to be KA'ed to
            if (neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
               // its a preferred parent
               addrPreferred = &(neighbors_vars.neighbors[i].addr_64b);
            } else {
               // its not a preferred parent
               // Note: commented out since policy is not to KA to non-preferred parents
               // addrOther =     &(neighbors_vars.neighbors[i].addr_64b);
            }
         }
      }
   }
   
   // return the EUI64 of the most urgent KA to send:
   // - if available, preferred parent
   // - if not, non-preferred parent
   if        (addrPreferred!=NULL) {
      return addrPreferred;
   } else if (addrOther!=NULL) {
      return addrOther;
   } else {
      return NULL;
   }
}

uint16_t neighbors_getNeighborMinBtneckCounter(uint8_t index){
   uint8_t i;
   uint16_t min_counter;

   min_counter = 0xffff;

   for (i=0; i<MAX_NUM_BTNECKS; i++){
      if (neighbors_vars.neighbors[index].btnecks[i].counter != 0
            && neighbors_vars.neighbors[index].btnecks[i].counter < min_counter)
         min_counter = neighbors_vars.neighbors[index].btnecks[i].counter;
   }

   return min_counter;
}
uint16_t neighbors_getMinBtneckCounter(){
   uint8_t i;
   uint16_t min_counter;

   min_counter = 0xffff;

   for (i=0; i<MAX_NUM_BTNECKS; i++){
      if (neighbors_vars.btnecks[i].counter != 0
            && neighbors_vars.btnecks[i].counter < min_counter)
         min_counter = neighbors_vars.btnecks[i].counter;
   }

   if (min_counter == 0xffff)
      return 0;
   else
      return min_counter;
}


//===== interrogators

/**
\brief Indicate whether some neighbor is a stable neighbor

\param[in] address The address of the neighbor, a full 128-bit IPv6 addres.

\returns TRUE if that neighbor is stable, FALSE otherwise.
*/
bool neighbors_isStableNeighbor(open_addr_t* address) {
   uint8_t     i;
   open_addr_t temp_addr_64b;
   open_addr_t temp_prefix;
   bool        returnVal;
   
   // by default, not stable
   returnVal  = FALSE;
   
   // but neighbor's IPv6 address in prefix and EUI64
   switch (address->type) {
      case ADDR_128B:
         packetfunctions_ip128bToMac64b(address,&temp_prefix,&temp_addr_64b);
         break;
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)0);
         return returnVal;
   }
   
   // iterate through neighbor table
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(&temp_addr_64b,i) && neighbors_vars.neighbors[i].stableNeighbor==TRUE) {
         returnVal  = TRUE;
         break;
      }
   }
   
   return returnVal;
}

/**
\brief Indicate whether some neighbor is a preferred neighbor.

\param[in] address The EUI64 address of the neighbor.

\returns TRUE if that neighbor is preferred, FALSE otherwise.
*/
bool neighbors_isPreferredParent(open_addr_t* address) {
   uint8_t i;
   bool    returnVal;
   
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   
   // by default, not preferred
   returnVal = FALSE;
   
   // iterate through neighbor table
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(address,i) && neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
         returnVal  = TRUE;
         break;
      }
   }
   
   ENABLE_INTERRUPTS();
   return returnVal;
}

/**
\brief Indicate whether some neighbor has a lower DAG rank that me.

\param[in] index The index of that neighbor in the neighbor table.

\returns TRUE if that neighbor has a lower DAG rank than me, FALSE otherwise.
*/
bool neighbors_isNeighborWithLowerDAGrank(uint8_t index) {
   bool    returnVal;
   
   if (neighbors_vars.neighbors[index].used==TRUE &&
       neighbors_vars.neighbors[index].DAGrank < neighbors_getMyDAGrank()) { 
      returnVal = TRUE;
   } else {
      returnVal = FALSE;
   }
   
   return returnVal;
}


/**
\brief Indicate whether some neighbor has a lower DAG rank that me.

\param[in] index The index of that neighbor in the neighbor table.

\returns TRUE if that neighbor has a lower DAG rank than me, FALSE otherwise.
*/
bool neighbors_isNeighborWithHigherDAGrank(uint8_t index) {
   bool    returnVal;
   
   if (neighbors_vars.neighbors[index].used==TRUE &&
       neighbors_vars.neighbors[index].DAGrank >= neighbors_getMyDAGrank()) { 
      returnVal = TRUE;
   } else {
      returnVal = FALSE;
   }
   
   return returnVal;
}

//===== updating neighbor information

/**
\brief Indicate some (non-ACK) packet was received from a neighbor.

This function should be called for each received (non-ACK) packet so neighbor
statistics in the neighbor table can be updated.

The fields which are updated are:
- numRx
- rssi
- asn
- stableNeighbor
- switchStabilityCounter

\param[in] l2_src MAC source address of the packet, i.e. the neighbor who sent
   the packet just received.
\param[in] rssi   RSSI with which this packet was received.
\param[in] asnTs  ASN at which this packet was received.
\param[in] joinPrioPresent Whether a join priority was present in the received
   packet.
\param[in] joinPrio The join priority present in the packet, if any.
*/
void neighbors_indicateRx(open_addr_t* l2_src,
                          int8_t       rssi,
                          asn_t*       asnTs,
                          bool         joinPrioPresent,
                          uint8_t      joinPrio) {
   uint8_t i;
   bool    newNeighbor;

   // update existing neighbor
   newNeighbor = TRUE;
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(l2_src,i)) {
         
         // this is not a new neighbor
         newNeighbor = FALSE;
         
         // update numRx, rssi, asn
         neighbors_vars.neighbors[i].numRx++;
         neighbors_vars.neighbors[i].rssi=rssi;
         memcpy(&neighbors_vars.neighbors[i].asn,asnTs,sizeof(asn_t));
         //update jp
         if (joinPrioPresent==TRUE){
            neighbors_vars.neighbors[i].joinPrio=joinPrio;
         }
         
         // update stableNeighbor, switchStabilityCounter
         if (neighbors_vars.neighbors[i].stableNeighbor==FALSE) {
            if (neighbors_vars.neighbors[i].rssi>BADNEIGHBORMAXRSSI) {
               neighbors_vars.neighbors[i].switchStabilityCounter++;
               if (neighbors_vars.neighbors[i].switchStabilityCounter>=SWITCHSTABILITYTHRESHOLD) {
                  neighbors_vars.neighbors[i].switchStabilityCounter=0;
                  neighbors_vars.neighbors[i].stableNeighbor=TRUE;
               }
            } else {
               neighbors_vars.neighbors[i].switchStabilityCounter=0;
            }
         } else if (neighbors_vars.neighbors[i].stableNeighbor==TRUE) {
            if (neighbors_vars.neighbors[i].rssi<GOODNEIGHBORMINRSSI) {
               neighbors_vars.neighbors[i].switchStabilityCounter++;
               if (neighbors_vars.neighbors[i].switchStabilityCounter>=SWITCHSTABILITYTHRESHOLD) {
                  neighbors_vars.neighbors[i].switchStabilityCounter=0;
                   neighbors_vars.neighbors[i].stableNeighbor=FALSE;
               }
            } else {
               neighbors_vars.neighbors[i].switchStabilityCounter=0;
            }
         }
         
         // stop looping
         break;
      }
   }
   
   // register new neighbor
   if (newNeighbor==TRUE) {
      registerNewNeighbor(l2_src, rssi, asnTs, joinPrioPresent,joinPrio);
   }
}

bool neighbors_isDAGroot(uint8_t (*address_1)[LENGTH_ADDR64b]){
   if (neighbors_vars.DAGroot_id < MAXNUMNEIGHBORS){
      if (packetfunctions_sameAddress64(address_1, 
            &(neighbors_vars.neighbors[neighbors_vars.DAGroot_id].addr_64b.addr_64b)) == TRUE)
         return TRUE;
   }
   return FALSE;
}

/**
\brief Indicate some packet was sent to some neighbor.

This function should be called for each transmitted (non-ACK) packet so
neighbor statistics in the neighbor table can be updated.

The fields which are updated are:
- numTx
- numTxACK
- asn

\param[in] l2_dest MAC destination address of the packet, i.e. the neighbor
   who I just sent the packet to.
\param[in] numTxAttempts Number of transmission attempts to this neighbor.
\param[in] was_finally_acked TRUE iff the packet was ACK'ed by the neighbor
   on final transmission attempt.
\param[in] asnTs ASN of the last transmission attempt.
*/
void neighbors_indicateTx(open_addr_t* l2_dest,
                          uint8_t      numTxAttempts,
                          bool         was_finally_acked,
                          asn_t*       asnTs) {
   uint8_t i;

   //todo-debug
      if (l2_dest->type == 0)
         openserial_printCritical(COMPONENT_IPHC, ERR_GENERIC,
                                     (errorparameter_t)l2_dest->type,
                                     (errorparameter_t)12);



   // don't run through this function if packet was sent to broadcast address
   if (packetfunctions_isBroadcastMulticast(l2_dest)==TRUE) {
      return;
   }
   
   // loop through neighbor table
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(l2_dest,i)) {
         // handle roll-over case
        
          if (neighbors_vars.neighbors[i].numTx>(0xff-numTxAttempts)) {
              neighbors_vars.neighbors[i].numWraps++; //counting the number of times that tx wraps.
              neighbors_vars.neighbors[i].numTx/=2;
              neighbors_vars.neighbors[i].numTxACK/=2;
           }
         // update statistics
        neighbors_vars.neighbors[i].numTx += numTxAttempts; 
        
        if (was_finally_acked==TRUE) {
            neighbors_vars.neighbors[i].numTxACK++;
            memcpy(&neighbors_vars.neighbors[i].asn,asnTs,sizeof(asn_t));
        }
        break;
      }
   }
}

/**
\brief Indicate I just received a RPL DIO from a neighbor.

This function should be called for each received a DIO is received so neighbor
routing information in the neighbor table can be updated.

The fields which are updated are:
- DAGrank

\param[in] msg The received message with msg->payload pointing to the DIO
   header.
*/
void neighbors_indicateRxDIO(OpenQueueEntry_t* msg) {
   uint8_t        i,j,k;
   uint8_t        nb_btnecks;

#ifdef _DEBUG_NEIGHBORS_
   char str[100];
   sprintf(str, "RX DIO");
   openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
#endif

   neighbors_vars.dio_counter++;

   // take ownership over the packet
   msg->owner = COMPONENT_NEIGHBORS;
   
   // update rank of that neighbor in table
   neighbors_vars.dio = (icmpv6rpl_dio_ht*)(msg->payload);
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(&(msg->l2_nextORpreviousHop),i)) {
         if (packetfunctions_sameAddress64((uint8_t(*)[LENGTH_ADDR64b])&neighbors_vars.dio->DODAGID[8],
                  (uint8_t(*)[LENGTH_ADDR64b])&msg->l3_sourceAdd.addr_128b[8]) == TRUE)
            neighbors_vars.DAGroot_id = i;
         if (
               neighbors_vars.dio->rank > neighbors_vars.neighbors[i].DAGrank &&
               neighbors_vars.dio->rank - neighbors_vars.neighbors[i].DAGrank >(DEFAULTLINKCOST*2*MINHOPRANKINCREASE)
            ) {
            // the new DAGrank looks suspiciously high, only increment a bit
            neighbors_vars.neighbors[i].DAGrank += (DEFAULTLINKCOST*2*MINHOPRANKINCREASE);
            openserial_printError(COMPONENT_NEIGHBORS,ERR_LARGE_DAGRANK,
                  (errorparameter_t)neighbors_vars.dio->rank,
                  (errorparameter_t)neighbors_vars.neighbors[i].DAGrank);
         } else {
            neighbors_vars.neighbors[i].DAGrank = neighbors_vars.dio->rank;
         }

         // update neighbor bottlecks
         for (j=0; j<MAX_NUM_BTNECKS; j++){
            neighbors_vars.neighbors[i].btnecks[j] = neighbors_vars.dio->btnecks[j];
            // special case for DAG root
            if (neighbors_isDAGroot(&neighbors_vars.neighbors[i].btnecks[j].addr_64b)){
               // replace DAG root counter by self counter 
               neighbors_vars.neighbors[i].btnecks[j].counter = sixtop_getBtneckCounter();
            }
         }
         break;
      }
   }
   
   if (idmanager_getIsDAGroot() == FALSE){
      // update bottlenecks
      neighbors_updateMyBottlenecksSet();

      if (neighbors_vars.bootstrap_period == TRUE){
         if (neighbors_vars.dio_counter == 3){
            neighbors_vars.bootstrap_period = FALSE;
            
            // init balance factors
            nb_btnecks = neighbors_getNumBtnecks();
            for (k=0; k<MAX_NUM_BTNECKS; k++)
               neighbors_vars.traffic_ratios[k] = (uint8_t)(100 / (float)nb_btnecks);
         }

      } else{
         // filter too constrained bottlenecks
         neighbors_filterBtnecks();

         // update parent set
         neighbors_updateMyParentsSet();

         // update my routing information
         neighbors_updateMyDAGrankAndNeighborPreference();

         // update my DAG rank    
         neighbors_updateMyDAGrankWorst();

         // update reserved tracks cells
         neighbors_updateReservedTracks();
      }
   } 

}

//===== write addresses

/**
\brief Write the 64-bit address of some neighbor to some location.
*/
void  neighbors_getNeighbor(open_addr_t* address, uint8_t addr_type, uint8_t index){
   switch(addr_type) {
      case ADDR_64B:
         memcpy(&(address->addr_64b),&(neighbors_vars.neighbors[index].addr_64b.addr_64b),LENGTH_ADDR64b);
         address->type=ADDR_64B;
         break;
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)addr_type,
                               (errorparameter_t)1);
         break; 
   }
}

//returns the whole entry concerning a neighbor
neighborRow_t *neighbors_getNeighborInfo(open_addr_t* address){
   uint8_t  i;

   for (i=0;i<MAXNUMNEIGHBORS;i++)
       if (neighbors_vars.neighbors[i].used==TRUE)
          if (packetfunctions_sameAddress(&(neighbors_vars.neighbors[i].addr_64b), address))
                return(&(neighbors_vars.neighbors[i]));

   //unfound
   return(NULL);
}


//===== managing routing info

/**
\brief Update my DAG rank and neighbor preference.

Call this function whenever some data is changed that could cause this mote's
routing decisions to change. Examples are:
- I received a DIO which updated by neighbor table. If this DIO indicated a
  very low DAGrank, I may want to change by routing parent.
- I became a DAGroot, so my DAGrank should be 0.
*/

void neighbors_updateMyDAGrankAndNeighborPreference() {
   uint8_t     i;
   uint16_t    rankIncrease;
   uint32_t    tentativeDAGrank; // 32-bit since is used to sum
   uint8_t     prefParentIdx;
   bool        prefParentFound;
   uint8_t     minRank;

   // if I'm a DAGroot, my DAGrank is always 0
   if ((idmanager_getIsDAGroot())==TRUE) {
      neighbors_vars.myDAGrank=0;
      return;
   }
   
   // reset my DAG rank to max value. May be lowered below.
   //neighbors_vars.myDAGrank  = MAXDAGRANK;
   
   // by default, I haven't found a preferred parent
   prefParentFound           = FALSE;
   prefParentIdx             = 0;
   
   //my current preferred parent
   open_addr_t pref_parent;
   neighbors_getPreferredParentEui64(&pref_parent);

   // loop through neighbor table, update myDAGrank
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         
         // reset parent preference
         neighbors_vars.neighbors[i].parentPreference = 0;
         
         // calculate link cost to this neighbor
         if (neighbors_vars.neighbors[i].numTx == 0) {
            rankIncrease = DEFAULTLINKCOST*MINHOPRANKINCREASE;
         } else {
            //6TiSCH minimal draft using OF0 for rank computation
#ifdef RPL_OF0
            rankIncrease = (uint16_t)
                  (
                        ((float)neighbors_vars.neighbors[i].numTx + 1)/((float)neighbors_vars.neighbors[i].numTxACK + 1)*
                        MINHOPRANKINCREASE
                  );
#endif

#ifdef RPL_OFFabrice
            rankIncrease = (uint16_t)
                  (
                        ((float)neighbors_vars.neighbors[i].numTx + 1)/((float)neighbors_vars.neighbors[i].numTxACK + 1)*
                        ((float)neighbors_vars.neighbors[i].numTx + 1)/((float)neighbors_vars.neighbors[i].numTxACK + 1)*
                        MINHOPRANKINCREASE
                  );
#endif
         }
         
         tentativeDAGrank = neighbors_vars.neighbors[i].DAGrank + rankIncrease;

#ifdef _DEBUG_ICMPv6RPL_RANK_
         char str[150];
          sprintf(str, "rank: neighbor ");
          openserial_ncat_uint8_t(str, (uint8_t)i, 150);
          strncat(str, ", PDR^-1= ", 150);
          openserial_ncat_uint8_t(str, (uint8_t)neighbors_vars.neighbors[i].numTx+1, 150);
          strncat(str, " / ", 150);
          openserial_ncat_uint8_t(str, (uint8_t)neighbors_vars.neighbors[i].numTxACK+1, 150);
          strncat(str, " - link metric ", 150);
          openserial_ncat_uint32_t(str, rankIncrease, 150);
          strncat(str, " - its rank ", 150);
          openserial_ncat_uint32_t(str, neighbors_vars.neighbors[i].DAGrank, 150);
          strncat(str, " - candidate rank  ", 150);
          openserial_ncat_uint32_t(str, tentativeDAGrank, 150);
          strncat(str, " - current best rank ", 150);
          openserial_ncat_uint32_t(str, neighbors_vars.neighbors[i].DAGrank, 150);
          openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));

#endif


         // prefered parent
         if (tentativeDAGrank < minRank && tentativeDAGrank<MAXDAGRANK){
            minRank = tentativeDAGrank;
            prefParentFound            = TRUE;
            prefParentIdx              = i;
         }
      }
   } 
   
   // update preferred parent
   if (prefParentFound) {
      neighbors_vars.neighbors[prefParentIdx].parentPreference       = MAXPREFERENCE;
      neighbors_vars.neighbors[prefParentIdx].stableNeighbor         = TRUE;
      neighbors_vars.neighbors[prefParentIdx].switchStabilityCounter = 0;
      #ifdef _DEBUG_NEIGHBORS_DETAIL_
         char strd[150];
         sprintf(strd, "NEIGH - New preferred parent:");
         openserial_ncat_uint32_t(strd, (uint32_t)prefParentIdx, 150);
         openserial_printf(COMPONENT_NEIGHBORS, strd, strlen(strd));
      #endif
   }

   //remove the old cells if the parent has changed
   if(!packetfunctions_sameAddress(&pref_parent, &(neighbors_vars.neighbors[prefParentIdx].addr_64b)))
      otf_notif_remove_parent(&pref_parent);
}


/**
\brief Update the node rank using the worst parent announcing the selected bottlenecks

**/
void neighbors_updateMyDAGrankWorst(){
   uint8_t i;
   dagrank_t max_rank;
   uint16_t rankIncrease;
   uint16_t tentativeDAGrank;

#ifdef _DEBUG_NEIGHBORS_
   char str[100];
   sprintf(str, "Update DAG rank");
   openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
#endif

   max_rank = 0;

   // loop through parent in parent set
   for (i=0; i<MAXNUMNEIGHBORS; i++){
      if (neighbors_vars.neighbors[i].inParentSet == TRUE){
         // compute offered rank
         rankIncrease = (uint16_t)
            (
             ((float)neighbors_vars.neighbors[i].numTx + 1)/((float)neighbors_vars.neighbors[i].numTxACK + 1)*
             MINHOPRANKINCREASE
            );
         tentativeDAGrank = neighbors_vars.neighbors[i].DAGrank + rankIncrease;

         // save parent offering highest rank
         if ( tentativeDAGrank > max_rank &&
               tentativeDAGrank<MAXDAGRANK) {
            max_rank = tentativeDAGrank;
         }
#ifdef _DEBUG_NEIGHBORS_
         char str[150];
         sprintf(str, "rank: neighbor ");
         openserial_ncat_uint32_t(str, (uint32_t)i, 150);
         strncat(str, ", PDR^-1= ", 150);
         openserial_ncat_uint32_t(str, (uint32_t)neighbors_vars.neighbors[i].numTx+1, 150);
         strncat(str, " / ", 150);
         openserial_ncat_uint32_t(str, (uint32_t)neighbors_vars.neighbors[i].numTxACK+1, 150);
         strncat(str, " - link metric ", 150);
         openserial_ncat_uint32_t(str, rankIncrease, 150);
         strncat(str, " - its rank ", 150);
         openserial_ncat_uint32_t(str, neighbors_vars.neighbors[i].DAGrank, 150);
         strncat(str, " - candidate rank  ", 150);
         openserial_ncat_uint32_t(str, tentativeDAGrank, 150);
         strncat(str, " - current max rank ", 150);
         openserial_ncat_uint32_t(str, max_rank, 150);
         openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
#endif
      }
   }

   // update my rank
   if (max_rank != 0){
      neighbors_vars.myDAGrank = max_rank;

      #ifdef _DEBUG_NEIGHBORS_DETAIL_
         char strd[150];
         sprintf(strd, "NEIGH - My new rank:");
         openserial_ncat_uint32_t(strd, (uint32_t)neighbors_vars.myDAGrank, 150);
         openserial_printf(COMPONENT_NEIGHBORS, strd, strlen(strd));
      #endif

   }
}

/**
\brief Update the bottlecks I advertise by saving only less constrained nodes

**/
void neighbors_updateMyBottlenecksSet(){
   uint8_t i,j,k;
   uint8_t max_counter, max_id;
   uint8_t empty_id; 
   bool btneck_exists;
   bool empty_found;

   // remove bottleneck that are not advertised by selected parents
   neighbors_removeUnadvertisedBtnecks();

   // For each used neighbor with lower rank, save less constrained bottlenecks
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if ( neighbors_vars.neighbors[i].DAGrank < neighbors_vars.myDAGrank
            && (neighbors_vars.neighbors[i].stableNeighbor)
         ){ 
         // for each neighbor btneck, insert, replace or update local btnecks
         for (j=0; j<MAX_NUM_BTNECKS; j++){
            if (neighbors_vars.neighbors[i].btnecks[j].counter != 0){
               // init
               btneck_exists = FALSE;
               empty_found = FALSE;
               empty_id = 0;
               max_id = 0;
               max_counter = 0;

               if (packetfunctions_sameAddress64(&neighbors_vars.neighbors[i].btnecks[j].addr_64b,
                     &(idmanager_getMyID(ADDR_64B)->addr_64b)
                  )){
                  char str[100];
                  sprintf(str, "ERR neighbor adversite myself");
                  openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
                  break;
               }

               // for each local btneck
               for (k=0; k<MAX_NUM_BTNECKS; k++){
                  if (neighbors_vars.btnecks[k].counter != 0){
                     // UPDATE existing bottleneck
                     if (packetfunctions_sameAddress64(&(neighbors_vars.btnecks[k].addr_64b),
                              &(neighbors_vars.neighbors[i].btnecks[j].addr_64b))){
                        neighbors_vars.btnecks[k].counter = neighbors_vars.neighbors[i].btnecks[j].counter;
                        btneck_exists = TRUE;
                        break;
                     } 
                     // save most contrained btneck 
                     if (neighbors_vars.btnecks[k].counter > max_counter){
                        max_counter = neighbors_vars.btnecks[k].counter;
                        max_id = k;
                     }
                  } else {
                     empty_found = TRUE;
                     empty_id = k; 
                  }
               }

               if (btneck_exists == FALSE){
                  if (empty_found){
                     // INSERT new bottleneck
                     memcpy(&neighbors_vars.btnecks[empty_id], &neighbors_vars.neighbors[i].btnecks[j], sizeof(btneck_t));
                  } else{                     
                     // REPLACE local max constrained bottleneck
                     if (neighbors_vars.neighbors[i].btnecks[j].counter < max_counter){
                        // copy bottleneck to local list
                        memcpy(&neighbors_vars.btnecks[max_id], &neighbors_vars.neighbors[i].btnecks[j], sizeof(btneck_t));
                        // -- note that the new bottleneck will inherit from previous traffic ratio
                     }
                  } 
               }    
            }
         }
      }
   }
}


// remove bottleneck that are too constrained
void neighbors_filterBtnecks(){
   uint8_t i;
   bool ratioAcceptable;

#ifdef _DEBUG_NEIGHBORS_
   char str[100];
   sprintf(str, "Filter Btneck");
   openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
#endif

   ratioAcceptable = FALSE;

   // Until traffic ratios are acceptable (>5%) for each bottleneck    
   while(ratioAcceptable == FALSE){
      // Calculate traffic ratio to send to each bottleneck
      neighbors_updateBalanceFactors();

      // Remove too coinstrained bottlenecks
      ratioAcceptable = TRUE;
      for (i=0; i<MAX_NUM_BTNECKS; i++){
         if (neighbors_vars.btnecks[i].counter != 0 && neighbors_vars.traffic_ratios[i] < 5){
            memset(&neighbors_vars.btnecks[i],0,sizeof(btneck_t));
            neighbors_vars.traffic_ratios[i] = 0;
            ratioAcceptable = FALSE;
            #ifdef _DEBUG_NEIGHBORS_DETAIL
               char strd[100];
               sprintf(strd, "NEIGH ----- Remove Btneck");
               openserial_printf(COMPONENT_NEIGHBORS, strd, strlen(strd));
            #endif
            break;
         }
      }
   }
}

/**
\brief Select the parents that advertise the btnecks I  have selected

**/
void  neighbors_updateMyParentsSet(){
   uint8_t i,j,k,l;
   bool btneck_found;

#ifdef _DEBUG_NEIGHBORS_
   char str[100];
   sprintf(str, "Update Parent Set");
   openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
#endif

   for (i=0; i<MAXNUMNEIGHBORS; i++){
      btneck_found = FALSE;
      for (j=0; j<MAX_NUM_BTNECKS; j++){
         if (neighbors_vars.neighbors[i].btnecks[j].counter != 0){
            for (k=0; k<MAX_NUM_BTNECKS; k++){
               if (neighbors_vars.btnecks[k].counter != 0){
                  if (packetfunctions_sameAddress64(&neighbors_vars.btnecks[k].addr_64b,
                           &neighbors_vars.neighbors[i].btnecks[j].addr_64b)
                     ){
                     btneck_found = TRUE;
                  }
               }
            }
         }
      }
      // If the parent doesn't annonce a selected btneck and is in the parent set, remove the parent
      if (!btneck_found && neighbors_vars.neighbors[i].inParentSet){
         // remove neighbor from parent set
         neighbors_vars.neighbors[i].inParentSet = FALSE;

         // remove existing tracks
         for (l=0; l<MAX_NUM_BTNECKS; l++){
            if (neighbors_vars.neighbors[i].btnecks[l].counter != 0){
               //sixtop_removeNeighborTracks(neighbors_vars.neighbors[i].btnecks[l].addr_64b, TRACK_BALANCING);
               //remove the old cells if the parent has changed
            }
         }
#ifdef _DEBUG_NEIGHBORS_
   char str[100];
   sprintf(str, "Remove Parent from Set");
   openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
#endif
      } else if (btneck_found && !neighbors_vars.neighbors[i].inParentSet){
#ifdef _DEBUG_NEIGHBORS_
   char str[100];
   sprintf(str, "Add Parent to Set");
   openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
#endif
         // add parent to set
         neighbors_vars.neighbors[i].inParentSet = TRUE;
      }
   }

/*
   // init parent set 
   for (i=0; i<MAXNUMNEIGHBORS; i++) {
      neighbors_vars.neighbors[i].inParentSet = FALSE;
   }

   // select parents that advertise the bottlenecks I have selected
   for (j=0;j <MAX_NUM_BTNECKS;j++){
      if (neighbors_vars.btnecks[j].counter != 0){
         for (k=0; k<MAXNUMNEIGHBORS; k++){
            if (neighbors_vars.neighbors[k].used==TRUE 
                  && neighbors_vars.neighbors[k].DAGrank < neighbors_vars.myDAGrank
                  && neighbors_vars.neighbors[k].inParentSet!=TRUE){
               for (l=0; l<MAX_NUM_BTNECKS; l++){
                  if (neighbors_vars.neighbors[k].btnecks[l].counter != 0){
                     if (packetfunctions_sameAddress64(
                              &(neighbors_vars.btnecks[j].addr_64b),
                              &(neighbors_vars.neighbors[k].btnecks[l].addr_64b))
                        ){
                        neighbors_vars.neighbors[k].inParentSet = TRUE;
                        #ifdef _DEBUG_NEIGHBORS_DETAIL_
                           char strd[100];
                           sprintf(strd, "NEIGH ----- Add parent in set");
                           openserial_printf(COMPONENT_NEIGHBORS, strd, strlen(strd));
         
                        #endif
                        break;
                     }
                  }
               }
            }
         }
      }
   }
*/
}

/**
\brief Compute the traffic ratio to send to each selected bottleneck
**/
void neighbors_updateBalanceFactors() {
   uint8_t i,j;
   uint16_t sum_counters;
   uint8_t btnecks_number;
   uint16_t optimal_value, diff_ratio, diff_counter;

   // init
   btnecks_number = 0;
   sum_counters = 0;

   // get the number of local btnecks and the sum of their counter
   for (i=0; i<MAX_NUM_BTNECKS; i++) {
      if (neighbors_vars.btnecks[i].counter != 0){
         sum_counters += (uint16_t)neighbors_vars.btnecks[i].counter;
         btnecks_number++;
      }
   }

   // get the balanced value objective
   optimal_value = (uint16_t)(sum_counters/(float)btnecks_number);

   // if counters different from objective, update balance factor
   for (j=0; j<MAX_NUM_BTNECKS; j++) {
      if (neighbors_vars.btnecks[j].counter != 0){
         diff_counter = optimal_value - neighbors_vars.btnecks[j].counter;
         if (diff_counter != 0){
            // compute ratio difference to optimal value
            diff_ratio = (uint16_t)(100 * diff_counter / (float)sum_counters);

            // if value not optimal +-5%, update ratio
            if (diff_ratio != 0){
               if ( (diff_ratio / 2) > 5 || (diff_ratio / 2) < -5) // don't change difference if < 5% (absolute value)
                  neighbors_vars.traffic_ratios[j] += diff_ratio / 2; // gradual increase
            }
         }
      }
   }
}

/**
\brief Reserve link with parents advertising selected tracks
**/
void neighbors_updateReservedTracks(){
   uint8_t i,j,k;
   track_t new_track;

#ifdef _DEBUG_NEIGHBORS_
   char str[100];
   sprintf(str, "Update reserved track");
   openserial_printf(COMPONENT_NEIGHBORS, str, strlen(str));
#endif

   // for each selected parent
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].inParentSet == TRUE){
         // for eacth of the selected parent's advertise bottleneck
         for (j=0; j<MAX_NUM_BTNECKS; j++){
            if(neighbors_vars.neighbors[i].btnecks[j].counter != 0){
               // for each of the bottleneck I advertise
               for (k=0; k<MAX_NUM_BTNECKS; k++){
                  if(neighbors_vars.btnecks[k].counter != 0){
                     // if this parent advertise a bottleneck I choose to advertise
                     if (packetfunctions_sameAddress64(
                              &neighbors_vars.neighbors[i].btnecks[j].addr_64b,
                              &neighbors_vars.btnecks[k].addr_64b)){
                        // create track
                        memcpy(&new_track.owner.addr_64b, &neighbors_vars.btnecks[k].addr_64b, LENGTH_ADDR64b);
                        new_track.owner.type = ADDR_64B;
                        new_track.instance = TRACK_BALANCING;

                        // if this parent does not already have a link with me for this track
                        if (schedule_getNbCellsWithTrackAndNeighbor(new_track,
                                 neighbors_vars.neighbors[i].addr_64b) == 0
                           ){
                           sixtop_addCells(&neighbors_vars.neighbors[i].addr_64b, TRACK_INIT_CELLS, new_track);
#ifdef _DEBUG_NEIGHBORS_DETAIL_PLU
   char strd[100];
   sprintf(strd, "test");
   openserial_ncat_uint8_t_hex(strd, (uint8_t)neighbors_vars.neighbors[i].addr_64b.addr_64b[6], 150);
   openserial_ncat_uint8_t_hex(strd, (uint8_t)neighbors_vars.neighbors[i].addr_64b.addr_64b[7], 150);
   openserial_printf(COMPONENT_NEIGHBORS, strd, strlen(strd));
#endif
                        }
                     }
                  }
               }
            }
         }
      }
   }

}

/**
\brief Remove bottlenecks that are not advertised by selected parents
**/
void neighbors_removeUnadvertisedBtnecks(){
   uint8_t i,j,k,l;
   uint8_t adv_counters[MAX_NUM_BTNECKS];
   
   // init adv counters
   memset(&adv_counters,0,sizeof(uint8_t)*MAX_NUM_BTNECKS);

   // for each neighbor's bottlenecks, if local btneck match, increase advertisement counter
   for (i=0;i<MAXNUMNEIGHBORS;i++){
      if (neighbors_vars.neighbors[i].used==TRUE
            && (neighbors_vars.neighbors[i].DAGrank < neighbors_vars.myDAGrank)
            && (neighbors_vars.neighbors[i].stableNeighbor)
         ){ 
         for (j=0; j<MAX_NUM_BTNECKS; j++){
            if (neighbors_vars.neighbors[i].btnecks[j].counter != 0){
               for (k=0; k<MAX_NUM_BTNECKS; k++){
                  if (neighbors_vars.btnecks[k].counter != 0){
                     if (packetfunctions_sameAddress64(&(neighbors_vars.btnecks[k].addr_64b),
                              &(neighbors_vars.neighbors[i].btnecks[j].addr_64b))){
                        adv_counters[k]++;
                     }
                  }
               }
            }
         }
      }
   }

   // If no parent advertise the bottleck remove it
   for (l=0; l<MAX_NUM_BTNECKS; l++){
      if (neighbors_vars.btnecks[l].counter != 0 && adv_counters == 0){
         memset(&neighbors_vars.btnecks[l],0,sizeof(btneck_t));
         neighbors_vars.traffic_ratios[l] = 0;
      }
   }
}

//===== maintenance

void  neighbors_removeOld() {
   uint8_t    i;
   uint16_t   timeSinceHeard;
   
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==1) {
         timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[i].asn);
         if (timeSinceHeard>DESYNCTIMEOUT) {
            removeNeighbor(i);
         }
      }
   } 
}

//===== debug

/**
\brief Triggers this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_neighbors() {
   debugNeighborEntry_t temp;
   neighbors_vars.debugRow=(neighbors_vars.debugRow+1)%MAXNUMNEIGHBORS;
   temp.row=neighbors_vars.debugRow;
   temp.neighborEntry=neighbors_vars.neighbors[neighbors_vars.debugRow];
   openserial_printStatus(STATUS_NEIGHBORS,(uint8_t*)&temp,sizeof(debugNeighborEntry_t));
   return TRUE;
}

//=========================== private =========================================

void registerNewNeighbor(open_addr_t* address,
                         int8_t       rssi,
                         asn_t*       asnTimestamp,
                         bool         joinPrioPresent,
                         uint8_t      joinPrio) {
   uint8_t  i,j;
   bool     iHaveAPreferedParent;
   // filter errors
   if (address->type!=ADDR_64B) {
      openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                            (errorparameter_t)address->type,
                            (errorparameter_t)2);
      return;
   }
   // add this neighbor
   if (isNeighbor(address)==FALSE) {
      i=0;
      while(i<MAXNUMNEIGHBORS) {
         if (neighbors_vars.neighbors[i].used==FALSE) {
            // add this neighbor
            neighbors_vars.neighbors[i].used                   = TRUE;
            neighbors_vars.neighbors[i].parentPreference       = 0;
            // neighbors_vars.neighbors[i].stableNeighbor         = FALSE;
            // Note: all new neighbors are consider stable
            neighbors_vars.neighbors[i].stableNeighbor         = TRUE;
            neighbors_vars.neighbors[i].switchStabilityCounter = 0;
            memcpy(&neighbors_vars.neighbors[i].addr_64b,address,sizeof(open_addr_t));
            neighbors_vars.neighbors[i].DAGrank                = DEFAULTDAGRANK;
            neighbors_vars.neighbors[i].rssi                   = rssi;
            neighbors_vars.neighbors[i].numRx                  = 1;
            neighbors_vars.neighbors[i].numTx                  = 0;
            neighbors_vars.neighbors[i].numTxACK               = 0;
            memcpy(&neighbors_vars.neighbors[i].asn,asnTimestamp,sizeof(asn_t));
            //update jp
            if (joinPrioPresent==TRUE){
               neighbors_vars.neighbors[i].joinPrio=joinPrio;
            }
            
            // do I already have a preferred parent ? -- TODO change to use JP
            iHaveAPreferedParent = FALSE;
            for (j=0;j<MAXNUMNEIGHBORS;j++) {
               if (neighbors_vars.neighbors[j].parentPreference==MAXPREFERENCE) {
                  iHaveAPreferedParent = TRUE;
               }
            }
            // if I have none, and I'm not DAGroot, the new neighbor is my preferred
            if (iHaveAPreferedParent==FALSE && idmanager_getIsDAGroot()==FALSE) {      
               neighbors_vars.neighbors[i].parentPreference     = MAXPREFERENCE;
            }
            break;
         }
         i++;
      }
      if (i==MAXNUMNEIGHBORS) {
         openserial_printError(COMPONENT_NEIGHBORS,ERR_NEIGHBORS_FULL,
                               (errorparameter_t)MAXNUMNEIGHBORS,
                               (errorparameter_t)0);
         return;
      }
   }
}

bool isNeighbor(open_addr_t* neighbor) {
   uint8_t i=0;
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (isThisRowMatching(neighbor,i)) {
         return TRUE;
      }
   }
   return FALSE;
}

void removeNeighbor(uint8_t neighborIndex) {
   neighbors_vars.neighbors[neighborIndex].used                      = FALSE;
   neighbors_vars.neighbors[neighborIndex].inParentSet               = FALSE;
   neighbors_vars.neighbors[neighborIndex].parentPreference          = 0;
   neighbors_vars.neighbors[neighborIndex].stableNeighbor            = FALSE;
   neighbors_vars.neighbors[neighborIndex].switchStabilityCounter    = 0;
   //neighbors_vars.neighbors[neighborIndex].addr_16b.type           = ADDR_NONE; // to save RAM
   neighbors_vars.neighbors[neighborIndex].addr_64b.type             = ADDR_NONE;
   //neighbors_vars.neighbors[neighborIndex].addr_128b.type          = ADDR_NONE; // to save RAM
   neighbors_vars.neighbors[neighborIndex].DAGrank                   = DEFAULTDAGRANK;
   neighbors_vars.neighbors[neighborIndex].rssi                      = 0;
   neighbors_vars.neighbors[neighborIndex].numRx                     = 0;
   neighbors_vars.neighbors[neighborIndex].numTx                     = 0;
   neighbors_vars.neighbors[neighborIndex].numTxACK                  = 0;
   neighbors_vars.neighbors[neighborIndex].asn.bytes0and1            = 0;
   neighbors_vars.neighbors[neighborIndex].asn.bytes2and3            = 0;
   neighbors_vars.neighbors[neighborIndex].asn.byte4                 = 0;
}

//=========================== helpers =========================================

bool isThisRowMatching(open_addr_t* address, uint8_t rowNumber) {
   switch (address->type) {
      case ADDR_64B:
         return neighbors_vars.neighbors[rowNumber].used &&
                packetfunctions_sameAddress(address,&neighbors_vars.neighbors[rowNumber].addr_64b);
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)3);
         return FALSE;
   }
}
