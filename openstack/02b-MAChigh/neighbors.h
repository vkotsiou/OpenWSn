#ifndef __NEIGHBORS_H
#define __NEIGHBORS_H

/**
\addtogroup MAChigh
\{
\addtogroup Neighbors
\{
*/
#include "opendefs.h"
#include "icmpv6rpl.h"


//=========================== define ==========================================

//Objective function
#define RPL_OFFabrice
//#define   RPL_OF0

#define MAXNUMNEIGHBORS           15
#define MAXNUMPARENTS           	 3
#define MAXPREFERENCE             2
#define BADNEIGHBORMAXRSSI        -70 //dBm
#define GOODNEIGHBORMINRSSI       -80 //dBm
#define SWITCHSTABILITYTHRESHOLD  2
#define DEFAULTLINKCOST           15

#define MAXDAGRANK                0xffff
#define DEFAULTDAGRANK            MAXDAGRANK

#ifdef RPL_OF0
#define DEFAULTDAGRANK            1
#define MINHOPRANKINCREASE        256  //default value in RPL and Minimal 6TiSCH draft
#endif

#ifdef RPL_OFFabrice
#define MINHOPRANKINCREASE        64
#endif

//=========================== typedef =========================================

BEGIN_PACK
typedef struct {
   bool             used;
   uint8_t          parentPreference;
   bool             inParentSet;
   bool             stableNeighbor;
   uint8_t          switchStabilityCounter;
   open_addr_t      addr_64b;
   dagrank_t        DAGrank;
   int8_t           rssi;
   uint8_t          numRx;
   uint8_t          numTx;
   uint8_t          numTxACK;
   uint8_t          numWraps;//number of times the tx counter wraps. can be removed if memory is a restriction. also check openvisualizer then.
   asn_t            asn;
   uint8_t          joinPrio;
   btneck_t         btnecks[MAX_NUM_BTNECKS];
} neighborRow_t;
END_PACK

BEGIN_PACK
typedef struct {
   uint8_t         row;
   neighborRow_t   neighborEntry;
} debugNeighborEntry_t;
END_PACK

BEGIN_PACK
typedef struct {
   uint8_t         last_addr_byte;   // last byte of the neighbor's address
   int8_t          rssi;
   uint8_t         parentPreference;
   dagrank_t       DAGrank;
   uint16_t        asn; 
} netDebugNeigborEntry_t;
END_PACK

BEGIN_PACK
typedef struct {
   uint8_t         row;
   btneck_t        btneck;
   uint16_t        ratio;
} debugBtneckEntry_t;
END_PACK

//=========================== module variables ================================
   
typedef struct {
   neighborRow_t        neighbors[MAXNUMNEIGHBORS];
   btneck_t             btnecks[MAX_NUM_BTNECKS];
   float                balance_factors[MAX_NUM_BTNECKS]; // percentage of data sent to each btneck 
   dagrank_t            myDAGrank;
   uint8_t              debugRow;
   uint8_t              debugBtRow;
   uint8_t              dio_counter; // number of received DIO
   uint8_t              bootstrap_period; // number of recv DIO before calculating balance ratio
   uint8_t              DAGroot_id;
   icmpv6rpl_dio_ht*    dio; //keep it global to be able to debug correctly.
} neighbors_vars_t;

//=========================== prototypes ======================================

void          neighbors_init(void);

// getters
dagrank_t     neighbors_getMyDAGrank(void);
uint16_t      neighbors_getOfferedDAGrank(neighborRow_t neighbor);
uint8_t       neighbors_getNumNeighbors(void);
uint8_t       neighbors_getNumBtnecks(void);
uint16_t      neighbors_getNeighborMinBtneckCounter(uint8_t index);
uint16_t      neighbors_getMinBtneckCounter(void);
bool          neighbors_getPreferredTrack(open_addr_t* addressToWrite);
bool          neighbors_getPreferredTrackParent(open_addr_t track_owner, open_addr_t* addressToWrite);
bool          neighbors_getPreferredParentEui64(open_addr_t* addressToWrite);
open_addr_t*  neighbors_getKANeighbor(uint16_t kaPeriod);
void          neighbors_getNeighborID(open_addr_t addr_64b);
void          neighbors_getAdvBtnecks(btneck_t* btnecks);
void          neighbors_getWorstParent(neighborRow_t* worstParent);

// interrogators
bool          neighbors_isStableNeighbor(open_addr_t* address);
bool          neighbors_isPreferredParent(open_addr_t* address);
bool          neighbors_isNeighborWithLowerDAGrank(uint8_t index);
bool          neighbors_isNeighborWithHigherDAGrank(uint8_t index);
bool          neighbors_isDAGroot(uint8_t (*address_1)[LENGTH_ADDR64b]);

// updating neighbor information
void          neighbors_indicateRx(
   open_addr_t*         l2_src,
   int8_t               rssi,
   asn_t*               asnTimestamp,
   bool                 joinPrioPresent,
   uint8_t              joinPrio
);
void          neighbors_indicateTx(
   open_addr_t*         dest,
   uint8_t              numTxAttempts,
   bool                 was_finally_acked,
   asn_t*               asnTimestamp
);
void          neighbors_indicateRxDIO(OpenQueueEntry_t* msg);

// get addresses
void          neighbors_getNeighbor(open_addr_t* address,uint8_t addr_type,uint8_t index);
//returns the whole entry concerning a neighbor
neighborRow_t *neighbors_getNeighborInfo(open_addr_t* address);
// managing routing info
void          neighbors_updateMyDAGrankAndNeighborPreference(void);
void          neighbors_updateMyDAGrankWorst(void);
void          neighbors_updateMyBottlenecksSet(void);
void          neighbors_filterBtnecks(void);
void          neighbors_updateMyParentsSet(void);
void          neighbors_updateBalanceFactors(void);
void          neighbors_updateReservedTracks(void);
void          neighbors_removeUnadvertisedBtnecks(void);

// maintenance
void          neighbors_removeOld(void);
// debug
bool          debugPrint_neighbors(void);
bool          debugPrint_btnecks(void);

/**
\}
\}
*/

#endif
