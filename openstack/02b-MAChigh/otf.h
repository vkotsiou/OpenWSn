#ifndef __OTF_H
#define __OTF_H

/**
\addtogroup MAChigh
\{
\addtogroup otf
\{
*/

#include "opendefs.h"


//to possibly allocate soft cells as soon as a packet is enqueued
#define OTF_AGRESSIVE



//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== module variables ================================

//=========================== prototypes ======================================

// admin
void      otf_init(void);
// notification from sixtop
void      otf_notif_addedCell(void);
void      otf_notif_removedCell(void);

/**
\}
\}
*/

//a packet is pushed to the MAC layer -> OTF notification
void     otf_notif_transmit(OpenQueueEntry_t* msg);


typedef struct{

}otf_vars_t;

#endif
