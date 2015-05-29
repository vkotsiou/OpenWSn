#ifndef __CEXAMPLE_H
#define __CEXAMPLE_H

/**
\addtogroup AppUdp
\{
\addtogroup cexample
\{
*/
#include "opencoap.h"


//=========================== define ==========================================

//=========================== typedef =========================================

typedef struct {
   coap_resource_desc_t desc;
   opentimer_id_t       timerId;
   track_t              track;
   uint16_t             seqnum;  //uniquely identifies this packet
} cexample_vars_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

void cexample_init(void);

/**
\}
\}
*/

#endif
