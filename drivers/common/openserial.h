/**
\brief Declaration of the "openserial" driver.

\author Fabien Chraim <chraim@eecs.berkeley.edu>, March 2012.
*/

#ifndef __OPENSERIAL_H
#define __OPENSERIAL_H

#include "opendefs.h"

#include "schedule.h"

/**
\addtogroup drivers
\{
\addtogroup OpenSerial
\{
*/

//=========================== define ==========================================

/**
\brief Number of bytes of the serial output buffer, in bytes.

\warning should be exactly 256 so wrap-around on the index does not require
         the use of a slow modulo operator.
*/
#define SERIAL_OUTPUT_BUFFER_SIZE 256 // leave at 256!

/**
\brief Number of bytes of the serial input buffer, in bytes.

\warning Do not pick a number greater than 255, since its filling level is
         encoded by a single byte in the code.
*/
#define SERIAL_INPUT_BUFFER_SIZE  200

/// Modes of the openserial module.
enum {
   MODE_OFF    = 0,  ///< The module is off, no serial activity.
   MODE_INPUT  = 1,  ///< The serial is listening or receiving bytes.
   MODE_STAT   = 2   ///< The serial is transmitting bytes for statistics
};
//MODE_OUTPUT = 2,  ///< The serial is transmitting bytes.

// frames sent mote->PC
#define SERFRAME_MOTE2PC_DATA               ((uint8_t)'D')
#define SERFRAME_MOTE2PC_STATUS             ((uint8_t)'S')
#define SERFRAME_MOTE2PC_INFO               ((uint8_t)'I')
#define SERFRAME_MOTE2PC_ERROR              ((uint8_t)'E')
#define SERFRAME_MOTE2PC_CRITICAL           ((uint8_t)'C')
#define SERFRAME_MOTE2PC_REQUEST            ((uint8_t)'R')
#define SERFRAME_MOTE2PC_STAT               ((uint8_t)'T')

// frames sent PC->mote
#define SERFRAME_PC2MOTE_SETROOT            ((uint8_t)'R')
#define SERFRAME_PC2MOTE_DATA               ((uint8_t)'D')
#define SERFRAME_PC2MOTE_TRIGGERSERIALECHO  ((uint8_t)'S')


enum{
   SERTYPE_DATA_GENERATION    = 0x01,
   SERTYPE_PKT_TX             = 0x02,
   SERTYPE_PKT_RX             = 0x03,
   SERTYPE_CELL_ADD           = 0x04,
   SERTYPE_CELL_REMOVE        = 0x05,
   SERTYPE_ACK_TX             = 0x06,
   SERTYPE_ACK_RX             = 0x07
};
//=========================== typedef =========================================



BEGIN_PACK
typedef struct{
   uint16_t    track_instance;
   uint8_t     track_owner[8];
   uint8_t     slotOffset;
   uint8_t     type;
   uint8_t     shared;
   uint8_t     channelOffset;
   uint8_t     neighbor[8];
} evtCellAdd_t;
END_PACK

BEGIN_PACK
typedef struct{
   uint16_t    track_instance;
   uint8_t     track_owner[8];
   uint8_t     slotOffset;
   uint8_t     type;
   uint8_t     shared;
   uint8_t     channelOffset;
   uint8_t     neighbor[8];
} evtCellRem_t;
END_PACK

BEGIN_PACK
typedef struct{
   uint16_t    track_instance;
   uint8_t     track_owner[8];
   uint16_t    seqnum;
} evtPktGen_t;
END_PACK

BEGIN_PACK
typedef struct{
   uint16_t    track_instance;
   uint8_t     track_owner[8];
   uint8_t     length;
   uint8_t     frame_type;
   uint8_t     l2Dest[8];
   uint8_t     txPower;
   uint8_t     numTxAttempts;
   uint8_t     l4_protocol;
   uint16_t    l4_sourcePortORicmpv6Type;
   uint16_t    l4_destination_port;
} evtPktTx_t;
END_PACK

BEGIN_PACK
typedef struct{
   uint16_t    track_instance;
   uint8_t     track_owner[8];
   uint8_t     length;
   uint8_t     frame_type;
   uint8_t     l2Src[8];
   uint8_t     rssi;
   uint8_t     lqi;
   uint8_t     crc;
} evtPktRx_t;
END_PACK


#define  OPENSERIAL_NBFRAMES        4




//=========================== module variables ================================

typedef struct {
   // admin
   uint8_t    mode;
   uint8_t    debugPrintCounter;
   // input
   uint8_t    reqFrame[1+1+2+1]; // flag (1B), command (2B), CRC (2B), flag (1B)
   uint8_t    reqFrameIdx;
   uint8_t    lastRxByte;
   bool       busyReceiving;
   bool       inputEscaping;
   uint16_t   inputCrc;
   uint8_t    inputBufFill;
   uint8_t    inputBuf[SERIAL_INPUT_BUFFER_SIZE];
   // output
/*   bool       outputBufFilled;
   uint16_t   outputCrc;
   uint8_t    outputBufIdxW;
   uint8_t    outputBufIdxR;
   uint8_t    outputBuf[SERIAL_OUTPUT_BUFFER_SIZE];
 */  //statOutput
 //  uint8_t    statOutputNb;
   uint8_t    statOutputCurrentR;   //index to push to serial
   uint8_t    statOutputCurrentW;   //index to write our data
   bool       statOutputBufFilled[OPENSERIAL_NBFRAMES];
   uint16_t   statOutputCrc[OPENSERIAL_NBFRAMES];
   uint8_t    statOutputBufIdxW[OPENSERIAL_NBFRAMES];
   uint8_t    statOutputBufIdxR[OPENSERIAL_NBFRAMES];
   uint8_t    statOutputBuf[OPENSERIAL_NBFRAMES][SERIAL_OUTPUT_BUFFER_SIZE];
} openserial_vars_t;

//=========================== prototypes ======================================
//statistics to openvisualizer
void  openserial_celladd(scheduleEntry_t* slotContainer);
void  openserial_cellremove(scheduleEntry_t* slotContainer);
void  openserial_statAckTx(void);
void  openserial_statAckRx(void);
void  openserial_statRx(OpenQueueEntry_t* msg);
void  openserial_statTx(OpenQueueEntry_t* msg);
void  openserial_statGen(uint16_t seqnum, track_t track);



void    openserial_init(void);
owerror_t openserial_printStat(uint8_t type, uint8_t calling_component, uint8_t *buffer, uint8_t length);
owerror_t openserial_printStatus(uint8_t statusElement, uint8_t* buffer, uint8_t length);
owerror_t openserial_printInfo(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2);
owerror_t openserial_printError(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2);
owerror_t openserial_printCritical(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2);
owerror_t openserial_printData(uint8_t* buffer, uint8_t length);
uint8_t openserial_getNumDataBytes(void);
uint8_t openserial_getInputBuffer(uint8_t* bufferToWrite, uint8_t maxNumBytes);
void    openserial_startInput(void);
void    openserial_startOutput(void);
void    openserial_stop(void);
bool    debugPrint_outBufferIndexes(void);
void    openserial_echo(uint8_t* but, uint8_t bufLen);

// interrupt handlers
void    isr_openserial_rx(void);
void    isr_openserial_tx(void);

/**
\}
\}
*/

#endif
