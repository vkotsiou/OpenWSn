/**
\brief Definition of the "openserial" driver.

\author Fabien Chraim <chraim@eecs.berkeley.edu>, March 2012.
*/

#include "opendefs.h"
#include "openserial.h"
#include "IEEE802154E.h"
#include "neighbors.h"
#include "sixtop.h"
#include "icmpv6echo.h"
#include "idmanager.h"
#include "openqueue.h"
#include "openbridge.h"
#include "leds.h"
#include "schedule.h"
#include "uart.h"
#include "opentimers.h"
#include "openhdlc.h"

//=========================== variables =======================================

openserial_vars_t openserial_vars;

//=========================== prototypes ======================================

owerror_t openserial_printInfoErrorCritical(
   char             severity,
   uint8_t          calling_component,
   uint8_t          error_code,
   errorparameter_t arg1,
   errorparameter_t arg2
);
// HDLC output
void outputHdlcOpen(void);
void outputHdlcWrite(uint8_t b);
void outputHdlcClose(void);
// HDLC stat output
void statOutputHdlcOpen(uint8_t i);
void statOutputHdlcWrite(uint8_t i, uint8_t b);
void statOutputHdlcClose(uint8_t i);
// HDLC input
void inputHdlcOpen(void);
void inputHdlcWrite(uint8_t b);
void inputHdlcClose(void);


//=========================== public ==========================================

void openserial_init() {
   uint16_t crc;
   uint8_t  i;
   
   // reset variable
   memset(&openserial_vars,0,sizeof(openserial_vars_t));
   
   // admin
   openserial_vars.mode                = MODE_OFF;
   openserial_vars.debugPrintCounter   = 0;
   
   // input
   openserial_vars.reqFrame[0]         = HDLC_FLAG;
   openserial_vars.reqFrame[1]         = SERFRAME_MOTE2PC_REQUEST;
   crc = HDLC_CRCINIT;
   crc = crcIteration(crc,openserial_vars.reqFrame[1]);
   crc = ~crc;
   openserial_vars.reqFrame[2]         = (crc>>0)&0xff;
   openserial_vars.reqFrame[3]         = (crc>>8)&0xff;
   openserial_vars.reqFrame[4]         = HDLC_FLAG;
   openserial_vars.reqFrameIdx         = 0;
   openserial_vars.lastRxByte          = HDLC_FLAG;
   openserial_vars.busyReceiving       = FALSE;
   openserial_vars.inputEscaping       = FALSE;
   openserial_vars.inputBufFill        = 0;
   
   // ouput
/*   openserial_vars.outputBufFilled     = FALSE;
   openserial_vars.outputBufIdxR       = 0;
   openserial_vars.outputBufIdxW       = 0;
  */
   //queue for stats
   openserial_vars.statOutputCurrentW  = 0;
   openserial_vars.statOutputCurrentR  = 0;
   for(i=0; i<OPENSERIAL_NBFRAMES; i++){
      openserial_vars.statOutputBufFilled[i] = FALSE;
      openserial_vars.statOutputBufIdxR[i]       = 0;
      openserial_vars.statOutputBufIdxW[i]       = 0;
   }

   // set callbacks
   uart_setCallbacks(isr_openserial_tx,
                     isr_openserial_rx);
}

/*
uint8_t  openserial_enough_space(uint8_t length){
   return(openserial_vars.outputBufIdxW - openserial_vars.outputBufIdxR + length < SERIAL_OUTPUT_BUFFER_SIZE);
}
*/

owerror_t openserial_printStatus(uint8_t statusElement,uint8_t* buffer, uint8_t length) {
   uint8_t i, pos;

   //find the first free position after the current position (if the current buffer is full)
   pos = openserial_vars.statOutputCurrentW;
   if (length + 12 + openserial_vars.statOutputBufIdxW[pos] - openserial_vars.statOutputBufIdxR[pos] > SERIAL_OUTPUT_BUFFER_SIZE){
      openserial_vars.statOutputCurrentW = (1 + openserial_vars.statOutputCurrentW) % OPENSERIAL_NBFRAMES;
      pos = openserial_vars.statOutputCurrentW;
   }
   openserial_vars.statOutputBufFilled[pos] = TRUE;


   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   statOutputHdlcOpen(pos);
   statOutputHdlcWrite(pos, SERFRAME_MOTE2PC_STATUS);
   statOutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   statOutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   statOutputHdlcWrite(pos, statusElement);
   for (i=0;i<length;i++){
      statOutputHdlcWrite(pos, buffer[i]);
   }
   statOutputHdlcClose(pos);
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

//a cell was inserted in the schedule
void openserial_celladd(scheduleEntry_t* slotContainer){

   #ifdef STATSERIAL

   evtCellAdd_t evt;
   evt.track_instance   = slotContainer->track.instance;
   memcpy(evt.track_owner, slotContainer->track.owner.addr_64b, 8);
   evt.slotOffset      = slotContainer->slotOffset;
   evt.type            = slotContainer->type;
   evt.shared          = slotContainer->shared;
   evt.channelOffset   = slotContainer->channelOffset;
   memcpy(evt.neighbor,      slotContainer->neighbor.addr_64b, 8);
   openserial_printStat(SERTYPE_CELL_ADD, COMPONENT_IEEE802154E, (uint8_t*)&evt, sizeof(evt));
   #endif
}

//a cell was removed in the schedule
void openserial_cellremove(scheduleEntry_t* slotContainer){

   #ifdef STATSERIAL

   evtCellRem_t evt;
   evt.track_instance   = slotContainer->track.instance;
   memcpy(evt.track_owner, slotContainer->track.owner.addr_64b, 8);
   evt.slotOffset      = slotContainer->slotOffset;
   evt.type            = slotContainer->type;
   evt.shared          = slotContainer->shared;
   evt.channelOffset   = slotContainer->channelOffset;
   memcpy(evt.neighbor,      slotContainer->neighbor.addr_64b, 8);
   openserial_printStat(SERTYPE_CELL_REMOVE, COMPONENT_IEEE802154E, (uint8_t*)&evt, sizeof(evt));

   #endif
}


//a ack was txed
void openserial_statAckTx(){

    #ifdef STATSERIAL

   #endif
}

//a ack was received
void openserial_statAckRx(){

    #ifdef STATSERIAL

   #endif
}

//push an event to track received frames
void openserial_statRx(OpenQueueEntry_t* msg){

   //stat for reception
   #ifdef STATSERIAL

      evtPktRx_t evt;
      evt.length           = msg->length;
      evt.rssi             = msg->l1_rssi;
      evt.lqi              = msg->l1_lqi;
      evt.crc              = msg->l1_crc;
      evt.track_instance   = msg->l2_track.instance;
      evt.frame_type       = msg->l2_frameType;
      memcpy(evt.track_owner, msg->l2_track.owner.addr_64b, 8);
      memcpy(evt.l2Src, msg->l2_nextORpreviousHop.addr_64b, 8);

      openserial_printStat(SERTYPE_PKT_RX, COMPONENT_IEEE802154E, (uint8_t*)&evt, sizeof(evt));
  #endif

}

//push an event to track transmitted frames
void openserial_statTx(OpenQueueEntry_t* msg){

   #ifdef STATSERIAL
      evtPktTx_t evt;
      evt.length           = msg->length;
      evt.txPower          = msg->l1_txPower;
      evt.track_instance   = msg->l2_track.instance;
      evt.numTxAttempts    = msg->l2_numTxAttempts;
      evt.l4_protocol      = msg->l4_protocol;
      evt.frame_type       = msg->l2_frameType;
      evt.l4_sourcePortORicmpv6Type = msg->l4_sourcePortORicmpv6Type;
      evt.l4_destination_port       = msg->l4_destination_port;

      memcpy(evt.track_owner, msg->l2_track.owner.addr_64b, 8);
      memcpy(evt.l2Dest,      msg->l2_nextORpreviousHop.addr_64b, 8);

     openserial_printStat(SERTYPE_PKT_TX, COMPONENT_IEEE802154E, (uint8_t*)&evt, sizeof(evt));
   #endif
}

//push an event to track generated frames
void openserial_statGen(uint16_t seqnum, track_t track){

   #ifdef STATSERIAL
      evtPktGen_t          dataGen;

      //info
      dataGen.seqnum          = seqnum ;
      dataGen.track_instance  = track.instance;
      memcpy(dataGen.track_owner, track.owner.addr_64b, 8);

      //memcpy(&(dataGen.track), &(cexample_vars.track), sizeof(cexample_vars.track));
      openserial_printStat(SERTYPE_DATA_GENERATION, COMPONENT_CEXAMPLE, (uint8_t*)&dataGen, sizeof(dataGen));
   #endif

}


owerror_t openserial_printStat(uint8_t type, uint8_t calling_component, uint8_t *buffer, uint8_t length) {
   uint8_t  asn[5];
   uint8_t  pos, i;


   INTERRUPT_DECLARATION();

   // retrieve ASN
   ieee154e_getAsn(asn);// byte01,byte23,byte4

   DISABLE_INTERRUPTS();

   //find the first free position after the current position (if the current buffer is full)
   pos = openserial_vars.statOutputCurrentW;
   if (length + 12 + openserial_vars.statOutputBufIdxW[pos] - openserial_vars.statOutputBufIdxR[pos] > SERIAL_OUTPUT_BUFFER_SIZE){
      openserial_vars.statOutputCurrentW = (1 + openserial_vars.statOutputCurrentW) % OPENSERIAL_NBFRAMES;
      pos = openserial_vars.statOutputCurrentW;
   }
   openserial_vars.statOutputBufFilled[pos] = TRUE;


   statOutputHdlcOpen(pos);
   statOutputHdlcWrite(pos, SERFRAME_MOTE2PC_STAT);
   statOutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   statOutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   statOutputHdlcWrite(pos, calling_component);
   statOutputHdlcWrite(pos, asn[0]);
   statOutputHdlcWrite(pos, asn[1]);
   statOutputHdlcWrite(pos, asn[2]);
   statOutputHdlcWrite(pos, asn[3]);
   statOutputHdlcWrite(pos, asn[4]);
   statOutputHdlcWrite(pos, type);
   for (i=0;i<length;i++){
      statOutputHdlcWrite(pos, buffer[i]);
   }
   statOutputHdlcClose(pos);
   ENABLE_INTERRUPTS();


   return E_SUCCESS;
}

owerror_t openserial_printInfoErrorCritical(
      char             severity,
      uint8_t          calling_component,
      uint8_t          error_code,
      errorparameter_t arg1,
      errorparameter_t arg2
   ) {
   uint8_t pos;

   //find the first free position after the current position (if the current buffer is full)
   pos = openserial_vars.statOutputCurrentW;
   if (12 + openserial_vars.statOutputBufIdxW[pos] - openserial_vars.statOutputBufIdxR[pos] > SERIAL_OUTPUT_BUFFER_SIZE){
      openserial_vars.statOutputCurrentW = (1 + openserial_vars.statOutputCurrentW) % OPENSERIAL_NBFRAMES;
      pos = openserial_vars.statOutputCurrentW;
   }
   openserial_vars.statOutputBufFilled[pos] = TRUE;


   INTERRUPT_DECLARATION();

   
   DISABLE_INTERRUPTS();
   statOutputHdlcOpen(pos);
   statOutputHdlcWrite(pos, severity);
   statOutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   statOutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   statOutputHdlcWrite(pos, calling_component);
   statOutputHdlcWrite(pos, error_code);
   statOutputHdlcWrite(pos, (uint8_t)((arg1 & 0xff00)>>8));
   statOutputHdlcWrite(pos, (uint8_t) (arg1 & 0x00ff));
   statOutputHdlcWrite(pos, (uint8_t)((arg2 & 0xff00)>>8));
   statOutputHdlcWrite(pos, (uint8_t) (arg2 & 0x00ff));
   statOutputHdlcClose(pos);
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printData(uint8_t* buffer, uint8_t length) {
   uint8_t  i;
   uint8_t  asn[5];

   uint8_t  pos;

   //find the first free position after the current position (if the current buffer is full)
   pos = openserial_vars.statOutputCurrentW;
   if (12 + openserial_vars.statOutputBufIdxW[pos] - openserial_vars.statOutputBufIdxR[pos] > SERIAL_OUTPUT_BUFFER_SIZE){
      openserial_vars.statOutputCurrentW = (1 + openserial_vars.statOutputCurrentW) % OPENSERIAL_NBFRAMES;
      pos = openserial_vars.statOutputCurrentW;
   }
   openserial_vars.statOutputBufFilled[pos] = TRUE;



   INTERRUPT_DECLARATION();
   
   // retrieve ASN
   ieee154e_getAsn(asn);// byte01,byte23,byte4
   
   DISABLE_INTERRUPTS();
   statOutputHdlcOpen(pos);
   statOutputHdlcWrite(pos, SERFRAME_MOTE2PC_DATA);
   statOutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   statOutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   statOutputHdlcWrite(pos, asn[0]);
   statOutputHdlcWrite(pos, asn[1]);
   statOutputHdlcWrite(pos, asn[2]);
   statOutputHdlcWrite(pos, asn[3]);
   statOutputHdlcWrite(pos, asn[4]);
   for (i=0;i<length;i++){
      statOutputHdlcWrite(pos, buffer[i]);
   }
   statOutputHdlcClose(pos);
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printInfo(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_INFO,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

owerror_t openserial_printError(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   // blink error LED, this is serious
   //leds_error_toggle();
   
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_ERROR,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

owerror_t openserial_printCritical(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   // blink error LED, this is serious
   leds_error_blink();
   
   // schedule for the mote to reboot in 10s
   opentimers_start(10000,
                    TIMER_ONESHOT,TIME_MS,
                    board_reset);
   
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_CRITICAL,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

uint8_t openserial_getNumDataBytes() {
   uint8_t inputBufFill;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();

   return inputBufFill-1; // removing the command byte
}

uint8_t openserial_getInputBuffer(uint8_t* bufferToWrite, uint8_t maxNumBytes) {
   uint8_t numBytesWritten;
   uint8_t inputBufFill;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();
   
   if (maxNumBytes<inputBufFill-1) {
      openserial_printError(COMPONENT_OPENSERIAL,ERR_GETDATA_ASKS_TOO_FEW_BYTES,
                            (errorparameter_t)maxNumBytes,
                            (errorparameter_t)inputBufFill-1);
      numBytesWritten = 0;
   } else {
      numBytesWritten = inputBufFill-1;
      memcpy(bufferToWrite,&(openserial_vars.inputBuf[1]),numBytesWritten);
   }
   
   return numBytesWritten;
}

void openserial_startInput() {
   INTERRUPT_DECLARATION();
   
   if (openserial_vars.inputBufFill>0) {
      openserial_printError(COMPONENT_OPENSERIAL,ERR_INPUTBUFFER_LENGTH,
                            (errorparameter_t)openserial_vars.inputBufFill,
                            (errorparameter_t)0);
      DISABLE_INTERRUPTS();
      openserial_vars.inputBufFill=0;
      ENABLE_INTERRUPTS();
   }
   
   uart_clearTxInterrupts();
   uart_clearRxInterrupts();      // clear possible pending interrupts
   uart_enableInterrupts();       // Enable USCI_A1 TX & RX interrupt
   
   DISABLE_INTERRUPTS();
   openserial_vars.busyReceiving  = FALSE;
   openserial_vars.mode           = MODE_INPUT;
   openserial_vars.reqFrameIdx    = 0;
#ifdef FASTSIM
   uart_writeBufferByLen_FASTSIM(
      openserial_vars.reqFrame,
      sizeof(openserial_vars.reqFrame)
   );
   openserial_vars.reqFrameIdx = sizeof(openserial_vars.reqFrame);
#else
   uart_writeByte(openserial_vars.reqFrame[openserial_vars.reqFrameIdx]);
#endif
   ENABLE_INTERRUPTS();
}

void openserial_startOutput() {
   //schedule a task to get new status in the output buffer
   uint8_t debugPrintCounter;
   
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   openserial_vars.debugPrintCounter = (openserial_vars.debugPrintCounter+1)%STATUS_MAX;
   debugPrintCounter = openserial_vars.debugPrintCounter;
   
   ENABLE_INTERRUPTS();
   // print debug information
   switch (debugPrintCounter) {
      case STATUS_ISSYNC:
         if (debugPrint_isSync()==TRUE) {
            break;
         }
      case STATUS_ID:
         if (debugPrint_id()==TRUE) {
            break;
         }
      case STATUS_DAGRANK:
         if (debugPrint_myDAGrank()==TRUE) {
            break;
         }
      case STATUS_OUTBUFFERINDEXES:
         if (debugPrint_outBufferIndexes()==TRUE) {
            break;
         }
      case STATUS_ASN:
         if (debugPrint_asn()==TRUE) {
            break;
         }
      case STATUS_MACSTATS:
         if (debugPrint_macStats()==TRUE) {
            break;
         }
      case STATUS_SCHEDULE:
         if(debugPrint_schedule()==TRUE) {
            break;
         }
      case STATUS_BACKOFF:
         if(debugPrint_backoff()==TRUE) {
            break;
         }
      case STATUS_QUEUE:
         if(debugPrint_queue()==TRUE) {
            break;
         }
      case STATUS_NEIGHBORS:
         if (debugPrint_neighbors()==TRUE) {
            break;
         }
      case STATUS_KAPERIOD:
         if (debugPrint_kaPeriod()==TRUE) {
            break;
         }
      default:
         DISABLE_INTERRUPTS();
         openserial_vars.debugPrintCounter=0;
         ENABLE_INTERRUPTS();
   }
   
   // flush buffer
   uart_clearTxInterrupts();
   uart_clearRxInterrupts();          // clear possible pending interrupts
   uart_enableInterrupts();           // Enable USCI_A1 TX & RX interrupt


   //STAT buffer
   uint8_t i = openserial_vars.statOutputCurrentR;

   //conflict OUTPUT / STAT
 /*  if(openserial_vars.outputBufFilled && openserial_vars.statOutputBufFilled[i]){
      openserial_printError(
            COMPONENT_OTF,
            ERR_GENERIC,
            (errorparameter_t)i,
            (errorparameter_t)openserial_vars.statOutputBufFilled[i]
      );
   }
*/
   DISABLE_INTERRUPTS();

   //STAT OUTPUT buffer
   if (openserial_vars.statOutputBufFilled[i]){

      openserial_vars.mode=MODE_STAT;

   #ifdef FASTSIM
      uart_writeCircularBuffer_FASTSIM(
         openserial_vars.statOutputBuf[i],
         &openserial_vars.statOutputBufIdxR[i],
         &openserial_vars.statOutputBufIdxW[i]
      );
   #else
      uart_writeByte(openserial_vars.statOutputBuf[i][openserial_vars.statOutputBufIdxR[i]++]);
   #endif
   }
/*   //OUTPUT buffer
   else if (openserial_vars.outputBufFilled){

      openserial_vars.mode=MODE_OUTPUT;
      if (openserial_vars.outputBufFilled) {
      #ifdef FASTSIM
         uart_writeCircularBuffer_FASTSIM(
               openserial_vars.outputBuf,
               &openserial_vars.outputBufIdxR,
               &openserial_vars.outputBufIdxW
         );
      #else
         uart_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
      #endif

      }
   }
*/
   else {
      openserial_stop();
   }

   ENABLE_INTERRUPTS();
}

void openserial_stop() {
   uint8_t inputBufFill;
   uint8_t cmdByte;
   bool busyReceiving;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   busyReceiving = openserial_vars.busyReceiving;
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();
   
   // disable USCI_A1 TX & RX interrupt
   uart_disableInterrupts();
   
   DISABLE_INTERRUPTS();
   openserial_vars.mode=MODE_OFF;
   ENABLE_INTERRUPTS();
   //the inputBuffer has to be reset if it is not reset where the data is read.
   //or the function openserial_getInputBuffer is called (which resets the buffer)
   if (busyReceiving==TRUE){
      openserial_printError(COMPONENT_OPENSERIAL,ERR_BUSY_RECEIVING,
                                  (errorparameter_t)0,
                                  (errorparameter_t)inputBufFill);
   }
   
   if (busyReceiving == FALSE && inputBufFill>0) {
      DISABLE_INTERRUPTS();
      cmdByte = openserial_vars.inputBuf[0];
      ENABLE_INTERRUPTS();
      switch (cmdByte) {
         case SERFRAME_PC2MOTE_SETROOT:
            idmanager_triggerAboutRoot();
            break;
         case SERFRAME_PC2MOTE_DATA:
            openbridge_triggerData();
            break;
         case SERFRAME_PC2MOTE_TRIGGERSERIALECHO:
            //echo function must reset input buffer after reading the data.
            openserial_echo(&openserial_vars.inputBuf[1],inputBufFill-1);
            break;   
         default:
            openserial_printError(COMPONENT_OPENSERIAL,ERR_UNSUPPORTED_COMMAND,
                                  (errorparameter_t)cmdByte,
                                  (errorparameter_t)0);
            //reset here as it is not being reset in any other callback
            DISABLE_INTERRUPTS();
            openserial_vars.inputBufFill = 0;
            ENABLE_INTERRUPTS();
            break;
      }
   }
   
   DISABLE_INTERRUPTS();
   openserial_vars.inputBufFill  = 0;
   openserial_vars.busyReceiving = FALSE;
   ENABLE_INTERRUPTS();
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_outBufferIndexes() {
   uint16_t temp_buffer[2];
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   temp_buffer[0] = openserial_vars.statOutputBufIdxW[openserial_vars.statOutputCurrentR];
   temp_buffer[1] = openserial_vars.statOutputBufIdxR[openserial_vars.statOutputCurrentR];
   ENABLE_INTERRUPTS();
   openserial_printStatus(STATUS_OUTBUFFERINDEXES,(uint8_t*)temp_buffer,sizeof(temp_buffer));
   return TRUE;
}

//=========================== private =========================================

//===== hdlc (stat output)

/**
\brief Start an HDLC frame in the output buffer.
*/
/*port_INLINE void outputHdlcOpen() {
   // initialize the value of the CRC
   openserial_vars.outputCrc                          = HDLC_CRCINIT;

   // write the opening HDLC flag
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]     = HDLC_FLAG;
}
*/
/**
\brief Add a byte to the outgoing HDLC frame being built.
*/
/*
port_INLINE void outputHdlcWrite(uint8_t b) {

    // iterate through CRC calculator
   openserial_vars.outputCrc = crcIteration(openserial_vars.outputCrc,b);

   // add byte to buffer
   if (b==HDLC_FLAG || b==HDLC_ESCAPE) {
      openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]  = HDLC_ESCAPE;
      b                                               = b^HDLC_ESCAPE_MASK;
   }
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]     = b;

}
*/
/**
\brief Finalize the outgoing HDLC frame.
*/
/*
port_INLINE void outputHdlcClose() {
   uint16_t   finalCrc;

   // finalize the calculation of the CRC
   finalCrc   = ~openserial_vars.outputCrc;

   // write the CRC value
   outputHdlcWrite((finalCrc>>0)&0xff);
   outputHdlcWrite((finalCrc>>8)&0xff);

   // write the closing HDLC flag
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]   = HDLC_FLAG;
}
*/

//===== hdlc (stat-output)

/**
\brief Start an HDLC frame in the output buffer.
*/
port_INLINE void statOutputHdlcOpen(uint8_t i) {
   // initialize the value of the CRC
   openserial_vars.statOutputCrc[i]                          = HDLC_CRCINIT;
   
   // write the opening HDLC flag
   openserial_vars.statOutputBuf[i][openserial_vars.statOutputBufIdxW[i]++]     = HDLC_FLAG;
}
/**
\brief Add a byte to the outgoing HDLC frame being built.
*/
port_INLINE void statOutputHdlcWrite(uint8_t i, uint8_t b) {
   
   // iterate through CRC calculator
   openserial_vars.statOutputCrc[i] = crcIteration(openserial_vars.statOutputCrc[i],b);
   
   // add byte to buffer
   if (b==HDLC_FLAG || b==HDLC_ESCAPE) {
      openserial_vars.statOutputBuf[i][openserial_vars.statOutputBufIdxW[i]++]  = HDLC_ESCAPE;
      b                                               = b^HDLC_ESCAPE_MASK;
   }
   openserial_vars.statOutputBuf[i][openserial_vars.statOutputBufIdxW[i]++]     = b;
   
}
/**
\brief Finalize the outgoing HDLC frame.
*/
port_INLINE void statOutputHdlcClose(uint8_t i) {
   uint16_t   finalCrc;
    
   // finalize the calculation of the CRC
   finalCrc   = ~openserial_vars.statOutputCrc[i];
   
   // write the CRC value
   statOutputHdlcWrite(i, (finalCrc>>0)&0xff);
   statOutputHdlcWrite(i, (finalCrc>>8)&0xff);
   
   // write the closing HDLC flag
   openserial_vars.statOutputBuf[i][openserial_vars.statOutputBufIdxW[i]++]   = HDLC_FLAG;
}


//===== hdlc (input)

/**
\brief Start an HDLC frame in the input buffer.
*/
port_INLINE void inputHdlcOpen() {
   // reset the input buffer index
   openserial_vars.inputBufFill                       = 0;
   
   // initialize the value of the CRC
   openserial_vars.inputCrc                           = HDLC_CRCINIT;
}
/**
\brief Add a byte to the incoming HDLC frame.
*/
port_INLINE void inputHdlcWrite(uint8_t b) {
   if (b==HDLC_ESCAPE) {
      openserial_vars.inputEscaping = TRUE;
   } else {
      if (openserial_vars.inputEscaping==TRUE) {
         b                             = b^HDLC_ESCAPE_MASK;
         openserial_vars.inputEscaping = FALSE;
      }
      
      // add byte to input buffer
      openserial_vars.inputBuf[openserial_vars.inputBufFill] = b;
      openserial_vars.inputBufFill++;
      
      // iterate through CRC calculator
      openserial_vars.inputCrc = crcIteration(openserial_vars.inputCrc,b);
   }
}
/**
\brief Finalize the incoming HDLC frame.
*/
port_INLINE void inputHdlcClose() {
   
   // verify the validity of the frame
   if (openserial_vars.inputCrc==HDLC_CRCGOOD) {
      // the CRC is correct
      
      // remove the CRC from the input buffer
      openserial_vars.inputBufFill    -= 2;
   } else {
      // the CRC is incorrect
      
      // drop the incoming fram
      openserial_vars.inputBufFill     = 0;
   }
}

//=========================== interrupt handlers ==============================

//executed in ISR, called from scheduler.c
void isr_openserial_tx() {
   uint8_t  i;
   switch (openserial_vars.mode) {
      case MODE_INPUT:
         openserial_vars.reqFrameIdx++;
         if (openserial_vars.reqFrameIdx<sizeof(openserial_vars.reqFrame)) {
            uart_writeByte(openserial_vars.reqFrame[openserial_vars.reqFrameIdx]);
         }
         break;
 /*     case MODE_OUTPUT:
         if (openserial_vars.outputBufIdxW==openserial_vars.outputBufIdxR) {
            openserial_vars.outputBufFilled = FALSE;
         }
         if (openserial_vars.outputBufFilled) {
            uart_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
         }
         break;
   */
      case MODE_STAT:
         i = openserial_vars.statOutputCurrentR;

         if (openserial_vars.statOutputBufIdxW[i]==openserial_vars.statOutputBufIdxR[i]) {
            openserial_vars.statOutputBufFilled[i] = FALSE;

            //considers the next buffer only if this one was entirely filled and the written one is the next one
            if (openserial_vars.statOutputCurrentW != openserial_vars.statOutputCurrentR)
               openserial_vars.statOutputCurrentR = (1 + openserial_vars.statOutputCurrentR) % OPENSERIAL_NBFRAMES;
           }
         if (openserial_vars.statOutputBufFilled[i]) {
            uart_writeByte(openserial_vars.statOutputBuf[i][openserial_vars.statOutputBufIdxR[i]++]);
         }
         break;
      case MODE_OFF:
      default:
         break;
   }
}

// executed in ISR, called from scheduler.c
void isr_openserial_rx() {
   uint8_t rxbyte;
   uint8_t inputBufFill;
   
   // stop if I'm not in input mode
   if (openserial_vars.mode!=MODE_INPUT) {
      return;
   }
   
   // read byte just received
   rxbyte = uart_readByte();
   //keep lenght
   inputBufFill=openserial_vars.inputBufFill;
   
   if        (
                openserial_vars.busyReceiving==FALSE  &&
                openserial_vars.lastRxByte==HDLC_FLAG &&
                rxbyte!=HDLC_FLAG
              ) {
      // start of frame
      
      // I'm now receiving
      openserial_vars.busyReceiving         = TRUE;
      
      // create the HDLC frame
      inputHdlcOpen();
      
      // add the byte just received
      inputHdlcWrite(rxbyte);
   } else if (
                openserial_vars.busyReceiving==TRUE   &&
                rxbyte!=HDLC_FLAG
             ) {
      // middle of frame
      
      // add the byte just received
      inputHdlcWrite(rxbyte);
      if (openserial_vars.inputBufFill+1>SERIAL_INPUT_BUFFER_SIZE){
         // input buffer overflow
         openserial_printError(COMPONENT_OPENSERIAL,ERR_INPUT_BUFFER_OVERFLOW,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         openserial_vars.inputBufFill       = 0;
         openserial_vars.busyReceiving      = FALSE;
         openserial_stop();
      }
   } else if (
                openserial_vars.busyReceiving==TRUE   &&
                rxbyte==HDLC_FLAG
              ) {
         // end of frame
         
         // finalize the HDLC frame
         inputHdlcClose();
         
         if (openserial_vars.inputBufFill==0){
            // invalid HDLC frame
            openserial_printError(COMPONENT_OPENSERIAL,ERR_WRONG_CRC_INPUT,
                                  (errorparameter_t)inputBufFill,
                                  (errorparameter_t)0);
         
         }
         
         openserial_vars.busyReceiving      = FALSE;
         openserial_stop();
   }
   
   openserial_vars.lastRxByte = rxbyte;
}

//======== SERIAL ECHO =============

void openserial_echo(uint8_t* buf, uint8_t bufLen){
   INTERRUPT_DECLARATION();
   // echo back what you received
   openserial_printData(
      buf,
      bufLen
   );
   
    DISABLE_INTERRUPTS();
    openserial_vars.inputBufFill = 0;
    ENABLE_INTERRUPTS();
}
