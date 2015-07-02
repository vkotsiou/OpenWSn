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
#include <stdio.h>

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
void OutputHdlcOpen(uint8_t i);
void OutputHdlcWrite(uint8_t i, uint8_t b);
void OutputHdlcClose(uint8_t i);
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
   
   //queue for stats
   openserial_vars.OutputCurrentW  = 0;
   openserial_vars.OutputCurrentR  = 0;
   for(i=0; i<OPENSERIAL_OUTPUT_NBBUFFERS; i++){
      openserial_vars.OutputBufFilled[i] = FALSE;
      openserial_vars.OutputBufIdxR[i]       = 0;
      openserial_vars.OutputBufIdxW[i]       = 0;
   }

   // set callbacks
   uart_setCallbacks(isr_openserial_tx,
                     isr_openserial_rx);
}


//return the ouput buffer we should now use
uint8_t openserial_get_output_buffer(uint8_t length){
   uint8_t     pos, space;


   //find the first free position after the current position (if the current buffer is full)
   pos = openserial_vars.OutputCurrentW;

   //the R pointer MUST be before the W pointer (R=read by the other side of the serial line, W=currently written by our module)
   if (openserial_vars.OutputBufIdxW[pos] > openserial_vars.OutputBufIdxR[pos])
      space = openserial_vars.OutputBufIdxW[pos] - openserial_vars.OutputBufIdxR[pos];
   else
      space = (uint8_t)256 - openserial_vars.OutputBufIdxR[pos] + openserial_vars.OutputBufIdxW[pos];


   //do we have enough space? (pessimistic case: 30% of the chars are escaped)
   if (length * 1.7 + 2 + space  > SERIAL_OUTPUT_BUFFER_SIZE){

      //the next buffer is already pending -> not anymore space
      if (openserial_vars.OutputCurrentR == openserial_vars.OutputCurrentW + 1)
         return(-1);

      //else, get the next buffer in the cycle
      openserial_vars.OutputCurrentW = (1 + openserial_vars.OutputCurrentW) % OPENSERIAL_OUTPUT_NBBUFFERS;
      pos = openserial_vars.OutputCurrentW;
   }

   return(pos);
}


owerror_t openserial_printStat(uint8_t type, uint8_t calling_component, uint8_t *buffer, uint8_t length) {
#ifdef OPENSERIAL_STAT
   uint8_t  asn[5];
   uint8_t  pos, i;

   INTERRUPT_DECLARATION();

   // retrieve ASN
   ieee154e_getAsn(asn);// byte01,byte23,byte4

   DISABLE_INTERRUPTS();

   pos = openserial_get_output_buffer(length + 9);
   if (pos >= OPENSERIAL_OUTPUT_NBBUFFERS){
      //leds_error_toggle();
      return(E_FAIL);
   }
   openserial_vars.OutputBufFilled[pos] = TRUE;

   OutputHdlcOpen(pos);
   OutputHdlcWrite(pos, SERFRAME_MOTE2PC_STAT);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   OutputHdlcWrite(pos, calling_component);
   OutputHdlcWrite(pos, asn[0]);
   OutputHdlcWrite(pos, asn[1]);
   OutputHdlcWrite(pos, asn[2]);
   OutputHdlcWrite(pos, asn[3]);
   OutputHdlcWrite(pos, asn[4]);
   OutputHdlcWrite(pos, type);
   for (i=0;i<length;i++){
      OutputHdlcWrite(pos, buffer[i]);
   }
   OutputHdlcClose(pos);
   ENABLE_INTERRUPTS();

#endif

   return E_SUCCESS;
}

owerror_t openserial_printf(uint8_t calling_component, char* buffer, uint8_t length) {
#ifdef OPENSERIAL_PRINTF
   uint8_t  i, pos;
   uint8_t  asn[5];

   pos = openserial_get_output_buffer(length + 3);
   if (pos >= OPENSERIAL_OUTPUT_NBBUFFERS){
      //leds_error_toggle();
      return(E_FAIL);
   }
   openserial_vars.OutputBufFilled[pos] = TRUE;


   INTERRUPT_DECLARATION();

   ieee154e_getAsn(asn);// byte01,byte23,byte4

   DISABLE_INTERRUPTS();
   OutputHdlcOpen(pos);
   OutputHdlcWrite(pos, SERFRAME_MOTE2PC_PRINTF);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   OutputHdlcWrite(pos, calling_component);
   OutputHdlcWrite(pos, asn[0]);
   OutputHdlcWrite(pos, asn[1]);
   OutputHdlcWrite(pos, asn[2]);
   OutputHdlcWrite(pos, asn[3]);
   OutputHdlcWrite(pos, asn[4]);

   for (i=0;i<length;i++){
      OutputHdlcWrite(pos, buffer[i]);
   }
   OutputHdlcClose(pos);
   ENABLE_INTERRUPTS();

#endif
   return E_SUCCESS;
}



owerror_t openserial_printStatus(uint8_t statusElement,uint8_t* buffer, uint8_t length) {
   uint8_t i, pos;

   pos = openserial_get_output_buffer(length + 3);
   if (pos >= OPENSERIAL_OUTPUT_NBBUFFERS){
      //leds_error_toggle();
      return(E_FAIL);
   }
   openserial_vars.OutputBufFilled[pos] = TRUE;


   INTERRUPT_DECLARATION();

   DISABLE_INTERRUPTS();
   OutputHdlcOpen(pos);
   OutputHdlcWrite(pos, SERFRAME_MOTE2PC_STATUS);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   OutputHdlcWrite(pos, statusElement);
   for (i=0;i<length;i++){
      OutputHdlcWrite(pos, buffer[i]);
   }
   OutputHdlcClose(pos);
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

   pos = openserial_get_output_buffer(8);
   if (pos >= OPENSERIAL_OUTPUT_NBBUFFERS){
      //leds_error_toggle();
      return(E_FAIL);
   }
   openserial_vars.OutputBufFilled[pos] = TRUE;



   INTERRUPT_DECLARATION();

   
   DISABLE_INTERRUPTS();
   OutputHdlcOpen(pos);
   OutputHdlcWrite(pos, severity);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   OutputHdlcWrite(pos, calling_component);
   OutputHdlcWrite(pos, error_code);
   OutputHdlcWrite(pos, (uint8_t)((arg1 & 0xff00)>>8));
   OutputHdlcWrite(pos, (uint8_t) (arg1 & 0x00ff));
   OutputHdlcWrite(pos, (uint8_t)((arg2 & 0xff00)>>8));
   OutputHdlcWrite(pos, (uint8_t) (arg2 & 0x00ff));
   OutputHdlcClose(pos);
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printData(uint8_t* buffer, uint8_t length) {
   uint8_t  i;
   uint8_t  asn[5];
   uint8_t  pos;

   pos = openserial_get_output_buffer(length + 8);
   if (pos >= OPENSERIAL_OUTPUT_NBBUFFERS){
      //leds_error_toggle();
      return(E_FAIL);
   }
   openserial_vars.OutputBufFilled[pos] = TRUE;



   INTERRUPT_DECLARATION();
   
   // retrieve ASN
   ieee154e_getAsn(asn);// byte01,byte23,byte4
   
   DISABLE_INTERRUPTS();
   OutputHdlcOpen(pos);
   OutputHdlcWrite(pos, SERFRAME_MOTE2PC_DATA);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   OutputHdlcWrite(pos, idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   OutputHdlcWrite(pos, asn[0]);
   OutputHdlcWrite(pos, asn[1]);
   OutputHdlcWrite(pos, asn[2]);
   OutputHdlcWrite(pos, asn[3]);
   OutputHdlcWrite(pos, asn[4]);
   for (i=0;i<length;i++){
      OutputHdlcWrite(pos, buffer[i]);
   }
   OutputHdlcClose(pos);
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
   //leds_error_blink();
   
   // schedule for the mote to reboot in 2s
   opentimers_start(2000,
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
   uint8_t  debugPrintCounter;
   uint8_t  i;
   
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
         if (debugPrint_schedule()==TRUE) {
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
   i = openserial_vars.OutputCurrentR;


   DISABLE_INTERRUPTS();

   //STAT OUTPUT buffer
   if (openserial_vars.OutputBufFilled[i]){

      openserial_vars.mode=MODE_OUTPUT;

   #ifdef FASTSIM
      uart_writeCircularBuffer_FASTSIM(
         openserial_vars.OutputBuf[i],
         &openserial_vars.OutputBufIdxR[i],
         &openserial_vars.OutputBufIdxW[i]
      );
   #else
      uart_writeByte(openserial_vars.OutputBuf[i][openserial_vars.OutputBufIdxR[i]++]);
   #endif
   }

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
   uint16_t temp_buffer[4];
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   temp_buffer[0] = openserial_vars.OutputBufIdxW[openserial_vars.OutputCurrentR];
   temp_buffer[1] = openserial_vars.OutputBufIdxR[openserial_vars.OutputCurrentR];
   temp_buffer[2] = openserial_vars.OutputCurrentW;
   temp_buffer[3] = openserial_vars.OutputCurrentR;
   ENABLE_INTERRUPTS();
   openserial_printStatus(STATUS_OUTBUFFERINDEXES,(uint8_t*)temp_buffer, 8);
   return TRUE;
}

//=========================== private =========================================


//===== hdlc (stat-output)

/**
\brief Start an HDLC frame in the output buffer.
*/
port_INLINE void OutputHdlcOpen(uint8_t i) {
   // initialize the value of the CRC
   openserial_vars.OutputCrc[i]                          = HDLC_CRCINIT;
   
   // write the opening HDLC flag
   openserial_vars.OutputBuf[i][openserial_vars.OutputBufIdxW[i]++]     = HDLC_FLAG;
}
/**
\brief Add a byte to the outgoing HDLC frame being built.
*/
port_INLINE void OutputHdlcWrite(uint8_t i, uint8_t b) {
   
   if (openserial_vars.OutputBufIdxW[i] + 1 == openserial_vars.OutputBufIdxR[i]){
      openserial_printCritical(COMPONENT_OPENSERIAL, ERR_OPENSERIAL_BUFFER_OVERFLOW,
            2,
            1);
      return;
   }

   // iterate through CRC calculator
   openserial_vars.OutputCrc[i] = crcIteration(openserial_vars.OutputCrc[i],b);
   
   // add byte to buffer
   if (b==HDLC_FLAG || b==HDLC_ESCAPE) {
      openserial_vars.OutputBuf[i][openserial_vars.OutputBufIdxW[i]++]  = HDLC_ESCAPE;
      b                                               = b^HDLC_ESCAPE_MASK;
   }
   openserial_vars.OutputBuf[i][openserial_vars.OutputBufIdxW[i]++]     = b;
   
}
/**
\brief Finalize the outgoing HDLC frame.
*/
port_INLINE void OutputHdlcClose(uint8_t i) {
   uint16_t   finalCrc;
    
   // finalize the calculation of the CRC
   finalCrc   = ~openserial_vars.OutputCrc[i];
   
   // write the CRC value
   OutputHdlcWrite(i, (finalCrc>>0)&0xff);
   OutputHdlcWrite(i, (finalCrc>>8)&0xff);
   
   // write the closing HDLC flag
   openserial_vars.OutputBuf[i][openserial_vars.OutputBufIdxW[i]++]   = HDLC_FLAG;
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

      case MODE_OUTPUT:
         i = openserial_vars.OutputCurrentR;

         //that's the end of this buffer
         if (openserial_vars.OutputBufIdxW[i]==openserial_vars.OutputBufIdxR[i]) {
            openserial_vars.OutputBufFilled[i] = FALSE;

            //considers the next buffer only if this one was entirely filled and the written one is the next one
            if (openserial_vars.OutputCurrentW != openserial_vars.OutputCurrentR)
               openserial_vars.OutputCurrentR = (1 + openserial_vars.OutputCurrentR) % OPENSERIAL_OUTPUT_NBBUFFERS;

         }

         //we push the next byte
         else if (openserial_vars.OutputBufFilled[i]) {
            uart_writeByte(openserial_vars.OutputBuf[i][openserial_vars.OutputBufIdxR[i]++]);
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
         openserial_printCritical(COMPONENT_OPENSERIAL,ERR_INPUT_BUFFER_OVERFLOW,
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




//========= SERIAL FOR STATS ======





//a cell was inserted in the schedule
void openserial_statCelladd(scheduleEntry_t* slotContainer){

   #ifdef OPENSERIAL_STAT

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
void openserial_statCellremove(scheduleEntry_t* slotContainer){

   #ifdef OPENSERIAL_STAT

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

   #ifdef OPENSERIAL_STAT

   openserial_printStat(SERTYPE_ACK_TX, COMPONENT_IEEE802154E, NULL, 0);

   #endif
}

//a ack was received
void openserial_statAckRx(){

    #ifdef OPENSERIAL_STAT

   openserial_printStat(SERTYPE_ACK_RX, COMPONENT_IEEE802154E, NULL, 0);

   #endif
}

//push an event to track received frames
void openserial_statRx(OpenQueueEntry_t* msg){

   //stat for reception
   #ifdef OPENSERIAL_STAT

      evtPktRx_t evt;
      evt.length           = msg->length;
      evt.rssi             = msg->l1_rssi;
      evt.lqi              = msg->l1_lqi;
      evt.crc              = msg->l1_crc;
      evt.track_instance   = msg->l2_track.instance;
      evt.frame_type       = msg->l2_frameType;
      evt.slotOffset       = schedule_getSlotOffset();
      evt.frequency        = calculateFrequency(schedule_getChannelOffset(), schedule_getType());
      memcpy(evt.track_owner, msg->l2_track.owner.addr_64b, 8);
      memcpy(evt.l2Src, msg->l2_nextORpreviousHop.addr_64b, 8);

      openserial_printStat(SERTYPE_PKT_RX, COMPONENT_IEEE802154E, (uint8_t*)&evt, sizeof(evtPktRx_t));
  #endif
}

//push an event to track transmitted frames
void openserial_statTx(OpenQueueEntry_t* msg){

   #ifdef OPENSERIAL_STAT
      evtPktTx_t evt;
      evt.length           = msg->length;
      evt.txPower          = msg->l1_txPower;
      evt.track_instance   = msg->l2_track.instance;
      evt.numTxAttempts    = msg->l2_numTxAttempts;
      evt.l4_protocol      = msg->l4_protocol;
      evt.frame_type       = msg->l2_frameType;
      evt.slotOffset       = schedule_getSlotOffset();
      evt.frequency        = calculateFrequency(schedule_getChannelOffset(), schedule_getType());
      evt.l4_sourcePortORicmpv6Type = msg->l4_sourcePortORicmpv6Type;
      evt.l4_destination_port       = msg->l4_destination_port;

      memcpy(evt.track_owner, msg->l2_track.owner.addr_64b, 8);
      memcpy(evt.l2Dest,      msg->l2_nextORpreviousHop.addr_64b, 8);

     openserial_printStat(SERTYPE_PKT_TX, COMPONENT_IEEE802154E, (uint8_t*)&evt, sizeof(evtPktTx_t));
   #endif

}


//a ack was txed
void openserial_statPktTimeout(OpenQueueEntry_t* msg){

#ifdef OPENSERIAL_STAT
     evtPktTx_t evt;
     evt.length           = msg->length;
     evt.txPower          = msg->l1_txPower;
     evt.track_instance   = msg->l2_track.instance;
     evt.numTxAttempts    = msg->l2_numTxAttempts;
     evt.l4_protocol      = msg->l4_protocol;
     evt.frame_type       = msg->l2_frameType;
     evt.slotOffset       = schedule_getSlotOffset();
     evt.frequency        = calculateFrequency(schedule_getChannelOffset(), schedule_getType());
     evt.l4_sourcePortORicmpv6Type = msg->l4_sourcePortORicmpv6Type;
     evt.l4_destination_port       = msg->l4_destination_port;

     memcpy(evt.track_owner, msg->l2_track.owner.addr_64b, 8);
     memcpy(evt.l2Dest,      msg->l2_nextORpreviousHop.addr_64b, 8);

    openserial_printStat(SERTYPE_PKT_TIMEOUT, COMPONENT_IEEE802154E, (uint8_t*)&evt, sizeof(evtPktTx_t));
  #endif
}


//push an event to track an erroneous frame
void openserial_statPktError(OpenQueueEntry_t* msg){

   #ifdef OPENSERIAL_STAT
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

     openserial_printStat(SERTYPE_PKT_ERROR, COMPONENT_IEEE802154E, (uint8_t*)&evt, sizeof(evtPktTx_t));
   #endif
}


//push an event to track generated frames
void openserial_statGen(uint16_t seqnum, track_t track){

   #ifdef OPENSERIAL_STAT
      evtPktGen_t          dataGen;

      //info
      dataGen.seqnum          = seqnum ;
      dataGen.track_instance  = track.instance;
      memcpy(dataGen.track_owner, track.owner.addr_64b, 8);

      //memcpy(&(dataGen.track), &(cexample_vars.track), sizeof(cexample_vars.track));
      openserial_printStat(SERTYPE_DATA_GENERATION, COMPONENT_CEXAMPLE, (uint8_t*)&dataGen, sizeof(dataGen));
   #endif

}


//append a uint8_t at the end of a string
char *openserial_ncat_uint8_t(char *str, uint8_t val, uint8_t length){
   uint8_t l = strlen(str);

   if (l + 3 > length)
      return(str);

   uint8_t a, b, c;

   a = val / 100;
   b = (val - a * 100)/10;
   c = val - a * 100 - b * 10;

   str[l] = '0' + a;
   str[l+1] = '0' + b;
   str[l+2] = '0' + c;
   str[l+3] = '\0';
   return(str);
}


//append a uint32_t at the end of a string (without the non significant zeros)
char *openserial_ncat_uint32_t(char *str, uint32_t val, uint8_t length){
   uint8_t l = strlen(str);

   if (l + 10 > length) //at most 10 digits
      return(str);

   uint8_t  digit, shift, i;
   uint32_t power;
   bool     nonzero = FALSE;


   power = 1000000000;
   shift = 0;           // to avoid non significant zeros
   for(i=0; i<10; i++){
      digit = val / power;
      if (digit != 0 || i == 9 || nonzero){
         nonzero = TRUE;
         str[l + shift] = '0' + digit;
         shift++;
      }
      val = val - power * digit;
      power = power / 10;
   }
   str[l+shift] = '\0';


   return(str);
}




//append a uint32_t at the end of a string (without the non significant zeros)
char *openserial_ncat_uint8_t_hex(char *str, uint8_t val, uint8_t length){
   uint8_t  l = strlen(str);
   uint8_t  c, shift;

   if (l + 2 > length) //at most 2 digits
      return(str);


   shift = 0;

   //first digit
   c = (val & 0xf0)  >> 4;


   if (c < 10)
      str[l+shift] = '0' + c;
   else if (c < 16)
      str[l+shift] = (uint8_t)'a' + c  - 10;
   else
      str[l+shift] = 'z';
   shift++;


   //second digit
   c = val & 0x0f;
   if (c < 10)
      str[l+shift] = '0' + c;
   else if (c < 16)
      str[l+shift] = 'a' + c  - 9;
   else
      str[l+shift] = 'z';
   shift++;


   str[l+shift] = '\0';

   return(str);
}


