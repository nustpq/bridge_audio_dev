/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "ssc.h"
#include <utility/trace.h>

extern unsigned char DBGU_WriteBuffer(
    AT91S_DBGU *usart,
    void *buffer,
    unsigned int size) ;
//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Configures a SSC peripheral. If the divided clock is not used, the master
/// clock frequency can be set to 0.
/// \note The emitter and transmitter are disabled by this function.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param id  Peripheral ID of the SSC.
//------------------------------------------------------------------------------
void SSC_Configure(AT91S_SSC *ssc,
                          unsigned int id,
                          unsigned int bitRate,
                          unsigned int masterClock)
{
    // Enable SSC peripheral clock
    unsigned char i = 200;
    AT91C_BASE_PMC->PMC_PCDR = 1 << id;
    while(i--) ;
    AT91C_BASE_PMC->PMC_PCER = 1 << id;

    // Reset, disable receiver & transmitter
    ssc->SSC_CR = AT91C_SSC_RXDIS | AT91C_SSC_TXDIS | AT91C_SSC_SWRST;
    ssc->SSC_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;

    // Configure clock frequency
    if (bitRate != 0) {
    
        ssc->SSC_CMR = masterClock / (2 * bitRate);
    }
    else {

        ssc->SSC_CMR = 0;
    }
}

//------------------------------------------------------------------------------
/// Configures the transmitter of a SSC peripheral. Several macros can be used
/// to compute the values of the Transmit Clock Mode Register (TCMR) and the
/// Transmit Frame Mode Register (TFMR) (see "SSC configuration macros").
/// \param ssc  Pointer to a AT91S_SSC instance.
/// \param tcmr  Transmit Clock Mode Register value.
/// \param tfmr  Transmit Frame Mode Register value.
//------------------------------------------------------------------------------
void SSC_ConfigureTransmitter(AT91S_SSC *ssc,
                                     unsigned int tcmr,
                                     unsigned int tfmr)
{
    ssc->SSC_TCR = 0 ;
    ssc->SSC_TNCR = 0 ;
    ssc->SSC_TCMR = tcmr;
    ssc->SSC_TFMR = tfmr;
}

//------------------------------------------------------------------------------
/// Configures the receiver of a SSC peripheral. Several macros can be used
/// to compute the values of the Receive Clock Mode Register (RCMR) and the
/// Receive Frame Mode Register (RFMR) (see "SSC configuration macros").
/// \param ssc  Pointer to a AT91S_SSC instance.
/// \param rcmr  Receive Clock Mode Register value.
/// \param rfmr  Receive Frame Mode Register value.
//------------------------------------------------------------------------------
void SSC_ConfigureReceiver(AT91S_SSC *ssc,
                                  unsigned int rcmr,
                                  unsigned int rfmr)
{
    ssc->SSC_RCR = 0;
    ssc->SSC_RNCR = 0;
    ssc->SSC_RCMR = rcmr;
    ssc->SSC_RFMR = rfmr;
}

//------------------------------------------------------------------------------
/// Enables the transmitter of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
/*
inline void delay_1us(void)
{
    unsigned int i = 20 ;
    //while(i--) ;
    i++;i--;i++;i--;i++;

}
inline void delay_10us(void)
{
    unsigned int i = 20 ;
    i++;i--;i++;i--;i++;i--;i++;i--;i++;i--;
    i++;i--;i++;i--;i++;i--;i++;i--;i++;i--;
    i++;i--;i++;i--;i++;i--;i++;i--;i++;i--;
    i++;i--;i++;i--;i++;i--;i++;i--;i++;i--;
    i++;i--;i++;i--;i++;i--;i++;i--;i++;i--;
    i++;i--;i++;i--;i++;i--;i++;i--;i++;i--;
}
*/
extern const Pin fm_reset ;

void delay(unsigned int times)
{
    times +=30 ;
    
    while(times--) ;
}

void SSC_EnableTransmitter(AT91S_SSC *ssc)
{
    ssc->SSC_CR = AT91C_SSC_TXEN  ;   
}

void SSC_EnTrmt_rst(AT91S_SSC *ssc,unsigned int time_us)
{
    //PIO_Set(&fm_reset);
    while(PIO_Get(&fm_reset)) ;
    delay(time_us);
    ssc->SSC_CR = AT91C_SSC_TXEN  ;
    //PIO_Clear(&fm_reset);
}


void SSC_EnableTransmitterPDC(AT91S_SSC *ssc)
{
    ssc->SSC_PTCR = AT91C_PDC_TXTEN  ;
}
//------------------------------------------------------------------------------
/// Disables the transmitter of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
void SSC_DisableTransmitter(AT91S_SSC *ssc)
{
    ssc->SSC_CR = AT91C_SSC_TXDIS;
}

void SSC_DisableTransmitterPDC(AT91S_SSC *ssc)
{
    ssc->SSC_PTCR = AT91C_PDC_TXTDIS  ;
}


void SSC_InitTransmitter(AT91S_SSC *ssc)
{
    ssc->SSC_TCR = 0 ;
    ssc->SSC_TNCR = 0 ;
    ssc->SSC_TPR  = 0 ;
    ssc->SSC_TNPR = 0 ;
    //ssc->SSC_CR = AT91C_SSC_TXDIS;
}

//------------------------------------------------------------------------------
/// Enables the receiver of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
void SSC_EnableReceiver(AT91S_SSC *ssc)
{
    ssc->SSC_CR = AT91C_SSC_RXEN ;
}


void SSC_EnableReceiverPDC(AT91S_SSC *ssc)
{
    ssc->SSC_PTCR = AT91C_PDC_RXTEN ;
}

//------------------------------------------------------------------------------
/// Disables the receiver of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
void SSC_DisableReceiver(AT91S_SSC *ssc)
{ 
    ssc->SSC_CR = AT91C_SSC_RXDIS ;
}

void SSC_DisableReceiverPDC(AT91S_SSC *ssc)
{
    ssc->SSC_PTCR = AT91C_PDC_RXTDIS ;
}

void SSC_InitReceiver(AT91S_SSC *ssc)
{
    ssc->SSC_RCR = 0;
    ssc->SSC_RNCR = 0;  
    ssc->SSC_RPR  = 0 ;
    ssc->SSC_RNPR = 0 ;
    //ssc->SSC_CR = AT91C_SSC_RXDIS ;
}

//------------------------------------------------------------------------------
/// Enables one or more interrupt sources of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param sources  Interrupt sources to enable.
//------------------------------------------------------------------------------
void SSC_EnableInterrupts(AT91S_SSC *ssc, unsigned int sources)
{
    ssc->SSC_IER = sources;
}

//------------------------------------------------------------------------------
/// Disables one or more interrupt sources of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param sources  Interrupt source to disable.
//------------------------------------------------------------------------------
void SSC_DisableInterrupts(AT91S_SSC *ssc, unsigned int sources)
{
    ssc->SSC_IDR = sources;
}

//------------------------------------------------------------------------------
/// Sends one data frame through a SSC peripheral. If another frame is currently
/// being sent, this function waits for the previous transfer to complete.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param frame  Data frame to send.
//------------------------------------------------------------------------------
void SSC_Write(AT91S_SSC *ssc, unsigned int frame)
{
    while ((ssc->SSC_SR & AT91C_SSC_TXRDY) == 0);
    ssc->SSC_THR = frame;
}

//------------------------------------------------------------------------------
/// Sends the contents of a data buffer a SSC peripheral, using the PDC. Returns
/// true if the buffer has been queued for transmission; otherwise returns
/// false.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param buffer  Data buffer to send.
/// \param length  Size of the data buffer.
//unsigned int addr ;
//------------------------------------------------------------------------------
unsigned char SSC_WriteBuffer(AT91S_SSC *ssc,
                                     void *buffer,
                                     unsigned int length)
{
   
    if (ssc->SSC_TCR == 0) { // Check if first bank is free

        ssc->SSC_TPR = (unsigned int) buffer;
        ssc->SSC_TCR = length;
        //ssc->SSC_PTCR = AT91C_PDC_TXTEN;      
        return 1;
        
    }  else if (ssc->SSC_TNCR == 0) { // Check if second bank is free

        ssc->SSC_TNPR = (unsigned int) buffer;
        ssc->SSC_TNCR = length;
        return 2;
    }      
    // No free banks
    return 0;
}

//------------------------------------------------------------------------------
/// Waits until one frame is received on a SSC peripheral, and returns it.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
unsigned int SSC_Read(AT91S_SSC *ssc)
{
    while ((ssc->SSC_SR & AT91C_SSC_RXRDY) == 0);
    return ssc->SSC_RHR;
}

//------------------------------------------------------------------------------
/// Reads data coming from a SSC peripheral receiver and stores it into the
/// provided buffer. Returns true if the buffer has been queued for reception;
/// otherwise returns false.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param buffer  Data buffer used for reception.
/// \param length  Size in bytes of the data buffer.
//------------------------------------------------------------------------------
unsigned char SSC_ReadBuffer(AT91S_SSC *ssc,
                                    void *buffer,
                                    unsigned int length)
{
    
    if (ssc->SSC_RCR == 0) { // Check if the first bank is free

        ssc->SSC_RPR = (unsigned int) buffer;
        ssc->SSC_RCR = length;
        //ssc->SSC_PTCR = AT91C_PDC_RXTEN;
        //DBGU_WriteBuffer(AT91C_BASE_DBGU,"ReadOver\r\n", 12) ;
        return 1;
        
    } else if (ssc->SSC_RNCR == 0) { // Check if second bank is free

        ssc->SSC_RNPR = (unsigned int) buffer;
        ssc->SSC_RNCR = length;
        return 1;
    }

    // No free bank
    return 0;
}


/******************************************************************************/

TCMR tcmr ;
TFMR tfmr ;
RCMR rcmr ;
RFMR rfmr ;


void InitSSC( unsigned int mclk  ) 

{   
  
    SSC_Configure( 
                   BOARD_AT73C213_SSC,
                   BOARD_AT73C213_SSC_ID,
                   0,//Frat * ((period+1)*2), //slave not gen clk
                   mclk );      
    
    tcmr.cks    = 2 ;   // TK pin
    rcmr.cks    = 2 ;   // RK pin
    tcmr.cko    = 0 ;   // input only
    rcmr.cko    = 0 ;   // input only  
    
    tcmr.cki    = 0;//outcki;  //  0: falling egde send
    rcmr.cki    = 1;//incki;  //  1: rising edge lock  
    tcmr.start  = 4; //low_left == 0 ? 5 : 4  ;  //4: falling edge trigger for low left, 5: rising edge trigger for high left,
    rcmr.start  = 4; 
    
    tcmr.sttdly = 1;//one clock cycle delay
    rcmr.sttdly = 1;//
    tcmr.period = 0;// period ;  slave not use
    rcmr.period = 0;//  
    
    tcmr.ckg    = 0 ; //slave not use
    rcmr.ckg    = 0 ; // 
       
    tfmr.fsos   = 0 ; //input only
    rfmr.fsos   = 0 ; // 
    
    tfmr.datnb  = 1 ; //2 slot data
    rfmr.datnb  = 1 ; //   
    tfmr.datlen = 31 ; //32bits
    rfmr.datlen = 31 ;
    
    tfmr.fslen  = 0 ; //frame sync is not used
    rfmr.fslen  = 0 ; //frame sync is not used
       
    tfmr.fsedge = 1 ;
    rfmr.fsedge = 1 ;
          
    tfmr.msbf   = 1 ;
    rfmr.msbf   = 1 ;   

    tfmr.datdef = 0 ;
    tfmr.fsden  = 0 ;
    
    rfmr.loop   = 0 ;    
    
    SSC_ConfigureTransmitter( BOARD_AT73C213_SSC,  tcmr.value,  tfmr.value   );
    SSC_ConfigureReceiver(    BOARD_AT73C213_SSC,  rcmr.value,  rfmr.value   );
    
    //SSC_EnableTransmitter( BOARD_AT73C213_SSC ) ; 
    //SSC_EnableReceiver(    BOARD_AT73C213_SSC ) ;  
   
    
}

