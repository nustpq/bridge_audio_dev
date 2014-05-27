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
//      Headers
//------------------------------------------------------------------------------

#include "dbgu.h"
#include <stdarg.h>
#include <board.h>
            
//------------------------------------------------------------------------------
//      Exported functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Initializes the DBGU with the given parameters, and enables both the
/// transmitter and the receiver.
/// \param mode  Operating mode to configure (see <Modes>).
/// \param baudrate  Desired baudrate.
/// \param mck  Frequency of the system master clock.
//------------------------------------------------------------------------------
void DBGU_Configure(unsigned int mode,
                           unsigned int baudrate,
                           unsigned int mck)
{   
    // Reset & disable receiver and transmitter, disable interrupts
    AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RSTRX | AT91C_US_RSTTX;
    AT91C_BASE_DBGU->DBGU_IDR = 0xFFFFFFFF;
    
    // Configure baud rate
    AT91C_BASE_DBGU->DBGU_BRGR = mck / (baudrate * 16);
    
    // Configure mode register
    AT91C_BASE_DBGU->DBGU_MR = mode;
    
    // Disable DMA channel
    AT91C_BASE_DBGU->DBGU_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;

    // Enable receiver and transmitter
    AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RXEN | AT91C_US_TXEN;
}

//------------------------------------------------------------------------------
/// Outputs a character on the DBGU line.
/// \param c  Character to send.
//------------------------------------------------------------------------------
void DBGU_PutChar(unsigned char c)
{
    // Wait for the transmitter to be ready
    //while ((AT91C_BASE_DBGU->DBGU_CSR & AT91C_US_TXEMPTY) == 0);
    
    // Send character
    AT91C_BASE_DBGU->DBGU_THR = c;
    
    // Wait for the transfer to complete
    while ((AT91C_BASE_DBGU->DBGU_CSR & AT91C_US_TXEMPTY) == 0);
}

//------------------------------------------------------------------------------
/// Reads and returns a character from the DBGU.
//------------------------------------------------------------------------------
unsigned char DBGU_GetChar()
{
    while ((AT91C_BASE_DBGU->DBGU_CSR & AT91C_US_RXRDY) == 0);
    return AT91C_BASE_DBGU->DBGU_RHR;
}

unsigned char DBGU_WriteBuffer(
    AT91S_DBGU *usart,
    void *buffer,
    unsigned int size)
{
    /*
    // Check if the first PDC bank is free
    if ((usart->DBGU_TCR == 0) && (usart->DBGU_TNCR == 0)) {

        usart->DBGU_TPR = (unsigned int) buffer;
        usart->DBGU_TCR = size;
        usart->DBGU_PTCR = AT91C_PDC_TXTEN;

        return 1;
    }
    // Check if the second PDC bank is free
    else if (usart->DBGU_TNCR == 0) {

        usart->DBGU_TNPR = (unsigned int) buffer;
        usart->DBGU_TNCR = size;

        return 1;
    }
    else {

        return 0;
    }
   */
   return 1;
}

unsigned char DBGU_ReadBuffer(AT91S_DBGU *usart,
                                      void *buffer,
                                      unsigned int size)
{
    // Check if the first PDC bank is free
    if ((usart->DBGU_RCR == 0) && (usart->DBGU_RNCR == 0)) {

        usart->DBGU_RPR = (unsigned int) buffer;
        usart->DBGU_RCR = size;
        usart->DBGU_PTCR = AT91C_PDC_RXTEN;

        return 1;
    }
    // Check if the second PDC bank is free
    else if (usart->DBGU_RNCR == 0) {

        usart->DBGU_RNPR = (unsigned int) buffer;
        usart->DBGU_RNCR = size;

        return 1;
    }
    else {

        return 0;
    }
}

