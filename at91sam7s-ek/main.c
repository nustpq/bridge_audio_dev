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
 
 /*
 *------------------------------------------------------------------------------
 *  Modification history:
 *
 ------------- Saturn II Audio  ------------------------------------------------
 * Planted to Saturn II platform          
 * Support bit-to-bit synchronous on two audio device
 * Enumeration succeed on Win7 , but still some issue on play/record if using Noah.
 *                                Aug.30th,2011 by PQ -- ""SaturnII Audio V1.3""
 *
 * 44.1kHz P&R supported on Win XP.
 *                                Sep.7th,2011 by PQ -- ""SaturnII Audio V1.4""

 * 22.05kHz P&R supported on Win XP.
 *                                Sep.13th,2011 by PQ -- ""SaturnII Audio V1.5""
 *
 * Fixed UART bugs that caused uart communication unstable bugs
 *                                Dec.30th,2011 by PQ -- ""SaturnII Audio V1.6""
 *
 * Changed USB int priority defines
 *                                Jan.11th,2012 by PQ -- ""SaturnII Audio V1.7""
 *
 * Changed SSC_PIN to Float before Install
 *                                Feb.21th,2012 by PQ -- ""SaturnII Audio V1.8""
 *
 * Changed SSC_PIN to Float before Install
 *                                Mar.3th,2012 by PQ -- "SaturnII Audio V2.0"
 * ----------------------------------------------------------------------------

 * Plated from SaturnII Audio V2.0
 *                                Mar.9th,2012 by PQ -- "J-III Audio V1.0"

 * mofified the the int priority and others for stability
 *                                Aug.8th,2012 by PQ -- "J-III Audio V1.2"

 * fast LED operation in ISR.    Aug.8th,2012 by PQ -- "J-III Audio V1.3"

 * shorten SSC ISR time cost"void ISR_SSC(void)".
                                 Aug.9th,2012 by PQ -- "J-III Audio V1.4"
 
 * changed PID, VID, USB2 to USB1.1 and feature selector unit in configurationDescriptors".
                                 Aug.9th,2012 by PQ -- "J-III Audio V1.5"
 
*  added FCLK sync in EnableSscSync() and EnableSscSync_rx().
*  Optimize sync mechanime
                                 Aug.9th,2012 by PQ -- "J-III Audio V1.6"

*  remove debug print to save time "AUDDSpeakerDriver_RequestHandler: Unsupported    entity/interface"
*  modified the void PIO_ConfigureIt() for ASSERT issue
*  improve UART_PRIORITY to highest
                                 Oct.23th,2012 by PQ -- "J-III Audio V1.7"

*  Add timeout in "while ( USBD_GetState() < USBD_STATE_CONFIGURED )" for INSTALL_TIME_OUT, in case of dead loop
*  improved the bug in function : delay_ms()
                                 Oct.24th,2012 by PQ -- "J-III Audio V1.8"

*  Use Thumb mode
   changed var " auddSpeakerDriverInterfaces " define in case of address error
                                  Oct.24th,2012 by PQ -- "J-III Audio V1.9"

////////////////////////////////////////////////////////////////////////////////
                                      
*  port for Audio on J-III 3.1 board
   Audio ID identifier changed : Check_Audio_ID()
                                  Nov.14th,2012 by PQ -- "J-III Audio V2.0"
                                      
*  Fixed 6dB gain issue and L/R revert issue 
*  by changed PDC length from 2bytes to 4bytes each : tfmr.datlen = 31
*  used __ramfunc for IO ISR:   void IO_synISR_Rec_RL()
                                  Jan.18th,2013 by PQ -- "J-III Audio V2.1"

*  Fixed MSB issue by changing PDC length from 2bytes to 4bytes each : tfmr.datlen = 31
*  uadjusted IO ISR:  
*  no problem
                                  Mar.4th,2013 by PQ -- "J-III Audio V2.2"

////////////////////////////////////////////////////////////////////////////////
///////////////               Bridge Audio Dev      ////////////////////////////   

* Used on AB01 for ACQUA Test
* Due to MCU MIPS 48MHz limitaion, and BCLK = 32FCLK for SSC, only <= 16k case is supported
* Here the SR is fixed to 16000.
*                                Mar.3th,2014 by PQ -- [FW:B:V0.1]

*/

//------------------------------------------------------------------------------ 
//         Headers
//------------------------------------------------------------------------------
#include <board.h>
#include <pio/pio.h>
#include <irq/irq.h>
#include <usart/usart.h>
#include <dbgu/dbgu.h>
#include <ioSyn.h>
#include <stdbool.h>
#include <app.h>
#include <utility/led.h>
#include <usb/device/core/USBD.h>
#include <tc/tc.h>
#include <noah.h>
#include <utility/trace.h>




void Head_Info( void )
{
  
    printf("\r\n\r\n");
    printf("---    Bridge Audio Dev Project     -----\r\n");    
    printf("------ HW version: %s --\n\r", BOARD_NAME);
    printf("------ SW version: %s --\n\r", fw_version); 
    printf("------ Board Name    : %s              -----\r\n", BOARD_NAME);
    printf("------ Compiled Time : %s %s by PQ -----\r\n", __DATE__, __TIME__);
    printf("------ TRACE_LEVEL = %d       ------\r\n",TRACE_LEVEL);  
    printf("------ Sample rate is fixed to [%2d] kHz   ------\r\n",FramRat);
           
}



int main(void)
{          
    IO_synInit() ;
    AT91_PIT_Init();
    //Check_Audio_ID();
    Uart_Init();     
    USART_Write(AT91C_BASE_US0, 0, 0); //send ACK for last reset cmd 
    
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);  
    Head_Info();
        
    LED_Configure(USBD_LEDPOWER);
    LED_Configure(USBD_LEDUDATA);    
    LED_Set(USBD_LEDPOWER); 
    LED_Set(USBD_LEDUDATA);

    delay_ms(2000);  

    printf("\r\nStart main loop...^_<");  
    
    I2s_Init( MCK );  
       
    Usb_Init() ;                
    while ( USBD_GetState() < USBD_STATE_CONFIGURED ) ; 
   
    I2S_RxOn   = 1 ;
    I2S_TxOn   = 1 ; 
    I2S_On     = 1 ; 
  
    
    while(1) {     
              
          //Audio_State_Control();
          InitI2S_Task() ;
          Debug_Info();   

       
    }//while(1)
    
    
    
    
}



