#define _I2S_C

#include <board.h>
#include <pio/pio.h>
#include <stdbool.h>
#include <pio/pio_it.h>
#include <ssc/ssc.h>
#include <irq/irq.h>
#include <ioSyn.h>
#include <usb/device/audio-speaker/AUDDSpeakerDriver.h>
#include <utility/trace.h>
#include "app.h"


extern const Pin Test;
extern const Pin Test_rx;
//------------------------------------------------------------------------------
/// Handles interrupts coming from the SSC. Sends remaining audio buffers
/// or stops the DAC transmission.
//------------------------------------------------------------------------------
static void ISR_SSC(void)
{
    void *pt;
    unsigned int len ;
 
    if ( BOARD_AT73C213_SSC->SSC_PTSR & AT91C_PDC_TXTEN )  //SSC OUT
    {            
        if ( (BOARD_AT73C213_SSC->SSC_SR & AT91C_SSC_ENDTX) != 0 ) //The register SSC_TCR has reached 0 since the last write in SSC_TCR or SSC_TNCR
        {    
            if (numBuffersToSend > 0) 
            {    
                pt  = (void *)buffers[outBufferIndex];
                len = bufferSizes[outBufferIndex] ;
                SSC_WriteBuffer(BOARD_AT73C213_SSC,pt ,len);
                outBufferIndex = (outBufferIndex + 1) % BUFFER_NUMBER_TXD;
                numBuffersToSend--;             
            }
            else 
            {  
                BOARD_AT73C213_SSC->SSC_IDR = AT91C_SSC_ENDTX; //SSC_DisableInterrupts(BOARD_AT73C213_SSC, AT91C_SSC_ENDTX);
                printf("\n\rTX_BUF=0\n\r"); 
            } 
                  
        } 
        
        if( (BOARD_AT73C213_SSC->SSC_SR & AT91C_SSC_TXBUFE) != 0 ) //Both SSC_TCR and SSC_TNCR have a value of 0.
        {  
             
            BOARD_AT73C213_SSC->SSC_IDR  = AT91C_SSC_TXBUFE | AT91C_SSC_ENDTX; //SSC_DisableInterrupts(BOARD_AT73C213_SSC,AT91C_SSC_TXBUFE | AT91C_SSC_ENDTX);
            BOARD_AT73C213_SSC->SSC_PTCR = AT91C_PDC_TXTDIS ;
            
            play_start       = true ;   //set flag for re-buffer
            I2S_TxOn         = 1 ;      //set_I2S_TX() ;           
            outBufferIndex   = inBufferIndex ;   // in case of issue happened
            numBuffersToSend = 0 ;   //clear the residual data   
            printf("\n\rTX_BUF=E\n\r"); 
        }


    }
    
        
    if( BOARD_AT73C213_SSC->SSC_PTSR & AT91C_PDC_RXTEN ) //SSC IN
    {          
        if( ( BOARD_AT73C213_SSC->SSC_SR & AT91C_SSC_ENDRX) != 0 )//End of PDC transfer when Receive Counter Register has arrived at zero.
        {             
            if(numBuffersToSend_rcd < BUFFER_NUMBER_RXD)//tx buf not full 
            {               
                pt  = (void *)buffers_rcd[inBufferIndex_rcd] ;              
                len = (inBufferIndex_rcd % sample_total) == sample_last ? sample_length_long : sample_length_short ;               
                SSC_ReadBuffer(BOARD_AT73C213_SSC,pt,len) ; 
                bufferSizes_rcd[inBufferIndex_rcd] = len;   
            
                inBufferIndex_rcd = (inBufferIndex_rcd + 1) % BUFFER_NUMBER_RXD ;
                numBuffersToSend_rcd++ ;   
            }           
            else// tx buf is full, stop SSC data input
            {   
                 BOARD_AT73C213_SSC->SSC_IDR = AT91C_SSC_ENDRX; //SSC_DisableInterrupts(BOARD_AT73C213_SSC, AT91C_SSC_ENDRX);
                 //printf("\n\r#RXBUF=0"); 
            }

        }
        
        if( (BOARD_AT73C213_SSC->SSC_SR & AT91C_SSC_RXBUFF) != 0)//Both SSC_RCR and SSC_RNCR have a value of 0.
        {   
            BOARD_AT73C213_SSC->SSC_IDR  = AT91C_SSC_RXBUFF | AT91C_SSC_ENDRX; //SSC_DisableInterrupts(BOARD_AT73C213_SSC,AT91C_SSC_RXBUFF | AT91C_SSC_ENDRX) ;
            BOARD_AT73C213_SSC->SSC_PTCR = AT91C_PDC_RXTDIS ;            
            record_start = true ;
            I2S_RxOn    = 1 ; //Set_I2S_RX() ;//clear the residual data
            //printf("\n\rRXBUFF");  
           
        }
    }
    
    
    
}


void I2s_Init( unsigned int mclk )
{
    
    PIO_Configure(&SSC_Pins, 1);

    IRQ_DisableIT(BOARD_AT73C213_SSC_ID) ; 

    InitSSC( mclk ) ;
    
    IRQ_ConfigureIT(
                      BOARD_AT73C213_SSC_ID,
                      AT91C_AIC_SRCTYPE_POSITIVE_EDGE | SSC_PRIORITY,
                      ISR_SSC
                   );
    
    IRQ_EnableIT(BOARD_AT73C213_SSC_ID) ;
   
    buffers     = (BUF *) txbuf ;
    buffers_rcd = (BUF *) rxbuf ;
    
}

