#define _USB_C

#include <pio/pio.h>
#include <stdbool.h>
#include <pio/pio_it.h>
#include <usart/usart.h>
#include <dbgu/dbgu.h>
#include <irq/irq.h>
#include <ssc/ssc.h>
#include <ioSyn.h>
#include <board.h>
#include <utility/led.h>
#include <usb/device/audio-speaker/AUDDSpeakerDriver.h>
#include <usb/device/audio-speaker/AUDDSpeakerChannel.h>
#include "app.h"
#include <usb/device/audio-speaker/AUDDSpeakerDriverDescriptors.h>
#include <utility/trace.h>

extern const Pin Test ;
extern const Pin RunLed ;
extern void SetBratInDescriptor(unsigned int ) ;

static const Pin pinVbus = PIN_USB_VBUS;

 

//------------------------------------------------------------------------------
/// Handles interrupts coming from PIO controllers.
//------------------------------------------------------------------------------
static void ISR_Vbus( void )
{
    //TRACE_INFO(trace_INFO, "VBUS ");

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        //TRACE_INFO(trace_INFO, "conn\n\r");
        USBD_Connect();
    }
    else {

        //TRACE_INFO(trace_INFO, "discon\n\r");
        USBD_Disconnect();
    }
}

void VBus_Configure( void )
{
    //TRACE_INFO(trace_INFO, "VBus configuration\n\r");

    // Configure PIO
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {
      
        // if VBUS present, force the connect
        //TRACE_INFO(trace_INFO, "conn\n\r");
        USBD_Connect();
    }
    else {
      
        USBD_Disconnect();
    }           
}

void AUDDSpeakerChannel_MuteChanged(AUDDSpeakerChannel *channel, unsigned char muted)
{
    // Master channel
    if (AUDDSpeakerChannel_GetNumber(channel) == AUDDSpeakerDriver_MASTERCHANNEL) {

        if (muted) {

            //TRACE_INFO(trace_INFO, "MuteMaster ");
            //AT73C213_SetMuteStatus(1, 1);
        }
        else {

            //TRACE_INFO(trace_INFO, "UnmuteMaster ");
            //AT73C213_SetMuteStatus(0, 0);
        }
    }
}



//unsigned char flag_a = 0;
//------------------------------------------------------------------------------
/// Invoked when a frame has been received.    
/// PC ==> MCU
//------------------------------------------------------------------------------
void FrameReceived(     unsigned int unused,
                        char status,
                        unsigned int transferred,
                        unsigned int remaining)
{
    //Play
    if ( status == USBD_STATUS_SUCCESS ) {         
        
        test_buf_id = 1 - test_buf_id ;
        bufferSizes[inBufferIndex] = transferred / AUDDSpeakerDriver_BYTESPERSAMPLE; 

        if ( numBuffersToSend < BUFFER_NUMBER_TXD ) { //enouth free buffer 
            AUDDSpeakerDriver_Read( test_buf[test_buf_id],//buffers[inBufferIndex],
                                    sample_length_long * AUDDSpeakerDriver_BYTESPERSAMPLE ,  //AUDDSpeakerDriver_BYTESPERFRAME + 4,
                                    (TransferCallback) FrameReceived,
                                    0); // No optional argument
        } else {  //usb out too fast              
            TRACE_WARNING("\n\rERROR: Play too fast, buffer is full!\n\r");  
        }        
        total_received += transferred ;     
     
    }  else {      
        TRACE_WARNING( "\n\rUsbDataReceived: Transfer error\n\r" );        
    }
   

    //pre-process data: double 16bit data to 32bit data for CODEC AD1937
    unsigned short  *pShort;
    unsigned int    *pInt, temp;  

    pShort = (unsigned short *)&test_buf[1-test_buf_id];
    pInt   = (unsigned int   *)&buffers[inBufferIndex];
    
    for( unsigned int i=0; i < AUDDSpeakerDriver_SAMPLESPERFRAME ; i++ ) {
        temp    = *pShort++ ;
        *pInt++ = temp + (temp<<16);
    }         
    // memset((unsigned char *)buffers[inBufferIndex],0x50,transferred);  //Debug use
    inBufferIndex = (inBufferIndex + 1) % BUFFER_NUMBER_TXD;
    numBuffersToSend++;


    
    if ( play_start  )  {
      
        if( numBuffersToSend >= PLAY_PRE_BUF_NUMBER ) {  //wait until received enough buffering data, then start SSC output...        
            play_start = false;        
           
            SSC_WriteBuffer(    BOARD_AT73C213_SSC,
                                buffers[outBufferIndex],
                                bufferSizes[outBufferIndex]); 
            //
            outBufferIndex = (outBufferIndex + 1) % BUFFER_NUMBER_TXD;
            numBuffersToSend--;
       
            SSC_WriteBuffer(    BOARD_AT73C213_SSC,
                                buffers[outBufferIndex],
                                bufferSizes[outBufferIndex]);
            //
            outBufferIndex = (outBufferIndex + 1) % BUFFER_NUMBER_TXD;
            numBuffersToSend--;      
            
            SSC_EnableTransmitter(BOARD_AT73C213_SSC);
            SSC_EnableTransmitterPDC(BOARD_AT73C213_SSC);
            SSC_EnableInterrupts(BOARD_AT73C213_SSC, AT91C_SSC_TXBUFE | AT91C_SSC_ENDTX) ;              
        }
        
    }    
  
    
    
}



//------------------------------------------------------------------------------
/// Invoked when a frame has been transmitted.    
/// MCU ==> PC
//------------------------------------------------------------------------------
void FrameSend(     unsigned int unused,
                    char status,
                    unsigned int transferred,
                    unsigned int remaining  )
{

    //Record        
    if ( status == USBD_STATUS_SUCCESS  ) { 
        AUDDSpeakerDriver_Write(  buffers_rcd[outBufferIndex_rcd],
                                  bufferSizes_rcd[outBufferIndex_rcd] * AUDDSpeakerDriver_BYTESPERSAMPLE,//AUDDSpeakerDriver_BYTESPERFRAME,
                                  (TransferCallback) FrameSend,
                                  0); // No optional argument 

        
        if(numBuffersToSend_rcd >0)  { //rec buf has data, ok
            outBufferIndex_rcd = (outBufferIndex_rcd + 1) % BUFFER_NUMBER_RXD;
            numBuffersToSend_rcd-- ;
            
        }  else  {  //usb in too fast. rec buf empty, which should not be empty unless SSC no data input, ERROR,   so reset index 
            outBufferIndex_rcd   = 0 ;
            numBuffersToSend_rcd = REC_PRE_BUF_NUMBER ;
            inBufferIndex_rcd    = REC_PRE_BUF_NUMBER ;
            TRACE_WARNING("\n\rRecord too fast, buffer is empty!\n\r"); 
            
        }   
        total_transmit += transferred ;
        //////////////////////////////////////////////////////////////////////
        
        
    //    
    //    //PIO_Set(&Test) ; 
    //    if( isRcdActive == 0 )  {
    //        //TRACE_INFO("<R_L>"); 
    //        isRcdActive = 1 ;
    //        //EnableSynInt_rx() ;
    //        //EnableSscSync_rx() ;            
    //        //StartRcd() ;
    //    }      
    //    
    }
}




void Usb_Init(void)
{  
    // USB audio driver initialization    
    SetBratInDescriptor(FramRat) ;
    //SetAudioName();      
    
    AUDDSpeakerDriver_Initialize() ;
    VBUS_CONFIGURE() ;
    
    // Turn off LEDs
    LED_Clear(USBD_LEDPOWER);
    LED_Clear(USBD_LEDUDATA);
    
}
