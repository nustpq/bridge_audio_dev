#include <pio/pio.h>
#include <stdbool.h>
#include <usb/device/audio-speaker/AUDDSpeakerDriver.h>
#include <ioSyn.h>
#include <ssc/ssc.h>
#include <board.h>
#include <utility/trace.h>
#include "app.h"


const char fw_version[] = "[FW:B:V0.1]";

volatile unsigned int  FramRat   =  16000 ; //Audio bridge default sample rate is 16k
volatile unsigned char rxbuf[120] ;   //rec
volatile unsigned char txbuf[I2S_BUF_LEN] ; //play
#ifdef DEBUG_AUDIO_DATA
unsigned char debug_buf[BUFFER_SIZE*2] ;
#endif

volatile unsigned int bufferSizes[BUFFER_NUMBER_TXD] ;
volatile unsigned int bufferSizes_rcd[BUFFER_NUMBER_RXD] ; 

unsigned int play_buf_empty_counter = 0 ; //record how  many times BUFFER_NUMBER_TXD = 0 
volatile unsigned char test_buf_id  = 0 ;
unsigned char test_buf[2][BUFFER_SIZE/2] ;

#ifdef DEBUG_AUDIO_DATA 
unsigned char  flag_b = 0;
unsigned char  flag_c = 0;
unsigned char  flag_d = 0;
#endif
unsigned char  flag_first_rec_sync = 0 ;

BUF *buffers ;
BUF *buffers_rcd ;

volatile unsigned int inBufferIndex         = 0 ;
volatile unsigned int inBufferIndex_rcd     = 0 ;

volatile bool play_start                    = true ;
volatile bool record_start                  = true ;

volatile unsigned int numBuffersToSend      = 0 ;
volatile unsigned int numBuffersToSend_rcd  = 0 ;

volatile unsigned int outBufferIndex        = 0 ;
volatile unsigned int outBufferIndex_rcd    = 0 ;
volatile unsigned int ssc_rxd_buf_len       = 0 ;
volatile unsigned int dacDelay ;


unsigned int total_received = 0 ;
unsigned int total_transmit = 0 ;
unsigned int error_bulkout_full  = 0 ;
unsigned int error_bulkout_empt  = 0 ;
unsigned int error_bulkin_full   = 0 ;
unsigned int error_bulkin_empt   = 0 ;

volatile bool bulkout_start     = false ;
volatile bool bulkin_start      = false ;
volatile bool bulkin_enable     = true;
volatile bool bulkout_enable    = true;

volatile unsigned int I2S_RxOn      = 0 ;
volatile unsigned int I2S_TxOn      = 0 ;
volatile unsigned int I2S_On        = 0 ;

volatile unsigned char usartBuffers[2][UART_BUFFER_SIZE] ;
volatile unsigned char usartCurrentBuffer = 0 ;

volatile unsigned char flag_sr_fraction    =  FLAG_SR_OTHERS ; 
volatile unsigned int sample_total         = 0;
volatile unsigned int sample_last          = 0;
volatile unsigned int sample_length_long   = 0;
volatile unsigned int sample_length_short  = 0;

unsigned char flag_play_interface_closed = 0 ;

const Pin Audio_ID_Pins[] =  {  PIN_AUDIO_ID2, PIN_AUDIO_ID1,PIN_AUDIO_ID0  } ;
unsigned char Audio_ID    =  0; 

AUDIO_CFG  Audio_Configure[2]; //[0]: rec config. [1]: play config.
unsigned char audio_cmd_index = AUDIO_CMD_IDLE ; 



      
//identify audio name
void Check_Audio_ID( void )
{ 
    unsigned char i     =   0 ;
    unsigned int  value =   0 ;
    
    PIO_Configure(Audio_ID_Pins, PIO_LISTSIZE(Audio_ID_Pins) ) ;
    
    for( i=0; i<PIO_LISTSIZE( Audio_ID_Pins ); i++ ) {      
        value <<= 1;
        value +=PIO_Get( &Audio_ID_Pins[i] );
    }     
    Audio_ID = value ; 
   
}
  
unsigned int CheckSum( unsigned int PreSum,unsigned int *pLoad, unsigned int Len )
{  
//    unsigned int sum = PreSum ;
//    
//    while(Len--)  {
//      
//        sum = *(pLoad++) + sum ; 
//    }
//    return(sum) ;  
    return 0;
}


void InitI2S_Task(void)
{
  
    unsigned int  i   ;
    unsigned int *pt  ;
    
    ///////////////////////////////////////////////////////////////////////////

    if( I2S_RxOn )   {      
      	I2S_RxOn = 0 ;                       
      	        
  		SSC_DisableReceiver( BOARD_AT73C213_SSC ) ;
        //reset MCU PDC counter
        SSC_InitReceiver( BOARD_AT73C213_SSC );
      	
        pt = (unsigned int *)rxbuf ; //somewhere init:   buffers_rcd = (BUF *) rxbuf ;
        for(i=0;i<I2S_BUF_LEN/4;i++)  {
            *(pt+i) = 0 ;             
        }           
    
        if( flag_sr_fraction == FLAG_SR_2205K )  {     //sample rate =  22.05kHz  
            sample_total        = 20; 
            sample_last         = 19;
            sample_length_long  = AUDDSpeakerDriver_SAMPLESPERFRAME_2205K_23;
            sample_length_short = AUDDSpeakerDriver_SAMPLESPERFRAME_2205K_22;
            for(i=0; i<REC_PRE_BUF_NUMBER; i++)  {
                    if(i % 20 == 19)  {
                       bufferSizes_rcd[i] = AUDDSpeakerDriver_SAMPLESPERFRAME_2205K_23;                       
                    } else {
                       bufferSizes_rcd[i] = AUDDSpeakerDriver_SAMPLESPERFRAME_2205K_22; 
                       
                    }
            }
        } else if( flag_sr_fraction == FLAG_SR_441K ) {   //sample rate = 44.1kHz    
            sample_total        = 10; 
            sample_last         =  9;   
            sample_length_long  = AUDDSpeakerDriver_SAMPLESPERFRAME_441K_45;
            sample_length_short = AUDDSpeakerDriver_SAMPLESPERFRAME_441K_44;
            for(i=0; i<REC_PRE_BUF_NUMBER; i++)  {       
                    if(i % 10 == 9)  {
                        bufferSizes_rcd[i] = AUDDSpeakerDriver_SAMPLESPERFRAME_441K_45;                        
                    } else {
                        bufferSizes_rcd[i] = AUDDSpeakerDriver_SAMPLESPERFRAME_441K_44; 
                        
                    } 
            }
                     
        } else  {                                       //other sample rate cases    
            sample_total        = 12;
            sample_last         = 11;
            sample_length_short = AUDDSpeakerDriver_SAMPLESPERFRAME;
            sample_length_long  = AUDDSpeakerDriver_SAMPLESPERFRAME;
            
            for(i=0; i<REC_PRE_BUF_NUMBER; i++)  {        
                bufferSizes_rcd[i] = AUDDSpeakerDriver_SAMPLESPERFRAME ;
            }                    
   
                     
        } 
    
        
        //init index for recording
        inBufferIndex_rcd    = REC_PRE_BUF_NUMBER ;
        outBufferIndex_rcd   = 0 ;
        numBuffersToSend_rcd = REC_PRE_BUF_NUMBER ;  

        //init PDC 
        if( SSC_ReadBuffer(   BOARD_AT73C213_SSC,
                              buffers_rcd[inBufferIndex_rcd],
                              sample_length_short
                          ) )   {           
            bufferSizes_rcd[inBufferIndex_rcd] =  sample_length_short  ;
            inBufferIndex_rcd = (inBufferIndex_rcd + 1) % BUFFER_NUMBER_RXD;
        }
              
        if( SSC_ReadBuffer(  BOARD_AT73C213_SSC,
                            buffers_rcd[inBufferIndex_rcd],
                            sample_length_short
                          ) )   {           
            bufferSizes_rcd[inBufferIndex_rcd] =  sample_length_short  ;
            inBufferIndex_rcd = (inBufferIndex_rcd + 1) % BUFFER_NUMBER_RXD;
        }                                  
       
        
//        SSC_EnableReceiver(BOARD_AT73C213_SSC);
//        SSC_EnableReceiverPDC(BOARD_AT73C213_SSC);
//        SSC_EnableInterrupts(BOARD_AT73C213_SSC, AT91C_SSC_RXBUFF | AT91C_SSC_ENDRX) ; 
         
    }
    
    ////////////////////////////////////////////////////////////////////////////
    
    if( I2S_TxOn )   {       
    
        I2S_TxOn = 0 ;          
        
		SSC_DisableTransmitter( BOARD_AT73C213_SSC ) ; 
        //reset MCU PDC counter
        SSC_InitTransmitter( BOARD_AT73C213_SSC );
           
        pt = (unsigned int *)txbuf ;  //somewhere init :   buffers     = (BUF *) txbuf ;
        for(i=0;i<I2S_BUF_LEN/4;i++)  {
            *(pt+i) = 0 ;
              
        }  
        
        numBuffersToSend = 0 ;
     
        
    }
    
    ////////////////////////////////////////////////////////////////////////////
            
    if( I2S_On )   {       

        I2S_On = 0 ;
        //prepare USB package
        inBufferIndex    = 0 ;
        outBufferIndex   = 0 ;
        numBuffersToSend = 0 ;       
      
        //play
        AUDDSpeakerDriver_Read(  test_buf[test_buf_id],//buffers[inBufferIndex],
                                 sample_length_long * AUDDSpeakerDriver_BYTESPERSAMPLE , //AUDDSpeakerDriver_BYTESPERFRAME + 4 ,
                                 (TransferCallback)FrameReceived,
                                 0 ) ;    
        
        //record
        AUDDSpeakerDriver_Write( NULL,
                                 0,
                                 (TransferCallback)FrameSend,
                                 0) ;  
 
        
    }
       
}


void Audio_State_Control( void )
{
    
    unsigned char err = 0 ;
        
    if( audio_cmd_index == AUDIO_CMD_IDLE ) {
        return;
    }
    
    if( USBD_GetState() < USBD_STATE_CONFIGURED && 
        audio_cmd_index != AUDIO_CMD_VERSION ) {
        err = ERR_USB_STATE;
        
    } else {
    
        switch( audio_cmd_index ) {            

            
            case AUDIO_CMD_START_PALYREC :  
               
            break;

            case AUDIO_CMD_STOP :             
                          
            break;   
        
            case AUDIO_CMD_CFG:  
            case AUDIO_CMD_VERSION:         
              
            break;         
            
            default:         
                err = ERR_CMD_TYPE;
            break;
        
        }
        
     }   
    
     if( audio_cmd_index == AUDIO_CMD_VERSION ) {        
            //USART_WriteBuffer( AT91C_BASE_US0, (void*)&fw_version[0], sizeof(fw_version) );         
            for(unsigned char i = 0; i< sizeof(fw_version) ; i++ ) {
                USART_Write( AT91C_BASE_US0, fw_version[i], 0 );
            }
            
     } else {          
            USART_Write( AT91C_BASE_US0, err, 0 );
            
     }
        
     audio_cmd_index = AUDIO_CMD_IDLE ;     
    
    
}


void Debug_Info( void )
{
     
//    if( Check_SysTick_State() == 0 ){ 
//        return;
//    }     
    
    delay_ms(100);
    
//    if( !(bulkout_start || bulkin_start) ) { 
//        printf( "\rWaitting for USB trans start...[Miss Stop = %d]",Stop_CMD_Miss_Counter);
//        return ; 
//    }
    
    //start print debug_after USB trans started
    //if( (total_received < 1920) && (total_transmit < 1920) ){  
       // return;
    //}    

    printf( "\rPlay:[%4.6f]MB,Buffered:[%3d]of[%3d],Empty Times:[%d].    Record:[%4.6f]MB,Buffered:[%3d]of[%3d]. ",
            total_received/1000000.0,  
            numBuffersToSend,
            BUFFER_NUMBER_TXD,
            play_buf_empty_counter,
            total_transmit/1000000.0,
            numBuffersToSend_rcd,
            BUFFER_NUMBER_RXD
   );        
            
            
           

}

