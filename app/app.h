/*****************************************************************
        DEFINES for Varies
***************************************************************/
#ifndef APP_H
#define APP_H





////////////////////////////////////////////////////////////////////////////////

#define DEBUG_AUDIO_DATA  //debug audio data use

////////////////////////////////////////////////////////////////////////////////

#define MCK                  BOARD_MCK
#define UART_BUFFER_SIZE     128// uart buf size

#define BUFFER_SIZE          128// 16*2*4 = 128//  384// 48*2*4 = 384   //192  //audio data transfered per USB frame, Max 48kHz:   48*2*2 =192
#define BUFFER_NUMBER_TXD    400 //must be a multiple of 20.  //140   play
#define BUFFER_NUMBER_RXD    400 //must be a multiple of 20.   //140  rec

#define I2S_BUF_LEN  (BUFFER_NUMBER_TXD > BUFFER_NUMBER_RXD? \
                     (BUFFER_NUMBER_TXD*BUFFER_SIZE) : (BUFFER_NUMBER_RXD*BUFFER_SIZE) )// max buf size

//this buffer may affect the MCU and PC time async time!!!
//#define PLAY_PRE_BUF_NUMBER  BUFFER_NUMBER_TXD/2 
#define PLAY_PRE_BUF_NUMBER  300  //because almost PC clcok is less than AB01 clock based on test
#define REC_PRE_BUF_NUMBER   BUFFER_NUMBER_RXD/2


// A programmable priority level : A higher level corresponds to a higher priority, 
// so level 0 is the lowest interrupt priority

#define USB_PRIORITY        4
#define SSC_PRIORITY        6
#define PIO_PRIORITY        5
#define UART_PRIORITY       7


//for var : flag_sr_fraction
#define FLAG_SR_2205K       2
#define FLAG_SR_441K        1
#define FLAG_SR_OTHERS      0


//Commands from Host MCU
#define  AUDIO_CMD_IDLE                 0x00
#define  AUDIO_CMD_START_REC            0x01
#define  AUDIO_CMD_START_PLAY           0x02
#define  AUDIO_CMD_START_PALYREC        0x03
#define  AUDIO_CMD_STOP                 0x04
#define  AUDIO_CMD_CFG                  0x05
#define  AUDIO_CMD_VERSION              0x06

//Error type
#define   ERR_USB_STATE                 0xF1
#define   ERR_CMD_TYPE                  0xFF


////////////////////////////////////////////////////////////////////////////////

typedef unsigned char BUF[BUFFER_SIZE]  ;//Audio data buffer

typedef struct {
  
  unsigned char type;//Rec: =0x00, Play: =0x01
  unsigned char channel_num; //1~6
  unsigned short int sample_rate;  
  
}AUDIO_CFG;


extern AUDIO_CFG  Audio_Configure[];
extern unsigned char audio_cmd_index;

#if defined( PIN_USB_VBUS )
    #define VBUS_CONFIGURE()    VBus_Configure()
#else 
    #define VBUS_CONFIGURE()    USBD_Connect()
#endif //#if defined(PIN_USB_VBUS)


#ifdef DEBUG_AUDIO_DATA
extern  unsigned char debug_buf[] ;
#endif

extern const char fw_version[];

//used for fast execution in ISR          
#define  LED_SET_POWER     { *(volatile unsigned int *)0xFFFFF634 = 1 << 25;}  //set power led          
#define  LED_CLEAR_POWER   { *(volatile unsigned int *)0xFFFFF630 = 1 << 25;}  //clear power led
#define  LED_SET_DATA      { *(volatile unsigned int *)0xFFFFF634 = 1 << 26;}  //set data led
#define  LED_CLEAR_DATA    { *(volatile unsigned int *)0xFFFFF630 = 1 << 26;}  //clear data led
#define  LED_TOGGLE_DATA   { if( *(volatile unsigned int *)0xFFFFF638 & (1 << 26) ){\
                                *(volatile unsigned int *)0xFFFFF634 = 1 << 26;\
                             } else {\
                                *(volatile unsigned int *)0xFFFFF630 = 1 << 26;\
                             }} 
                        


extern unsigned char  flag_b ;
extern unsigned char  flag_c ;
extern unsigned char  flag_d ;
extern unsigned char  flag_first_rec_sync;

extern volatile unsigned int FramRat ;
extern volatile unsigned char rxbuf[] ;
extern volatile unsigned char txbuf[] ;
extern BUF *buffers ;
extern BUF *buffers_rcd ;

extern volatile unsigned int bufferSizes[];
extern volatile unsigned int bufferSizes_rcd[] ;
extern volatile unsigned int inBufferIndex ;
extern volatile unsigned int inBufferIndex_rcd ;
extern volatile bool         play_start ;
extern volatile bool         record_start ;
extern volatile unsigned int numBuffersToSend ;
extern volatile unsigned int numBuffersToSend_rcd ;
extern volatile unsigned int outBufferIndex ;
extern volatile unsigned int outBufferIndex_rcd ;
extern volatile unsigned int ssc_rxd_buf_len ;
extern volatile unsigned int dacDelay;

extern unsigned int total_received ;
extern unsigned int total_transmit ;
extern unsigned int error_bulkout_full  ;
extern unsigned int error_bulkout_empt  ;
extern unsigned int error_bulkin_full   ;
extern unsigned int error_bulkin_empt   ;

            
extern volatile bool bulkin_enable;
extern volatile bool bulkout_enable;
extern volatile bool bulkout_start;
extern volatile bool bulkin_start;

extern volatile unsigned char usartBuffers[][UART_BUFFER_SIZE];
extern volatile unsigned char usartCurrentBuffer ;
//extern volatile unsigned char usartBuffers1[][UART_BUFFER_SIZE];
//extern volatile unsigned char usartCurrentBuffer1 ;
//extern volatile unsigned char usartBuffers2[][UART_BUFFER_SIZE];
//extern volatile unsigned char usartCurrentBuffer2 ;

#ifdef _UART_C
#else
extern void Uart_Init(void) ;
#endif

#ifdef _I2S_C
#else
extern void I2s_Init(  unsigned int mclk  ) ;
#endif

#ifndef _USB_C
 
extern void VBus_Configure( void ) ;
extern void FrameReceived(unsigned int unused,char status,unsigned int transferred,unsigned int remaining) ;
extern void FrameSend(unsigned int unused, char status,unsigned int transferred,unsigned int remaining) ;
extern void Usb_Init(void) ;
#endif

extern void Debug_Info( void );

extern void CheckSum_Task(void) ;
extern void  Set_I2S_RX(void) ;
extern void  Set_I2S_TX(void) ;
extern void  Set_I2S(void) ;
extern void InitI2S_Task(void) ;

extern volatile unsigned int I2S_RxOn   ;
extern volatile unsigned int I2S_TxOn   ;
extern volatile unsigned int I2S_On     ;


extern void Check_Audio_ID( void );

extern unsigned char flag_play_interface_closed;
extern volatile unsigned char flag_sr_fraction  ;
extern volatile unsigned int  sample_total       ;
extern volatile unsigned int  sample_last        ;
extern volatile unsigned int  sample_length_long ;
extern volatile unsigned int  sample_length_short;

extern unsigned char Audio_ID ;
extern          char AUDIO_ID_S[];


extern unsigned char test_buf[][BUFFER_SIZE/2] ;
extern volatile unsigned char test_buf_id;

extern unsigned int play_buf_empty_counter;



#endif //#ifndef APP_H
