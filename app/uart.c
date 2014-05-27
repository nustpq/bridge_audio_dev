#define _UART_C

#include <pio/pio.h>
#include <stdbool.h>
#include <usart/usart.h>
#include <dbgu/dbgu.h>
#include <utility/trace.h>
#include <irq/irq.h>
#include <board.h>
#include "app.h"



#define  RULER_CMD_SET_AUDIO_CFG        0x01
#define  RULER_CMD_START_AUDIO          0x02
#define  RULER_CMD_STOP_AUDIO           0x03
#define  RULER_CMD_GET_AUDIO_VERSION    0x0B

#define CMD_STAT_SYNC1     0
#define CMD_STAT_SYNC2     1
#define CMD_STAT_FLAG      2
#define CMD_STAT_CMD1      3
#define CMD_STAT_CMD2      4
#define CMD_STAT_DATA      5

#define CMD_DATA_SYNC1     0xEB
#define CMD_DATA_SYNC2     0x90


#define UART_TIMEOUT_BIT  ( 5 * 10 ) // 50=5*10  timeout in 5 Bytes' time  


static const Pin Uart_Pins[] = {  

    PIN_USART0_RXD,
    PIN_USART0_TXD,
    //PIN_USART1_RXD,
    //PIN_USART1_TXD
      
};


unsigned char UART_CMD_Buffer[4];
unsigned char state_mac      = CMD_STAT_SYNC1 ;
unsigned char PcCmdCounter   = 0; 


void pcInt(  unsigned char ch )
{    
    unsigned char *pChar = UART_CMD_Buffer;
    
    switch( state_mac ) {   
        
        case CMD_STAT_SYNC1 :        
            if(ch == CMD_DATA_SYNC1)  {
                state_mac = CMD_STAT_SYNC2 ;
            }
        break;
        
        case CMD_STAT_SYNC2 :
            if(ch == CMD_DATA_SYNC2)  {           
                 state_mac     =  CMD_STAT_FLAG;
                 PcCmdCounter  = 0 ; 
            } else {              
                state_mac = CMD_STAT_SYNC1;                
            }
        break ;
        
        case CMD_STAT_FLAG :   
       
            switch( ch )  {
                case RULER_CMD_SET_AUDIO_CFG : 
                    state_mac =  CMD_STAT_CMD1 ;
                    break ;
                case RULER_CMD_START_AUDIO :
                    state_mac =  CMD_STAT_CMD2 ;
                    break ;                
                case RULER_CMD_STOP_AUDIO :
                    audio_cmd_index = AUDIO_CMD_STOP ; 
                    state_mac = CMD_STAT_SYNC1; 
                    break ;  
                case RULER_CMD_GET_AUDIO_VERSION  :
                    audio_cmd_index = AUDIO_CMD_VERSION ; 
                    state_mac = CMD_STAT_SYNC1; 
                break ;  
                        
                default :
                    break ;                        
            }
         
        break ;
        
        case CMD_STAT_CMD1 :            
             UART_CMD_Buffer[PcCmdCounter++] = ch;       
             state_mac  = CMD_STAT_DATA ;
          
        break ;
        
        case CMD_STAT_CMD2 :  
             audio_cmd_index = ch;  //
             PcCmdCounter    = 0;
             state_mac       = CMD_STAT_SYNC1 ;
          
        break ;
        
        case CMD_STAT_DATA :
            *(pChar+PcCmdCounter) = ch; 
            PcCmdCounter++;
            if( PcCmdCounter > 3 ) { //check verflow
               Audio_Configure[(*pChar)&0x01] = *(AUDIO_CFG *)pChar;                
               audio_cmd_index = AUDIO_CMD_CFG ; 
               PcCmdCounter = 0 ;        
               state_mac = CMD_STAT_SYNC1;                
            } 
        break ;
                
        default :
            state_mac     = CMD_STAT_SYNC1;
            PcCmdCounter  = 0 ;
        break ;
        
    } 
    
    
}


//------------------------------------------------------------------------------
/// Handles interrupts coming from USART #0.
//------------------------------------------------------------------------------
static void ISR_Usart0()
{
  
    unsigned int    i ;
    unsigned int    status ;
    unsigned short  data_received;
  
    status        = AT91C_BASE_US0->US_CSR;
    data_received = UART_BUFFER_SIZE - ( AT91C_BASE_US0->US_RCR );
    
    // Buffer has been read successfully
    if ( (status & AT91C_US_ENDRX) != 0 ) {

        // Disable timer
        //AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;         
        
        for( i=0; i < data_received; i++)  { //analyze the data
          
            pcInt(usartBuffers[usartCurrentBuffer][i]) ;
        } 
        
        usartCurrentBuffer  = 1 - usartCurrentBuffer; //pingpong buffer index
        USART_ReadBuffer(AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer], UART_BUFFER_SIZE); // Restart read on buffer

        // Restart timer
        //AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    }
    
    if ( (status & AT91C_US_TIMEOUT) != 0 ) {

        // Disable timer
        //AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;         
        
        AT91C_BASE_US0->US_RCR = 0;
        
        for( i=0; i< data_received ; i++)  {
          
            pcInt(usartBuffers[usartCurrentBuffer][i]) ;
        }        
        
        usartCurrentBuffer = 1 - usartCurrentBuffer;
        USART_ReadBuffer(AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer], UART_BUFFER_SIZE);    // Restart read on buffer
        
        AT91C_BASE_US0->US_CR   = AT91C_US_STTTO; //restart timeout counter
        AT91C_BASE_US0->US_RTOR = UART_TIMEOUT_BIT;
        // Restart timer
        //AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    }
    /*
    // Buffer has been sent
    if ((status & AT91C_US_TXBUFE) != 0) {

        AT91C_BASE_US0->US_IDR = AT91C_US_TXBUFE;
    }
    */
    
    
    
}



/*
static void ISR_Timer1()
{
    unsigned char size1;
    unsigned int status1 = AT91C_BASE_TC1->TC_SR;

    if ((status1 & AT91C_TC_CPCS) != 0) {
    
        size1 = UART_BUFFER_SIZE - AT91C_BASE_US1->US_RCR;
        if (size1==0) {

           AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
           return;
        }
        AT91C_BASE_US1->US_RCR = 0;
        
        unsigned int i ;
        for(i=0;i<size1;i++)
        {
            pcInt(usartBuffers1[usartCurrentBuffer1][i]) ;
        }  
        DBGU_WriteBuffer(AT91C_BASE_DBGU,(void *)usartBuffers1[usartCurrentBuffer1],size1);
        // Restart read on buffer
        usartCurrentBuffer1 = 1 - usartCurrentBuffer1;
        USART_ReadBuffer(AT91C_BASE_US1,(void *)usartBuffers1[usartCurrentBuffer1],UART_BUFFER_SIZE);
        
        AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    }
}
*/                   
//------------------------------------------------------------------------------
/// Handles interrupts coming from USART #0.
//------------------------------------------------------------------------------
/*
static void ISR_Usart1()
{
    unsigned int status = AT91C_BASE_US1->US_CSR;

    // Buffer has been read successfully
    if ((status & AT91C_US_ENDRX) != 0) {

        // Disable timer
        AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;
        
        unsigned int i ;
        for(i=0;i<UART_BUFFER_SIZE;i++)
        {
            pcInt(usartBuffers1[usartCurrentBuffer1][i]) ;
        }  
        
        //DBGU_WriteBuffer(AT91C_BASE_DBGU,(void *)usartBuffers1[usartCurrentBuffer1],UART_BUFFER_SIZE);  
        // Restart read on buffer
        
        usartCurrentBuffer1 = 1 - usartCurrentBuffer1;
        USART_ReadBuffer(AT91C_BASE_US1,(void *)usartBuffers1[usartCurrentBuffer1],UART_BUFFER_SIZE);

        // Restart timer
        AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    }

    // Buffer has been sent
    if ((status & AT91C_US_TXBUFE) != 0) {

        AT91C_BASE_US1->US_IDR = AT91C_US_TXBUFE;
    }
}
*/

/*
static void ISR_Timer2()
{
    unsigned char size2;
    unsigned int status2 = AT91C_BASE_TC2->TC_SR;

    //size1 = UART_BUFFER_SIZE - AT91C_BASE_US1->US_RCR;
    if ((status2 & AT91C_TC_CPCS) != 0) {
    
        size2 = UART_BUFFER_SIZE - AT91C_BASE_DBGU->DBGU_RCR;
        if (size2==0) {

           AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
           return;
        }
        AT91C_BASE_DBGU->DBGU_RCR = 0;
        
        unsigned int i ;
        for(i=0;i<size2;i++)
        {
            pcInt(usartBuffers2[usartCurrentBuffer2][i]) ;
        } 
        
        //DBGU_WriteBuffer(AT91C_BASE_DBGU,(void *)usartBuffers2[usartCurrentBuffer2], size2) ;
        usartCurrentBuffer2 = 1 - usartCurrentBuffer2;
        DBGU_ReadBuffer(AT91C_BASE_DBGU,(void *)usartBuffers2[usartCurrentBuffer2],UART_BUFFER_SIZE);
        
        AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    }
}
*/
//------------------------------------------------------------------------------
/// Handles interrupts coming from USART #0.
//------------------------------------------------------------------------------
/*
static void ISR_UsartDbug()
{
    unsigned int status = AT91C_BASE_DBGU->DBGU_CSR;

    // Buffer has been read successfully
    if ((status & AT91C_US_ENDRX) != 0) {

        // Disable timer
        AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKDIS;
        
        unsigned int i ;
        for(i=0;i<UART_BUFFER_SIZE;i++)
        {
            pcInt(usartBuffers2[usartCurrentBuffer2][i]) ;
        }         
         
        //DBGU_WriteBuffer(AT91C_BASE_DBGU,(void *)usartBuffers2[usartCurrentBuffer2], UART_BUFFER_SIZE) ;
        usartCurrentBuffer2 = 1 - usartCurrentBuffer2;       
        DBGU_ReadBuffer(AT91C_BASE_DBGU, (void *)usartBuffers2[usartCurrentBuffer2], UART_BUFFER_SIZE);

        // Restart timer
        AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    }

    // Buffer has been sent
    if ((status & AT91C_US_TXBUFE) != 0) {

        AT91C_BASE_DBGU->DBGU_IDR = AT91C_US_TXBUFE;
    }
}
*/


void Uart_Init(void)
{
       
    PIO_Configure(Uart_Pins, PIO_LISTSIZE(Uart_Pins));
    
    usartCurrentBuffer = 0 ;
    //usartCurrentBuffer1= 0 ;
    //usartCurrentBuffer2= 0 ;    
    
    // Configure USART0¡¡¡¡//Comm between DP/DC and HOST mcu    
    AT91C_BASE_PMC->PMC_PCER    = 1 << AT91C_ID_US0;
    AT91C_BASE_US0->US_IDR      = 0xFFFFFFFF;
    AT91C_BASE_US0->US_IER      = AT91C_US_ENDRX | AT91C_US_TIMEOUT; 
    USART_Configure(AT91C_BASE_US0,USART_MODE_ASYNCHRONOUS,115200,MCK);
    USART_SetTransmitterEnabled(AT91C_BASE_US0, 1);
    USART_SetReceiverEnabled(AT91C_BASE_US0, 1);
    IRQ_ConfigureIT(AT91C_ID_US0, UART_PRIORITY, ISR_Usart0); ///priority chaned to max?
    
    AT91C_BASE_US0->US_CR   = AT91C_US_STTTO; //restart timeout counter
    AT91C_BASE_US0->US_RTOR = UART_TIMEOUT_BIT;
    
    USART_ReadBuffer(AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer],UART_BUFFER_SIZE);    
    IRQ_EnableIT(AT91C_ID_US0);  
    
    IRQ_EnableIT(AT91C_ID_SYS); 
      
    
    /*
    // Configure timer 0    
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC0);
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC0->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC0->TC_CMR = AT91C_TC_CLKS_TIMER_DIV5_CLOCK
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;
    AT91C_BASE_TC0->TC_RC = 0x00FF;
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;
    IRQ_ConfigureIT(AT91C_ID_TC0, 0, ISR_Timer0);
    IRQ_EnableIT(AT91C_ID_TC0);    
    
    // Configure timer 1
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC1);
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC1->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC1->TC_CMR = AT91C_TC_CLKS_TIMER_DIV5_CLOCK
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;
    AT91C_BASE_TC1->TC_RC = 0x00FF;
    AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS;
    IRQ_ConfigureIT(AT91C_ID_TC1, 0, ISR_Timer1);
    IRQ_EnableIT(AT91C_ID_TC1);
    
    // Configure timer 2
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC2);
    AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC2->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC2->TC_CMR = AT91C_TC_CLKS_TIMER_DIV5_CLOCK
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;
    AT91C_BASE_TC2->TC_RC = 0x00FF;
    AT91C_BASE_TC2->TC_IER = AT91C_TC_CPCS;    
    IRQ_ConfigureIT(AT91C_ID_TC2, 0, ISR_Timer2);
    IRQ_EnableIT(AT91C_ID_TC2);
    */    
    
    /*
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    */
    
}
