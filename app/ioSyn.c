#include <board.h>
#include <pio/pio.h>
#include <stdbool.h>
#include <pio/pio_it.h>
#include <ssc/ssc.h>
#include <irq/irq.h>
#include <ioSyn.h>
#include <app.h>
#include <utility/trace.h>

extern const Pin Test;
// pin for synchronous action defines

#ifdef  TRACE_DBGU //normal : UART_DBG pin as UART
const Pin Test      =  {1 << 1, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT} ;
#else  //debug: use UART_DBG pin for test trigger debug
const Pin Test      =  {1 << 27, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} ; //debug no UART_DBG_Rx
const Pin Test_rx   =  {1 << 28, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} ; //debug no UART_DBG_Tx
#endif

const Pin StRcd_2   =  {1 << 4, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_OPENDRAIN|PIO_PULLUP} ;
const Pin StRcd     =  {1 << 6, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_OPENDRAIN|PIO_PULLUP} ;
//const Pin TxEn      =  {1 << 7, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} ;
//const Pin RxEn      =  {1 << 8, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} ;
const Pin TxEn      =  {1 << 7, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_OPENDRAIN|PIO_PULLUP} ; //J3.1
const Pin RxEn      =  {1 << 8, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_OPENDRAIN|PIO_PULLUP} ;
const Pin pinSyn    =  {1 << 9, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP};
const Pin pinSyn_rx =  {1 << 10,AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP};
const Pin SynClk    =  {1 << 11,AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP};

const Pin SSC_Pins        = PINS_SSC_TX;
const Pin SSC_Pins_Float  = PINS_SSC_TX_FLOAT;




volatile unsigned char usbsyn = 0;


/************************* enables **********************************/
void  EnableSynInt(void)
{   
    PIO_EnableIt(&pinSyn);    
}
    
void  EnableSscSync(void)
{          
    while( !PIO_Get(&SynClk) ) ;
    while(  PIO_Get(&SynClk) ) ; 
    PIO_Set(&TxEn) ;    
}


void  EnableSynInt_rx(void)
{  
    PIO_EnableIt(&pinSyn_rx);  
}

void  EnableSscSync_rx(void)
{  
    while( !PIO_Get(&SynClk) ) ;
    while(  PIO_Get(&SynClk) ) ; 
    PIO_Set(&RxEn) ;      
}


/***************************** disables ********************************/
void  DisableSyn(void)
{
    PIO_DisableIt(&pinSyn);
}


void  DisableSscSync(void)
{
    PIO_Clear(&TxEn) ;
}


void  DisableSyn_rx(void) 
{
    PIO_DisableIt(&pinSyn_rx) ;
}


void  DisableSscSync_rx(void)
{
    PIO_Clear(&RxEn) ;
}


void StopRcd(void)
{
    PIO_Clear(&StRcd) ;
}

void StartRcd(void)
{
    usbsyn = 1 ;
}




/*****************************************************************************/
// StRcd pins on DC and DP are OD circuit, Only both DP and DC StRcd pins are 
// released(clear,low), then the output is in high state, like : lines and
void StartRcd_Task(void)
{    
    if(usbsyn)  {       
        usbsyn = 0 ;  
        PIO_Set(&StRcd) ;//set to high  
    }
    
}




void IO_synInit(void)
{
     
//    PIO_Configure(&pinSyn, 1) ;
//    PIO_Configure(&SynClk, 1) ;
//    PIO_Configure(&TxEn, 1) ;    
//    PIO_Configure(&pinSyn_rx, 1) ;
//    PIO_Configure(&RxEn, 1) ;
//    PIO_Configure(&StRcd, 1) ;
//    PIO_Configure(&StRcd_2, 1) ;
 
//#ifdef  TRACE_DBGU //normal : UART_DBG pin as UART
//    PIO_Configure(&Test, 1) ;  
//#else   
//    PIO_Configure(&Test, 1) ; 
//    PIO_Configure(&Test_rx, 1) ; 
//#endif
    
    PIO_Configure(&SSC_Pins_Float, 1);  //init I2S first to float to avoid level conflict on FLCK,BCLK

//    PIO_Clear(&TxEn) ;
//    PIO_Clear(&RxEn) ;
//    PIO_Clear(&StRcd);//toggle to GND 
//    PIO_Clear(&StRcd_2);
//#ifdef  TRACE_DBGU //normal : UART_DBG pin as UART    
//    PIO_Set(&Test) ;
//#else
//    PIO_Set(&Test) ;
//    PIO_Set(&Test_rx) ;
//#endif    
    
//    PIO_InitializeInterrupts( PIO_PRIORITY );
    
    
}




/*********************   Sync pin interrupt service   *************************/


//Play :  FCLK = 0 left ;FCLK =1 right
__ramfunc void IO_synISR_Play_LR( void ) 
{         
    pinSyn.pio->PIO_IDR = pinSyn.mask; //PIO_DisableIt(&pinSyn);//DisableSyn() ;     
    //Test.pio->PIO_SODR  = Test.mask;   
       
    while(   SynClk.pio->PIO_PDSR & SynClk.mask )  ; 
    while( !(SynClk.pio->PIO_PDSR & SynClk.mask )) ; 
    BOARD_AT73C213_SSC->SSC_CR = AT91C_SSC_TXEN ;  //SSC_EnableTransmitter(BOARD_AT73C213_SSC);
    BOARD_AT73C213_SSC->SSC_PTCR = AT91C_PDC_TXTEN ; //SSC_EnableTransmitterPDC(BOARD_AT73C213_SSC);
       
    //TxEn.pio->PIO_CODR           = TxEn.mask; //PIO_Clear(&TxEn) ;//DisableSscSync() ;
    BOARD_AT73C213_SSC->SSC_IER  = AT91C_SSC_TXBUFE | AT91C_SSC_ENDTX;//SSC_EnableInterrupts(BOARD_AT73C213_SSC, AT91C_SSC_TXBUFE | AT91C_SSC_ENDTX) ; 
    
    //Test.pio->PIO_CODR = Test.mask; 
    //TRACE_INFO( " P[LR] ");  
}


//Play:   FCLK = 1 left ; FCLK = 0 right
__ramfunc void IO_synISR_Play_RL( void )  
{    
    pinSyn.pio->PIO_IDR = pinSyn.mask; //PIO_DisableIt(&pinSyn);//DisableSyn() ;    
    //Test.pio->PIO_SODR  = Test.mask; 
    
    while(   SynClk.pio->PIO_PDSR & SynClk.mask )  ; 
    while( !(SynClk.pio->PIO_PDSR & SynClk.mask )) ;
    while(   SynClk.pio->PIO_PDSR & SynClk.mask )  ;  
    BOARD_AT73C213_SSC->SSC_CR = AT91C_SSC_TXEN ;
    BOARD_AT73C213_SSC->SSC_PTCR = AT91C_PDC_TXTEN ; //SSC_EnableTransmitterPDC(BOARD_AT73C213_SSC); 
     
    //TxEn.pio->PIO_CODR           = TxEn.mask; //PIO_Clear(&TxEn) ;//DisableSscSync() ;
    BOARD_AT73C213_SSC->SSC_IER  = AT91C_SSC_TXBUFE | AT91C_SSC_ENDTX;//SSC_EnableInterrupts(BOARD_AT73C213_SSC, AT91C_SSC_TXBUFE | AT91C_SSC_ENDTX) ; 
         
    //Test.pio->PIO_CODR = Test.mask;  
    //TRACE_INFO( " P[RL] ");

}




//Record :  FCLK = 0 left ; FCLK =1 right
__ramfunc void IO_synISR_Rec_LR( void )   
{    
    pinSyn_rx.pio->PIO_IDR = pinSyn_rx.mask;//PIO_DisableIt(&pinSyn_rx) ;//DisableSyn_rx() ;    
    //Test_rx.pio->PIO_SODR  = Test_rx.mask;     
    
    while(   SynClk.pio->PIO_PDSR & SynClk.mask )  ; 
    while( !(SynClk.pio->PIO_PDSR & SynClk.mask )) ; 
    BOARD_AT73C213_SSC->SSC_CR = AT91C_SSC_RXEN ;
    BOARD_AT73C213_SSC->SSC_RHR;
    BOARD_AT73C213_SSC->SSC_PTCR = AT91C_PDC_RXTEN ;//SSC_EnableReceiverPDC(BOARD_AT73C213_SSC);         
    
    //RxEn.pio->PIO_CODR           = RxEn.mask; //PIO_Clear(&RxEn) ;//DisableSscSync_rx() ;
    BOARD_AT73C213_SSC->SSC_IER  = AT91C_SSC_RXBUFF | AT91C_SSC_ENDRX;//SSC_EnableInterrupts(BOARD_AT73C213_SSC, AT91C_SSC_RXBUFF | AT91C_SSC_ENDRX) ;  
       
  
    //Test_rx.pio->PIO_CODR = Test_rx.mask; 
    //TRACE_INFO( " R[LR] ");

}


//Record:   FCLK = 1 left ; FCLK = 0 right
__ramfunc void IO_synISR_Rec_RL( void ) 
{
    pinSyn_rx.pio->PIO_IDR = pinSyn_rx.mask;//PIO_DisableIt(&pinSyn_rx) ;//DisableSyn_rx() ;  
    //Test_rx.pio->PIO_SODR  = Test_rx.mask; //PIO_Set(&Test_rx) ;
    
    while(   SynClk.pio->PIO_PDSR & SynClk.mask )  ; 
    while( !(SynClk.pio->PIO_PDSR & SynClk.mask )) ;
    while(   SynClk.pio->PIO_PDSR & SynClk.mask )  ;
    BOARD_AT73C213_SSC->SSC_CR = AT91C_SSC_RXEN ;
    BOARD_AT73C213_SSC->SSC_RHR;
    BOARD_AT73C213_SSC->SSC_PTCR = AT91C_PDC_RXTEN ;//SSC_EnableReceiverPDC(BOARD_AT73C213_SSC); 
    
    //RxEn.pio->PIO_CODR           = RxEn.mask; //PIO_Clear(&RxEn) ;//DisableSscSync_rx() ;
    BOARD_AT73C213_SSC->SSC_IER  = AT91C_SSC_RXBUFF | AT91C_SSC_ENDRX;//SSC_EnableInterrupts(BOARD_AT73C213_SSC, AT91C_SSC_RXBUFF | AT91C_SSC_ENDRX) ;  
    
    //Test_rx.pio->PIO_CODR        = Test_rx.mask;     
    //TRACE_INFO( " R[RL] ");
}

