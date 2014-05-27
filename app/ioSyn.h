#ifndef __IO_SYN_H
#define __IO_SYN_H

extern void IO_synInit(void);

extern void  EnableSynInt(void) ;
extern void  EnableSscSync(void) ;
extern void  DisableSyn(void) ;
extern void  DisableSscSync(void) ;

extern void  EnableSynInt_rx(void) ;
extern void  EnableSscSync_rx(void) ;
extern void  DisableSyn_rx(void) ;
extern void  DisableSscSync_rx(void) ;

extern void  StartRcd(void) ;
extern void  StopRcd(void) ;
extern void  StartRcd_Task(void) ;

extern void  IO_synISR_Play_RL( void )  ;
extern void  IO_synISR_Rec_RL( void )  ;
extern void  IO_synISR_Play_LR( void )  ;
extern void  IO_synISR_Rec_LR( void )  ;



extern const Pin pinSyn    ;
extern const Pin pinSyn_rx ;
extern const Pin SynClk    ;
extern const Pin TxEn      ;
extern const Pin RxEn      ;
extern const Pin StRcd     ;
extern const Pin Tst       ;
extern const Pin SSC_Pins  ;

#endif
