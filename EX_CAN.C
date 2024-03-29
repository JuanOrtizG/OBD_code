/*
@autor: Juan Ortiz
@versión: 
@fecha:
@Tema: Protocolo de Comunicación CAN
*/

#include "18F4580.h"
#fuses HS,NOPROTECT,NOLVP,NOWDT
#use delay(clock=16000000)
//#use delay(oscillator=20Mhz, clock = 80Mhz)
#use rs232(baud=9600, xmit=PIN_C6, rcv=PIN_C7)

#DEFINE CAN_DO_DEBUG          FALSE
#DEFINE CAN_USE_EXTENDED_ID    FALSE

//#DEFINE Set_1000K_Baud            TRUE
//#DEFINE Set_500K_Baud             TRUE   //
#DEFINE Set_250K_Baud             TRUE   //
//#DEFINE Set_200K_Baud             TRUE   //
//#DEFINE Set_125K_Baud             TRUE   //


#include "can-18F4580.c"

/*
int16 ms;

#int_timer2
void isr_timer2(void) {
   ms++; //keep a running timer that increments every milli-second
}
*/


//int1 can_getd(int32 & id, int * data, int & len, struct rx_stat & stat)
void obd_get()
{
   struct rx_stat rxstat;
   int32 rx_id;
   int in_data[8];
   int rx_len;
   int i;
   
   if ( can_kbhit() )   //if data is waiting in buffer...
      {
         if(can_getd(rx_id, &in_data[0], rx_len, rxstat)) { //...then get data from buffer
           // printf("\r\nGOT: BUFF=%U ID=%LU LEN=%U OVF=%U ", rxstat.buffer, rx_id, rx_len, rxstat.err_ovfl);
           // printf("FILT=%U RTR=%U EXT=%U INV=%U", rxstat.filthit, rxstat.rtr, rxstat.ext, rxstat.inv);
            printf("\r\n    RECIBIENDO DATA = ");
            for ( i=0;i<rx_len;i++) {
               printf("%X ",in_data[i]);
            }
            printf("\r\n");
         }
         else {
            printf("\r\nFAIL on GETD\r\n");
         }

      }
}


//void obd_put(int time)
void obd_put()
{
   //send a request (tx_rtr=1) for 8 bytes of data (tx_len=8) from id 24 (tx_id=24)
   int out_data[8]={1,2,3,4,5,5,5,5};
   int32 tx_id=12;
   int1 tx_rtr=0;
   int1 tx_ext=0;
   int tx_len=8;
   int tx_pri=3;

   int i;
   
   //every two seconds, send new data if transmit buffer is empty
      // if ( can_tbe() && (ms > 2000)) 
      if ( can_tbe() )
      {
         //ms=time;
         i=can_putd(tx_id, out_data, tx_len,tx_pri,tx_ext,tx_rtr); //put data on transmit buffer
         if (i != 0xFF) { //success, a transmit buffer was open
            // printf("\r\nPUT %U: ID=%LU LEN=%U ", i, tx_id, tx_len);
            // printf("PRI=%U EXT=%U RTR=%U\r\n   DATA = ", tx_pri, tx_ext, tx_rtr);
            printf("\r\n    ENVIANDO DATA = ");
            for (i=0;i<tx_len;i++) {
               printf("%X ",out_data[i]);
            }
            printf("\r\n");
         }
         else { //fail, no transmit buffer was open
            printf("\r\nFAIL on PUTD\r\n");
         }
      }
}
void main() {
   
   printf("\r\n\r\nCCS CAN EXAMPLE\r\n");

   //setup_timer_2(T2_DIV_BY_4,79,16);   //setup up timer2 to interrupt every 1ms if using 20Mhz clock

   can_init();
   enable_interrupts(INT_TIMER2);   //enable timer2 interrupt
   enable_interrupts(GLOBAL);       //enable all interrupts (else timer2 wont happen)

   printf("\r\nRunning...");

   while(TRUE)
   {
      printf ("ciclo.. \n");
      obd_get();
      delay_ms(500);
      obd_put();
      delay_ms(500);
   
      
   }
}
