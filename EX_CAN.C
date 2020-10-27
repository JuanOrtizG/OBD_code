/*
@autor: Juan Ortiz
@versión: 
@fecha:
@Tema: Protocolo de Comunicación CAN
*/

#include "18F4580.h"
#fuses HS,NOPROTECT,NOLVP,NOWDT
#use delay(clock=20000000)
#use rs232(baud=9600, xmit=PIN_C6, rcv=PIN_C7)

#DEFINE CAN_DO_DEBUG          TRUE
#DEFINE CAN_USE_EXTENDED_ID    FALSE

//#DEFINE Set_Standard_125k_Baud    TRUE   //FUNCIONA
//#DEFINE Set_500K_Baud             TRUE   //Configurar a TRUE para usar .. NO FUNCIONA A 500KBPS
#DEFINE Set_250K_Baud               TRUE   //Configurar a TRUE para usar .. FUNCIONA
//#DEFINE Set_125K_Baud             FALSE   //Configurar a TRUE para usar  -- AUN NO PROBADO


#include "can-18F4580.c"

int16 ms;

#int_timer2
void isr_timer2(void) {
   ms++; //keep a running timer that increments every milli-second
}

void main() {
   struct rx_stat rxstat;
   int32 rx_id;
   int in_data[8];
   int rx_len;

//send a request (tx_rtr=1) for 8 bytes of data (tx_len=8) from id 24 (tx_id=24)
   int out_data[8];
   int32 tx_id=4;
   int1 tx_rtr=0;
   int1 tx_ext=0;
   int tx_len=8;
   int tx_pri=3;

   int i;

   for (i=0;i<8;i++) {
      out_data[i]=0;
      in_data[i]=0;
   }

   printf("\r\n\r\nCCS CAN EXAMPLE\r\n");

   setup_timer_2(T2_DIV_BY_4,79,16);   //setup up timer2 to interrupt every 1ms if using 20Mhz clock

   can_init();

   enable_interrupts(INT_TIMER2);   //enable timer2 interrupt
   enable_interrupts(GLOBAL);       //enable all interrupts (else timer2 wont happen)

   printf("\r\nRunning...");

   while(TRUE)
   {
   
      if ( can_kbhit() )   //if data is waiting in buffer...
      {
         if(can_getd(rx_id, &in_data[0], rx_len, rxstat)) { //...then get data from buffer
            printf("\r\nGOT: BUFF=%U ID=%LU LEN=%U OVF=%U ", rxstat.buffer, rx_id, rx_len, rxstat.err_ovfl);
            printf("FILT=%U RTR=%U EXT=%U INV=%U", rxstat.filthit, rxstat.rtr, rxstat.ext, rxstat.inv);
            printf("\r\n    DATA = ");
            for (i=0;i<rx_len;i++) {
               printf("%X ",in_data[i]);
            }
            printf("\r\n");
         }
         else {
            printf("\r\nFAIL on GETD\r\n");
         }

      }
   
      //every two seconds, send new data if transmit buffer is empty
      if ( can_tbe() && (ms > 2000))
      {
         ms=0;
         i=can_putd(tx_id, out_data, tx_len,tx_pri,tx_ext,tx_rtr); //put data on transmit buffer
         if (i != 0xFF) { //success, a transmit buffer was open
            printf("\r\nPUT %U: ID=%LU LEN=%U ", i, tx_id, tx_len);
            printf("PRI=%U EXT=%U RTR=%U\r\n   DATA = ", tx_pri, tx_ext, tx_rtr);
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
}
