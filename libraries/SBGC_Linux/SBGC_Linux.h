#ifndef  __SBGC_Linux__
#define  __SBGC_Linux__

#include <inttypes.h>
#include <stdio.h>
#include <SBGC.h>


#define LED_PIN  13

extern SBGC_Parser sbgc_parser; 

void SBGC_Demo_setup(int fd);
inline void LED_ON() {   printf("LED_ON\n"); }
inline void LED_OFF() {    printf("LED_OFF\n"); }
void blink_led(uint8_t cnt);





#endif
