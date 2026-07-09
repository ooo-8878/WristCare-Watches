#ifndef __MYRTC_H
#define __MYRTC_H

#include "stm32f4xx.h"


void my_rtc_init(void);
void myrtc_alarm_init(void);
void myrtc_alarm_set_time(uint8_t rtc_h12,uint8_t hours,uint8_t minutes);

#endif