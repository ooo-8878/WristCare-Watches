#ifndef __VIBRATION_MOTOR_H
#define __VIBRATION_MOTOR_H


typedef struct __motor_t
{
    uint8_t sta;
    uint32_t time;
}motor_t;

#define VIBRATION_MOTOR(x)  (x) ? (PBout(1)=1) : (PBout(1)=0)

void vibration_motor_init(void);

#endif