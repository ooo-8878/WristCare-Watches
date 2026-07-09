#ifndef __DHT11_H
#define __DHT11_H

#define DHT11_Port        GPIOG
#define DHT11_RCC         RCC_AHB1Periph_GPIOG
#define DHT11_Pin         GPIO_Pin_9 //data引脚
#define DHT11_Data_R()    PGin(9)
#define DHT11_Data_W()    PGout(9) 

void dht11_init(void);
void dht11_start(void);
int8_t dht11_ack(void);
int8_t dht11_receive_byte(void);
int8_t dht11_get_tem_hum(float* tem,uint8_t* hum);

#endif