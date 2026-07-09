#ifndef __USART_H
#define __USART_H


extern volatile uint8_t  g_usart1_rx_buf[1280];
extern volatile uint32_t g_usart1_rx_cnt;
extern volatile uint32_t g_usart1_rx_end;



void usart1_init(uint32_t baud);

void usart1_send_bytes(uint8_t *buf,uint32_t len);


#endif



