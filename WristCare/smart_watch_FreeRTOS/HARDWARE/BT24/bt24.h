#ifndef __BT24_H
#define __BT24_H


typedef struct __bt24_t{
    uint8_t rx_buf[128];
    uint8_t rx_len;
    uint8_t rx_flag;
}bt24_t;

extern bt24_t bt24_data;

void bt24_init(uint32_t baud);
void bt24_sendstr(const char *str);
void bt24_sendbuf(const char *buf,uint16_t len);
uint8_t bt24_connect_status(void);
void bt24_clear_struct(bt24_t *data);

#endif