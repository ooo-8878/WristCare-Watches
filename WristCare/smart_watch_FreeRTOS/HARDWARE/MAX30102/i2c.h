#ifndef __I2C_H
#define __I2C_H

#define SCL_W	PCout(7)
#define SDA_W	PCout(9)
#define SDA_R	PCin(9)


uint8_t i2c_recv_byte(void);
void i2c_send_byte(uint8_t byte);
void i2c_stop(void);
void i2c_start(void);
void i2c_init(void);
uint8_t i2c_wait_ack(void);
void i2c_ack(uint8_t ack);

void IIC_ReadBytes(u8 deviceAddr, u8 writeAddr,u8* data,u8 dataLength);
void IIC_Read_One_Byte(u8 daddr,u8 addr,u8* data);
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);



#endif
