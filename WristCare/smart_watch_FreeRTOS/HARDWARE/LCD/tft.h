 /*作    者:粤嵌.温工*/
#ifndef __TFT_H
#define __TFT_H

#define LCD_WIDTH 240
#define LCD_HEIGHT 280

/* 部分tft屏需要偏移量 */
#define X_OFFSET	20
#define Y_OFFSET	20

#define LCD_SOFT_SPI_ENABLE  0
#define LCD_DMA_ENABLE		 1

#define RED 	0XF800   	// 红色
#define GREEN 	0X07E0 		// 绿色
#define BLUE 	0X001F  	// 蓝色
#define WHITE 	0XFFFF 		// 白色
#define BLACK 	0X0000 		// 黑色

#if LCD_SOFT_SPI_ENABLE

#define SPI_CS_0 PEout(13) = 0
#define SPI_CS_1 PEout(13) = 1

#define SPI_SCK_0 PEout(11) = 0
#define SPI_SCK_1 PEout(11) = 1

#define SPI_SDA_0 PEout(9) = 0
#define SPI_SDA_1 PEout(9) = 1

#define LCD_RST_0 PEout(7) = 0
#define LCD_RST_1 PEout(7) = 1

#define LCD_DC_0 PEout(15) = 0
#define LCD_DC_1 PEout(15) = 1

#define LCD_BLK_0 PDout(9) = 0
#define LCD_BLK_1 PDout(9) = 1

#else

#define SPI_CS_0 PGout(6) = 0
#define SPI_CS_1 PGout(6) = 1

#define SPI_SCK_0 PBout(15) = 0
#define SPI_SCK_1 PBout(15) = 1

#define SPI_SDA_0 PDout(10) = 0
#define SPI_SDA_1 PDout(10) = 1

#define LCD_RST_0 PGout(8) = 0
#define LCD_RST_1 PGout(8) = 1

#define LCD_DC_0 PGout(7) = 0
#define LCD_DC_1 PGout(7) = 1

#define LCD_BLK_0 PBout(4) = 0
#define LCD_BLK_1 PBout(4) = 1

#endif

extern uint16_t g_lcd_width ;
extern uint16_t g_lcd_height;

void lcd_init(void);
void lcd_set_brightness(uint8_t brightness);
void lcd_addr_set(uint32_t x_s, uint32_t y_s, uint32_t x_e, uint32_t y_e);
void lcd_fill(uint32_t x_s, uint32_t y_s, uint32_t x_len, uint32_t y_len,uint32_t color);
void lcd_clear(uint32_t color);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t dat);
void lcd_draw_point(uint32_t x, uint32_t y, uint32_t color);
void lcd_draw_picture(uint32_t x_s, uint32_t y_s, uint32_t width, uint32_t height, const uint8_t *pic);
void lcd_show_chn(uint32_t x, uint32_t y,uint8_t no, uint32_t fc, uint32_t bc,uint32_t font_size);
void lcd_show_string(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t font_size,uint8_t mode);
void lcd_show_integer(uint32_t x,uint32_t y,uint32_t num,uint32_t len,uint32_t fc,uint32_t bc,uint32_t font_size);
void lcd_show_float(uint32_t x,uint32_t y,float num,uint32_t len,uint32_t fc,uint32_t bc,uint32_t font_size);
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);
void lcd_draw_circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color);
void lcd_draw_line(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color);
void lcd_set_direction(uint32_t dir);
uint32_t lcd_get_direction(void);
void spi1_tx_dma_init(uint32_t DMA_Memory0BaseAddr, uint16_t DMA_BufferSize, uint32_t DMA_MemoryDataSize, uint32_t DMA_MemoryInc);
void spi1_tx_dma_start(void);
void spi1_tx_dma_stop(void);
#endif
