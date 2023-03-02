/*
 * lcd.h
 *
 *  Created on: Dec 5, 2022
 *      Author: aybuke.sokmen
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

/* SSD1306 width in pixels */
#ifndef lcd_width
#define lcd_width            128
#endif
/* SSD1306 LCD height in pixels */
#ifndef lcd_height
#define lcd_height          64
#endif


static uint8_t lcd_Buffer[lcd_width * lcd_height / 8];

typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} lcd_t;


/* Private variable */
static lcd_t SSD1306;


#define lcd_RIGHT_HORIZONTAL_SCROLL              0x26
#define lcd_LEFT_HORIZONTAL_SCROLL               0x27
#define lcd_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define lcd_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define lcd_DEACTIVATE_SCROLL                    0x2E // Stop scroll
#define lcd_ACTIVATE_SCROLL                      0x2F // Start scroll
#define lcd_SET_VERTICAL_SCROLL_AREA             0xA3 // Set scroll range

#define lcd_NORMALDISPLAY       0xA6
#define lcd_INVERTDISPLAY       0xA7




void lcd_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (
		x >= lcd_width ||
		y >= lcd_height
	) {
		/* Error */
		return;
	}

	/* Check if pixels are inverted */
	if (SSD1306.Inverted) {
		color = (SSD1306_COLOR_t)!color;
	}

	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer[x + (y / 8) * lcd_width] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * lcd_width] &= ~(1 << (y % 8));
	}
}



#endif /* INC_LCD_H_ */
