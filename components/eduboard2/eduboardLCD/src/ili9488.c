#include "../../eduboard2.h"
#include <driver/gpio.h>
#include "ili9488.h"

#define TAG "ili9488"

#ifdef CONFIG_LCD_ILI9488

// #define SPI_BUFFER_MAXLENGTH 2880	//1440
#define SPI_BUFFER_MAXLENGTH 3072	//1440
#define COLORS_MAXLENGTH SPI_BUFFER_MAXLENGTH/3

static const int SPI_Command_Mode = 0;
static const int SPI_Data_Mode = 1;
//static const int SPI_Frequency = SPI_MASTER_FREQ_10M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_20M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_26M;
static const int SPI_Frequency = SPI_MASTER_FREQ_40M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_80M;

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

TFT_t * lcddevice = NULL;

void ili9488_spi_master_init(TFT_t * dev, int16_t PIN_MOSI, int16_t PIN_SCLK, int16_t PIN_CS, int16_t PIN_DC, int16_t PIN_RESET, int16_t PIN_BL)
{
	//ESP_LOGI(TAG, "GPIO_DC=%d",GPIO_DC);
	gpio_reset_pin( PIN_DC );
	gpio_set_direction( PIN_DC, GPIO_MODE_OUTPUT );
	gpio_set_level( PIN_DC, 0 );

	//ESP_LOGI(TAG, "GPIO_RESET=%d",GPIO_RESET);
	if ( PIN_RESET >= 0 ) {
		//gpio_pad_select_gpio( GPIO_RESET );
		gpio_reset_pin( PIN_RESET );
		gpio_set_direction( PIN_RESET, GPIO_MODE_OUTPUT );
		gpio_set_level( PIN_RESET, 1 );
		delayMS(50);
		gpio_set_level( PIN_RESET, 0 );
		delayMS(50);
		gpio_set_level( PIN_RESET, 1 );
		delayMS(50);
	}

	//ESP_LOGI(TAG, "GPIO_BL=%d",GPIO_BL);
	if ( PIN_BL >= 0 ) {
		gpio_reset_pin(PIN_BL);
		gpio_set_direction( PIN_BL, GPIO_MODE_OUTPUT );
		gpio_set_level( PIN_BL, 0 );
	}

	gpspi_init(&dev->_SPIHandle, PIN_MOSI, -1, PIN_SCLK, PIN_CS, SPI_Frequency, true);
	dev->_dc = PIN_DC;
	dev->_bl = PIN_BL;
}

bool ili9488_spi_write_cmd_data(uint8_t data) {
	gpio_set_level( lcddevice->_dc, SPI_Data_Mode );
	return gpspi_write_data(&lcddevice->_SPIHandle, &data, 1);
}
bool ili9488_spi_write_cmd(uint8_t cmd) {
	gpio_set_level( lcddevice->_dc, SPI_Command_Mode );
	return gpspi_write_data(&lcddevice->_SPIHandle, &cmd, 1);
}


bool IRAM_ATTR ili9488_spi_write_colors(uint16_t *colors565, uint32_t length) {
	uint8_t colors666[length*3];
	for(uint32_t i = 0; i < length; i++) {
		colors666[(i*3)+0] = (0x1F&(colors565[i]>>11))*2;
		colors666[(i*3)+0] <<= 2;
		colors666[(i*3)+1] = (0x3F&(colors565[i]>>5));
		colors666[(i*3)+1] <<= 2;
		colors666[(i*3)+2] = (0x1F&(colors565[i]))*2;
		colors666[(i*3)+2] <<= 2;
	}
	if(length > COLORS_MAXLENGTH) {
		ESP_LOGE(TAG, "bufferlength too long!");
		return false;
	}
	gpio_set_level( lcddevice->_dc, SPI_Data_Mode );
	return gpspi_write_data(&lcddevice->_SPIHandle, &colors666[0], length * 3);
}
bool ili9488_lcd_setpos(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2) {
	ili9488_spi_write_cmd(ILI9488_CMD_COLUMN_ADDRESS_SET);
	ili9488_spi_write_cmd_data(x1>>8);
	ili9488_spi_write_cmd_data(x1&0xff);
	ili9488_spi_write_cmd_data(x2>>8);
	ili9488_spi_write_cmd_data(x2&0xff);
	ili9488_spi_write_cmd(ILI9488_CMD_PAGE_ADDRESS_SET);
	ili9488_spi_write_cmd_data(y1>>8);
	ili9488_spi_write_cmd_data(y1&0xff);
	ili9488_spi_write_cmd_data(y2>>8);
	ili9488_spi_write_cmd_data(y2&0xff);

    ili9488_spi_write_cmd(ILI9488_CMD_MEMORY_WRITE);
	return true;
}

void ili9488_init(TFT_t * dev, int width, int height, int offsetx, int offsety)
{
	lcddevice = dev;
	lcddevice->_width = width;
	lcddevice->_height = height;
	lcddevice->_offsetx = offsetx;
	lcddevice->_offsety = offsety;
	lcddevice->_font_direction = DIRECTION0;
	lcddevice->_font_fill = false;
	lcddevice->_font_underline = false;

	ESP_LOGI(TAG, "ILI9488 initialization.");

  	ili9488_spi_write_cmd(ILI9488_CMD_SOFTWARE_RESET);  //Exit Sleep
	vTaskDelay(100 / portTICK_PERIOD_MS);

	ili9488_spi_write_cmd(ILI9488_CMD_POSITIVE_GAMMA_CORRECTION);
	ili9488_spi_write_cmd_data(0x00);
	ili9488_spi_write_cmd_data(0x03);
	ili9488_spi_write_cmd_data(0x09);
	ili9488_spi_write_cmd_data(0x08);
	ili9488_spi_write_cmd_data(0x16);
	ili9488_spi_write_cmd_data(0x0A);
	ili9488_spi_write_cmd_data(0x3F);
	ili9488_spi_write_cmd_data(0x78);
	ili9488_spi_write_cmd_data(0x4C);
	ili9488_spi_write_cmd_data(0x09);
	ili9488_spi_write_cmd_data(0x0A);
	ili9488_spi_write_cmd_data(0x08);
	ili9488_spi_write_cmd_data(0x16);
	ili9488_spi_write_cmd_data(0x1A);
	ili9488_spi_write_cmd_data(0x0F);
	

	ili9488_spi_write_cmd(ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION);
	ili9488_spi_write_cmd_data(0x00);
	ili9488_spi_write_cmd_data(0x16);
	ili9488_spi_write_cmd_data(0x19);
	ili9488_spi_write_cmd_data(0x03);
	ili9488_spi_write_cmd_data(0x0F);
	ili9488_spi_write_cmd_data(0x05);
	ili9488_spi_write_cmd_data(0x32);
	ili9488_spi_write_cmd_data(0x45);
	ili9488_spi_write_cmd_data(0x46);
	ili9488_spi_write_cmd_data(0x04);
	ili9488_spi_write_cmd_data(0x0E);
	ili9488_spi_write_cmd_data(0x0D);
	ili9488_spi_write_cmd_data(0x35);
	ili9488_spi_write_cmd_data(0x37);
	ili9488_spi_write_cmd_data(0x0F);
	
	
	 
	ili9488_spi_write_cmd(ILI9488_CMD_POWER_CONTROL_1);      //Power Control 1 
	ili9488_spi_write_cmd_data(0x17);    //Vreg1out 
	ili9488_spi_write_cmd_data(0x15);    //Verg2out 
	 
	ili9488_spi_write_cmd(ILI9488_CMD_POWER_CONTROL_2);      //Power Control 2     
	ili9488_spi_write_cmd_data(0x41);    //VGH,VGL 
	 
	ili9488_spi_write_cmd(ILI9488_CMD_VCOM_CONTROL_1);      //Power Control 3 
	ili9488_spi_write_cmd_data(0x00); 
	ili9488_spi_write_cmd_data(0x12);    //Vcom 
	ili9488_spi_write_cmd_data(0x80);
	 
	ili9488_spi_write_cmd(ILI9488_CMD_MEMORY_ACCESS_CONTROL);      //Memory Access 
	ili9488_spi_write_cmd_data(0x48); 
	 
	ili9488_spi_write_cmd(ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET);      // Interface Pixel Format 
	ili9488_spi_write_cmd_data(0x66); 	  //18 bit    
	//ili9488_spi_write_cmd_data(0x55); 	  //16 bit    
	 
	ili9488_spi_write_cmd(ILI9488_CMD_INTERFACE_MODE_CONTROL);      // Interface Mode Control 
	ili9488_spi_write_cmd_data(0x80);     			 //SDO NOT USE
	 
	ili9488_spi_write_cmd(ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL);      //Frame rate 
	ili9488_spi_write_cmd_data(0xA0);    //60Hz 
	 
	ili9488_spi_write_cmd(ILI9488_CMD_DISPLAY_INVERSION_CONTROL);      //Display Inversion Control 
	ili9488_spi_write_cmd_data(0x02);    //2-dot 
	 
	ili9488_spi_write_cmd(ILI9488_CMD_DISPLAY_FUNCTION_CONTROL);      //Display Function Control  RGB/MCU Interface Control 
	ili9488_spi_write_cmd_data(0x02);    //MCU 
	ili9488_spi_write_cmd_data(0x02);    //Source,Gate scan dieection 
	  
	ili9488_spi_write_cmd(ILI9488_CMD_SET_IMAGE_FUNCTION);      // Set Image Functio
	ili9488_spi_write_cmd_data(0x00);    // Disable 24 bit data
	 
	ili9488_spi_write_cmd(ILI9488_CMD_ADJUST_CONTROL_3);      // Adjust Control 
	ili9488_spi_write_cmd_data(0xA9);     
	ili9488_spi_write_cmd_data(0x51);     
	ili9488_spi_write_cmd_data(0x2C);     
	ili9488_spi_write_cmd_data(0x82);    // D7 stream, loose 

	ili9488_spi_write_cmd(ILI9488_CMD_SLEEP_OUT); //Sleep out
	vTaskDelay(120 / portTICK_PERIOD_MS);
	ili9488_spi_write_cmd(ILI9488_CMD_DISPLAY_ON);
  
	///Enable backlight
	ESP_LOGI(TAG, "Enable backlight.");
	if(lcddevice->_bl >= 0) {
		gpio_set_level( lcddevice->_bl, 1 );
	}
}

void ili9488_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
	if (x >= lcddevice->_width) return;
	if (y >= lcddevice->_height) return;

	
	uint16_t _x = x + lcddevice->_offsetx;
	uint16_t _y = y + lcddevice->_offsety;
	
	ili9488_lcd_setpos(_x,_x,_y,_y);
	uint16_t colors[1] = {color};
	ili9488_spi_write_colors(&colors[0], 1);
}

void ili9488_DrawMultiPixels(uint16_t x, uint16_t y, uint16_t size, uint16_t * colors) {
	if (x+size > lcddevice->_width) return;
	if (y >= lcddevice->_height) return;
	
	uint16_t _x1 = x + lcddevice->_offsetx;
	uint16_t _x2 = _x1 + size;
	uint16_t _y1 = y + lcddevice->_offsety;
	uint16_t _y2 = _y1;
	//uint16_t i = 0;
	ili9488_lcd_setpos(_x1, _x2, _y1, _y2);
	if(size <= COLORS_MAXLENGTH) {
		ili9488_spi_write_colors(&colors[0], size);
	} else {
		ESP_LOGE(TAG, "Size of Colors array too large");
	}
}

void ili9488_DrawArea(uint16_t x, uint16_t y, uint16_t size_x, uint16_t size_y, uint16_t * colors) 
{
	if (x+size_x-1 > lcddevice->_width) {ESP_LOGE(TAG, "ERROR"); return;}
	if (y+size_y-1 >= lcddevice->_height) {ESP_LOGE(TAG, "ERROR"); return;}
	uint16_t _x1 = x + lcddevice->_offsetx;
	uint16_t _x2 = _x1 + size_x-1;
	uint16_t _y1 = y + lcddevice->_offsety;
	uint16_t _y2 = _y1 + size_y-1;
	//uint16_t i = 0;
	ili9488_lcd_setpos(_x1, _x2, _y1, _y2);
	if(size_x * size_y <= COLORS_MAXLENGTH) {
		ili9488_spi_write_colors(&colors[0], size_x * size_y);
	} else {
		ESP_LOGE(TAG, "Size of Colors array too large");
	}
}

void ili9488_DrawMultiLines(uint16_t start_y, uint16_t lines, uint16_t * colors) {
	uint16_t _x1 = 0;
	uint16_t _x2 = lcddevice->_width-1;
	uint16_t _y1 = start_y; 
	uint16_t _y2 = start_y + lines - 1;
	ili9488_lcd_setpos(_x1, _x2, _y1, _y2);
	#define LINES_PER_UPDATE 3 //Maxed by Transfer-Buffer of SPI Unit
	for(int y = 0; y < lines; y+= LINES_PER_UPDATE) {
		uint32_t pos = y*lcddevice->_width;
		ili9488_spi_write_colors(&colors[pos], lcddevice->_width*LINES_PER_UPDATE);
	}
}

void ili9488_DrawFillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	if (x1 >= lcddevice->_width) {ESP_LOGE(TAG, "ERROR"); return;}
	if (x2 >= lcddevice->_width) x2=lcddevice->_width-1;
	if (y1 >= lcddevice->_height) {ESP_LOGE(TAG, "ERROR"); return;}
	if (y2 >= lcddevice->_height) y2=lcddevice->_height-1;

	uint16_t _x1 = x1 + lcddevice->_offsetx;
	uint16_t _x2 = x2 + lcddevice->_offsetx;
	uint16_t _y1 = y1 + lcddevice->_offsety;
	uint16_t _y2 = y2 + lcddevice->_offsety;
	ili9488_lcd_setpos(_x1, _x2, _y1, _y2);
	uint32_t size = (x2-x1+1) * (y2-y1+1);
	uint16_t colors[COLORS_MAXLENGTH];
	
	for(uint16_t i = 0; i < COLORS_MAXLENGTH; i++) {
		colors[i] = color;
	}
	while(size > COLORS_MAXLENGTH) {
		ili9488_spi_write_colors(&colors[0], COLORS_MAXLENGTH);
		size -= COLORS_MAXLENGTH;
	}
	ili9488_spi_write_colors(&colors[0], size);
}

// Display OFF
void ili9488_DisplayOff() {
	ili9488_spi_write_cmd(ILI9488_CMD_DISPLAY_OFF);	//Display off
}
 
// Display ON
void ili9488_DisplayOn() {
	ili9488_spi_write_cmd(ILI9488_CMD_DISPLAY_ON);	//Display on
}

void ili9488_InversionOff() {
	ili9488_spi_write_cmd(ILI9488_CMD_DISP_INVERSION_OFF);	//Display Inversion Off
}

// Display Inversion On
void ili9488_InversionOn() {
	ili9488_spi_write_cmd(ILI9488_CMD_DISP_INVERSION_ON);	//Display Inversion On
}

#endif