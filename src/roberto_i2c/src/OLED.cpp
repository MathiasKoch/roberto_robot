
#include "OLED_GFX.h"
#include "OLED.h"
#include "I2CBus.h"

  

inline void OLED::fastI2Cwrite(char* tbuf, uint32_t len) {
	_i2c.write(tbuf, len);
}

// the most basic function, set a single pixel
void OLED::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
	uint8_t * p = poledbuff ;
	
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
    return;

		// check rotation, move pixel around if necessary
	switch (getRotation()) 
	{
		case 1:
			swap(x, y);
			x = WIDTH - x - 1;
		break;
		
		case 2:
			x = WIDTH - x - 1;
			y = HEIGHT - y - 1;
		break;
		
		case 3:
			swap(x, y);
			y = HEIGHT - y - 1;
		break;
	}  

	// Get where to do the change in the buffer
	p = poledbuff + (x + (y/8)*ssd1306_lcdwidth );
	
	// x is which column
	if (color == WHITE) 
		*p |=  (1<<(y%8));  
	else
		*p &= ~(1<<(y%8)); 
}

void OLED::init(I2CBus i2c, int8_t i2c_addr, int8_t width, int8_t height) {

	_i2c = i2c;
	_i2c_addr = i2c_addr;
	// Lcd size
	ssd1306_lcdwidth  = width;
	ssd1306_lcdheight = height;
	
	// Empty pointer to OLED buffer
	poledbuff = NULL;

	if (poledbuff)
		free(poledbuff);
		
	// Allocate memory for OLED buffer
	poledbuff = (uint8_t *) malloc ( (ssd1306_lcdwidth * ssd1306_lcdheight / 8 )); 
}



void OLED::close(void){
	// De-Allocate memory for OLED buffer if any
	if (poledbuff)
		free(poledbuff);
		
	poledbuff = NULL;
}
	
void OLED::begin(void){
	uint8_t multiplex;
	uint8_t chargepump;
	uint8_t compins;
	uint8_t contrast;
	uint8_t precharge;

	constructor(ssd1306_lcdwidth, ssd1306_lcdheight);
	
	multiplex 	= 0x3F;
	compins 	= 0x12;
	contrast	= 0x9F;
	
	chargepump = 0x10;
	precharge  = 0x22;
	
	ssd1306_command(SSD_Display_Off);                    // 0xAE
	ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV, 0x80);      // 0xD5 + the suggested ratio 0x80
	ssd1306_command(SSD1306_SETMULTIPLEX, multiplex); 
	ssd1306_command(SSD1306_SETDISPLAYOFFSET, 0x00);        // 0xD3 + no offset
	ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
	ssd1306_command(SSD1306_CHARGEPUMP, chargepump); 
	ssd1306_command(SSD1306_MEMORYMODE, 0x00);              // 0x20 0x0 act like ks0108
	ssd1306_command(SSD1306_SEGREMAP | 0x1);
	ssd1306_command(SSD1306_COMSCANDEC);
	ssd1306_command(SSD1306_SETCOMPINS, compins);  // 0xDA
	ssd1306_command(SSD_Set_ContrastLevel, contrast);
	ssd1306_command(SSD1306_SETPRECHARGE, precharge); // 0xd9
	ssd1306_command(SSD1306_SETVCOMDETECT, 0x40);  // 0xDB
	ssd1306_command(SSD1306_DISPLAYALLON_RESUME);    // 0xA4
	ssd1306_command(SSD1306_Normal_Display);         // 0xA6

	// Reset to default value in case of 
	// no reset pin available on OLED
	ssd1306_command(0x21, 0, 127); 
	ssd1306_command(0x22, 0,   7); 
	stopscroll();
	
	// Empty uninitialized buffer
	clearDisplay();
	ssd1306_command(SSD_Display_On);							//--turn on oled panel
}


void OLED::invertDisplay(uint8_t i) 
{
  if (i) 
    ssd1306_command(SSD_Inverse_Display);
  else 
    ssd1306_command(SSD1306_Normal_Display);
}

void OLED::ssd1306_command(uint8_t c){ 
	char buff[2] ;

	// Clear D/C to switch to command mode
	buff[0] = SSD_Command_Mode ; 
	buff[1] = c;

	// Write Data on I2C
	fastI2Cwrite(buff, sizeof(buff));
}

void OLED::ssd1306_command(uint8_t c0, uint8_t c1) { 
	char buff[3];

	buff[0] = SSD_Command_Mode;
	buff[1] = c0;
	buff[2] = c1;


	// Write Data on I2C
	fastI2Cwrite(buff, 3);
}

void OLED::ssd1306_command(uint8_t c0, uint8_t c1, uint8_t c2){ 
	char buff[4];
	buff[0] = SSD_Command_Mode; 
	buff[1] = c0;
	buff[2] = c1;
	buff[3] = c2;

	// Write Data on I2C
	fastI2Cwrite(buff, sizeof(buff));
}


// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void OLED::startscrollright(uint8_t start, uint8_t stop){
	ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(0XFF);
	ssd1306_command(SSD_Activate_Scroll);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void OLED::startscrollleft(uint8_t start, uint8_t stop){
	ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(0XFF);
	ssd1306_command(SSD_Activate_Scroll);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void OLED::startscrolldiagright(uint8_t start, uint8_t stop){
	ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);	
	ssd1306_command(0X00);
	ssd1306_command(ssd1306_lcdheight);
	ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(SSD_Activate_Scroll);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void OLED::startscrolldiagleft(uint8_t start, uint8_t stop){
	ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);	
	ssd1306_command(0X00);
	ssd1306_command(ssd1306_lcdheight);
	ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(SSD_Activate_Scroll);
}

void OLED::stopscroll(void){
	ssd1306_command(SSD_Deactivate_Scroll);
}

void OLED::ssd1306_data(uint8_t c){
	char buff[2] ;
	
	// Setup D/C to switch to data mode
	buff[0] = SSD_Data_Mode; 
	buff[1] = c;

	// Write on i2c
	fastI2Cwrite(buff, sizeof(buff));
}

void OLED::display(void) {
	ssd1306_command(SSD1306_SETLOWCOLUMN  | 0x0); // low col = 0
	ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0); // hi col = 0
	ssd1306_command(SSD1306_SETSTARTLINE  | 0x0); // line #0

	uint16_t i=0 ;
	
	// pointer to OLED data buffer
	uint8_t * p = poledbuff;

	char buff[17] ;
	uint8_t x ;

	// Setup D/C to switch to data mode
	buff[0] = SSD_Data_Mode; 

		// loop trough all OLED buffer and 
    // send a bunch of 16 data byte in one xmission
    for (i=0; i<(ssd1306_lcdwidth*ssd1306_lcdheight/8); i+=16){
    	for (x=1; x<=16; x++) 
			buff[x] = *p++;
		fastI2Cwrite(buff,  17);
    }
}

// clear everything (in the buffer)
void OLED::clearDisplay(void) {
  memset(poledbuff, 0, (ssd1306_lcdwidth*ssd1306_lcdheight/8));
}
