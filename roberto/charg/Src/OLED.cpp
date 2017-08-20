
#include "OLED.h"
#include <cstring>


OLED::OLED(){

}
  

int OLED::fastI2Cwrite(uint8_t cmd, uint8_t* tbuf, uint32_t len) {
	//if (!m_settings->I2CWrite(_i2c_addr, cmd, len, tbuf, "Failed to write display data"))
	if (!HAL_I2C_Mem_Write(&hi2c1,_i2c_addr,cmd, 1, tbuf, len, 10))
        return false;
    return true;
}


bool OLED::init(uint8_t i2c_addr, uint8_t width, uint8_t height) {
	_i2c_addr = i2c_addr;

	available = true;
	uint8_t b = SSD1306_Normal_Display;
	
	if (!HAL_I2C_Mem_Write(&hi2c1,_i2c_addr,SSD_Command_Mode, 1, &b, sizeof(b), 10)){
		available = false;
		return false;
	}

	// Lcd size
	ssd1306_lcdwidth  = width;
	ssd1306_lcdheight = height;
	
	// Empty pointer to OLED buffer
	poledbuff = NULL;

	if (poledbuff)
		free(poledbuff);
		
	// Allocate memory for OLED buffer
	poledbuff = (uint8_t *) malloc ( (ssd1306_lcdwidth * ssd1306_lcdheight / 8 )); 
	return true;
}



void OLED::close(void){
	// De-Allocate memory for OLED buffer if any
	if(!available)
		return;

	if (poledbuff)
		free(poledbuff);
		
	poledbuff = NULL;
}
	
int OLED::begin(void){
	uint8_t multiplex;
	uint8_t chargepump;
	uint8_t compins;
	uint8_t contrast;
	uint8_t precharge;

	if(!available)
		return false;
	
	multiplex 	= 0x3F;
	compins 	= 0x12;
	contrast	= 0xCF;
	
	chargepump = 0x14;
	precharge  = 0xF1;
	
	ssd1306_command(SSD_Display_Off);                    // 0xAE
	ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV, 0x80);      // 0xD5 + the suggested ratio 0x80
	ssd1306_command(SSD1306_SETMULTIPLEX, multiplex); 
	ssd1306_command(SSD1306_SETDISPLAYOFFSET, 0x00);        // 0xD3 + no offset
	ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
	ssd1306_command(SSD1306_CHARGEPUMP, chargepump); 
	ssd1306_command(SSD1306_MEMORYMODE, 0x00);              // 0x20 0x0 act like ks0108
	ssd1306_command(SSD1306_SEGREMAP | 0x0);
	ssd1306_command(SSD1306_COMSCANDEC);
	ssd1306_command(SSD1306_SETCOMPINS, compins);  // 0xDA
	ssd1306_command(SSD_Set_ContrastLevel, contrast);
	ssd1306_command(SSD1306_SETPRECHARGE, precharge); // 0xd9
	ssd1306_command(SSD1306_SETVCOMDETECT, 0x40);  // 0xDB
	ssd1306_command(SSD1306_DISPLAYALLON_RESUME);    // 0xA4
	ssd1306_command(SSD1306_Normal_Display);         // 0xA6

	// Reset to default value in case of 
	// no reset pin available on OLED
	//ssd1306_command(0x21, 0, 127); 
	//ssd1306_command(0x22, 0,   7); 
	//stopscroll();
	
	// Empty uninitialized buffer
	clearDisplay();
	ssd1306_command(SSD_Display_On);							//--turn on oled panel*/
	return true;
}


void OLED::invertDisplay(uint8_t i) 
{
	if(!available)
		return;

	if (i) 
		ssd1306_command(SSD_Inverse_Display);
	else 
		ssd1306_command(SSD1306_Normal_Display);
}

void OLED::ssd1306_command(uint8_t c){ 
	// Write Data on I2C
	fastI2Cwrite(SSD_Command_Mode, &c, sizeof(c));
}

void OLED::ssd1306_command(uint8_t c0, uint8_t c1) { 
	unsigned char buff[2];

	buff[0] = c0;
	buff[1] = c1;


	// Write Data on I2C
	fastI2Cwrite(SSD_Command_Mode, buff, 2);
}

void OLED::ssd1306_command(uint8_t c0, uint8_t c1, uint8_t c2){ 
	unsigned char buff[3];
	buff[0] = c0;
	buff[1] = c1;
	buff[2] = c2;

	// Write Data on I2C
	fastI2Cwrite(SSD_Command_Mode, buff, sizeof(buff));
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
	// Write on i2c
	fastI2Cwrite(SSD_Data_Mode, &c, sizeof(c));
}

void OLED::display(void) {
	if(!available)
		return;

	ssd1306_command(SSD1306_SETLOWCOLUMN  | 0x0); // low col = 0
	ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0); // hi col = 0
	ssd1306_command(SSD1306_SETSTARTLINE  | 0x0); // line #0

	uint16_t i=0 ;
	
	// pointer to OLED data buffer
	uint8_t * p = poledbuff;

	unsigned char buff[16] ;
	uint8_t x ;

		// loop trough all OLED buffer and 
    // send a bunch of 16 data byte in one xmission
    for (i=0; i<(ssd1306_lcdwidth*ssd1306_lcdheight/8); i+=16){
    	for (x=0; x<16; x++) 
			buff[x] = *p++;
		fastI2Cwrite(SSD_Data_Mode, buff,  16);
    }
}

void OLED::setBuffer(uint8_t * buf) {
	if(!available)
		return;
	std::memcpy(poledbuff, buf, (ssd1306_lcdwidth*ssd1306_lcdheight/8));
}


// clear everything (in the buffer)
void OLED::clearDisplay(void) {
	if(!available)
		return;
	std::memset(poledbuff, 0, (ssd1306_lcdwidth*ssd1306_lcdheight/8));
}
