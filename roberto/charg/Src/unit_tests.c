
#include "unit_tests.h"
#include "usbd_cdc_if.h"


void jump_testmode(){

}



void test_USB_TX(uint16_t cnt){
	uint8_t data[] = "R";
  	for(uint16_t i = 0; i < cnt; i++){ 
		USB_write(data,1);
  	}
}

void test_USB_RX(uint16_t cnt){
	uint8_t data[16];
	uint32_t nRead;
	uint16_t err;
	uint8_t result;
	uint32_t totalLen;
	uint32_t timeout = 10000;
	uint32_t t0, t1;
	while(1){
		// Wait with time-start until first byte is ready
		while(USB_available() <= 0);
		nRead = 0;
		err = 0;
		t0 = HAL_GetTick();
	  	while(nRead < cnt){// && (HAL_GetTick()-t0 < timeout)){
	  		totalLen = 16;
	  		data[0] = '.';
			result = USB_read(data, &totalLen);
			if(result == USBD_OK && totalLen != 0){
				nRead += totalLen;
			}
			//if(data[0] != "T"){
			//	err++;
			//}
	  	}
	  	t1 = HAL_GetTick();

	  	if(nRead > 0){
	  		// Not a timeout!
		  	uint32_t Bps = ((nRead*1000)/(t1-t0));

		  	uint8_t dataOut[16];
		  	snprintf(dataOut, 16, "S:%d", (int)Bps);
		  	USB_write(dataOut, sizeof(dataOut));
	  	}else{
	  		while(USB_available() > 0){
	  			totalLen = 16;
	  			USB_read(data, &totalLen);
	  		}
	  	}
	}
}

void test_USB_LOOP(void){
	uint8_t data[16];
	uint8_t result;
	uint32_t totalLen;
	while(1){
		totalLen = 16;
		result = USB_read(data, &totalLen);
		if(result == USBD_OK && totalLen != 0){
			USB_write(data, totalLen);
		}
	}
}