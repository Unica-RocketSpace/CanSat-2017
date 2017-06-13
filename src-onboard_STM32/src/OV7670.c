/*
 * Ov7670.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: arndtjenssen
 */
#include <stdbool.h>

#include "OV7670.h"
#include "i2c.h"

OV7670_struct OV7670 =
		{
				.capture_request = false,
				.capture_done = true,
				.busy = false,
				.init_success = false,
				.line_counter = 0,
				.last_line_counter = 0,
				.camera_mode = MODE_RGB565,
				.bufferPos = 0,
				//int bufferFullFunctionPtr = 0;
				//int readImageStartFunctionPtr = 0;
				//int readImageStopFunctionPtr = 0;
				.edgeEnhacementEnabled = false,
				.denoiseEnabled = false,
				.buffer = {0}
		};

/*
void OV7670_setSerial(HardwareSerial *s) {
	OV7670.serial = s;
}
*/
/*void OV7670_nightMode(bool enable) {
	if (enable) {
		i2c_write(OV7670_I2C_ADDR, REG_COM11, (uint8_t*)(COM11_EXP | COM11_HZAUTO), 1);
		i2c_write(OV7670_I2C_ADDR, REG_COM11, (uint8_t*)(COM11_EXP|COM11_HZAUTO|COM11_NIGHT|COM11_NIGHT_FR8), 1);
	} else {
		i2c_write(OV7670_I2C_ADDR, REG_COM11, (uint8_t*)(COM11_EXP|COM11_HZAUTO|COM11_NIGHT|COM11_NIGHT_FR8), 1);
		i2c_write(OV7670_I2C_ADDR, REG_COM11, (uint8_t*)(COM11_EXP|COM11_HZAUTO), 1);
	}
}

// -2 (low contrast ) to +2 (high contrast)
void OV7670_contrast(int8_t value) {
	static const uint8_t values[] = {0x60, 0x50, 0x40, 0x38, 0x30};

	value = min(max((value + 2), 0), 4);
	i2c_write(OV7670_I2C_ADDR, REG_CONTRAST, *values[value], 1);
}

// -2 (dark) to +2 (bright)
void OV7670_brightness(int8_t value) {
	static const uint8_t values[] = {0xb0, 0x98, 0x00, 0x18, 0x30};

	value = min(max((value + 2), 0), 4);
	i2c_write(OV7670_I2C_ADDR, REG_BRIGHT, values[value]);
}

// 0 - Normal, 1 - Antique, 2 - BluOV7670_read_one_byteish, 3 - Greenish
// 4 - Redish, 5 - B&W, 6 - Negative, 7 - B&W negative
void OV7670_specialEffect(uint8_t value) {
	value = min(max((value + 2), 0), 4);
	transfer_regvals(ov7670_effects[value]);
}

// 0 to disable, > 0 enable and set edge enhancement factor
void OV7670_edgeEnhancement(uint8_t value) {
	uint8_t v = COM16_AWBGAIN | (OV7670.denoiseEnabled ? COM16_DENOISE : 0);

	if (value == 0) {
		i2c_write(OV7670_I2C_ADDR, REG_COM16, v);
		i2c_write(OV7670_I2C_ADDR, REG_EDGE, 0);
	} else {
		i2c_write(OV7670_I2C_ADDR, REG_COM16, v | COM16_EDGE);
		i2c_write(OV7670_I2C_ADDR, REG_EDGE, value);
	}
	OV7670.edgeEnhacementEnabled = (value > 0);
}

// 0 to disable, > 0 enable and set denoise factor
void OV7670_denoise(uint8_t value) {
	uint8_t v = COM16_AWBGAIN | (OV7670.edgeEnhacementEnabled ? COM16_EDGE : 0);

	if (value == 0) {
		i2c_write(OV7670_I2C_ADDR, REG_COM16, v);
		i2c_write(OV7670_I2C_ADDR, REG_EDGE, 0);
	} else {
		i2c_write(OV7670_I2C_ADDR, REG_COM16, v | COM16_DENOISE);
		i2c_write(OV7670_I2C_ADDR, REG_DENOISE_STRENGTH, value);
	}
	OV7670.denoiseEnabled = (value > 0);
}
*/

/*
void OV7670_vsync_handler() {
	if (OV7670.capture_request) {
		WRITE_RESET;
		WRITE_ENABLE;
		OV7670.capture_request = false;
		OV7670.capture_done = false;
	} else {
		WRITE_DISABLE;
		WRITE_RESET;
		if (OV7670.busy) {
			OV7670.busy = false;
			OV7670.capture_done = true;
		}
	}
	//last_line_counter = line_counter;
	//line_counter = 0;
}

void OV7670_href_handler() {
	OV7670.line_counter++;
}

void OV7670_capture_next() {
	OV7670.capture_request = true;
	OV7670.busy = true;
}

uint8_t OV7670_captured() {
	uint8_t result;
   if (OV7670.busy) {
       result = false ;
   } else {
       result = OV7670.capture_done;
       OV7670.capture_done = false;
   }
   return result ;
}

uint8_t OV7670_read_one_byte() {
	uint8_t b;

	READ_CLOCK_HIGH;
	b = DATA_PIN;
	READ_CLOCK_LOW;

	return b;
}

void OV7670_read_stop() {
	read_one_byte();
	READ_CLOCK_HIGH;
	READ_CLOCK_LOW;
}

void OV7670_capture_image() {
  capture_next();
  while(captured() == false);
  _delay_us(10);

OV7670.bufferPos = 0;
  if (readImageStartFunctionPtr) {
  	(*readImageStartFunctionPtr)();
  }

  READ_RESET;

  int r=0, g=0, b=0, d1 = 0, d2 = 0, d3 = 0, d4 = 0;
  uint16_t index = 0;

  // read image
  for (int y = 0; y < SIZEY; y++) {
  	for (int x = 0; x < SIZEX; x++) {

  		if (OV7670.camera_mode != MODE_YUV) {
    		d1 = read_one_byte();
    		d2 = read_one_byte();
  		}

  		switch (OV7670.camera_mode) {
  			case MODE_RGB444:
  	  		b = (d1 & 0x0F) << 4;
  				g = (d2 & 0xF0);
  				r = (d2 & 0x0F) << 4;
  				break;
  			case MODE_RGB555:
  	      b = (d1 & 0x1F) << 3;
  	      g = (((d1 & 0xE0) >> 2) | ((d2 & 0x03) << 6));
  	      r = (d2 & 0x7c) << 1;
  				break;
  			case MODE_RGB565:
  	      b = (d1 & 0x1F) << 3;
  	      g = (((d1 & 0xE0) >> 3) | ((d2 & 0x07) << 5));
  	      r = (d2 & 0xF8);
  				break;
  			case MODE_YUV:
  				if (index % 2 == 0) {
  	    		d1 = read_one_byte(); // U0
  	    		d2 = read_one_byte(); // Y0
    	  		d3 = read_one_byte(); // V0
    	  		d4 = read_one_byte(); // Y1

            // b = d2 + 1.77200 * (d1 - 128);
            // g = d2 - 0.34414 * (d1 - 128) - 0.71414 * (d3 - 128);
            // r = d2 + 1.40200 * (d3 - 128);

            b = d2 + 1.4075 * (d1 - 128);
            g = d2 - 0.3455 * (d1 - 128) - 0.7169 * (d3 - 128);
            r = d2 + 1.7790 * (d3 - 128);
  				} else {
            // b = d4 + 1.77200 * (d1 - 128);
            // g = d4 - 0.34414 * (d1 - 128) - 0.71414 * (d3 - 128);
            // r = d4 + 1.40200 * (d3 - 128);

            b = d4 + 1.4075 * (d1 - 128);
            g = d4 - 0.3455 * (d1 - 128) - 0.7169 * (d3 - 128);
            r = d4 + 1.7790 * (d3 - 128);
  				}

          b = min(max(b, 0), 255);
          g = min(max(g, 0), 255);
          r = min(max(r, 0), 255);

          index++;
  				break;
  		}

  		OV7670.buffer[OV7670.bufferPos] = r;
  		OV7670.buffer[OV7670.bufferPos + 1] = g;
  		OV7670.buffer[OV7670.bufferPos + 2] = b;
  		OV7670.bufferPos += 3;
  		if (OV7670.bufferPos >= BUFFER_SIZE) {
  			if (bufferFullFunctionPtr) {
  				(*bufferFullFunctionPtr)(OV7670.buffer);
  			}

  			OV7670.bufferPos = 0;
  		}
  	}
  }

  read_stop();

  if (readImageStopFunctionPtr) {
  	(*readImageStopFunctionPtr)();
  }

}*/


/**
 * transfers a regval list via SCCB to camera
 *
 * 1: success
 * other values: failure
 */
uint8_t OV7670_transfer_regvals(struct regval_list *list) {
	uint8_t ret = 0;
	uint8_t i = 0;

	for(;;) {
		// end marker check
		if ((list[i].reg_num == EM) && (list[i].value == EM)) {
			return 1;
		}

		ret = i2c_write(OV7670_I2C_ADDR, list[i].reg_num, &list[i].value, 1);
		if (!ret) {
			return i;
		}

		// delay for reset command
		if ((list[i].reg_num == REG_COM7) && (list[i].value == COM7_RESET)) {
			for (uint16_t f = 0; f < 65000; f++){}
		}

		i++;
	}

	return 0;
}


/*uint8_t OV7670_init_rgb444_qvga() {
	uint8_t ret = 0;

	//i2c_write(OV7670_I2C_ADDR, REG_COM7, COM7_RGB | COM7_QQVGA);
	ret = OV7670_transfer_regvals(ov7670_fmt_rgb444);
	if (ret != 1) return ret;

	return OV7670_transfer_regvals(ov7670_qqvga);
}

uint8_t OV7670_init_rgb555_qvga() {
	uint8_t ret = 0;

	//i2c_write(OV7670_I2C_ADDR, REG_COM7, COM7_RGB | COM7_QQVGA);
	ret = OV7670_transfer_regvals(ov7670_fmt_rgb555);
	if (ret != 1) return ret;

	return OV7670_transfer_regvals(ov7670_qqvga);
}
*/

uint8_t OV7670_init_rgb565_qvga() {
	uint8_t ret = 0;
	uint8_t reg_data = (COM7_RGB | COM7_QVGA);
	i2c_write(OV7670_I2C_ADDR, REG_COM7, &reg_data, 1);
	ret = OV7670_transfer_regvals(ov7670_fmt_rgb565);
	if (ret != 1) return ret;

	return OV7670_transfer_regvals(ov7670_qvga);
}

/*uint8_t OV7670_init_yuv_qvga() {
	uint8_t ret = 0;

	//i2c_write(OV7670_I2C_ADDR, REG_COM7, COM7_YUV);
	ret = OV7670_transfer_regvals(ov7670_fmt_yuv422);
	if (ret != 1) return ret;

	return OV7670_transfer_regvals(ov7670_qqvga);
}*/

/**
 * returns 1 if camera was initialized succesful
 * mode: MODE_RGB444, MODE_RGB555, MODE_RGB565, MODE_YUV
 */
uint8_t OV7670_reset(uint8_t mode) {
	uint8_t ret = 0;
	OV7670.camera_mode = mode;

	OV7670_init_camera_reset();

	switch (OV7670.camera_mode) {
	/*case MODE_RGB444:
		ret = OV7670_init_rgb444_qvga();
		if (ret != 1) return ret;
		break;
	case MODE_RGB555:
		ret = OV7670_init_rgb555_qvga();
		if (ret != 1) return ret;
		break;*/
	case MODE_RGB565:
		ret = OV7670_init_rgb565_qvga();
		if (ret != 1) return ret;
		break;
	/*case MODE_YUV:
		ret = OV7670_init_yuv_qvga();
		if (ret != 1) return ret;
		break;*/

	}

	OV7670_init_negative_vsync();			//зачем?
	ret = OV7670_init_default_values();		//зачем?

	return ret;
}

void OV7670_init() {
	OV7670.init_success = OV7670_reset(MODE_RGB565);
}


void OV7670_init_negative_vsync() {
	//i2c_write(OV7670_I2C_ADDR, REG_COM10, COM10_VS_NEG);
}

void OV7670_init_camera_reset()
{
	uint8_t reg_data = COM7_RESET;
	i2c_write(OV7670_I2C_ADDR, REG_COM7, &reg_data, 1);
	for (uint16_t f = 0; f < 65000; f++){}
}

uint8_t OV7670_init_default_values() {
	return OV7670_transfer_regvals(ov7670_default);
}
