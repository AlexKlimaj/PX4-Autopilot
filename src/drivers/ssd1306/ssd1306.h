/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>


#include "ssd1306_spi.h"
#include "ssd1306_fonts.h"

#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))

// Header Values
#define JUMPTABLE_BYTES 4

#define JUMPTABLE_LSB   1
#define JUMPTABLE_SIZE  2
#define JUMPTABLE_WIDTH 3
#define JUMPTABLE_START 4

#define WIDTH_POS 0
#define HEIGHT_POS 1
#define FIRST_CHAR_POS 2
#define CHAR_NUM_POS 3

// Display commands
#define CHARGEPUMP 0x8D
#define COLUMNADDR 0x21
#define COMSCANDEC 0xC8
#define COMSCANINC 0xC0
#define DISPLAYALLON 0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 0xAE
#define DISPLAYON 0xAF
#define EXTERNALVCC 0x1
#define INVERTDISPLAY 0xA7
#define MEMORYMODE 0x20
#define NORMALDISPLAY 0xA6
#define PAGEADDR 0x22
#define SEGREMAP 0xA0
#define SETCOMPINS 0xDA
#define SETCONTRAST 0x81
#define SETDISPLAYCLOCKDIV 0xD5
#define SETDISPLAYOFFSET 0xD3
#define SETHIGHCOLUMN 0x10
#define SETLOWCOLUMN 0x00
#define SETMULTIPLEX 0xA8
#define SETPRECHARGE 0xD9
#define SETSEGMENTREMAP 0xA1
#define SETSTARTLINE 0x40
#define SETVCOMDETECT 0xDB
#define SWITCHCAPVCC 0x2

enum OLEDDISPLAY_TEXT_ALIGNMENT {
	TEXT_ALIGN_LEFT = 0,
	TEXT_ALIGN_RIGHT = 1,
	TEXT_ALIGN_CENTER = 2,
	TEXT_ALIGN_CENTER_BOTH = 3
};

enum OLEDDISPLAY_GEOMETRY {
	GEOMETRY_128_64   = 0,
	GEOMETRY_128_32,
};

typedef char (*FontTableLookupFunction)(const uint8_t ch);
char DefaultFontTableLookup(const uint8_t ch);


class SSD1306 : public I2CSPIDriver<SSD1306>
{
public:
	SSD1306(I2CSPIBusOption bus_option, const int bus, SSD1306_SPI* interface);
	virtual ~SSD1306();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

private:

	void updateStatus(const battery_status_s& data);

	// Driver specific
	void sendCommand(uint8_t command);

	int init();
	void sendInitCommands();
	void resetDisplay(void);

	void display(void);

	void setGeometry(OLEDDISPLAY_GEOMETRY g, uint16_t width = 0, uint16_t height = 0);

	void drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData);
	void drawStringInternal(int16_t xMove, int16_t yMove, const char* text, uint16_t textLength, uint16_t textWidth);

	void drawString(int16_t x, int16_t y, const char* text);
	uint16_t getStringWidth(const char* text, uint16_t length);

	// Sets the current font. Available default fonts:
	// ArialMT_Plain_10, ArialMT_Plain_16, ArialMT_Plain_24
	void setFont(const uint8_t *fontData);
	void setFontTableLookupFunction(FontTableLookupFunction function);

	void clear(void);
	void displayOn(void);
	void displayOff(void);
	void flipScreenVertically();


	SSD1306_SPI* _interface {};

	uint8_t* _buffer {};

	OLEDDISPLAY_GEOMETRY _geometry = GEOMETRY_128_32;
	OLEDDISPLAY_TEXT_ALIGNMENT _textAlignment = TEXT_ALIGN_LEFT;

	uint16_t  _displayWidth = 128;
	uint16_t  _displayHeight = 32;
	uint16_t  _displayBufferSize = 128*32/8;

	const uint8_t* _fontData = ArialMT_Plain_16; // pointer to the font data structure

	FontTableLookupFunction _fontTableLookupFunction = DefaultFontTableLookup;


	uORB::Subscription _battery_sub{ORB_ID(battery_status)};

};