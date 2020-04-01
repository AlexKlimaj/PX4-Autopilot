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

#include "ssd1306.h"

namespace ssd1306
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// SPI IMPL /////////////////////////////////////////////////////////////////////////////////////////////////////
SSD1306_SPI::SSD1306_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode)
	: SPI("SSD1306_SPI", nullptr, bus, device, spi_mode, bus_frequency)
{
}

int SSD1306_SPI::init()
{
	return SPI::init();
}

int SSD1306_SPI::probe()
{
	printf("HES PROBING ME\n");
	return PX4_OK;
}

void SSD1306_SPI::writeBytes(uint8_t* data, size_t size)
{
	transfer(data, nullptr, size);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// SSD1306 IMPL /////////////////////////////////////////////////////////////////////////////////////////////////////
SSD1306_SPI* _interface {};

uint8_t* _buffer {};

OLEDDISPLAY_GEOMETRY _geometry = GEOMETRY_128_32;
OLEDDISPLAY_TEXT_ALIGNMENT _textAlignment = TEXT_ALIGN_LEFT;

static constexpr uint16_t DISPLAY_WIDTH = 128;
static constexpr uint16_t DISPLAY_HEIGHT = 32;
static constexpr uint16_t DISPLAY_BUFFER_SIZE = DISPLAY_WIDTH * DISPLAY_HEIGHT / 8;

const uint8_t* _fontData = ArialMT_Plain_16; // pointer to the font data structure

FontTableLookupFunction _fontTableLookupFunction = DefaultFontTableLookup;
int init()
{
	if (!_buffer)
	{
		_buffer = new uint8_t[DISPLAY_BUFFER_SIZE];

		if(!_buffer)
		{
			printf("Not enough memory to create display\n");
			return PX4_ERROR;
		}
	}

	// Now create SPI interface
	_interface = new SSD1306_SPI(1, 268435567, 2000000); // add args
	_interface->init();
	up_udelay(10000);

	// First reset the device
	stm32_gpiowrite(GPIO_SPI1_OLED_RST, false);
	up_udelay(10000);
	stm32_gpiowrite(GPIO_SPI1_OLED_RST, true);

	sendInitCommands();
	resetDisplay();

	up_udelay(10000);

	return PX4_OK;
}

int clean_up()
{
	delete[] _buffer;
	delete _interface;

	return OK;
}

void updateStatus(float v, float i, float soc)
{
	clear();

	char text_temp[20] = {};
	const char* str;

	snprintf(text_temp, sizeof(text_temp), "mV: %d", (int)(v*1000));
	str = text_temp;
	drawString(0, 0, str);

	// TODO: verify we are properly handling negatives here.
	snprintf(text_temp, sizeof(text_temp), "mA: %d", (int)(i*1000));

	str = text_temp;
	drawString(0, 16, str);

	snprintf(text_temp, sizeof(text_temp), "%d%%", (int)(soc*100));
	str = text_temp;
	drawString(85, 0, str);

	display();
}

void display(void)
{
	sendCommand(COLUMNADDR);
	sendCommand(0x0);
	sendCommand(0x7F);

	sendCommand(PAGEADDR);
	sendCommand(0x0);

	if (_geometry == GEOMETRY_128_64)
	{
		sendCommand(0x7);
	}
	else if (_geometry == GEOMETRY_128_32)
	{
		sendCommand(0x3);
	}

	sendData(_buffer, DISPLAY_BUFFER_SIZE);
}

void sendInitCommands(void)
{
	sendCommand(DISPLAYOFF);
	sendCommand(SETDISPLAYCLOCKDIV);
	sendCommand(0xF0); // Increase speed of the display max ~96Hz
	sendCommand(SETMULTIPLEX);
	sendCommand(DISPLAY_HEIGHT - 1);
	sendCommand(SETDISPLAYOFFSET);
	sendCommand(0x00);
	sendCommand(SETSTARTLINE);
	sendCommand(CHARGEPUMP);
	sendCommand(0x14);
	sendCommand(MEMORYMODE);
	sendCommand(0x00);
	sendCommand(SEGREMAP);
	sendCommand(COMSCANINC);
	sendCommand(SETCOMPINS);

	if (_geometry == GEOMETRY_128_64)
	{
		sendCommand(0x12);
	}
	else if (_geometry == GEOMETRY_128_32)
	{
		sendCommand(0x02);
	}

	sendCommand(SETCONTRAST);

	if (_geometry == GEOMETRY_128_64)
	{
		sendCommand(0xCF);
	}
	else if (_geometry == GEOMETRY_128_32)
	{
		sendCommand(0x8F);
	}

	sendCommand(SETPRECHARGE);
	sendCommand(0xF1);
	sendCommand(SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
	sendCommand(0x40);          //0x40 default, to lower the contrast, put 0
	sendCommand(DISPLAYALLON_RESUME);
	sendCommand(NORMALDISPLAY);
	sendCommand(0x2e);            // stop scroll
	sendCommand(DISPLAYON);
}

void drawString(int16_t xMove, int16_t yMove, const char* text)
{
	uint16_t lineHeight = pgm_read_byte(_fontData + HEIGHT_POS);

	uint16_t yOffset = 0;
	// If the string should be centered vertically too then we need to know how heigh the string is.
	if (_textAlignment == TEXT_ALIGN_CENTER_BOTH)
	{
		uint16_t lb = 0;
		// Find number of linebreaks in text
		for (uint16_t i=0;text[i] != 0; i++)
		{
			lb += (text[i] == 10);
		}

		// Calculate center
		yOffset = (lb * lineHeight) / 2;
	}

	uint16_t length = strlen(text);
	drawStringInternal(xMove, yMove - yOffset, text, length, getStringWidth(text, length));
}

void drawStringInternal(int16_t xMove, int16_t yMove, const char* text, uint16_t textLength, uint16_t textWidth)
{
	uint8_t textHeight       = pgm_read_byte(_fontData + HEIGHT_POS);
	uint8_t firstChar        = pgm_read_byte(_fontData + FIRST_CHAR_POS);
	uint16_t sizeOfJumpTable = pgm_read_byte(_fontData + CHAR_NUM_POS)  * JUMPTABLE_BYTES;

	uint16_t cursorX         = 0;
	uint16_t cursorY         = 0;

	switch (_textAlignment)
	{
		case TEXT_ALIGN_CENTER_BOTH:
			yMove -= textHeight >> 1;
		// Fallthrough
		case TEXT_ALIGN_CENTER:
			xMove -= textWidth >> 1; // divide by 2
			break;
		case TEXT_ALIGN_RIGHT:
			xMove -= textWidth;
			break;
		case TEXT_ALIGN_LEFT:
			break;
	}

	// Don't draw anything if it is not on the screen.
	if (xMove + textWidth  < 0 || xMove > DISPLAY_WIDTH )
	{
		return;
	}

	if (yMove + textHeight < 0 || yMove > DISPLAY_WIDTH )
	{
		return;
	}

	for (uint16_t j = 0; j < textLength; j++)
	{
		int16_t xPos = xMove + cursorX;
		int16_t yPos = yMove + cursorY;

		uint8_t code = text[j];
		if (code >= firstChar)
		{
			uint8_t charCode = code - firstChar;

			// 4 Bytes per char code
			uint8_t msbJumpToChar    = pgm_read_byte( _fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES );                  // MSB  \ JumpAddress
			uint8_t lsbJumpToChar    = pgm_read_byte( _fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB);   // LSB /
			uint8_t charByteSize     = pgm_read_byte( _fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE);  // Size
			uint8_t currentCharWidth = pgm_read_byte( _fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH); // Width

			// Test if the char is drawable
			if (!(msbJumpToChar == 255 && lsbJumpToChar == 255))
			{
				// Get the position of the char data
				uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
				drawInternal(xPos, yPos, currentCharWidth, textHeight, _fontData, charDataPosition, charByteSize);
			}

			cursorX += currentCharWidth;
		}
	}
}

void drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData)
{
	if (width < 0 || height < 0)
	{
		return;
	}
	if (yMove + height < 0 || yMove > DISPLAY_HEIGHT)
	{
		return;
	}
	if (xMove + width  < 0 || xMove > DISPLAY_WIDTH)
	{
		return;
	}

	uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
	int8_t   yOffset      = yMove & 7;

	bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

	int16_t initYMove   = yMove;
	int8_t  initYOffset = yOffset;


	for (uint16_t i = 0; i < bytesInData; i++)
	{
		// Reset if next horizontal drawing phase is started.
		if ( i % rasterHeight == 0)
		{
			yMove   = initYMove;
			yOffset = initYOffset;
		}

		uint8_t currentByte = pgm_read_byte(data + offset + i);

		int16_t xPos = xMove + (i / rasterHeight);
		int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * DISPLAY_WIDTH;

		int16_t dataPos    = xPos  + yPos;

		if (dataPos >=  0  && dataPos < DISPLAY_BUFFER_SIZE &&
			xPos    >=  0  && xPos    < DISPLAY_WIDTH )
		{

			if (yOffset >= 0)
			{
				_buffer[dataPos] |= currentByte << yOffset;

				if (dataPos < (DISPLAY_BUFFER_SIZE - DISPLAY_WIDTH))
				{
					_buffer[dataPos + DISPLAY_WIDTH] |= currentByte >> (8 - yOffset);
				}
			}
			else
			{
				// Make new offset position
				yOffset = -yOffset;

				_buffer[dataPos] |= currentByte >> yOffset;

				// Prepare for next iteration by moving one block up
				yMove -= 8;

				// and setting the new yOffset
				yOffset = 8 - yOffset;
			}
		}
	}
}

uint16_t getStringWidth(const char* text, uint16_t length)
{
	uint16_t firstChar = pgm_read_byte(_fontData + FIRST_CHAR_POS);
	uint16_t stringWidth = 0;
	uint16_t maxWidth = 0;

	while (length--)
	{
		stringWidth += pgm_read_byte(_fontData + JUMPTABLE_START + (text[length] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH);
		if (text[length] == 10)
		{
			maxWidth = (maxWidth > stringWidth) ? maxWidth : stringWidth;
			stringWidth = 0;
		}
	}

	return (maxWidth > stringWidth) ? maxWidth : stringWidth;
}

void sendCommand(uint8_t command)
{
	stm32_gpiowrite(GPIO_SPI1_OLED_DC, false); // deassert data control line
	_interface->writeBytes(&command, 1);
}

void sendData(uint8_t* data, size_t size)
{
	stm32_gpiowrite(GPIO_SPI1_OLED_DC, true); // assert data control line
	_interface->writeBytes(data, size);
}

void resetDisplay(void)
{
	clear();
	display();
	flipScreenVertically(); // We've got to flip it for it to be correct for our use
}

void flipScreenVertically()
{
	sendCommand(SEGREMAP | 0x01);
	sendCommand(COMSCANDEC); // Rotate screen 180 Deg
}

void displayOn(void)
{
	sendCommand(DISPLAYON);
}

void displayOff(void)
{
	sendCommand(DISPLAYOFF);
}

void clear(void)
{
	memset(_buffer, 0, DISPLAY_BUFFER_SIZE);
}

void setFontTableLookupFunction(FontTableLookupFunction function)
{
	_fontTableLookupFunction = function;
}

char DefaultFontTableLookup(const uint8_t ch)
{
	// UTF-8 to font table index converter
	// Code form http://playground.arduino.cc/Main/Utf8ascii
	static uint8_t LASTCHAR;

	if (ch < 128)
	{ // Standard ASCII-set 0..0x7F handling
		LASTCHAR = 0;
		return ch;
	}

	uint8_t last = LASTCHAR;   // get last char
	LASTCHAR = ch;

	switch (last)
	{    // conversion depnding on first UTF8-character
		case 0xC2: return (uint8_t) ch;
		case 0xC3: return (uint8_t) (ch | 0xC0);
		case 0x82: if (ch == 0xAC) return (uint8_t) 0x80;    // special case Euro-symbol
	}

	return (uint8_t) 0; // otherwise: return zero, if character has to be ignored
}

} // end namespace ssd1306