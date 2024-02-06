/****************************************************************************
 *
 *   Copyright (c) 2020, 2021 PX4 Development Team. All rights reserved.
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

#include "NMEAParser.hpp"
#include <cstdio>

NMEAParser::NMEAParser()
    : _buffer_length(0)
{}

NMEAParser::~NMEAParser()
{}

int NMEAParser::parse(const uint8_t* buffer, int length)
{
    // Append new data to the internal buffer
    if (_buffer_length + length < NMEA_PARSER_BUFFER_SIZE) {
        memcpy(_buffer + _buffer_length, buffer, length);
        _buffer_length += length;
    } else {
        // std::cout << "overflow" << std::endl;
        // Handle buffer overflow here.
    }

    // Process the buffer and return the number of messages parsed
    return processBuffer();
}

const char* findCharInArray(const char* start, char c, int length)
{
    for (int i = 0; i < length; i++) {
        if (start[i] == c) {
            return &start[i];
        }
    }
    return nullptr;
}

int NMEAParser::processBuffer()
{
    int messagesParsed = 0;
    int startPos = 0;
    int bytes_remaining = _buffer_length;

    while (1) {
        // Find the start ($) of an NMEA message, handling data with NULL values
        const char* start = findCharInArray(_buffer + startPos, '$', bytes_remaining);

        if (bytes_remaining == 0) {
            break;
        }

        if (!start) {
            // std::cout << "missingstart: bytes_remaining:" << bytes_remaining << std::endl;
            // std::string msg(_buffer + startPos, bytes_remaining);
            // std::cout << "missing start: " << msg.c_str() << std::endl;
            break;
        }

        // Find the end (*) of the NMEA message, starting from the found start position
        const char* end = findCharInArray(start, '*', _buffer + _buffer_length - start);
        if (!end) {
            // std::cout << "missing end" << std::endl;
            break; // No end found from the start position, or message is likely incomplete
        }

        // Found start and end of message
        int messageLength = end - start + 5; // * (1), crc (2), crlf (2)

        // $GNGGA,,,,,,0,00,0.0,,M,,M,,*56
        // auto strmsg = std::string(start, messageLength);
        // std::cout << strmsg << std::endl;
        // $ + data + * + checksum

        // Check if the length from start to the end of message including checksum and CRLF is within the buffer
        if (messageLength > bytes_remaining) {
            // std::cout << "incomplete message" << std::endl;
            break; // Incomplete message
        }

        if (verifyChecksum(start, messageLength)) {
            messagesParsed++;

            // TODO: handle_message()

            // Update startPos for the next message, including CRLF
            startPos = (start - _buffer) + messageLength; // +3 for '*CS',, +2 for CRLF

        } else {
            // std::cout << "Invalid checksum" << std::endl;
            // If checksum is invalid or message incomplete, move startPos just after this '$'
            startPos = (start - _buffer) + 1;
        }

        bytes_remaining = _buffer_length - startPos;
    }

    // Shift remaining data to the beginning of the buffer
    if (startPos < _buffer_length) {
        memmove(_buffer, _buffer + startPos, bytes_remaining);
        _buffer_length -= startPos;

        // std::cout << "bytes_remaining:" << bytes_remaining << std::endl;

        for (int i = 0; i < _buffer_length; i++) {
            printf("%c", _buffer[i]);
        }
        printf("\n\n");

    } else {
        _buffer_length = 0;
    }

    return messagesParsed;
}


bool NMEAParser::verifyChecksum(const char* nmeaMessage, int length)
{
    const char* asteriskPos = strchr(nmeaMessage, '*');

    unsigned char calculatedChecksum = 0;
    for (const char* p = nmeaMessage + 1; p < asteriskPos; p++) {
        calculatedChecksum ^= *p;
    }

    char receivedChecksumStr[3] = {asteriskPos[1], asteriskPos[2], '\0'};
    int messageChecksum = 0;
    sscanf(receivedChecksumStr, "%X", &messageChecksum);

    return messageChecksum == calculatedChecksum;
}


/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */

// int NMEAParser::handleMessage(int len)
// {
// 	// why???
// 	if (len < 7) {
// 		return 0;
// 	}

// 	// For each message type a certain number of commas are expected
// 	int comma_count = 0;

// 	for (int i = 0 ; i < len; i++) {
// 		if (_rx_buffer[i] == ',') { comma_count++; }
// 	}

// 	// buffer pointer used for iteration over buffer
// 	char *bufptr = (char *)(_rx_buffer + 6);
// 	int ret = PX4_OK;

// 	if ((memcmp(_rx_buffer + 3, "ZDA,", 4) == 0) && (comma_count == 6)) {
// 		handle_ZDA(bufptr);

// 	} else if ((memcmp(_rx_buffer + 3, "GGA,", 4) == 0) && (comma_count >= 14)) {
// 		handle_GGA(bufptr);

// 	} else if (memcmp(_rx_buffer + 3, "HDT,", 4) == 0 && comma_count == 2) {
// 		handle_HDT(bufptr);

// 	} else if ((memcmp(_rx_buffer + 3, "GNS,", 4) == 0) && (comma_count >= 12)) {
// 		handle_GNS(bufptr);

// 	} else if ((memcmp(_rx_buffer + 3, "RMC,", 4) == 0) && (comma_count >= 11)) {
// 		handle_RMC(bufptr);

// 	} else if ((memcmp(_rx_buffer + 3, "GST,", 4) == 0) && (comma_count == 8)) {
// 		handle_GST(bufptr);

// 	} else if ((memcmp(_rx_buffer + 3, "GSA,", 4) == 0) && (comma_count >= 17)) {
// 		handle_GSA(bufptr);

// 	} else if ((memcmp(_rx_buffer + 3, "GSV,", 4) == 0)) {
// 		handle_GSV(bufptr);

// 	} else if ((memcmp(_rx_buffer + 3, "VTG,", 4) == 0) && (comma_count >= 8)) {
// 		handle_VTG(bufptr);
// 	}


// 	// Whats down here?

// 	if (_sat_num_gga > 0) {
// 		_gps_position->satellites_used = _sat_num_gga;

// 	} else if (_SVNUM_received && _SVINFO_received && _FIX_received) {

// 		_sat_num_gsv = _sat_num_gpgsv + _sat_num_glgsv + _sat_num_gagsv
// 			       + _sat_num_gbgsv + _sat_num_bdgsv;
// 		_gps_position->satellites_used = MAX(_sat_num_gns, _sat_num_gsv);
// 	}

// 	if (_VEL_received && _POS_received) {
// 		ret = 1;
// 		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
// 		_clock_set = false;
// 		_VEL_received = false;
// 		_POS_received = false;
// 		_rate_count_vel++;
// 		_rate_count_lat_lon++;
// 	}

// 	return ret;
// }