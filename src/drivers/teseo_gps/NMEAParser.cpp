/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <math.h>
#include <time.h>


// Append data to the internal buffer, process the buffer, and return the number of messages parsed.
int NMEAParser::parse(const uint8_t* buffer, unsigned length)
{
    if (_buffer_length + length < sizeof(_buffer)) {
        memcpy(_buffer + _buffer_length, buffer, length);
        _buffer_length += length;

    } else {
        PX4_INFO("buffer overflow -- clearing");
        _buffer_length = 0;
    }

    return process_buffer();
}

// Process the buffer and return the number of messages parsed.
int NMEAParser::process_buffer()
{
    int messages_parsed = 0;
    int start_pos = 0;
    int bytes_remaining = _buffer_length;
    int message_length = 0;

    while (bytes_remaining != 0) {

        bool found_message_frame = false;
        const char* start = nullptr;
        const char* end = nullptr;

        // Find the start ($) and end (*) an NMEA message
        for (int i = start_pos; i < _buffer_length; i++) {
            if (_buffer[i] == '$') {
                start = &_buffer[i];

            } else if (_buffer[i] == '*') {
                end = &_buffer[i];
            }

            message_length = end - start + 5;

            if (start && end && end > start && message_length <= bytes_remaining) {
                found_message_frame = true;
                break;
            }
        }

        if (!found_message_frame) {
            // 1. No start or end characters found
            // 2. No start character found _before_ an end character
            // 3. Start found but not end (incomplete, most common)
            // 4. Start and end found but we may be missing the checksum and CR LF bytes
            break;
        }

        if (validate_checksum(start, message_length)) {
            messages_parsed++;

            // Updates GNSS struct
            handle_nmea_message(start, message_length);

            // Increment to the start of next expected message
            start_pos = (start - _buffer) + message_length;

        } else {
            PX4_INFO("Invalid checksum");
            // If checksum is invalid or message incomplete, move start_pos just after this '$'
            start_pos = (start - _buffer) + 1;
        }

        bytes_remaining = _buffer_length - start_pos;
    }

    // If buffer iterator start_pos isn't pointing to the end of the buffer, shift remaining data to the beginning of the buffer
    if (start_pos < _buffer_length) {
        memmove(_buffer, _buffer + start_pos, bytes_remaining);
        _buffer_length -= start_pos;

#if defined(DEBUG_BUILD)
        PX4_INFO("Incomplete message");
        PX4_INFO("bytes_remaining: %d", bytes_remaining);

        for (size_t i = 0; i < _buffer_length; i++) {
            printf("%c", _buffer[i]);
        }

        printf("\n\n");
#endif

    } else {
        _buffer_length = 0;
    }

    return messages_parsed;
}

// Handles a NMEA message which has already been validated.
void NMEAParser::handle_nmea_message(const char* buffer, int length)
{
    // For each message type a certain number of commas are expected
    int comma_count = 0;

    for (int i = 0 ; i < length; i++) {
        if (buffer[i] == ',') {
            comma_count++;
        }
    }

    // The ID starts after the first 3 bytes ($GN)
    // The data starts after the first 6 bytes ($GNRMC)

    if ((memcmp(buffer + 3, "GGA,", 4) == 0) && (comma_count >= 14)) {
        _gga = handle_GGA(buffer + 6);

    } else if ((memcmp(buffer + 3, "RMC,", 4) == 0) && (comma_count >= 11)) {
        _rmc = handle_RMC(buffer + 6);

    } else if ((memcmp(buffer + 3, "GST,", 4) == 0) && (comma_count == 8)) {
        _gst = handle_GST(buffer + 6);

    } else if ((memcmp(buffer + 3, "GSA,", 4) == 0) && (comma_count >= 17)) {
        _gsa = handle_GSA(buffer + 6);

    } else if ((memcmp(buffer + 3, "VTG,", 4) == 0) && (comma_count >= 8)) {
        _vtg = handle_VTG(buffer + 6);

    } else if ((memcmp(buffer + 1, "PSTM,", 5) == 0)) {
        PX4_INFO("Got PSTM return: %s", buffer);

        // TODO: GSV, GBS
    } else {
        char msg[4];
        memcpy(msg, buffer + 3, 3);
        msg[3] = '\0';
        PX4_INFO("unknown message: %s", msg);
    }
}

void NMEAParser::send_test_command()
{
// Reset to defaults
// Enable SBAS and report SBAS
// Enables Galileo and BeiDou
// Set baudrate to 115200
// Set fix rate to 10Hz
// save params
// reset to apply

// $PSTMRESTOREPAR
// $PSTMSETPAR,1200,0x1000000C,1
// $PSTMSETPAR,1227,0x3C0,1
// $PSTMSETPAR,1102,0xA
// $PSTMSETPAR,1303,0.1
// $PSTMSAVEPAR
// $PSTMSRR


}

bool NMEAParser::validate_checksum(const char* nmea_message, int length)
{
    // Example --  $GNGSA,A,1,,,,,,,,,,,,,0.0,0.0,0.0,4*36<CR><LF>
    int recv_checksum = 0;
    unsigned char calc_checksum = 0;
    char recv_checksum_str[3] = {nmea_message[length - 4], nmea_message[length - 3], '\0'};
    sscanf(recv_checksum_str, "%X", &recv_checksum);

    // Start after $ and end before *
    for (int i = 1; i < length - 5; i++) {
        calc_checksum ^= nmea_message[i];
    }

    return recv_checksum == calc_checksum;
}
