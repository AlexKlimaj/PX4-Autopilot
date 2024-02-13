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

#pragma once

#include <cstring>
#include "msgs/GGA.hpp"
#include "msgs/GSA.hpp"
#include "msgs/GST.hpp"
#include "msgs/RMC.hpp"
#include "msgs/VTG.hpp"

#include "px4_platform_common/log.h"


class NMEAParser
{
public:
    int parse(const uint8_t* buffer, unsigned length); // Returns the number of messages parsed

    RMC_Data* RMC() { return &_rmc; };
    GGA_Data* GGA() { return &_gga; };
    VTG_Data* VTG() { return &_vtg; };
    GST_Data* GST() { return &_gst; };
    GSA_Data* GSA() { return &_gsa; };

    void send_test_command();

private:
    // Parses an NMEA message and updates the NMEA data structures
    void handle_nmea_message(const char* buffer, int length);


    // Process the buffer and return the number of messages parsed
    int process_buffer();
    bool validate_checksum(const char* nmeaMessage, int length);

    static const int BUFFER_SIZE = 2048;
    char _buffer[2048] = {};
    int _buffer_length = 0;

    RMC_Data _rmc;
    GGA_Data _gga;
    VTG_Data _vtg;
    GST_Data _gst;
    GSA_Data _gsa;
    // GBS_Data _gbs;
};
