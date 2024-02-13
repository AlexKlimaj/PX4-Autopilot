#include <cstdlib>

struct GST_Data {
	// $<TalkerID>GST 	-- GN (multi-constellation), GP (GPS), GL (GLONASS), GA (GALILEO), BD (BEIDOU), QZ (QZSS)
	// <Timestamp> 		-- UTC time of GPS sample, hhmmss.sss
	// <EHPE> 			-- Equivalent Horizontal Position Error, dd.d in m
	// <Semi-major> 	-- Standard deviation (meters) of semi-major axis of error ellipse, dd.d in m
	// <Semi-minor> 	-- Standard deviation (meters) of semi-minor axis of error ellipse, dd.d in m
	// <Angle> 			-- Orientation of semi-major axis of error ellipse (true north degrees), dd.d in degree
	// <LatErr> 		-- Standard deviation (meters) of latitude error, dd.d in m
	// <LonErr> 		-- Standard deviation (meters) of longitude error, dd.d in m
	// <Alt Err Dev> 	-- Standard deviation (meters) of altitude error, dd.d in m

	float timestamp = 0.f;
	float ehpe = 0.f;
	float semi_major_stddev = 0.f;
	float semi_minor_stddev = 0.f;
	float angle = 0.f;
	float lat_err = 0.f;
	float lon_err = 0.f;
	float alt_err = 0.f;
};

inline GST_Data handle_GST(const char* msg)
{
	// $GNGST,205512.000,16.5,5.6,4.5,0.8,5.0,5.0,6.7*41

	// Global Positioning System Pseudorange Noise Statistics.

	// $<TalkerID>GST 	-- GN (multi-constellation), GP (GPS), GL (GLONASS), GA (GALILEO), BD (BEIDOU), QZ (QZSS)
	// <Timestamp> 		-- UTC time of GPS sample, hhmmss.sss
	// <EHPE> 			-- Equivalent Horizontal Position Error, dd.d in m
	// <Semi-major> 	-- Standard deviation (meters) of semi-major axis of error ellipse, dd.d in m
	// <Semi-minor> 	-- Standard deviation (meters) of semi-minor axis of error ellipse, dd.d in m
	// <Angle> 			-- Orientation of semi-major axis of error ellipse (true north degrees), dd.d in degree
	// <LatErr> 		-- Standard deviation (meters) of latitude error, dd.d in m
	// <LonErr> 		-- Standard deviation (meters) of longitude error, dd.d in m
	// <Alt Err Dev> 	-- Standard deviation (meters) of altitude error, dd.d in m
	// *
	// <checksum>
	// <cr><lf>

	GST_Data gst = {};
	char* endp = nullptr;

	if (msg && *(++msg) != ',') { gst.timestamp = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gst.ehpe	 = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gst.semi_major_stddev = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gst.semi_minor_stddev = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gst.angle = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gst.lat_err = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gst.lon_err = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gst.alt_err = strtof(msg, &endp); msg = endp; }

	return gst;
}