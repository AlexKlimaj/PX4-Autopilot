#include <cstdlib>

struct GGA_Data {
	// $GPGGA,183417.000,04814.03970,N,01128.52205,E,0,00,99.0,495.53,M,47.6,M*53

	// Global Positioning System Fixed data

	// $<TalkerID>GGA 	-- GN (multi-constellation), GP (GPS), GL (GLONASS), GA (GALILEO), BD (BEIDOU), QZ (QZSS)
	// <Timestamp>	 	-- UTC time of GPS sample, hhmmss.sss
	// <Lat> 			-- Latitude as degrees:, DDDMM.MMMMM
	// <N/S> 			-- Latitude direction, "N" or "S"
	// <Long>			-- Longitude as degrees:, DDDMM.MMMMM
	// <E/W> 			-- Longitude direction, "E" or "W"
	// <GPSQual> 		-- 0 = Fix not available or invalid, 1 = GPS PS Mode fix valid, 2 = Differential GPS SPS Mode fix valid, 6 = Estimated (dead reckoning) mode
	// <Sats> 			-- Satellites in use
	// <HDOP> 			-- Horizontal Dilution of Precision, max: 99.0
	// <Alt> 			-- Height above mean sea level, max: 100000m
	// <AltVal> 		-- Reference Unit for Altitude ("M" = meters)
	// <GeoSep> 		-- Geoidal Separation measure in "M" = meters
	// <GeoVal> 		-- Reference Unit for GeoSep ("M" = meters)
	// <DGPSAge> 		-- Not supported
	// <DGPSRef> 		-- Not supported
	// *
	// <checksum>
	// <cr><lf>

	float timestamp = 0.0;
	double lat = 0.0;
	char ns = '?';
	double lon = 0.0;
	char ew = '?';
	int fix_quality = 0;
	int sats = 0;
	float hdop = 0.f;
	int alt = 0;
	char alt_val = '?';
	int geo_sep = 0;
	char geo_val = '?';
};

inline GGA_Data handle_GGA(const char* msg)
{
	GGA_Data gga = {};
	char* endp = nullptr;

	if (msg && *(++msg) != ',') { gga.timestamp = strtod(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gga.lat = strtod(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gga.ns = *(msg++); }

	if (msg && *(++msg) != ',') { gga.lon = strtod(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gga.ew = *(msg++); }

	if (msg && *(++msg) != ',') { gga.fix_quality = strtol(msg, &endp, 10); msg = endp; }

	if (msg && *(++msg) != ',') { gga.sats = strtol(msg, &endp, 10); msg = endp; }

	if (msg && *(++msg) != ',') { gga.hdop = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { gga.alt = strtol(msg, &endp, 10); msg = endp; }

	if (msg && *(++msg) != ',') { gga.alt_val = *(msg++); }

	if (msg && *(++msg) != ',') { gga.geo_sep = strtol(msg, &endp, 10); msg = endp; }

	if (msg && *(++msg) != ',') { gga.geo_val = *(msg++); }

	return gga;
}