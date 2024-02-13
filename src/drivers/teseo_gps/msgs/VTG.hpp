#include <cstdlib>

struct VTG_Data {
	// $GPVTG,73.2,T,,M,0.2,N,0.4,K,D*50
	// $GNVTG,,T,,M,,N,,K,N*32

	// Course over ground and ground speed, this message provides the actual course and speed relative to ground.

	// $<TalkerID>VTG 	-- GN (multi-constellation), GP (GPS), GL (GLONASS), GA (GALILEO), BD (BEIDOU), QZ (QZSS)
	// <TMGT> 			-- Track in reference to "true" earth poles, ddd.d in degrees
	// <T> 				-- Indicates "terrestrial"
	// <TMGM>			-- Track in reference to "magnetic" earth poles, ddd.d in degrees
	// <M> 				-- Indicates "magnetic"
	// <SoGN>			-- Speed over Ground in knots
	// <N> 				-- Indicates "knots"
	// <SoGK>			-- Speed over Ground in kilometers per hour, ddd.d in km/h
	// <K> 				-- Indicates "kilometres"
	// <D> 				-- Mode indicator: A = Autonomous mode, D = Differential mode, E = Estimated mode, N = none??(jake observed)
	// *
	// <checksum>
	// <cr><lf>

	float track_true = 0.f;
	char T = '?';
	float track_mag = 0.f;
	char M = '?';
	float ground_speed_knots = 0.f;
	char N = '?';
	float ground_speed_kmh = 0.f;
	char K = '?';
	char mode = '?';
};

inline VTG_Data handle_VTG(const char* msg)
{
	VTG_Data vtg = {};
	char* endp = nullptr;

	if (msg && *(++msg) != ',') { vtg.track_true = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { vtg.T = *(msg++); }

	if (msg && *(++msg) != ',') { vtg.track_mag = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { vtg.M = *(msg++); }

	if (msg && *(++msg) != ',') { vtg.ground_speed_knots = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { vtg.N = *(msg++); }

	if (msg && *(++msg) != ',') { vtg.ground_speed_kmh = strtof(msg, &endp); msg = endp; }

	if (msg && *(++msg) != ',') { vtg.K = *(msg++); }

	if (msg && *(++msg) != ',') { vtg.mode = *(msg++); }

	return vtg;
}