#include "UavcanCanard.hpp"

extern "C" __EXPORT int uavcan_canard_main(int argc, char *argv[])
{
	return UavcanCanard::main(argc, argv);
}
