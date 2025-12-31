#pragma once

#include <assert.h>

#ifndef CANARD_ASSERT
# define CANARD_ASSERT(x) assert(x)
#endif

// libcanard does not allocate internally; it expects the application to provide
// memory for transfers. PX4 will supply this in the driver implementation.
