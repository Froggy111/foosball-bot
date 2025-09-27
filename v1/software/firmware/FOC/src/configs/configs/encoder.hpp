#pragma once

#define ABZ 1
#define MT6701 2

#define ENCODER_TYPE ABZ

#if ENCODER_TYPE == ABZ
#include "configs/encoder/ABZ.hpp"
#elif ENCODER_TYPE == MT6701
#include "configs/encoder/MT6701.hpp"
#endif
