#pragma once

#include "config.hpp"

#if ENCODER_TYPE == ABZ
#include "ABZ.hpp"
#elif ENCODER_TYPE == MT6701
#include "MT6701.hpp"
#endif
