#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "stdio.h"

class Lidar
{
public:
    void begin();
    bool measure(void);
    uint16_t getDistance(void);
    uint16_t getStrength(void);
private:
    uint16_t _distance = 0;
    uint16_t _strength = 0;
};
