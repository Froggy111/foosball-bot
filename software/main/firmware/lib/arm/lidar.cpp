#include <Arduino.h>
#include "lidar.hpp"

void Lidar::begin()
{
    Serial.begin(115200);
}

bool Lidar::measure(void)
{
    uint8_t       TFbuff[9] = {0};
    long          checksum  = 0 ;
    while(Serial.available()){
        TFbuff[0] = Serial.read();
        checksum += TFbuff[0];
        if(TFbuff[0] == 'Y'){
            TFbuff[1] = Serial.read();
            checksum += TFbuff[1];
            if(TFbuff[1] == 'Y'){
                for(int i = 2;i < 8;i++){
                    TFbuff[i] = Serial.read();
                    checksum += TFbuff[i];
                }
                TFbuff[8] = Serial.read();
                checksum &= 0xff;
                if(checksum == TFbuff[8]){
                    _distance = TFbuff[2]+TFbuff[3]*256;
                    _strength = TFbuff[4]+TFbuff[5]*256;
                    return true;
                }else{
                    checksum  = 0;
                }
            }else{
                checksum  = 0;
            }
        }else{
            checksum  = 0;
        }
    }
    return false;
}

uint16_t Lidar::getDistance(void)
{
    return _distance;
}

uint16_t Lidar::getStrength(void)
{
    return _strength;
}