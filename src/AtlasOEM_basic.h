#ifndef _ATLASOEM_BASIC_H
#define _ATLASOEM_BASIC_H

//=================================== Atlas device type OEM
#define UNKNOWN_DEVICE    0
#define EC_OEM_DEVICE     4
#define PH_OEM_DEVICE     1
#define DO_OEM_DEVICE     3
#define ORP_OEM_DEVICE    2

#define UNKNOWN_VERSION   0

//=================================== interrupt control register 
#define DISABLED_INTERRUPT    0
#define HIGH_ON_INTERRUPT     2
#define LOW_ON_INTERRUPT      4
#define CHANGE_ON_INTERRUPT   8


//================================== address for unlock
#define address_unlock_A  0x55            
#define address_unlock_B  0xAA

//=================================== EC_OEM register data
// general register of atlas-OEM
#define device_type             0x00
#define firmware_version        0x01
#define addr_lock               0x02
#define new_addr_register       0x03
#define int_ctrl                0x04
#define led_ctrl                0x05
#define sleep_ctrl              0x06
#define data_available          0x07

#define NOP __asm__ __volatile__ ("nop\n\t")    // delay 62.5ns on a 16MHz AtMega

#include <Arduino.h>
#include <Wire.h>
template <typename T>
uint8_t getAddrressDevice(const T deviceType)
{
  if( deviceType < 1 || deviceType > 4 ) return UNKNOWN_DEVICE;
  else
  {
    for ( uint8_t addr = 1; addr < 128; addr++)
    {
      Wire.beginTransmission(addr);                                                            
      Wire.write(device_type);                                                                                
      if( Wire.endTransmission() == 0 )
      {
        Wire.requestFrom(addr,1); 
        if(Wire.available() == 1)
        {
          if (Wire.read() == deviceType)
          {
            Wire.endTransmission();
            return addr;
            break;
          };
        }
        Wire.endTransmission();
      }
    }
  }
  return UNKNOWN_DEVICE;
}

#endif
