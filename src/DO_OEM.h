#ifndef _DO_OEM_H
#define _DO_OEM_H

#include <stdio.h>
#include <stdarg.h>
#include <Arduino.h>
#include <Wire.h>

#include "AtlasOEM_basic.h"
#include "StabilityDetector.h"

#define NOP __asm__ __volatile__ ("nop\n\t") 
#define NONE_INT    255

#define NONE_DATA   sqrt(-1)

#define i2c_id            0x67              //default I2C address   
#define one_byte_read     0x01              //used in a function to read data from the device  
#define two_byte_read     0x02              //used in a function to read data from the device
#define four_byte_read    0x04             //used in a function to read data from the device

#define on_DO             true
#define off_DO            false

#define DO_HIBERNATES     true
#define DO_ACTIVES        false

//=================================== DO_OEM register data
// specific ec' register
#define calib_reg               0x08   // 1bytes
#define calib_confirm           0x09
#define salinity_compen_reg     0x0A   // 4bytes
#define pressure_compen_reg     0x0E   // 4bytes
#define temp_compen_reg         0x12   // 4bytes
#define salinity_confirm_reg    0x16   // 4bytes
#define pressure_confirm_reg    0x1A   // 4bytes
#define temp_confirm_reg        0x1E   // 4bytes
#define DO_in_mg_reg            0x22   // 4bytes
#define DO_in_sat_reg           0x26   // 4bytes
//========================================================

//=================================== Code for confirmation calibration
#define NO_CALIBRATION                   0
#define ATMOSPHERE_CALIBRATION           1
#define ZERO_DO_CALIBRATION              2
#define BOTH_CALIBRATION                 3      // both atmospheriic and 0 Dissolved Oxygen
//=======================================================================
class DO_OEM
{
  private:
    uint8_t int_pin;
    byte device_addr;
    union data_handler move_data;
    struct param_OEM_DO param;
    struct param_OEM_DO_compensation compensation;
    bool  new_reading_available,
          hibernation_status,
          status_presence;       
    uint8_t firm_version,
            int_control,
            type_device;
    StabilityDetector   dissolvedOxygenStability;

    bool i2c_write_long(byte reg, unsigned long data);
    bool i2c_write_byte(byte reg, byte data);
    void i2c_read(byte reg, byte number_of_bytes_to_read, unsigned long timeout = 500UL);
    void delayForMillis(unsigned long timeout);
  public:
    DO_OEM(uint8_t pin = NONE_INT, uint8_t addr_ = i2c_id);
    
    void init(bool led_ = off_DO , bool hibernate_ = false, uint8_t int_CTRL = DISABLED_INTERRUPT );
// device information
    byte getDeviceType(void);
    byte getFirmwareVersion(void);
// address control
    bool isLockedAddress(void);
    bool setLockedAddress(bool lock);
    bool setDeviceAddress(uint8_t new_address);
    byte getStoredAddr(void){return device_addr;}; 
// control register
    uint8_t isInterruptAvailable(void);
    bool    setInterruptAvailable(uint8_t mode = CHANGE_ON_INTERRUPT);
    bool    isLedOn(void);
    bool    setLedOn(bool stat);
    bool    isHibernate(void);
    bool    setHibernate(void);
    bool    wakeUp(void);
    bool    isNewDataAvailable(void);
    bool    clearNewDataRegister(void);
// calibration register
    uint8_t getStatusCalibration(void);
    bool    clearCalibrationData(void);
    bool    dissolvedCalibration(uint8_t typeCal = ATMOSPHERE_CALIBRATION);
// compensation register
// request data
    bool  requestCompensationData(void);
    float getPressureCompensation(bool fromStored = true );                     // if true, need run requestCompensationData() before
    float getSalinityCompensation(bool fromStored = true );
    float getTemperatureCompensation(bool fromStored = true );
    param_OEM_DO_compensation getAllCompensation(void){return compensation;};   // need to run requestCompensationData before
// set data    
    bool setPressureCompensation(float compenValue);
    bool setSalinityCompensation(float compenValue);
    bool setTemperatureCompensation(float compenValue);
    bool setCompensation(float pressureCompen = NONE_DATA, float salinityCompen = NONE_DATA, float temperatureCompen = NONE_DATA);
// reading parameter
    bool  singleReading(void);
    float getDataInMilli(bool fromStored = true);         //true : taking data from stored-memory ( run singleReading() first ); false : request data from module
    float getDataInSaturation(bool fromStored = true);
    param_OEM_DO getAllParam(void);
// Stability function
    bool isSaturationStable(void) {return dissolvedOxygenStability.isStable();};
    void setSaturationPrecision(float precision) {this->dissolvedOxygenStability.setPrecision(precision); };
    uint8_t getSaturationStableCount(void){return dissolvedOxygenStability.getStableCount();};
    float getSaturationDeviasion(void){return dissolvedOxygenStability.getDeviasionValue();};
};

#endif
