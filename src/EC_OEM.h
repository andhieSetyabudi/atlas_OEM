#ifndef _EC_OEM_H
#define _EC_OEM_H

#include <stdio.h>
#include <stdarg.h>
#include <Arduino.h>
#include <Wire.h>

#include "AtlasOEM_basic.h"
#include "StabilityDetector.h"

#define NOP __asm__ __volatile__ ("nop\n\t") 
#define NONE_INT    255


#define i2c_id            0x64              //default I2C address   
#define one_byte_read     0x01              //used in a function to read data from the device  
#define two_byte_read     0x02              //used in a function to read data from the device
#define four_byte_read    0x04             //used in a function to read data from the device

#define on_EC             true
#define off_EC            false

#define EC_HIBERNATES     true
#define EC_ACTIVES        false


//=================================== Code for confirmation calibration
#define NO_CALIBRATION            0
#define DRY_CALIBRATION           1
#define SINGLE_POINT_CALIBRATION  2
#define LOW_POINT_CALIBRATION     3
#define HIGH_POINT_CALIBRATION    4

//=================================== EC_OEM register data
// specific ec' register
#define probe_type              0x08    // 2bytes
#define calib_reg               0x0A    // 4bytes
#define calib_request           0x0E
#define calib_confirm           0x0F
#define temp_compen             0x10   // 4bytes
#define temp_confirm            0x14   // 4bytes
#define ec_read_reg             0x18   // 4bytes
#define tds_read_reg            0x1C   // 4bytes
#define pss_read_reg            0x20   // 4bytes
//========================================================


class EC_OEM{
  private:
    uint8_t int_pin;
    byte device_addr;
    union data_handler move_data;
    struct param_OEM_EC param;
    bool  new_reading_available,
          hibernation_status,
          status_presence;       
    uint8_t firm_version,
            int_control,
            type_device;

    StabilityDetector   SalinityStability;
    
    bool i2c_write_long(byte reg, unsigned long data);
    bool i2c_write_byte(byte reg, byte data);
    void i2c_read(byte reg, byte number_of_bytes_to_read, unsigned long timeout = 500UL);
    void delayForMillis(unsigned long timeout);
  public:
    EC_OEM(uint8_t pin = NONE_INT, uint8_t addr_ = i2c_id);
    void init(bool led_ = off_EC , bool hibernate_ = false, uint8_t int_CTRL = DISABLED_INTERRUPT );
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
// probe type    
    float getProbeType(void);
    bool  setProbeType(float type);
// calibration register
    uint8_t getStatusCalibration(void);
    bool    clearCalibrationData(void);
    bool    setCalibration(uint8_t mode=DRY_CALIBRATION, float value=0);
// temperature compensation
    float getTempCompensationValue(void);
    bool  setTempCompensation(float value);
// reading parameter ; Conductivity, Salinity & TDS
    bool  singleReading(void);  
    float getSalinity(bool fromStored = true) ;     //true : taking data from stored-memory ( run singleReading() first ); false : request data from module
    float getConductivity(bool fromStored = true) ;
    float getTDS(bool fromStored = true) ;  
    param_OEM_EC getAllParam(void);
// Stability function
    bool isSalinityStable(void) {return SalinityStability.isStable();};
    void setSalinityPrecision(float precision) {this->SalinityStability.setPrecision(precision); };
    uint8_t getSalinityStableCount(void){return SalinityStability.getStableCount();};
    float getSalinityDeviasion(void){return SalinityStability.getDeviasionValue();};
    
};
#endif
