#include "DO_OEM.h"

void DO_OEM::delayForMillis(unsigned long timeout)
{
  unsigned long t = millis();
  while(millis() - t <= timeout){}    // do nothing  
}

// basic function for I2C/SMbus access
bool DO_OEM::i2c_write_long(byte reg, unsigned long data) {                                     //used to write a 4 bytes to a register: i2c_write_longe(register to start at,unsigned long data )                                                                                 
  Wire.beginTransmission(device_addr);                                                          
  Wire.write(reg);                                                                              
  for (byte i = 3; i >= 0; i--) {                                                                    //with this code we write multiple bytes in reverse
    Wire.write(move_data.i2c_data[i]);
  }
  if(Wire.endTransmission() == 0)return true;
  else return false;                                                                         //end the I2C data transmission
}

bool DO_OEM::i2c_write_byte(byte reg, byte data) {                                              //used to write a single byte to a register: i2c_write_byte(register to write to, byte data)
  Wire.beginTransmission(device_addr);                                                         
  Wire.write(reg);                                                                              
  Wire.write(data);                                                                             
  if(Wire.endTransmission() == 0)return true;
  else return false;                                                                         //end the I2C data transmission
}

void DO_OEM::i2c_read(byte reg, byte number_of_bytes_to_read, unsigned long timeout) {                                           //used to read 1,2,and 4 bytes: i2c_read(starting register,number of bytes to read)
  move_data.answ = 0;
  Wire.beginTransmission(device_addr);                                                            
  Wire.write(reg);                                                                                
  if( Wire.endTransmission() == 0)
  {
    unsigned long dt = millis();
    Wire.requestFrom(device_addr, (byte)number_of_bytes_to_read);                                   //call the device and request to read X bytes
    while( millis() - dt <= timeout)
    {
      if(Wire.available() ==  (byte)number_of_bytes_to_read)
      {
          for (byte i = number_of_bytes_to_read; i > 0; i--)
            move_data.i2c_data[i - 1] = Wire.read();
          break;  
      }  
    }
    Wire.endTransmission();     
  }                                                                        
}


// main methods of lib

DO_OEM::DO_OEM(uint8_t pin, uint8_t addr_)
{
  this->int_pin               = pin;
  this->device_addr           = addr_;
  this->param.inMilligrams    = 0;
  this->param.inSaturation    = 0;
  this->new_reading_available = false;
  this->hibernation_status    = false;
  this->firm_version          = UNKNOWN_VERSION;
  this->int_control           = 0;
  this->status_presence       = false;
  this->type_device           = UNKNOWN_DEVICE;
  this->dissolvedOxygenStability.setPrecision(0.02f);
}

void DO_OEM::init(bool led_, bool hibernate_, uint8_t int_CTRL)
{
  Wire.beginTransmission(device_addr);                                                            
  Wire.write(device_type);                                                                                
  if( Wire.endTransmission() == 0 )
  {
    if(getDeviceType() == DO_OEM_DEVICE )
    {
      this->status_presence = true;
    }
    else
    {
      this->device_addr = getAddrressDevice(DO_OEM_DEVICE);
      Serial.println(device_addr);
      if ( device_addr == DO_OEM_DEVICE )
        this->status_presence = true;
      else
        this->status_presence = false;
    }     
  }
  if(status_presence)
  {
    if(led_) setLedOn(led_);
    if(hibernate_)
    {
      if(!isHibernate()) setHibernate();
    }
    else
    {
      if(isHibernate()) wakeUp();
    }
    setInterruptAvailable(int_CTRL);
    clearNewDataRegister();
  }
}

byte DO_OEM::getDeviceType(void)
{
  this->type_device = UNKNOWN_DEVICE;
  i2c_read(device_type, one_byte_read);
  this->type_device = move_data.i2c_data[0];
  return type_device;
}

byte DO_OEM::getFirmwareVersion(void)
{
  this->firm_version = UNKNOWN_VERSION;
  i2c_read(firmware_version, one_byte_read);
  this->firm_version = move_data.i2c_data[0];
  return firm_version;
}

bool DO_OEM::isLockedAddress(void)
{
  i2c_read(addr_lock, one_byte_read);
  if(move_data.i2c_data[0]>0) return true;
  else                        return false;
}

bool DO_OEM::setLockedAddress(bool lock)
{
  if(lock)    // lock address
  {
    if ( i2c_write_byte(addr_lock, 0x00))
    {
      i2c_read(addr_lock, one_byte_read);               
      if (move_data.i2c_data[0])return true;
    }
  }
  else        // unlock address
  {
    if(i2c_write_byte(addr_lock, address_unlock_A))
    {
      if( i2c_write_byte(addr_lock, address_unlock_B) )
      {
        i2c_read(addr_lock, one_byte_read);
        if (move_data.i2c_data[0] == 0) return true;
      }
    }
  }
  return false;
}

bool DO_OEM::setDeviceAddress(uint8_t new_address)
{
  if (new_address < 1 || new_address > 127)return false;
  else
  {
      i2c_read(addr_lock, one_byte_read);                                    
      if (move_data.i2c_data[0])            // addr is locked
      {
        if ( !setLockedAddress(false) ) return false;   // can't unlock
      };
      if(i2c_write_byte(new_addr_register, new_address))  return true;
      else                                                return false;
  }
}

uint8_t DO_OEM::isInterruptAvailable(void)
{
  i2c_read(int_ctrl, one_byte_read); 
  if( move_data.i2c_data[0] == HIGH_ON_INTERRUPT || move_data.i2c_data[0] == LOW_ON_INTERRUPT || move_data.i2c_data[0] == CHANGE_ON_INTERRUPT )
    return move_data.i2c_data[0];
  else 
    return DISABLED_INTERRUPT;
}

bool DO_OEM::setInterruptAvailable(uint8_t mode)
{
  if( mode == DISABLED_INTERRUPT || mode == HIGH_ON_INTERRUPT || mode == LOW_ON_INTERRUPT || mode == CHANGE_ON_INTERRUPT )
  {
    if ( isInterruptAvailable() == mode ) return true;
    else
    {
      if( i2c_write_byte(int_ctrl, mode) )
      {
        i2c_read(int_ctrl, one_byte_read);
        if( move_data.i2c_data[0] == mode ) return true;
        else                                return false;
      }
      else
        return false;
    }
  }
}

bool DO_OEM::isLedOn(void)
{
   i2c_read(led_ctrl, one_byte_read);
   if (move_data.i2c_data[0])       return on_DO;   
   if (move_data.i2c_data[0] == 0)  return off_DO;
}

bool DO_OEM::setLedOn(bool stat)
{
  byte stat_byte = stat?0x01:0x00;
  if(i2c_write_byte(led_ctrl, stat_byte))
  {
    i2c_read(led_ctrl, one_byte_read);
    if (move_data.i2c_data[0] == stat_byte)  return true;
    else                                     return false;
  }
  else
    return false;
}

bool DO_OEM::isHibernate(void)
{
  i2c_read(sleep_ctrl, one_byte_read);
  if( move_data.i2c_data[0] == 0 ) return DO_HIBERNATES;
  if( move_data.i2c_data[0] == 1 ) return DO_ACTIVES;
}

bool DO_OEM::setHibernate(void)
{
    if(i2c_write_byte(sleep_ctrl, 0x00))
    {
      i2c_read(sleep_ctrl, one_byte_read);
      if (move_data.i2c_data[0] == 0)   return true;
      else                              return false;
    }
    else
      return false;
}

bool DO_OEM::wakeUp(void)
{
  if(i2c_write_byte(sleep_ctrl, 0x01))
  {
    i2c_read(sleep_ctrl, one_byte_read);
    if (move_data.i2c_data[0] == 1) return true;
    else                            return false;
  }
  else
    return false;
}

bool DO_OEM::isNewDataAvailable(void)
{
  i2c_read(data_available, one_byte_read);
  if (move_data.i2c_data[0])  return true;
  if (move_data.i2c_data[0]=0)return false;
}

bool DO_OEM::clearNewDataRegister(void)
{
  if ( !isNewDataAvailable() ) return true;
  else
  {
    if (i2c_write_byte(data_available, 0x00))
    if ( !isNewDataAvailable() ) return true;
    else                         return false;
  }
}

uint8_t DO_OEM::getStatusCalibration(void)
{
  i2c_read(calib_confirm, one_byte_read);
  if (move_data.i2c_data[0] == 0) return NO_CALIBRATION;
  if (move_data.i2c_data[0] == 1) return ATMOSPHERE_CALIBRATION;
  if (move_data.i2c_data[0] == 2) return ZERO_DO_CALIBRATION;
  if (move_data.i2c_data[0] == 3) return BOTH_CALIBRATION;
}

bool DO_OEM::clearCalibrationData(void)
{
  if( i2c_write_byte(calib_reg, 0x01) )
  {
    delayForMillis(40);    // don't use delay-function
    if(getStatusCalibration() == NO_CALIBRATION ) return true;
    else                                          return false;
  }
  else
    return false;
}

bool DO_OEM::dissolvedCalibration(uint8_t typeCal)
{
  if(typeCal < ATMOSPHERE_CALIBRATION || typeCal > ZERO_DO_CALIBRATION ) return false;
  else
  {
    if ( typeCal == ATMOSPHERE_CALIBRATION )
    {
      i2c_write_byte(calib_reg, 0x02);    
      delayForMillis(40);                  
      byte statusCal = getStatusCalibration();
      if (statusCal == ATMOSPHERE_CALIBRATION || statusCal == BOTH_CALIBRATION) return true;
    }
    else
    {
      i2c_write_byte(calib_reg, 0x03);
      delayForMillis(40);             
      byte statusCal = getStatusCalibration();
      if (statusCal == ZERO_DO_CALIBRATION || statusCal == BOTH_CALIBRATION) return true;
    };
  };
  return false;
}

bool DO_OEM::requestCompensationData(void)
{
  if(isHibernate())
  {
    wakeUp(); 
    delayForMillis(5);
    if(isHibernate()) return false;
  }
  i2c_read(salinity_confirm_reg, four_byte_read); 
    compensation.salinity = move_data.answ;       
    compensation.salinity /= 100;                 
  NOP;
  i2c_read(pressure_confirm_reg, four_byte_read); 
    compensation.pressure = move_data.answ;       
    compensation.pressure /= 100;                 
  NOP;
  i2c_read(temp_confirm_reg, four_byte_read);  
    compensation.temperature = move_data.answ;                             
    compensation.temperature /= 100; 
  NOP;
  return true;
}

float DO_OEM::getPressureCompensation(bool fromStored)
{
  if(fromStored)
   {
      if(isHibernate())
      {
        wakeUp();
        NOP;
      }
      return compensation.pressure;
   }
   else
   {
      if(isHibernate())
      {
        wakeUp();
        NOP; NOP;
        if ( isHibernate() ) return sqrt(-1);
      }
      i2c_read(pressure_confirm_reg, four_byte_read); 
      compensation.pressure = move_data.answ;       
      compensation.pressure /= 100;             
   };
   return compensation.pressure;
}

float DO_OEM::getSalinityCompensation(bool fromStored)
{
   if(fromStored)
   {
      if(isHibernate())
      {
        wakeUp();
        NOP;
      }
      return compensation.salinity;
   }
   else
   {
      if(isHibernate())
      {
        wakeUp();
        NOP; NOP;
        if ( isHibernate() ) return sqrt(-1);
      }
      i2c_read(salinity_confirm_reg, four_byte_read); 
      compensation.salinity = move_data.answ;       
      compensation.salinity /= 100;             
   };
   return compensation.salinity;
}

float DO_OEM::getTemperatureCompensation(bool fromStored)
{
   if(fromStored)
   {
      if(isHibernate())
      {
        wakeUp();
        NOP;
      }
      return compensation.temperature;
   }
   else
   {
      if(isHibernate())
      {
        wakeUp();
        NOP; NOP;
        if ( isHibernate() ) return sqrt(-1);
      }
      i2c_read(temp_confirm_reg, four_byte_read); 
      compensation.temperature = move_data.answ;       
      compensation.temperature /= 100;             
   };
   return compensation.temperature;
}


bool DO_OEM::setPressureCompensation(float compenValue)
{
  compenValue *= 100;
  move_data.answ = compenValue;                                
  if(i2c_write_long(pressure_compen_reg, move_data.answ)) return true;
  else                                                    return false;
}

bool DO_OEM::setSalinityCompensation(float compenValue)
{
  compenValue *= 100;
  move_data.answ = compenValue;                                
  if(i2c_write_long(salinity_compen_reg, move_data.answ)) return true;
  else                                                    return false;
}

bool DO_OEM::setTemperatureCompensation(float compenValue)
{
  compenValue *= 100;
  move_data.answ = compenValue;                                
  if(i2c_write_long(temp_compen_reg, move_data.answ)) return true;
  else                                                return false;
}

bool DO_OEM::setCompensation(float pressureCompen, float salinityCompen, float temperatureCompen)
{
  bool result = false;
  if(!isnan(pressureCompen))
  {
    result = setPressureCompensation(pressureCompen);
  };
  if(!isnan(salinityCompen))
  {
    result = setSalinityCompensation(salinityCompen);
  };
  if(!isnan(temperatureCompen))
  {
    result = setTemperatureCompensation(temperatureCompen);
  };
  return result;
}

bool DO_OEM::singleReading(void)
{
  if(isHibernate())
  {
    wakeUp();
    NOP;
    if(isHibernate()) return false;
  };
  if(isNewDataAvailable())
  {
    i2c_read(DO_in_mg_reg, four_byte_read);                                                  
    param.inMilligrams = move_data.answ;                                                                    
    param.inMilligrams /= 100;                                                                              
    NOP;
    i2c_read(DO_in_sat_reg, four_byte_read);
    param.inSaturation = move_data.answ;
    param.inSaturation /= 100;
    dissolvedOxygenStability.pushToBuffer(param.inSaturation);
    NOP;
    return clearNewDataRegister();
  }
  else
    return false;
}

float DO_OEM::getDataInMilli(bool fromStored)
{
  if(fromStored)
   {
      if(isHibernate())
      {
        wakeUp();
        NOP;
      }
      return param.inMilligrams;
   }
   else
   {
      if(isHibernate())
      {
        wakeUp();
        NOP; NOP;
        if ( isHibernate() ) return sqrt(-1);
      }
      if(isNewDataAvailable())
      {
        NOP;
        i2c_read(DO_in_mg_reg, four_byte_read);
        param.inMilligrams = move_data.answ;
        param.inMilligrams /= 100;
        clearNewDataRegister();
      }
   }
   return param.inMilligrams;
}

float DO_OEM::getDataInSaturation(bool fromStored)
{
  if(fromStored)
   {
      if(isHibernate())
      {
        wakeUp();
        NOP;
      }
      return param.inSaturation;
   }
   else
   {
      if(isHibernate())
      {
        wakeUp();
        NOP; NOP;
        if ( isHibernate() ) return sqrt(-1);
      }
      if(isNewDataAvailable())
      {
        NOP;
        i2c_read(DO_in_sat_reg, four_byte_read);
        param.inSaturation = move_data.answ;
        param.inSaturation /= 100;
        clearNewDataRegister();
      }
   }
   return param.inSaturation;
}

param_OEM_DO DO_OEM::getAllParam(void)
{
  if(isHibernate())
  {
    wakeUp();
    NOP;
  }
  return param;
}
