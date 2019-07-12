#include "EC_OEM.h"

EC_OEM alpha;
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  alpha.init();
  Serial.println("Device addr : "+String(alpha.getStoredAddr()) );
  Serial.println("Device type : "+String(alpha.getDeviceType()) );
  Serial.println("Firmware : "+String(alpha.getFirmwareVersion()) ); 
}
struct param_OEM_EC parameter;
void loop()
{
  
  alpha.singleReading();
  parameter = alpha.getAllParam();
  Serial.println("salinity= " + String(parameter.salinity)+
                 "\nconductivity= " +String(parameter.conductivity)+
                 "\ntds= " +String(parameter.tds)+
                 "\nSalinity stable = "+(alpha.isSalinityStable()?"yes":"no")
                 );
  delay(1000);
}
