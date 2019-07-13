#include "EC_OEM.h"
#include "DO_OEM.h"

EC_OEM alpha;
DO_OEM beta;
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  alpha.init();
  beta.init();
  Serial.println("Device addr EC: "+String(alpha.getStoredAddr()) );
  Serial.println("Device type EC: "+String(alpha.getDeviceType()) );
  Serial.println("Firmware EC: "+String(alpha.getFirmwareVersion()) ); 

  Serial.println("Device addr DO: "+String(beta.getStoredAddr()) );
  Serial.println("Device type DO: "+String(beta.getDeviceType()) );
  Serial.println("Firmware DO: "+String(beta.getFirmwareVersion()) ); 
}
struct param_OEM_EC parameter;
struct param_OEM_DO parameter2;
void loop()
{
  
  alpha.singleReading();
  beta.singleReading();
  parameter = alpha.getAllParam();
  parameter2 = beta.getAllParam();
  Serial.println("salinity= " + String(parameter.salinity)+
                 "\nconductivity= " +String(parameter.conductivity)+
                 "\ntds= " +String(parameter.tds)+
                 "\nSalinity stable = "+(alpha.isSalinityStable()?"yes":"no")
                 );
                 
  Serial.println("DO in mg= " + String(parameter2.inMilligrams)+
                 "\nDO in saturation= " +String(parameter2.inSaturation)+
                 "\nDO Saturation stable = "+(beta.isSaturationStable()?"yes":"no")
                 );
  delay(1000);
}
