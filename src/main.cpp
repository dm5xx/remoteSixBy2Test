#include <Arduino.h>

#include <Wire.h>
#include "Adafruit_MCP23017.h"
 

#define OUTPUTTEST
#define INPUTTEST

Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
 
// Timer-Variablen
uint8_t mcp_addr1 = 0; // Adresse 0x20 / 0
uint8_t mcp_addr2 = 1; // Adresse 0x20 / 0

bool state = false;

void setup() {  
  Serial.begin(115200);
  Serial.println("Test starting..");


  mcp1.begin(mcp_addr1);
  mcp2.begin(mcp_addr2);

  for(int i= 0; i < 16; i++)
  {
      mcp1.pinMode(i, OUTPUT); // GPB0 an mcp1 als Output definieren
  }

  for(int j= 0; j < 16; j++)
  {
      mcp2.pinMode(j, INPUT); // GPB0 an mcp1 als Output definieren
      mcp2.pullUp(j, HIGH);  // turn on a 100K pullup internally
  }

}
 
void loop() {
Serial.println("LOOP"); 

  state = !state;

  for(int i= 0; i < 16; i++)
  {
    Serial.println("Output"); 
    Serial.println(state); 
    mcp1.digitalWrite(i, state);

    delay(400);
  }

  for(int j= 0; j < 16; j++)
  {
    Serial.println(mcp2.digitalRead(j)); 
    delay(100);
  }
  delay(1000);
}



/// I2C Scanner
// #include <Wire.h>
 
 
// void setup()
// {
//   Wire.begin();
 
//   Serial.begin(115200);
//   while (!Serial);             // Leonardo: wait for serial monitor
//   Serial.println("\nI2C Scanner");
// }
 
 
// void loop()
// {
//   byte error, address;
//   int nDevices;
 
//   Serial.println("Scanning...");
 
//   nDevices = 0;
//   for(address = 1; address < 127; address++ )
//   {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();
 
//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.print(address,HEX);
//       Serial.println("  !");
 
//       nDevices++;
//     }
//     else if (error==4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.println(address,HEX);
//     }    
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");
 
//   delay(5000);           // wait 5 seconds for next scan
// }