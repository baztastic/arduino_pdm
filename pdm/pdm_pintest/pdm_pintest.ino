#include "Adafruit_ASFcore.h"
#include "Adafruit_ZeroPDM.h"
#include <SPI.h>

#define SAMPLERATE_HZ 44100
#define DECIMATION    64

#define SERIALPORT Serial

void setup() {
  // put your setup code here, to run once:
  // Configure serial port.
  SERIALPORT.begin(9600);
  SERIALPORT.println("SAMD PDM Demo");
  pinMode(13, OUTPUT);

    // Initialize the PDM/I2S receiver

  // use analog output A0 @ full rez
  analogWriteResolution(10);

}

void loop() {
  // put your main code here, to run repeatedly:
  int i ;
  int j ;
  for(i = 0; i < 20; i++) {
    for(j = 0; j < 20; j++) {
      Adafruit_ZeroPDM pdm = Adafruit_ZeroPDM(i, j);

      if (!pdm.begin()) {
//        SERIALPORT.println("Failed to configure PDM");
        continue;
      }
      else {
        SERIALPORT.println("PDM configured");
        SERIALPORT.print("clk = ");
        SERIALPORT.print(i);
        SERIALPORT.print(" data = ");
        SERIALPORT.println(j);
        continue;
      }
    }
  }
  while(1);
}
