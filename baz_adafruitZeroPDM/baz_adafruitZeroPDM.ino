//#include <Adafruit_ASFcore.h>
//#include <clock.h>
//#include <clock_feature.h>
//#include <compiler.h>
//#include <gclk.h>
#include <i2s.h>
//#include <interrupt.h>
//#include <interrupt_sam_nvic.h>
//#include <parts.h>
//#include <pinmux.h>
//#include <power.h>
//#include <reset.h>
//#include <status_codes.h>
//#include <system.h>
//#include <system_interrupt.h>
//#include <system_interrupt_features.h>
//#include <tc.h>
//#include <tc_interrupt.h>
//#include <wdt.h>

#include <Adafruit_ZeroPDM.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Adafruit_ZeroPDM::Adafruit_ZeroPDM(6, 4);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(200);                       // wait for a second
 }
