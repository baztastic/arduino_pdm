#include "Adafruit_ASFcore.h"
#include "Adafruit_ZeroPDM.h"
#include <SPI.h>
#include "arduinoFFT.h"

#define SAMPLERATE_HZ 44100
#define DECIMATION    64

// Create PDM receiver object, with Clock and Data pins used (not all pins available)
Adafruit_ZeroPDM pdm_m = Adafruit_ZeroPDM(1, 2);  // audible
Adafruit_ZeroPDM pdm_u = Adafruit_ZeroPDM(6, 9);  // ultrasonic
// clk = 1, 6
// data = 2, 9

// fft stuff
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

#define CHANNEL A0
const uint16_t samples = 512; //This value MUST ALWAYS be a power of 2
double samplingFrequency = 3000;
int sumSampleRate;
int oldSumMicros = 0;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal_m[samples];
double vReal_u[samples];
double vImag_m[samples];
double vImag_u[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

uint16_t sample_index = 0;
bool first_run = true;
// fft stuff


#define SERIALPORT Serial

// a windowed sinc filter for 44 khz, 64 samples
uint16_t sincfilter[DECIMATION] = {0, 2, 9, 21, 39, 63, 94, 132, 179, 236, 302, 379, 467, 565, 674, 792, 920, 1055, 1196, 1341, 1487, 1633, 1776, 1913, 2042, 2159, 2263, 2352, 2422, 2474, 2506, 2516, 2506, 2474, 2422, 2352, 2263, 2159, 2042, 1913, 1776, 1633, 1487, 1341, 1196, 1055, 920, 792, 674, 565, 467, 379, 302, 236, 179, 132, 94, 63, 39, 21, 9, 2, 0, 0};

void setup() {
  // Configure serial port.
  SERIALPORT.begin(9600);
  SERIALPORT.println("SAMD PDM Demo");
  pinMode(13, OUTPUT);

  // Initialize the PDM/I2S receiver
  if (!pdm_m.begin()) {
    SERIALPORT.println("Failed to initialize Audible I2S/PDM!");
    while (1);
  }
  if (!pdm_u.begin()) {
    SERIALPORT.println("Failed to initialize Ultrasonic I2S/PDM!");
    while (1);
  }

  SERIALPORT.println("PDM initialized");

  // Configure PDM receiver, sample rate
  if (!pdm_m.configure(SAMPLERATE_HZ * DECIMATION / 32)) {
    SERIALPORT.println("Failed to configure Audible PDM");
    while (1);
  }
  if (!pdm_u.configure(SAMPLERATE_HZ * DECIMATION / 32)) {
    SERIALPORT.println("Failed to configure Ultrasonic PDM");
    while (1);
  }
  SERIALPORT.println("PDM configured");
}

uint16_t readings_m[DECIMATION / 16];  // 4 x 16 bit words = 64 samples to match our filter!
uint16_t readings_u[DECIMATION / 16];  // 4 x 16 bit words = 64 samples to match our filter!

void loop() {
  for (uint8_t i=0; i < (DECIMATION/16) ; i++) {
     readings_m[i] = pdm_m.read() & 0xFFFF;
  }

  for (uint8_t i=0; i < (DECIMATION/16) ; i++) {
     readings_u[i] = pdm_u.read() & 0xFFFF;
  }

  uint16_t runningsum_m = 0;
  uint16_t runningsum_u = 0;
  
  for (uint8_t samplenum = 0; samplenum < (DECIMATION/16); samplenum++) {
    uint16_t sample = readings_m[samplenum];
    for (int8_t i=0; i<16; i++) {
      // start at the MSB which is the 'first' bit to come down the line, chronologically
      if (sample & 0x8000) {
        runningsum_m += sincfilter[samplenum * 16 + i];
      }
      sample <<= 1;
    }
   sample = readings_u[samplenum];
   for (int8_t i=0; i<16; i++) {
      // start at the MSB which is the 'first' bit to come down the line, chronologically
      if (sample & 0x8000) {
        runningsum_u += sincfilter[samplenum * 16 + i];
      }
      sample <<= 1;
    }
  }

// since we wait for the samples from I2S peripheral, we dont need to delay, we will 'naturally'
// wait the right amount of time between analog writes
  
//  output_data(runningsum_m);
//  graph(runningsum_u);

  double fft_runningsum_m = 1000*((double)(runningsum_m >> 6) - 512.0) / 512.0;  // number between -1000.0 and 1000.0
  double fft_runningsum_u = 1000*((double)(runningsum_u >> 6) - 512.0) / 512.0;  // number between -1000.0 and 1000.0
  
  if(sample_index < samples) {
    if(first_run == true) {
      sumSampleRate += micros() - oldSumMicros;
    }
    vReal_m[sample_index] = fft_runningsum_m;
    sample_index++;
  }
  else {
    if(first_run == true) {
      first_run = false;
      double magic_barry_factor = (1000.0/7096.057);
//      double magic_barry_factor = 1.0;
      samplingFrequency = sumSampleRate/samples * magic_barry_factor;
      SERIALPORT.print(samplingFrequency);
      SERIALPORT.println("Hz (FFT sampling frequency)");
    }
    PrintFFT(*vReal_m, *vImag_m, "Audio");
    PrintFFT(*vReal_u, *vImag_u, "Ultra");
    sample_index = 0;
    memset(vReal_m,0,sizeof(vReal_m));
    memset(vImag_m,0,sizeof(vImag_m));
    memset(vReal_u,0,sizeof(vReal_u));
    memset(vImag_u,0,sizeof(vImag_u));
  }
}

void output_data(uint16_t runningsum) {
  SERIALPORT.print(micros());
  SERIALPORT.print("\t");
  SERIALPORT.println(runningsum >> 6); // convert 16 bit -> 10 bit  
}

void graph(uint16_t runningsum) {
  int temp = runningsum >> 6;
  int lowbound = 300;
  int upbound = 700;
  if(temp <=lowbound) {temp = lowbound;}
  if(temp >=upbound) {temp = upbound;}
  temp = temp - lowbound;
  temp = temp/2;
  for(int16_t i=0; i<temp; i++){
    SERIALPORT.print(" ");
    if(i > 1000){break;}
  }
  SERIALPORT.println("|");
}

void PrintFFT(double vReal, double vImag, char type[5]){
  /* Print the results of the sampling according to time */
//  SERIALPORT.println("Data:");
//  PrintVector(vReal, samples, SCL_TIME);
  
  FFT.Windowing(&vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  
//  SERIALPORT.println("Weighed data:");
//  PrintVector(vReal, samples, SCL_TIME);
  
  FFT.Compute(&vReal, &vImag, samples, FFT_FORWARD); /* Compute FFT */
  
//  SERIALPORT.println("Computed Real values:");
//  PrintVector(vReal, samples, SCL_INDEX);
  
//  SERIALPORT.println("Computed Imaginary values:");
//  PrintVector(vImag, samples, SCL_INDEX);
  
  FFT.ComplexToMagnitude(&vReal, &vImag, samples); /* Compute magnitudes */
  
//  SERIALPORT.println("Computed magnitudes:");
//  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);



  double peak_values[2];
  double x = FFT.MajorPeak(&vReal, samples, samplingFrequency, peak_values);
  if(peak_values[1] > 1000){
    SERIALPORT.print(type);
    SERIALPORT.print("\t");
    SERIALPORT.print(peak_values[0], 6);
    SERIALPORT.print("\t");
    SERIALPORT.println(peak_values[1], 6);
  }
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    SERIALPORT.print(abscissa, 6);
    SERIALPORT.print("\t");
    SERIALPORT.print(vData[i], 4);
    SERIALPORT.println();
  }
  SERIALPORT.println();
}

