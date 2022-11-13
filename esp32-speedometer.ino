#include "arduinoFFT.h"

#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
arduinoFFT FFT;

const int analogInput = 2; // ADC2-2
int messageCounter = 0;
int sampleCounter;
const uint16_t samples = 1024;
double vReal[samples];
double vImag[samples];
unsigned int delayUs;
const double samplingFrequency = 1000;  // Hz
char message[64];
char mac[7];
const char compile_version[] = __DATE__ " " __TIME__;
int64_t microSecondsSinceBoot;
int64_t lastSampleTime = 0;
int64_t nextDelay;

void output(char* message) {
  Serial.print (message);
  SerialBT.print (message);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //sets the baud rate of the UART for serial transmission
  while (!Serial);

  if (SerialBT.begin("ESP32test")) {   // sets Bluetooth device name
    output ("SerialBT.begin successful.\n");
  }

  // some initialization stuff
  FFT = arduinoFFT();
  delayUs = 1000; // = 1000 Hz
  sampleCounter = 0;

  const uint8_t* point = esp_bt_dev_get_address();
  sprintf(mac, "%02X%02X%02X", (int)point[3], (int)point[4], (int)point[5]);
  
// debug code
  double cycles = 10.0;
  for (int i = 0; i < samples; ++i) {
    vReal[i] = int8_t((50.0 * (sin((i * (twoPi * cycles)) / samples))) / 2.0);
    vImag[i] = 0.0;
  }
}

double majorPeak () {
  // measure the performance
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  return FFT.MajorPeak(vReal, samples, samplingFrequency);
}

void loop() {
  // put your main code here, to run repeatedly:
  microSecondsSinceBoot = esp_timer_get_time();
  if (microSecondsSinceBoot > 1000 + lastSampleTime)
  {
    vReal[sampleCounter] = analogRead (analogInput);
  
    if (sampleCounter == 999) {
      sprintf (message, ">%s:SU#%03d=%f<\n", mac, messageCounter, 0.001 * (microSecondsSinceBoot / 1000));
      output(message);
      sprintf (message, ">%s:VF#%03d=%f<\n", mac, messageCounter, majorPeak());
      output(message);
      sprintf (message, ">%s:SB#%03d=%s<\n", mac, 0, __DATE__ " " __TIME__);
      output(message);
      sampleCounter = -1;
      messageCounter += 1;
    }
    lastSampleTime += 1000;
    sampleCounter = sampleCounter + 1;
  } else {
    delayMicroseconds (10);
  }
}
