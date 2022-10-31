#include "arduinoFFT.h"

#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
arduinoFFT FFT;

int messageCounter = 0;
int sampleCounter;
const uint16_t samples = 1024; // max value for Arduino nano (already exceeds ram by 206 bytes, but still runs)
double vReal[samples];
double vImag[samples];
unsigned int delayUs;
const double samplingFrequency = 1000;  // Hz
char message[64];
char mac[7];
const char compile_version[] = __DATE__ " " __TIME__;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //sets the baud rate of the UART for serial transmission
  while (!Serial);

  if (SerialBT.begin("ESP32test")) {   // sets Bluetooth device name
    Serial.println ("SerialBT.begin successful.");
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
  delay (2000);
  sprintf (message, ">%s:SB#%03d=%s<\n", mac, 0, __DATE__ " " __TIME__);
  SerialBT.print(message);
}

double majorPeak () {
  // measure the performance
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  return FFT.MajorPeak(vReal, samples, samplingFrequency);
}

void loop() {
  // put your main code here, to run repeatedly:
  vReal[sampleCounter] = analogRead (34);

  if (sampleCounter == 999) {
    sprintf (message, ">%s:VF#%03d=%f<\n", mac, messageCounter, majorPeak());
    Serial.print (message);
    SerialBT.print (message);
    sampleCounter = -1;
    messageCounter += 1;
    delay(1000);
  }
  sampleCounter = sampleCounter + 1;
}
