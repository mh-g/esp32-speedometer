#include "arduinoFFT.h"

#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
arduinoFFT FFT;

const int analogInput = 2; // used IO pin: ADC2-2
int messageCounter = 0; // counter for SPEED messages
int sampleCounter; // counter for FFT input samples
const uint16_t samples = 1024; // length of FFT
double vReal[samples]; // FFT buffer
double vImag[samples];
double maxAmplitude = 0.0; // maximum amplitude of the last FFT input gathering cycle
unsigned int delayUs = 1000000 / samples; // delay between two samples (design: 1024 samples per second)
char message[64]; // buffer for output message preparation
char mac[7]; // hex representation of the last three bytes of the MAC address
const char compile_version[] = __DATE__ " " __TIME__;
int64_t microSecondsSinceBoot; // current time
int64_t lastSampleTime = 0; // time of last sample

// general output function, to easily tailor output to serial and/or bluetooth
void output(char* message) {
  Serial.print (message);
  SerialBT.print (message);
}

void setup() {
  // comms initialization
  Serial.begin(115200); //sets the baud rate of the UART for serial transmission
  while (!Serial);

  if (SerialBT.begin("ESP32test")) {   // sets Bluetooth device name
    output ("SerialBT.begin successful.\n");
  }

  // get MAC address
  const uint8_t* point = esp_bt_dev_get_address();
  sprintf(mac, "%02X%02X%02X", (int)point[3], (int)point[4], (int)point[5]);

  // prepare FFT
  FFT = arduinoFFT();
  sampleCounter = 0;

  // sample storage initialization
  for (int i = 0; i < samples; ++i) {
    vReal[i] = 0.0;
    vImag[i] = 0.0;
  }
}

double majorPeak () {
  // measure the performance
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  return FFT.MajorPeak(vReal, samples, 1.0 * samples); // sampling frequency: samples filled in 1 second
}

void loop() {
  // determine if enough time has passed to obtain a new sample value
  microSecondsSinceBoot = esp_timer_get_time();
  if (microSecondsSinceBoot >= delayUs + lastSampleTime)
  {
    // get next sample
    vReal[sampleCounter] = analogRead (analogInput);
    if (vReal[sampleCounter] > maxAmplitude)
    {
      maxAmplitude = vReal[sampleCounter];
    }
    sampleCounter = sampleCounter + 1;

    // calculate FFT and output all messages
    if (sampleCounter == samples) {
      sprintf (message, ">%s:SU#%03d=%f<\n", mac, messageCounter, 0.001 * (microSecondsSinceBoot / 1000));
      output(message);
      sprintf (message, ">%s:VF#%03d=%f<\n", mac, messageCounter, majorPeak());
      output(message);
      sprintf (message, ">%s:VA#%03d=%f<\n", mac, messageCounter, maxAmplitude);
      output(message);
      sprintf (message, ">%s:SB#%03d=%s<\n", mac, 0, __DATE__ " " __TIME__);
      output(message);
      
      // increase message counter (always in the range [0..999])
      messageCounter += 1;
      if (messageCounter == 1000) messageCounter = 0;
      
      // reset sample counter and maximum amplitude
      sampleCounter = 0;
      maxAmplitude = 0.0;
    }
    lastSampleTime += delayUs;
  } else { // wait a little more before trying to sample again
    delayMicroseconds (10);
  }
}
