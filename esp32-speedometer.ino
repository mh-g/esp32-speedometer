#include "float.h"
#include "arduinoFFT.h"

#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// BME interface using https://github.com/finitespace/BME280 (BME280 by Tyler Glenn)
#include "BME280I2C.h"
#include "EnvironmentCalculations.h"
#include "Wire.h"

#define SEALEVELPRESSURE_HPA (1013.25)
#define BMESUBSAMPLE (25)

// library objects
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
BluetoothSerial SerialBT;
arduinoFFT FFT;

// average environmental data
struct envContainer {
  float minimum;
  float maximum;
  float sum;
  int samples;

  public:
    // initialize
    envContainer () { startOver (); }
    
    void startOver () {
      minimum = FLT_MAX;
      maximum = FLT_MIN;
      sum = 0.0;
      samples = 0;
    }

    // process another sample
    void addSample (float value) {
      sum += value;
      ++samples;
      if (value < minimum) minimum = value;
      if (value > maximum) maximum = value;
    }

    // return latest measurement
    float getValue () { return sum / samples; }

    // indicate if measurement is of high quality
    bool goodQuality () { return samples >= BMESUBSAMPLE / 2; }
};

const int analogInput = 26; // used IO pin: 26 (ADC2-9)
int messageCounter = 0; // counter for SPEED messages
int sampleCounter; // counter for FFT input samples
const int outputFrequency = 2; // number of FFTs per second
const uint16_t samples = 1024; // length of FFT
double vReal[samples]; // FFT buffer
double vImag[samples];
double maxAmplitude = 0.0; // maximum amplitude of the last FFT input gathering cycle
unsigned int delayUs = 1000000 / outputFrequency / samples; // delay between two samples (design: 1024 samples per second)
bool bmeAvailable;
int bmeSubsampler;
envContainer envTemperature;
envContainer envPressure;
envContainer envAltitude;
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

  // we can also slow down the CPU a bit to save some mA
  setCpuFrequencyMhz(80);

  if (SerialBT.begin("ESP32speedometer")) {   // sets Bluetooth device name
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

  // initialize BME280 for environmental data and storage classes
  Wire.begin();
  bmeAvailable = bme.begin();
  if (bmeAvailable) {
    switch (bme.chipModel()) {
      case BME280::ChipModel_BME280:
        output("Found BME280 sensor. Success.\n");
        break;
      case BME280::ChipModel_BMP280:
        output("Found BMP280 sensor. No Humidity available.\n");
        break;
      default:
        output("Found UNKNOWN sensor. Error!\n");
        bmeAvailable = false;    
    }
  } else {
    output("No BME280/BMP280 found. No environmental data available.\n");
  }
  if (bmeAvailable) {
    bmeSubsampler = BMESUBSAMPLE;
    envTemperature.startOver();
    envPressure.startOver();
    envAltitude.startOver();
  }
}

double majorPeak () {
  // measure the performance
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  return FFT.MajorPeak(vReal, samples, 1.0 * outputFrequency * samples); // sampling frequency: samples filled in 1 second
}

void loop() {
  // determine if enough time has passed to obtain a new sample value
  microSecondsSinceBoot = esp_timer_get_time();
  if (microSecondsSinceBoot >= delayUs + lastSampleTime)
  {
    // get next environmental data samples
    if (bmeAvailable) {
      --bmeSubsampler;
      if (bmeSubsampler == 0) {
        bmeSubsampler = BMESUBSAMPLE;
        float temp(NAN), hum(NAN), pres(NAN);
        
        BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
        BME280::PresUnit presUnit(BME280::PresUnit_Pa);
        
        bme.read(pres, temp, hum, tempUnit, presUnit);
  
        envTemperature.addSample(temp);
//        envHumidity.addSample(hum);
        envPressure.addSample(pres / 100.0F);
        envAltitude.addSample(
          Altitude(pres / 100.0f, EnvironmentCalculations::AltitudeUnit::AltitudeUnit_Meters, 
            SEALEVELPRESSURE_HPA, temp, EnvironmentCalculations::TempUnit::TempUnit_Celsius));
      }
    }
    
    // get next hub dynamo voltage sample
    vReal[sampleCounter] = analogRead (analogInput);
    vImag[sampleCounter] = 0;
    if (vReal[sampleCounter] > maxAmplitude)
    {
      maxAmplitude = vReal[sampleCounter];
    }
    sampleCounter = sampleCounter + 1;

    // calculate FFT and output all messages
    if (sampleCounter == samples) {
      // uptime
      sprintf (message, ">%s:SU#%03d=%f<\n", mac, messageCounter, 0.001 * (microSecondsSinceBoot / 1000));
      output(message);
      // hub dynamo data
      sprintf (message, ">%s:VF#%03d=%f<\n", mac, messageCounter, majorPeak());
      output(message);
      sprintf (message, ">%s:VA#%03d=%f<\n", mac, messageCounter, maxAmplitude);
      output(message);
      // environmental data
      if (bmeAvailable) {
        sprintf (message, ">%s:EL#%03d=%f<\n", mac, messageCounter, envAltitude.getValue());
        output(message);
        sprintf (message, ">%s:EP#%03d=%f<\n", mac, messageCounter, envPressure.getValue());
        output(message);
        sprintf (message, ">%s:ET#%03d=%f<\n", mac, messageCounter, envTemperature.getValue());
        output(message);
      }
      // software version
      sprintf (message, ">%s:SB#%03d=%s<\n", mac, messageCounter, __DATE__ " " __TIME__);
      output(message);
      
      // increase message counter (always in the range [0..999])
      messageCounter += 1;
      if (messageCounter == 1000) messageCounter = 0;
      
      // reset sample counter and maximum amplitude
      sampleCounter = 0;
      maxAmplitude = 0.0;

      // reset environmental values
      bmeSubsampler = BMESUBSAMPLE;
      envTemperature.startOver();
      envPressure.startOver();
      envAltitude.startOver();
    }
    lastSampleTime += delayUs;
  } else { // wait a little more before trying to sample again
    delayMicroseconds (10);
  }
}
