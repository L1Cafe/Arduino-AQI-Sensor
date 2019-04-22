#include <bme680.h>
#include <Adafruit_BME680.h>
#include <bme680_defs.h>

#include <Arduino.h>

// Analog 0 A0 is for MiCS5524, for CO and VOC
// Digital D2 for PM2.5 TX (UART)
// SDA and SCL for:
// - 0x77 for BME680 --OK
// - 0x58 for SGP30 for TVOC eCO2 -- OK
//    SDA/SDI is GRAY WIRE
//    SCK/SCL is ORANGE WIRE
// TODO: https://github.com/jbanaszczyk/pms5003

#include <Wire.h>

// GENERICS
float temperature; // in Celsius
float humidity;    // in %

// SGP30
#include "Adafruit_SGP30.h"
Adafruit_SGP30 sgp;
uint16_t TVOC; // Total Volatile Organic Compounds in ppb
uint16_t eCO2; // equivalent CO2 in ppm
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

// BME680
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
Adafruit_BME680 bme;
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
float pressure; // atmospheric pressure in hPa
float gas;      // ambient gas in KOhms

// MiCS5524
#define MiCS5524 A0
int gasses;

// PMS5003
#include <SoftwareSerial.h>
#define PMS5003 2    // IN from sensor
#define PMS5003off 3 // The sensor does not read incoming data, so leave D3 disconnected
SoftwareSerial pmsSerial(PMS5003, PMS5003off);
struct pmdata
{
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard; // Standard concentration units
  uint16_t pm10_env, pm25_env, pm100_env; // Environmental concentration units
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um; // Particle count per 0.1L of air
  uint16_t unused;
  uint16_t checksum;
};
struct pmdata pms5003data;

void setup()
{
  Serial.begin(115200);

  // BME680
  if (!bme.begin())
  {
    Serial.println("BME680 sensor not found");
    while (1);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // SGP30
  if (!sgp.begin())
  {
    Serial.println("SGP30 sensor not found");
    while (1);
  }
  // Serial.print("SGP30 serial #: ");
  // Serial.print(sgp.serialnumber[0], HEX);
  // Serial.print(sgp.serialnumber[1], HEX);
  // Serial.println(sgp.serialnumber[2], HEX);
  // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
  //sgp.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!

  // PMS5003
  pmsSerial.begin(9600);

  // Startup sequence
  Serial.println("Starting up...");
  delay(10000); // 10 second startup delay
}

boolean readPMSdata(Stream *s)
{
  if (!s->available())
  {
    return false;
  }
  if (s->peek() != 0x42) // Magic number
  {
    s->read();
    return false;
  }

  if (s->available() < 32)
  {
    return false;
  }
  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  for (uint8_t i = 0; i < 30; i++)
  {
    sum += buffer[i];
  }

  // The data comes in endian'd, let's fix it
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++)
  {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  };

  // Place it into struct
  memcpy((void *)&pms5003data, (void *)buffer_u16, 30);

  if(sum != pms5003data.checksum)
  {
    Serial.println("PMS5003 checksum failure");
    return false;
  }
  return true;
}

void loop()
{
  // BME680
  if (!bme.performReading())
  {
    Serial.println("BME680 measurement failed");
  }
  else
  {
    humidity = bme.humidity;
    temperature = bme.temperature;
    pressure = bme.pressure / 100.0;
    gas = bme.gas_resistance / 1000.0;
  }

  // SGP30
  // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));
  if (!sgp.IAQmeasure())
  {
    Serial.println("SGP30 measurement failed");
  }
  else
  {
    TVOC = sgp.TVOC;
    eCO2 = sgp.eCO2;
  }

  // MiCS5524 simple voltage reading
  gasses = analogRead(A0);

  // PMS5003
  if (readPMSdata(&pmsSerial))
  {
  }
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("=== BEGINNING OF TRANSMISSION ================");
  Serial.println("Basic atmospheric values =====================");

  Serial.print("Temperature:\t\t\t\t\t");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Humidity:\t\t\t\t\t\t");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Pressure:\t\t\t\t\t\t");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Ambient gas:\t\t\t\t\t");
  Serial.print(gas);
  Serial.println(" kOhms");

  Serial.println("Pollution values =============================");

  Serial.print("PIV:\t\t\t\t\t\t\t"); /* Pollution Index Value */
  Serial.println(gasses);

  Serial.print("eCO2:\t\t\t\t\t\t\t");
  Serial.println(eCO2);

  Serial.print("TVOC:\t\t\t\t\t\t\t");
  Serial.println(TVOC);

  Serial.println("Particulate matter values ====================");
  Serial.print("PM 1.0:\t\t\t\t\t\t\t");
  Serial.println(pms5003data.pm10_env);
  Serial.print("PM 2.5:\t\t\t\t\t\t\t");
  Serial.println(pms5003data.pm25_env);
  Serial.print("PM 10:\t\t\t\t\t\t\t");
  Serial.println(pms5003data.pm100_env);
  Serial.print("Particles > 0.3um / 0.1L air:\t");
  Serial.println(pms5003data.particles_03um);
  Serial.print("Particles > 0.5um / 0.1L air:\t");
  Serial.println(pms5003data.particles_05um);
  Serial.print("Particles > 10um / 0.1L air:\t");
  Serial.println(pms5003data.particles_10um);
  Serial.print("Particles > 25um / 0.1L air:\t");
  Serial.println(pms5003data.particles_25um);
  Serial.print("Particles > 50um / 0.1L air:\t");
  Serial.println(pms5003data.particles_50um);
  Serial.print("Particles > 100um / 0.1L air:\t");
  Serial.println(pms5003data.particles_100um);

  Serial.println("=== END OF TRANSMISSION ======================");

  delay(5000);
}