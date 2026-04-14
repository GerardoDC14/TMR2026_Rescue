#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// =========================
// Pines y configuración
// =========================
#define MQ2_PIN 12
#define I2C_SDA 21
#define I2C_SCL 22

// =========================
// Configuración MQ-2
// =========================
const int MQ2_CLEAN_ADC = 3750;
const int MQ2_MAX_ADC   = 4095;
const float MQ2_MAX_PPM = 2000.0;

// =========================
// Objetos de sensores
// =========================
Adafruit_AMG88xx amg;
Adafruit_LIS3MDL lis3mdl;

// =========================
// Variables
// =========================
float pixels[64];

float mq2_ppm = 0.0;

// Lecturas actuales del magnetómetro
float mag_x = 0.0;
float mag_y = 0.0;
float mag_z = 0.0;

// Línea base inicial del magnetómetro
float mag_x0 = 0.0;
float mag_y0 = 0.0;
float mag_z0 = 0.0;

// Delta respecto a línea base
float mag_dx = 0.0;
float mag_dy = 0.0;
float mag_dz = 0.0;

// =========================
// Setup
// =========================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Iniciando sensores...");

  Wire.begin(I2C_SDA, I2C_SCL);

  // MQ-2
  pinMode(MQ2_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);

  // LIS3MDL
  if (!lis3mdl.begin_I2C())
  {
    Serial.println("Error: no se encontró el LIS3MDL");
    while (1) delay(10);
  }
  Serial.println("LIS3MDL detectado");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);

  // Prueba cambiar rango si acercas imanes fuertes
  lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);

  // AMG8833
  if (!amg.begin())
  {
    Serial.println("Error: no se encontró el AMG8833");
    while (1) delay(10);
  }
  Serial.println("AMG8833 detectado");

  // Calibración rápida inicial del magnetómetro
  calibrarMagnetometro();

  Serial.println("Sensores iniciados correctamente");
}

// =========================
// Loop
// =========================
void loop()
{
  leerMQ2();
  leerLIS3MDL();
  leerAMG8833();
  enviarSerialCompacto();

  delay(1000);
}

// =========================
// MQ-2
// =========================
void leerMQ2()
{
  const int muestras = 10;
  long suma = 0;

  for (int i = 0; i < muestras; i++)
  {
    suma += analogRead(MQ2_PIN);
    delay(5);
  }

  int raw = suma / muestras;

  if (raw <= MQ2_CLEAN_ADC)
  {
    mq2_ppm = 0.0;
  }
  else
  {
    mq2_ppm = ((float)(raw - MQ2_CLEAN_ADC) / (MQ2_MAX_ADC - MQ2_CLEAN_ADC)) * MQ2_MAX_PPM;
  }

  if (mq2_ppm < 0.0) mq2_ppm = 0.0;
  if (mq2_ppm > MQ2_MAX_PPM) mq2_ppm = MQ2_MAX_PPM;
}

// =========================
// LIS3MDL
// =========================
void calibrarMagnetometro()
{
  const int muestras = 50;
  float sx = 0.0, sy = 0.0, sz = 0.0;

  Serial.println("Calibrando magnetómetro... no acerques imanes");

  for (int i = 0; i < muestras; i++)
  {
    sensors_event_t event;
    lis3mdl.getEvent(&event);

    sx += event.magnetic.x;
    sy += event.magnetic.y;
    sz += event.magnetic.z;

    delay(20);
  }

  mag_x0 = sx / muestras;
  mag_y0 = sy / muestras;
  mag_z0 = sz / muestras;

  Serial.print("Baseline MAG: ");
  Serial.print(mag_x0, 2); Serial.print(", ");
  Serial.print(mag_y0, 2); Serial.print(", ");
  Serial.println(mag_z0, 2);
}

void leerLIS3MDL()
{
  sensors_event_t event;
  lis3mdl.getEvent(&event);

  mag_x = event.magnetic.x;
  mag_y = event.magnetic.y;
  mag_z = event.magnetic.z;

  mag_dx = mag_x - mag_x0;
  mag_dy = mag_y - mag_y0;
  mag_dz = mag_z - mag_z0;
}

// =========================
// AMG8833
// =========================
void leerAMG8833()
{
  amg.readPixels(pixels);
}

// =========================
// Envío serial compacto
// Formato:
// <MQ2:ppm|MAG:dx,dy,dz|THERM:p0,p1,...,p63>
// =========================
void enviarSerialCompacto()
{
  Serial.print("<");

  Serial.print("MQ2:");
  Serial.print(mq2_ppm, 1);

  Serial.print("|");

  Serial.print("MAG:");
  Serial.print(mag_dx, 2);
  Serial.print(",");
  Serial.print(mag_dy, 2);
  Serial.print(",");
  Serial.print(mag_dz, 2);

  Serial.print("|");

  Serial.print("THERM:");
  for (int i = 0; i < 64; i++)
  {
    Serial.print(pixels[i], 1);
    if (i < 63) Serial.print(",");
  }

  Serial.println(">");
}