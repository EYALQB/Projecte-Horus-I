#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SensirionI2CScd4x.h>
#include <SD.h>
#include "DHT.h"

#define DHTPIN 2        // Pin del DHT22
#define DHTTYPE DHT22   // Tipo del sensor DHT

#define LED_PIN 7       // Pin del LED

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;
SensirionI2CScd4x scd4x;
File dataFile;

const int chipSelect = 53; // Pin CS para la tarjeta SD


void setup() {
  Serial.begin(9600);
  dht.begin();
  bmp.begin();

  uint16_t error;
  char errorMessage[256];
  Wire.begin();
  scd4x.begin(Wire);
  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  
// Start Measurement
  error = scd4x.startPeriodicMeasurement();

 
  pinMode(chipSelect, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Apagar el LED inicialmente

  if (!SD.begin(chipSelect)) {
    Serial.println("Error al inicializar la tarjeta SD.");
    return;
  }

  dataFile = SD.open("datos.txt", FILE_WRITE);

  if (!dataFile) {
    Serial.println("Error al abrir el archivo.");
    return;
  }
  
  // Escribir el encabezado en el archivo
  dataFile.println("Temperatura (°C),Humedad (%), TemperaturaBMP, Presión (hPa), Altitud, CO2 (ppm),Temperatura CO2, Humedad CO2,Índice UV");
  dataFile.close();
}

void loop() {
  uint16_t error;
  char errorMessage[256];
// Read Measurement
  uint16_t co2 = 0;
  float temperatureCO2 = 0.0f;
  float humidityCO2 = 0.0f;
  bool isDataReady = false;
  error = scd4x.getDataReadyFlag(isDataReady);

    if (!isDataReady) {
        return;
    }
  error = scd4x.readMeasurement(co2, temperatureCO2, humidityCO2);
  
  float temperatureDHT = dht.readTemperature();
  float humidityDHT = dht.readHumidity();
  float temperatureBMP = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F; // Convertir a hPa
  float altitudeBMP = bmp.readAltitude(1013.25);
  
  float uv = analogRead(A0) * (5.0 / 1023.0); // Convertir a voltaje UV
  float uvIndex = uv * 0.1; // Calibración del sensor GUVA-S12SD

  dataFile = SD.open("datos.txt", FILE_WRITE);

  if (dataFile) {
    char temperatureDHTStr[10];
    char humidityDHTStr[10];
    char temperatureBMPStr[10];
    char pressureStr[10];
    char altitudeBMPStr[10];
    char co2Str[10];
    char temperatureCO2Str[10];
    char humidityCO2Str[10];
    char uvIndexStr[10];

    // Convertir las variables a cadenas de texto
    dtostrf(temperatureDHT, 6, 2, temperatureDHTStr);
    dtostrf(humidityDHT, 6, 2, humidityDHTStr);
    dtostrf(temperatureBMP, 6, 2, temperatureBMPStr);
    dtostrf(pressure, 6, 2, pressureStr);
    dtostrf(altitudeBMP, 6, 2, altitudeBMPStr);
    dtostrf(co2, 6, 0, co2Str);
    dtostrf(temperatureCO2, 6, 2, temperatureCO2Str);  
    dtostrf(humidityCO2, 6, 2, humidityCO2Str);   
    dtostrf(uvIndex, 6, 2, uvIndexStr);

    char dataLine[100];
    // Imprimir los datos en una línea separados por comas utilizando snprintf
    snprintf(dataLine, sizeof(dataLine), "%s,%s,%s,%s,%s,%s,%s,%s,%s", temperatureDHTStr, humidityDHTStr, temperatureBMPStr, pressureStr, altitudeBMPStr, co2Str, temperatureCO2Str, humidityCO2Str, uvIndexStr);

//    // Imprimir los datos en el monitor serial
//    Serial.print("Temperatura DHT: ");
//    Serial.print(temperatureDHTStr);
//    Serial.println(" °C");
//    Serial.print("Humedad DHT: ");
//    Serial.print(humidityDHTStr);
//    Serial.println(" %");
//    Serial.print("Presión BMP: ");
//    Serial.print(pressureStr);
//    Serial.println(" hPa");
//    Serial.print("CO2: ");
//    Serial.print(co2Str);
//    Serial.println(" ppm");
//    Serial.print("Índice UV: ");
//    Serial.print(uvIndexStr);
//    Serial.println();

    dataFile.println(dataLine);

    dataFile.close();

    Serial.println("Datos almacenados en la tarjeta SD.");

    // Encender el LED
    digitalWrite(LED_PIN, HIGH);
    delay(100);  // Retraso para el parpadeo
    digitalWrite(LED_PIN, LOW);
  } else {
    Serial.println("Error al abrir el archivo.");
  }

  delay(2000); // Esperar 2 segundos antes de tomar más mediciones
}
