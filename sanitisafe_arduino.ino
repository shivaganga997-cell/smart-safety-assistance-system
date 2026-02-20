/*
 * ============================================================
 *  SANITISAFE - Arduino Mega with LoRa
 *  Collects sanitation worker safety data & location,
 *  then transmits to Municipal Safety System via LoRa.
 * ============================================================
 *
 *  Hardware Connections (Arduino Mega):
 *  ------------------------------------
 *  LoRa Module (SX1276/RFM95):
 *    NSS  -> Pin 53 (SS)
 *    MOSI -> Pin 51
 *    MISO -> Pin 50
 *    SCK  -> Pin 52
 *    RST  -> Pin 9
 *    DIO0 -> Pin 2 (interrupt)
 *
 *  GPS Module (NEO-6M / NEO-8M) via Serial1:
 *    TX   -> Pin 19 (RX1)
 *    RX   -> Pin 18 (TX1)
 *
 *  Gas Sensor MQ-4 (Methane/Toxic Gas):
 *    AO   -> A0
 *
 *  Heart Rate Sensor (MAX30100/Pulse):
 *    AO   -> A1  (analog fallback)
 *    SDA  -> Pin 20 (I2C)
 *    SCL  -> Pin 21 (I2C)
 *
 *  Temperature/Humidity (DHT22):
 *    DATA -> Pin 7
 *
 *  Panic Button:
 *    One side -> Pin 3 (INPUT_PULLUP)
 *    Other    -> GND
 *
 *  Fall Detection (MPU-6050 IMU):
 *    SDA -> Pin 20
 *    SCL -> Pin 21
 *
 *  Status LED:
 *    Green  -> Pin 4  (Worker Safe)
 *    Red    -> Pin 5  (Alert)
 *    Yellow -> Pin 6  (Transmitting)
 *
 *  Buzzer:
 *    +    -> Pin 8
 *
 *  Libraries required:
 *    - LoRa by Sandeep Mistry     (v0.8.0+)
 *    - TinyGPS++ by Mikal Hart     (v1.0.3+)
 *    - DHT sensor library by Adafruit
 *    - MPU6050 by Electronic Cats
 *    - ArduinoJson by Benoit Blanchon (v6+)
 * ============================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <DHT.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

// ──────────────────────────────────────────
//  PIN DEFINITIONS
// ──────────────────────────────────────────
#define LORA_SS   53
#define LORA_RST   9
#define LORA_DIO0  2

#define GAS_SENSOR_PIN  A0
#define PULSE_PIN       A1

#define DHT_PIN   7
#define DHT_TYPE  DHT22

#define PANIC_BTN_PIN 3

#define LED_GREEN  4
#define LED_RED    5
#define LED_YELLOW 6
#define BUZZER_PIN 8

// ──────────────────────────────────────────
//  LORA SETTINGS
// ──────────────────────────────────────────
#define LORA_FREQUENCY     915E6   // 915 MHz (change to 868E6 for EU)
#define LORA_TX_POWER      17      // dBm (max 20)
#define LORA_SPREADING     10      // SF10 for long range
#define LORA_BANDWIDTH     125E3   // 125 kHz
#define LORA_CODING_RATE   5       // 4/5

// ──────────────────────────────────────────
//  THRESHOLDS
// ──────────────────────────────────────────
#define GAS_THRESHOLD          600   // Raw ADC value (tune per sensor)
#define FALL_ACCEL_THRESHOLD   2.5   // g-force
#define TEMP_HIGH_THRESHOLD    40.0  // °C
#define PULSE_LOW_THRESHOLD    50    // BPM
#define PULSE_HIGH_THRESHOLD   120   // BPM
#define TRANSMIT_INTERVAL    30000   // ms between normal transmissions
#define ALERT_INTERVAL        5000   // ms between alert transmissions

// ──────────────────────────────────────────
//  WORKER / DEVICE IDENTITY
// ──────────────────────────────────────────
#define WORKER_ID  "W-001"   // Unique ID per device
#define ZONE_ID    "Z-04"    // Deployment zone

// ──────────────────────────────────────────
//  OBJECT INSTANCES
// ──────────────────────────────────────────
TinyGPSPlus  gps;
DHT          dht(DHT_PIN, DHT_TYPE);
MPU6050      mpu;

// ──────────────────────────────────────────
//  GLOBAL STATE
// ──────────────────────────────────────────
struct WorkerData {
  float    latitude;
  float    longitude;
  float    gpsSpeed;      // km/h
  bool     gpsValid;
  float    temperature;   // °C
  float    humidity;      // %
  int      gasLevel;      // raw ADC
  bool     gasAlert;
  int      heartRate;     // BPM (estimated)
  bool     panicPressed;
  bool     fallDetected;
  float    accelMag;      // g
  String   status;        // "SAFE" | "ALERT"
  unsigned long timestamp;
};

WorkerData worker;

unsigned long lastTransmitTime = 0;
unsigned long lastSensorRead   = 0;
bool          inAlertState     = false;

// ──────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);   // GPS UART

  Serial.println(F("=== SANITISAFE Booting ==="));

  // Pin modes
  pinMode(LED_GREEN,    OUTPUT);
  pinMode(LED_RED,      OUTPUT);
  pinMode(LED_YELLOW,   OUTPUT);
  pinMode(BUZZER_PIN,   OUTPUT);
  pinMode(PANIC_BTN_PIN, INPUT_PULLUP);

  // Boot indication
  digitalWrite(LED_YELLOW, HIGH);

  // Init I2C
  Wire.begin();

  // Init DHT
  dht.begin();

  // Init MPU-6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("[WARN] MPU-6050 not found. Fall detection disabled."));
  } else {
    Serial.println(F("[OK] MPU-6050 connected."));
    // Configure sensitivity
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  // ±4g
  }

  // Init LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println(F("[ERR] LoRa init failed! Halting."));
    while (true) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
      delay(200);
    }
  }
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SPREADING);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.enableCrc();
  Serial.println(F("[OK] LoRa initialised."));

  // Attach panic button interrupt
  attachInterrupt(digitalPinToInterrupt(PANIC_BTN_PIN), onPanicPressed, FALLING);

  // Startup beep
  tone(BUZZER_PIN, 1000, 200);
  delay(300);
  tone(BUZZER_PIN, 1200, 200);

  worker.panicPressed = false;
  worker.fallDetected = false;
  worker.status       = "SAFE";

  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN,  HIGH);

  Serial.println(F("=== SANITISAFE Ready ==="));
}

// ──────────────────────────────────────────
//  MAIN LOOP
// ──────────────────────────────────────────
void loop() {
  // 1. Feed GPS parser continuously
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  // 2. Read all sensors every 2 seconds
  if (millis() - lastSensorRead >= 2000) {
    lastSensorRead = millis();
    readGPS();
    readEnvironmental();
    readGasSensor();
    readHeartRate();
    detectFall();
    evaluateWorkerStatus();
    printStatus();
  }

  // 3. Transmit via LoRa
  unsigned long txInterval = inAlertState ? ALERT_INTERVAL : TRANSMIT_INTERVAL;
  if (millis() - lastTransmitTime >= txInterval) {
    lastTransmitTime = millis();
    transmitData();
  }

  // 4. Handle indicators
  updateIndicators();
}

// ──────────────────────────────────────────
//  SENSOR FUNCTIONS
// ──────────────────────────────────────────

void readGPS() {
  if (gps.location.isValid() && gps.location.age() < 5000) {
    worker.latitude  = gps.location.lat();
    worker.longitude = gps.location.lng();
    worker.gpsSpeed  = gps.speed.kmph();
    worker.gpsValid  = true;
  } else {
    worker.gpsValid = false;
  }
}

void readEnvironmental() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) worker.temperature = t;
  if (!isnan(h)) worker.humidity    = h;
}

void readGasSensor() {
  worker.gasLevel = analogRead(GAS_SENSOR_PIN);
  worker.gasAlert = (worker.gasLevel > GAS_THRESHOLD);
}

/*
 * Heart rate: simple analog pulse estimate.
 * Replace with MAX30100 library calls if module is present.
 */
void readHeartRate() {
  // Placeholder: map raw ADC to BPM range 40–160
  int raw = analogRead(PULSE_PIN);
  worker.heartRate = map(raw, 0, 1023, 40, 160);
}

void detectFall() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert raw to g  (±4g range → 8192 LSB/g)
  float gx = ax / 8192.0f;
  float gy = ay / 8192.0f;
  float gz = az / 8192.0f;

  // Magnitude of acceleration vector
  worker.accelMag = sqrt(gx*gx + gy*gy + gz*gz);

  // Free-fall followed by impact: simplified single-threshold detection
  if (worker.accelMag > FALL_ACCEL_THRESHOLD) {
    worker.fallDetected = true;
  }
}

// ──────────────────────────────────────────
//  STATUS EVALUATION
// ──────────────────────────────────────────
void evaluateWorkerStatus() {
  bool alert = false;

  if (worker.gasAlert)      alert = true;
  if (worker.panicPressed)  alert = true;
  if (worker.fallDetected)  alert = true;
  if (worker.temperature > TEMP_HIGH_THRESHOLD) alert = true;
  if (worker.heartRate < PULSE_LOW_THRESHOLD || worker.heartRate > PULSE_HIGH_THRESHOLD) alert = true;

  inAlertState   = alert;
  worker.status  = alert ? "ALERT" : "SAFE";
  worker.timestamp = millis();
}

// ──────────────────────────────────────────
//  LORA TRANSMISSION
// ──────────────────────────────────────────
void transmitData() {
  // Build compact JSON payload
  StaticJsonDocument<256> doc;

  doc["id"]   = WORKER_ID;
  doc["zone"] = ZONE_ID;
  doc["stat"] = worker.status;
  doc["t"]    = (int)(worker.timestamp / 1000);  // seconds uptime

  if (worker.gpsValid) {
    doc["lat"] = serialized(String(worker.latitude,  6));
    doc["lon"] = serialized(String(worker.longitude, 6));
    doc["spd"] = (int)worker.gpsSpeed;
  } else {
    doc["lat"] = nullptr;
    doc["lon"] = nullptr;
  }

  doc["tmp"]  = (int)worker.temperature;
  doc["hum"]  = (int)worker.humidity;
  doc["gas"]  = worker.gasLevel;
  doc["bpm"]  = worker.heartRate;
  doc["fall"] = worker.fallDetected;
  doc["panic"]= worker.panicPressed;

  // Encode JSON to string
  char payload[256];
  size_t len = serializeJson(doc, payload);

  // Blink yellow LED during TX
  digitalWrite(LED_YELLOW, HIGH);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)payload, len);
  bool ok = LoRa.endPacket();

  digitalWrite(LED_YELLOW, LOW);

  if (ok) {
    Serial.print(F("[TX] Sent: "));
    Serial.println(payload);
  } else {
    Serial.println(F("[TX] LoRa send failed."));
  }

  // Clear one-shot flags after transmission
  worker.panicPressed = false;
  worker.fallDetected = false;
}

// ──────────────────────────────────────────
//  INDICATORS
// ──────────────────────────────────────────
void updateIndicators() {
  if (inAlertState) {
    // Fast red blink + buzzer
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      lastBlink = millis();
      digitalWrite(LED_RED,   !digitalRead(LED_RED));
      digitalWrite(LED_GREEN, LOW);
      tone(BUZZER_PIN, 880, 250);
    }
  } else {
    digitalWrite(LED_RED,   LOW);
    digitalWrite(LED_GREEN, HIGH);
    noTone(BUZZER_PIN);
  }
}

// ──────────────────────────────────────────
//  PANIC BUTTON ISR
// ──────────────────────────────────────────
void onPanicPressed() {
  // Debounce inside ISR using a static timestamp
  static unsigned long lastPress = 0;
  if (millis() - lastPress > 1000) {
    lastPress = millis();
    worker.panicPressed = true;
  }
}

// ──────────────────────────────────────────
//  DEBUG PRINT
// ──────────────────────────────────────────
void printStatus() {
  Serial.println(F("--- Worker Status ---"));
  Serial.print(F("Status : ")); Serial.println(worker.status);

  if (worker.gpsValid) {
    Serial.print(F("GPS    : "));
    Serial.print(worker.latitude, 6); Serial.print(F(", "));
    Serial.println(worker.longitude, 6);
  } else {
    Serial.println(F("GPS    : No fix"));
  }

  Serial.print(F("Temp   : ")); Serial.print(worker.temperature); Serial.println(F(" °C"));
  Serial.print(F("Hum    : ")); Serial.print(worker.humidity);    Serial.println(F(" %"));
  Serial.print(F("Gas    : ")); Serial.print(worker.gasLevel);
  if (worker.gasAlert) Serial.print(F(" !! ALERT"));
  Serial.println();
  Serial.print(F("BPM    : ")); Serial.println(worker.heartRate);
  Serial.print(F("AccelG : ")); Serial.println(worker.accelMag);
  Serial.print(F("Fall   : ")); Serial.println(worker.fallDetected   ? "YES" : "no");
  Serial.print(F("Panic  : ")); Serial.println(worker.panicPressed   ? "YES" : "no");
  Serial.println(F("---------------------"));
}
