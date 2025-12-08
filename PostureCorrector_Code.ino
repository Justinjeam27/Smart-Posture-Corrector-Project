#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- HARDWARE SETTINGS ---
// ESP8266 Connections:
// ESP TX -> Arduino Pin 2
// ESP RX -> Arduino Pin 3 (via Voltage Divider)
SoftwareSerial esp8266(2, 3); // RX, TX

Adafruit_MPU6050 mpu;
const int BUZZER_PIN = 9;

// --- WIFI & THINGSPEAK SETTINGS ---
// NOTE: Credentials removed for GitHub security. 
// Update these lines on your local board before uploading.
String wifiSSID = "YOUR_WIFI_NAME_HERE";      
String wifiPass = "YOUR_WIFI_PASSWORD_HERE"; 

String apiKey = "YOUR_API_KEY_HERE"; // Your ThingSpeak Write Key
String host = "api.thingspeak.com";
String port = "80";

// --- POSTURE SETTINGS ---
const float SLOUCH_THRESHOLD = 25.0; 
float baselineAngle = 0;
bool isCalibrated = false;

// --- TIMERS ---
unsigned long lastConnectionTime = 0;
const unsigned long postingInterval = 20000; // Send data every 20 seconds

void setup() {
  // 1. Start Serial Monitors
  Serial.begin(115200);       // For your computer
  esp8266.begin(115200);      // For the Wi-Fi Module
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("--- SMART POSTURE CORRECTOR STARTING ---");

  // 2. Initialize Sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW); delay(100); }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 3. Connect to Wi-Fi
  connectToWifi();

  // 4. Calibrate Posture
  calibratePosture();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate Posture
  float currentAngle = atan2(a.acceleration.x, a.acceleration.z) * 57.29578;
  float slouchAngle = abs(currentAngle - baselineAngle);

  // Check Slouch
  if (slouchAngle > SLOUCH_THRESHOLD) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // --- SEND DATA TO THINGSPEAK ---
  if (millis() - lastConnectionTime > postingInterval) {
    sendToThingSpeak(slouchAngle);
    lastConnectionTime = millis();
  }
  
  delay(100);
}

// --- HELPER FUNCTIONS ---

void calibratePosture() {
  Serial.println(">>> CALIBRATION: Stand Straight for 3 Seconds... <<<");
  digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW);
  delay(3000); 
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  baselineAngle = atan2(a.acceleration.x, a.acceleration.z) * 57.29578;
  
  Serial.print("Baseline set to: "); Serial.println(baselineAngle);
  digitalWrite(BUZZER_PIN, HIGH); delay(600); digitalWrite(BUZZER_PIN, LOW);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  sendData("AT+RST\r\n", 2000, false); 
  sendData("AT+CWMODE=1\r\n", 1000, false);
  String cmd = "AT+CWJAP=\"" + wifiSSID + "\",\"" + wifiPass + "\"\r\n";
  sendData(cmd, 5000, true);
  Serial.println("Wi-Fi Connected!");
}

void sendToThingSpeak(float angle) {
  String cmd = "AT+CIPSTART=\"TCP\",\"" + host + "\"," + port + "\r\n";
  sendData(cmd, 1000, false);

  String url = "GET /update?api_key=" + apiKey + "&field1=" + String(angle) + "\r\n";
  String sCmd = "AT+CIPSEND=" + String(url.length()) + "\r\n";
  sendData(sCmd, 1000, false);
  sendData(url, 1000, true);
  
  Serial.print("Data Sent: Angle = "); Serial.println(angle);
}

String sendData(String command, const int timeout, boolean debug) {
  String response = "";
  esp8266.print(command);
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (esp8266.available()) {
      char c = esp8266.read();
      response += c;
    }
  }
  if (debug) { 
    Serial.print(response); 
  }
  return response;
}
