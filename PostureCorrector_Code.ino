/*
 * PROJECT: IoT Smart Posture Corrector
 * AUTHOR: Justin Jeam Crisostomo
 * HARDWARE: Arduino Uno, MPU-6050 Accelerometer, ESP-01S Wi-Fi Module, Active Buzzer
 * DESCRIPTION: Detects spinal tilt in real-time. Alerts user via buzzer if slouching 
 * and uploads data to ThingSpeak cloud for long-term tracking.
 */

#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ================================================================
// 1. HARDWARE CONFIGURATION
// ================================================================
// We use SoftwareSerial to create a second serial port for the Wi-Fi module
// because the main hardware serial is used for the computer USB connection.
SoftwareSerial esp8266(4, 3); // Pin 2 = RX (Connects to ESP TX), Pin 3 = TX (Connects to ESP RX)

Adafruit_MPU6050 mpu;   // Create the sensor object
const int BUZZER_PIN = 9; // The pin controlling the haptic feedback

// ================================================================
// 2. WI-FI & CLOUD CREDENTIALS
// ================================================================
// Note: ESP8266 only supports 2.4GHz networks.
String wifiSSID = "Type_Wifi_Name_here";      
String wifiPass = "Type_Wifi_Password_here"; 

// ThingSpeak Settings
String apiKey = "Type_your_API_Key"; // Write API Key for Channel ID
String host = "api.thingspeak.com"; // The server we are sending data to the host link
String port = "80";                 // Standard HTTP port

// ================================================================
// 3. POSTURE LOGIC VARIABLES
// ================================================================
const float SLOUCH_THRESHOLD = 25.0; // Angle (degrees) that triggers the alarm
float baselineAngle = 0;             // Stores the user's "perfect posture" angle
bool isCalibrated = false;           // Flag to check if start-up is done

// ================================================================
// 4. TIMER VARIABLES (Non-blocking delay)
// ================================================================
unsigned long lastConnectionTime = 0;    // Tracks the last time we sent data
const unsigned long postingInterval = 20000; // Wait 20 seconds between uploads (to respect API limits)

void setup() {
  // Start the USB Serial Monitor for debugging
  Serial.begin(115200);       
  
  // ================================================================
  //  BAUD RATE STABILIZATION SEQUENCE
  // ================================================================
  // The ESP8266 defaults to 115200, but SoftwareSerial on Arduino Uno
  // is unstable at that speed. We must force it down to 9600.
  esp8266.begin(115200);      
  delay(100);
  
  // Send AT command to permanently set ESP8266 to 9600 baud
  esp8266.println("AT+UART_DEF=9600,8,1,0,0"); 
  delay(2000); 
  
  // Restart the serial connection at the new, stable speed
  esp8266.end();  
  esp8266.begin(9600); 
  // ================================================================
  
  // Initialize Output Pins
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off

  Serial.println("--- SMART POSTURE CORRECTOR STARTING ---");

  // Initialize the MPU-6050 Sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    // If sensor fails, beep forever to warn the user
    while (1) { digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW); delay(100); }
  }
  Serial.println("MPU6050 Found!");
  
  // Set Sensor Sensitivity
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Run Startup Routines
  connectToWifi();     // Connect to the hotspot/router
  calibratePosture();  // Set the zero baseline for the specific user
}

void loop() {
  // 1. READ SENSOR DATA
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 2. CALCULATE POSTURE ANGLE
  // Uses Trigonometry (atan2) to convert X and Z acceleration into an angle
  float currentAngle = atan2(a.acceleration.x, a.acceleration.z) * 57.29578; // 57.29... converts Radians to Degrees
  
  // Calculate absolute deviation from the baseline
  float slouchAngle = abs(currentAngle - baselineAngle);

  // 3. HAPTIC FEEDBACK LOGIC
  if (slouchAngle > SLOUCH_THRESHOLD) {
    digitalWrite(BUZZER_PIN, HIGH); // Bad Posture: Buzz ON
  } else {
    digitalWrite(BUZZER_PIN, LOW);  // Good Posture: Buzz OFF
  }

  // 4. CLOUD UPLOAD (Timer Logic)
  // Only upload if 20 seconds have passed since the last upload
  if (millis() - lastConnectionTime > postingInterval) {
    sendToThingSpeak(slouchAngle);
    lastConnectionTime = millis(); // Reset the timer
  }
  
  delay(100); // Small stability delay
}

// ================================================================
// HELPER FUNCTIONS
// ================================================================

// Captures the user's current position to use as "0 degrees"
void calibratePosture() {
  Serial.println(">>> CALIBRATION: Stand Straight for 3 Seconds... <<<");
  // Short beep to signal start of calibration
  digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW);
  delay(3000); // Wait for user to settle
  
  // Read sensor and set global baseline
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  baselineAngle = atan2(a.acceleration.x, a.acceleration.z) * 57.29578;
  
  Serial.print("Baseline set to: "); Serial.println(baselineAngle);
  // Long beep to signal success
  digitalWrite(BUZZER_PIN, HIGH); delay(600); digitalWrite(BUZZER_PIN, LOW);
}

// Handles the AT commands to join the Wi-Fi network
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  sendData("AT+RST\r\n", 2000, false);        // Reset module
  sendData("AT+CWMODE=1\r\n", 1000, false);   // Set to Station Mode (Client)
  
  // Join the Access Point (Hotspot)
  String cmd = "AT+CWJAP=\"" + wifiSSID + "\",\"" + wifiPass + "\"\r\n";
  sendData(cmd, 10000, true); // 10s timeout to allow for connection handshake
}

// Constructs the HTTP Packet to send data to ThingSpeak
void sendToThingSpeak(float angle) {
  // 1. Open TCP Connection to ThingSpeak Server
  String cmd = "AT+CIPSTART=\"TCP\",\"" + host + "\"," + port + "\r\n";
  sendData(cmd, 2000, false);

  // 2. Construct HTTP GET Request
  // IMPORTANT: We use HTTP/1.1 and include the "Host" header.
  // This fixes the "400 Bad Request" error by telling the server exactly who we want to talk to.
  String url = "GET /update?api_key=" + apiKey + "&field1=" + String(angle) + " HTTP/1.1\r\n" +
               "Host: api.thingspeak.com\r\n" + 
               "Connection: close\r\n\r\n";

  // 3. Send the Data Length first (Required by ESP8266)
  String sCmd = "AT+CIPSEND=" + String(url.length()) + "\r\n";
  sendData(sCmd, 1000, false);
  
  // 4. Send the actual Data Packet
  sendData(url, 2000, true);
  
  Serial.print("Data Sent: Angle = "); Serial.println(angle);
}

// Sends AT commands to ESP8266 and waits for a response
String sendData(String command, const int timeout, boolean debug) {
  String response = "";
  esp8266.print(command); // Send command to module
  
  long int time = millis();
  // Wait for the specific timeout duration
  while ((time + timeout) > millis()) {
    while (esp8266.available()) {
      char c = esp8266.read(); // Read character from module
      response += c;
    }
  }
  
  // Print response to Serial Monitor if debug is true
  if (debug) { 
    Serial.print(response); 
  }
  return response;
}
