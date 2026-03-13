#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <ArduinoJson.h>

// ======= Wi-Fi Config (Main AP) =======
const char* ssid = "main";
const char* password = "12345678"; // You can change this

ESP8266WebServer server(80);

// ======= Servo Config =======
Servo myServo;
const int servoPin = 2; // D4 on NodeMCU (change if needed)
const int servoMin = 0;   // Left-most angle
const int servoMax = 180; // Right-most angle

// ======= Camera Movement Config =======
float targetAngle = 90;   // Current target position
float smoothedAngle = 90; // Smoothed position
float alpha = 0.5;        // Smoothing factor (0.0 to 1.0)
float deadband = 2;     // Minimum angle change to apply

// ======= Difference Config =======
float maxDiff = 400.0;  // +/-5 cm difference = full servo swing (adjust later)

// ======= Web Handler =======
void handleData() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (!error && doc.containsKey("difference")) {
      float diff = doc["difference"];
      
      // Map difference to servo angle
      diff = constrain(diff, -maxDiff, maxDiff);
      targetAngle = map(diff, -maxDiff, maxDiff, servoMin, servoMax);
    }
  }
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);
  myServo.write(smoothedAngle); // Start centered

  // Start AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("AP Started. IP: ");
  Serial.println(WiFi.softAPIP());

  // Setup server
  server.on("/update", HTTP_POST, handleData);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  // Apply smoothing
  smoothedAngle = smoothedAngle + alpha * (targetAngle - smoothedAngle);

  // Apply deadband
  if (abs(smoothedAngle - myServo.read()) > deadband) {
    myServo.write(smoothedAngle);
    Serial.print("Moving servo to: ");
    Serial.println(smoothedAngle);
  }

  delay(200); // Small delay for stability
}
