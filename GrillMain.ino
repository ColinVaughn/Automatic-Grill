#include <SPI.h>
#include <Adafruit_MAX31855.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
#else
  #include <WiFi.h>
  #include <WebServer.h>
#endif

// Thermocouple pins for MAX6675/MAX31855
#define MAXDO   12
#define MAXCS   10
#define MAXCLK  13

// Create the thermocouple instance
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// Stepper motor pins
#define STEP_PIN 9
#define DIR_PIN  8

// Create an instance of the stepper motor
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// PID variables
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// LCD pins for Arduino Mega
#ifdef ESP8266
const int rs = 5, en = 4, d4 = 0, d5 = 2, d6 = 14, d7 = 12;
#else
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
#endif

// Initialize the LCD library with the LCD pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Wi-Fi credentials for Access Point
#ifdef ESP8266
const char ap_ssid[] PROGMEM = "ESP8684_AP";
const char ap_password[] PROGMEM = "password";
#else
const char* ap_ssid = "ESP8684_AP";
const char* ap_password = "password";
#endif

// Create an instance of the server
#ifdef ESP8266
ESP8266WebServer server(80);
#else
WebServer server(80);
#endif

// Pressure sensor pin
const int pressurePin = A0;

// Relay control pin
const int relayPin = 3;

// Pressure thresholds
const float maxSafePressure = 30.0; // in psi
const float reducePressureThreshold = 15.0; // in psi
const float minPressureThreshold = 2.0; // in psi
const float startupPressure = 10.0; // in psi

// Rotary encoder pins
const int CLK = 2; // CLK pin
const int DT = 3;  // DT pin
const int SW = 4;  // Switch pin

volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

bool settingTemperature = false;
int setTemperature = 25; // Initial set temperature

bool shutdownMode = false;
bool startupMode = false;

float readPressure() {
  int sensorValue = analogRead(pressurePin);
  // Convert the analog reading (0-1023) to voltage (0-5V)
  float voltage = sensorValue * (5.0 / 1023.0);
  // Convert voltage to pressure (0-100 psi)
  float pressure = (voltage - 0.5) * (100.0 / 4.0);
  return pressure;
}

// Interrupt service routine for rotary encoder
#ifdef ESP8266
void IRAM_ATTR updateEncoder() {
#else
void updateEncoder() {
#endif
  int MSB = digitalRead(CLK); // Read MSB input
  int LSB = digitalRead(DT);  // Read LSB input

  int encoded = (MSB << 1) | LSB; // Convert the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; // Add current reading to previous one to find the direction

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue++;
  } 
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue--;
  }
  lastEncoded = encoded; // Remember this value for next time
  if (encoderValue != lastencoderValue) {
    if (!settingTemperature) {
      // Enter temperature setting mode if not already in it
      settingTemperature = true;
      setTemperature = Setpoint; // Initialize set temperature to current temperature
    }
    else {
      // Update set temperature based on encoder value
      setTemperature = constrain(Setpoint + (encoderValue * 5), 0, 100); // Increment by 5 degrees
    }
    lastencoderValue = encoderValue; // Remember this value for next time
  }
}

// Handle set temperature URL
void handleSetTemp() {
  if (server.hasArg("temp")) {
    Setpoint = server.arg("temp").toFloat();
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle shutdown URL
void handleShutdown() {
  shutdownMode = true;
  startupMode = false;
  server.send(200, "text/html", "<script>toggleButton('shutdown');</script>");
}

// Handle startup URL
void handleStartup() {
  startupMode = true;
  shutdownMode = false;
  server.send(200, "text/html", "<script>toggleButton('startup');</script>");
}
void handleTemperature() {
  server.send(200, "text/plain", String(Input));
}

void handlePressure() {
  server.send(200, "text/plain", String(readPressure()));
}



void handleRoot() {
  String html = FPSTR(
    "<html>\
    <head>\
      <title>Temperature Control</title>\
      <script>\
        function toggleButton(action) {\
          if (action === 'startup') {\
            document.getElementById('startupForm').style.display = 'none';\
            document.getElementById('shutdownForm').style.display = 'block';\
          } else if (action === 'shutdown') {\
            document.getElementById('shutdownForm').style.display = 'none';\
            document.getElementById('startupForm').style.display = 'block';\
          }\
        }\
        function updateTemperature() {\
          var xhttp = new XMLHttpRequest();\
          xhttp.onreadystatechange = function() {\
            if (this.readyState == 4 && this.status == 200) {\
              document.getElementById('temperature').innerHTML = this.responseText + ' C';\
            }\
          };\
          xhttp.open('GET', '/temperature', true);\
          xhttp.send();\
        }\
        function updatePressure() {\
          var xhttp = new XMLHttpRequest();\
          xhttp.onreadystatechange = function() {\
            if (this.readyState == 4 && this.status == 200) {\
              document.getElementById('pressure').innerHTML = this.responseText + ' psi';\
            }\
          };\
          xhttp.open('GET', '/pressure', true);\
          xhttp.send();\
        }\
        setInterval(function() { updateTemperature(); updatePressure(); }, 1000); \
      </script>\
    </head>\
    <body>\
      <h1>Temperature Control</h1>\
      <p>Current Temperature: <span id='temperature'>");
  html += Input;
  html += F(" C</span></p>\
      <p>Current Pressure: <span id='pressure'>");
  html += readPressure();
  html += F(" psi</span></p>\
      <form id=\"setTempForm\" action=\"/setTemp\" method=\"GET\">\
        <label for=\"temp\">Set Temperature (C):</label>\
        <input type=\"text\" id=\"temp\" name=\"temp\">\
        <input type=\"submit\" value=\"Set\">\
      </form>");
  
  // Display Startup/Shutdown buttons with JavaScript actions
  html += F("<form id=\"startupForm\" action=\"/startup\" method=\"GET\" style=\"display:");
  html += (shutdownMode ? "none" : "block");
  html += F(";\">\
        <input type=\"submit\" value=\"Startup\" onclick=\"toggleButton('startup')\">\
      </form>");
  
  html += F("<form id=\"shutdownForm\" action=\"/shutdown\" method=\"GET\" style=\"display:");
  html += (shutdownMode ? "block" : "none");
  html += F(";\">\
        <input type=\"submit\" value=\"Shutdown\" onclick=\"toggleButton('shutdown')\">\
      </form>");

  html += F("</body>\
    </html>");
  server.send(200, "text/html", html);
}
void setup() {
  // Start serial communication for debugging purposes
  Serial.begin(115200);

  // Initialize the LCD
  lcd.begin(20, 4); // Adjust for 20x4 LCD
  lcd.print("Temp: ");
  
  // Initialize the stepper motor
  stepper.setMaxSpeed(1000); // Set maximum speed for the stepper motor
  stepper.setAcceleration(500); // Set acceleration for the stepper motor

  // Set the desired temperature
  Setpoint = 25.0; // Example setpoint temperature in Celsius

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-1000, 1000); // Stepper motor position limits

  // Initialize the relay pin
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Assume HIGH turns the relay off (valve closed)

  // Set up the ESP8684 or Arduino Mega as an Access Point
  #ifdef ESP8266
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("Access Point started");

  // Print the IP address
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  #endif

  // Rotary encoder setup
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP); // Use internal pull-up resistor
  
  // Attach interrupt for CLK pin
  attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);

  // Define server routes
  server.on("/", handleRoot);
  server.on("/setTemp", handleSetTemp);
  server.on("/shutdown", handleShutdown);
  server.on("/startup", handleStartup);
  server.on("/temperature", handleTemperature);
  server.on("/pressure", handlePressure);
  
  // Start the server
  server.begin();
  Serial.println("Server started");
}



void loop() {
  // Handle client requests
  server.handleClient();

  // Read the temperature from the thermocouple
  Input = thermocouple.readCelsius();
  
  // Check if reading failed
  if (isnan(Input)) {
    Serial.println("Thermocouple reading failed");
    lcd.setCursor(6, 0);
    lcd.print("Error   ");
    return;
  }

  // Print the temperature to the serial monitor
  Serial.print("Current Temperature: ");
  Serial.println(Input);

  // Print the temperature to the LCD
  lcd.setCursor(0, 0); // Adjust for 20x4 LCD
  lcd.print("Current Temp: ");
  lcd.print(Input);
  lcd.print(" C   "); // Add spaces to ensure any previous characters are overwritten

  // Read the pressure sensor
  float pressure = readPressure();

  // Print the pressure to the serial monitor
  Serial.print("Current Pressure: ");
  Serial.println(pressure);

  // Print the pressure to the LCD
  lcd.setCursor(0, 1); // Adjust for 20x4 LCD
  lcd.print("Pressure: ");
  lcd.print(pressure);
  lcd.print(" psi");

  if (shutdownMode) {
    // Shutdown mode
    Serial.println("Shutdown mode activated.");
    // Reduce the pressure and close the valve
    while (pressure > 0) {
      stepper.moveTo(stepper.currentPosition() - 10);
      stepper.run();
      pressure = readPressure();
      lcd.setCursor(0, 1);
      lcd.print("Pressure: ");
      lcd.print(pressure);
      lcd.print(" psi");
      delay(100);
    }
    digitalWrite(relayPin, LOW); // Close the valve
    Serial.println("Valve closed.");
    lcd.setCursor(0, 1);
    lcd.print("Shutdown complete.");
    while (true) {
      // Stay in shutdown mode
      server.handleClient(); // Still handle client requests
      delay(100);
    }
  }

  if (startupMode) {
    // Startup mode
    Serial.println("Startup mode activated.");
    // Increase the pressure and open the valve
    while (pressure < startupPressure) {
      stepper.moveTo(stepper.currentPosition() + 10);
      stepper.run();
      pressure = readPressure();
      lcd.setCursor(0, 1);
      lcd.print("Pressure: ");
      lcd.print(pressure);
      lcd.print(" psi");
      delay(100);
    }
    digitalWrite(relayPin, HIGH); // Open the valve
    Serial.println("Valve opened.");
    lcd.setCursor(0, 1);
    lcd.print("Startup complete.");
    startupMode = false; // Exit startup mode
  }

  // Check pressure levels
  if (pressure > maxSafePressure) {
    // Turn off the relay to close the valve
    digitalWrite(relayPin, LOW);
    Serial.println("Pressure too high! Valve closed.");
    lcd.setCursor(0, 1);
    lcd.print("Valve closed!     ");
    return;
  } else if (pressure > reducePressureThreshold) {
    // Decrease the stepper motor position to reduce pressure
    stepper.moveTo(stepper.currentPosition() - 10);
    Serial.println("Reducing pressure...");
  } else if (pressure < minPressureThreshold) {
    // Stop the stepper motor because the tank is probably low
    stepper.stop();
    Serial.println("Pressure too low! Motor stopped.");
  }

  // Check if temperature setting mode is activated
  if (settingTemperature) {
    // Display set temperature on LCD
    lcd.setCursor(0, 2); // Adjust for 20x4 LCD
    lcd.print("Set Temp: ");
    lcd.print(setTemperature);
    lcd.print(" C");
  }

  // Handle encoder button press to confirm temperature setting
  if (digitalRead(SW) == LOW) {
    Setpoint = setTemperature;
    settingTemperature = false;
    Serial.println("Temperature set to: " + String(setTemperature) + " C");
  }

  // Compute PID output
  myPID.Compute();

  // Control the stepper motor based on PID output
  stepper.moveTo((int)Output);
  stepper.run();

  // Wait for a second before taking another reading
  delay(1000);
}


