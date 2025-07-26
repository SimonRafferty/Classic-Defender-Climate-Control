#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <SimpleKalmanFilter.h>

// --- Pin Definitions ---
// Outputs
const int AC_PIN = 12;
const int HEATER_VALVE_PIN = 13;
const int FAN_LOW_PIN = 27;
const int FAN_MED_PIN = 33;
const int FAN_HIGH_PIN = 32;

// Inputs
const int DEFROST_SWITCH_PIN = 15; // Active HIGH
const int AUTO_MODE_SWITCH_PIN = 14; // Active HIGH

// Analog Inputs (Use ADC1 channels for WiFi compatibility)
// Note: A2=GPIO2, A3=GPIO15 (Used by Auto switch!), A4=GPIO4.
// Let's re-assign Analog inputs to pins usable with WiFi:
// Assuming Huzzah ESP32 Feather pin markings:
// A2 -> IO34 (Used for PT100) - Check Huzzah Pinout! Confirm this. If needed, change.
// A3 -> IO39 (Used for Fan Speed Set) - Check Huzzah Pinout! Confirm this. If needed, change.
// A4 -> IO36 (Used for Temp Set) - Check Huzzah Pinout! Confirm this. If needed, change.
// !!! IMPORTANT: Verify these Analog pin assignments against your specific Huzzah board version !!!
// Using common ESP32 dev board pins for example if A2/3/4 aren't ADC1:
const int INTERIOR_TEMP_PIN = 34; // ADC1_CH6 (Formerly A2 concept)
const int TEMP_SET_PIN = 39;       // ADC1_CH3 (Formerly A3 concept)
const int FAN_SET_PIN = 36;      // ADC1_CH0 (Formerly A4 concept)
SimpleKalmanFilter Filter1(2, 2, 0.01);


// --- WiFi Configuration ---
const char* WIFI_SSID = "Climate";
const char* WIFI_PASS = "Defender";
IPAddress localIP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1); // Often .1, adjust if needed
IPAddress subnet(255, 255, 255, 0);
const char* OTA_HOSTNAME = "esp32-climate-control";
// const char* OTA_PASSWORD = "ota_password"; // Optional: Uncomment and set password

// --- Control Logic Constants ---
// Analog Ranges (Raw ADC 12-bit)
const int ADC_MIN_FAN = 2000;
const int ADC_MAX_FAN = 3200;
const int ADC_MAX_TEMP = 3300;
const int ADC_MIN_TEMP = 2300;
const int PT100_ADC_MIN_VALID = 100;  // Example: Define reasonable min ADC for PT100
const int PT100_ADC_MAX_VALID = 4000; // Example: Define reasonable max ADC for PT100

// Temperature Settings
const float TEMP_SET_MIN_C = 14.0;
const float TEMP_SET_MAX_C = 28.0;
const float TEMP_HYSTERESIS = 1.0; // Degrees C for Auto mode switching

// Fan Speed Settings (Corresponds to Off, Low, Med, High)
const int FAN_SET_LEVELS = 4;
const int FAN_OFF = 0;
const int FAN_LOW = 1;
const int FAN_MED = 2;
const int FAN_HIGH = 3;

// Manual Mode PWM % Thresholds for Temp Setting
const float MANUAL_AC_FULL_DUTY_THRESHOLD = 0.10; // <= 10%
const float MANUAL_AC_MAX_PROPORTIONAL_THRESHOLD = 0.50; // >10% to <= 50%
const float MANUAL_HEAT_MIN_PROPORTIONAL_THRESHOLD = 0.50; // >50% to < 90%
const float MANUAL_HEAT_FULL_DUTY_THRESHOLD = 0.90; // >= 90%

// Timing intervals (milliseconds)
const unsigned long PWM_INTERVAL_MS = 30000; // 10 seconds for slow PWM cycle
const unsigned long AUTO_UPDATE_INTERVAL_MS = 20000; // 5 seconds minimum between Auto mode changes
const unsigned long CONTROL_LOOP_DELAY_MS = 100; // How often the main control logic runs
const unsigned long WEB_LOOP_DELAY_MS = 10;      // How often the web server/OTA task runs

// --- Global State Variables ---
// Inputs
volatile int rawFanSet = 0;
volatile int rawTempSet = 0;
volatile int rawInteriorTemp = 0;
volatile bool defrostSwitchActive = false; // true if LOW
volatile bool autoModeSwitchActive = false; // true if LOW
volatile float interiorTempC = 20.0; // Default value
volatile float setTempC = 21.0;     // Default value
volatile int setFanSpeed = FAN_OFF;  // 0=Off, 1=Low, 2=Med, 3=High

// Control Mode
enum ControlMode { MANUAL, AUTO, DEFROST };
volatile ControlMode currentMode = MANUAL;

// Heating Mode
enum HeatingMode { HEAT, IDLE, COOL };
volatile HeatingMode currentHeatingMode = IDLE;

// Outputs Targets
volatile bool targetACState = false;
volatile bool targetHeaterValveState = false;
volatile int targetFanSpeed = FAN_OFF; // 0-3

// PWM State (Manual Mode)
unsigned long lastPwmUpdateTime = 0;
float manualAcDutyCycle = 0.0;   // 0.0 to 1.0
float manualHeatDutyCycle = 0.0; // 0.0 to 1.0
bool currentAcPwmState = false;
bool currentHeaterPwmState = false;

// Auto Mode State
unsigned long lastAutoUpdateTime = 0;

// Error Status
volatile bool pt100Error = false;
String errorMessage = "";

// --- RTOS Task Handles ---
TaskHandle_t webTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;

// --- Web Server ---
WebServer server(80);

// --- PT100 Function ---
// Provided function to read PT100 temperature
float readPT100(int adc_value) {
    // Linear calibration based on our known point (2579 ADC = 14.7°C)
    // Higher ADC value corresponds to higher temperature

    // Using a slope of 0.01°C per ADC count (typical for this setup)
    const float slope = 0.3603; // Deg C per ADC count - TUNE THIS if needed

    // Calculate offset to match our calibration point exactly
    const float offset = 17.5 - (slope * 1300.0); // Use float literal

    // Calculate temperature using linear formula
    float temperature = (slope * (float)adc_value) + offset; // Cast ADC to float

    float tempfiltered = Filter1.updateEstimate(temperature); //Kalman Filter

    return tempfiltered;
}

// --- Helper Functions ---

// Maps an inverted analog reading to a desired range
float mapInvertedAnalog(int val, int valMin, int valMax, float outMin, float outMax) {
    // Clamp input value
      val = constrain(val, valMin, valMax);

      // Invert the input mapping: map MAX to outMin, MIN to outMax
      return outMax - float(val - valMin) / float(valMax - valMin) * float(outMax - outMin);

    
    //return map(val, valMin, valMax, outMin, outMax);
    
}

int mapInvertedAnalogToInt(int val, int valMin, int valMax, int outMin, int outMax) {
    // Clamp input value
    val = constrain(val, valMin, valMax);
    // Invert the input mapping: map MAX to outMin, MIN to outMax
    // Add 0.5 before casting to int for rounding
    return (int)(map(val, valMin, valMax, outMax * 100, outMin * 100) / 100.0 + 0.5);
}


// Reads all inputs and updates global state variables
void readInputs() {
    // Read Analog Inputs
    rawFanSet = analogRead(FAN_SET_PIN);
    rawTempSet = analogRead(TEMP_SET_PIN);
    rawInteriorTemp = analogRead(INTERIOR_TEMP_PIN);

    // --- Calculate Scaled Values ---

    // Fan Speed Setting (Inverted, 0-3)
    // Divide the ADC range into FAN_SET_LEVELS segments
    int fanAdcStep = (ADC_MAX_FAN - ADC_MIN_FAN) / FAN_SET_LEVELS;
    // Determine the segment based on raw value, remembering it's inverted
    setFanSpeed = FAN_SET_LEVELS - 1 - ((constrain(rawFanSet, ADC_MIN_FAN, ADC_MAX_FAN)-ADC_MIN_FAN) / fanAdcStep)+0.5;
    setFanSpeed = constrain(setFanSpeed, FAN_OFF, FAN_HIGH); // Ensure valid range

    // Temperature Setting (Inverted, MIN_TEMP_C to MAX_TEMP_C)
    setTempC = mapInvertedAnalog(rawTempSet, ADC_MIN_TEMP, ADC_MAX_TEMP, TEMP_SET_MIN_C, TEMP_SET_MAX_C);

    // Interior Temperature (PT100)
    // Check if reading is within a plausible ADC range first
    if (rawInteriorTemp < PT100_ADC_MIN_VALID || rawInteriorTemp > PT100_ADC_MAX_VALID) {
        pt100Error = true;
        errorMessage = "PT100 Reading Out of Range!";
        interiorTempC = -99.9; // Indicate error clearly
        Serial.println("ERROR: PT100 ADC reading out of valid range.");
    } else {
        pt100Error = false;
        errorMessage = ""; // Clear error on valid reading
        interiorTempC = readPT100(rawInteriorTemp);
        // Optional: Add sanity check for calculated temperature range
        if (interiorTempC < -20.0 || interiorTempC > 80.0) { // Example range
             pt100Error = true;
             errorMessage = "PT100 Calculated Temp Unlikely!";
             Serial.printf("WARN: PT100 Calculated Temp (%f C) seems unlikely.\n", interiorTempC);
        }
    }


    // Read Digital Inputs (Active LOW)
    defrostSwitchActive = (digitalRead(DEFROST_SWITCH_PIN) == LOW);
    autoModeSwitchActive = (digitalRead(AUTO_MODE_SWITCH_PIN) == LOW);

    // Serial Debugging
    Serial.printf("Inputs - Raw Fan: %d, Raw TempSet: %d, Raw PT100: %d\n", rawFanSet, rawTempSet, rawInteriorTemp);
    Serial.printf("Inputs - Set Fan: %d, Set Temp: %.1f C, Int Temp: %.1f C %s\n", setFanSpeed, setTempC, interiorTempC, pt100Error ? "(ERROR)" : "");
    Serial.printf("Inputs - Defrost: %s, Auto: %s\n", defrostSwitchActive ? "ON" : "OFF", autoModeSwitchActive ? "ON" : "OFF");
}

// Determines the current operating mode
void determineMode() {
    if (pt100Error) {
        // Force manual mode if PT100 reading is invalid
        if (currentMode != MANUAL) {
            Serial.println("PT100 Error detected! Forcing MANUAL mode.");
        }
        currentMode = MANUAL;
    } else if (defrostSwitchActive) {
        currentMode = DEFROST;
    } else if (autoModeSwitchActive) {
        currentMode = AUTO;
    } else {
        currentMode = MANUAL;
    }
    // Serial.printf("Current Mode: %d\n", currentMode); // DEBUG
}

// Applies the target fan speed, ensuring only one output is high
void updateFanOutputs(int speed) {
    digitalWrite(FAN_LOW_PIN, (speed == FAN_LOW) ? HIGH : LOW);
    digitalWrite(FAN_MED_PIN, (speed == FAN_MED) ? HIGH : LOW);
    digitalWrite(FAN_HIGH_PIN, (speed == FAN_HIGH) ? HIGH : LOW);
    // Serial.printf("Fan Output - Speed: %d (L:%d M:%d H:%d)\n", speed, digitalRead(FAN_LOW_PIN), digitalRead(FAN_MED_PIN), digitalRead(FAN_HIGH_PIN)); // DEBUG
}

// Handles Manual Mode Logic
void handleManualMode() {
    unsigned long now = millis();

    // Set fan speed directly from user input
    targetFanSpeed = setFanSpeed;

    // Calculate desired AC and Heater duty cycles based on set temperature percentage
    float tempSetPercent = (setTempC - TEMP_SET_MIN_C) / (TEMP_SET_MAX_C - TEMP_SET_MIN_C);
    tempSetPercent = constrain(tempSetPercent, 0.0, 1.0); // Clamp to 0-1 range

    manualAcDutyCycle = 0.0;
    manualHeatDutyCycle = 0.0;

    if (tempSetPercent <= MANUAL_AC_FULL_DUTY_THRESHOLD) { // <= 10%
        manualAcDutyCycle = 1.0; // 100% AC
    } else if (tempSetPercent <= MANUAL_AC_MAX_PROPORTIONAL_THRESHOLD) { // >10% to <= 50%
        // AC proportional: 100% at 10%, 0% at 50%
        manualAcDutyCycle = map(tempSetPercent, MANUAL_AC_FULL_DUTY_THRESHOLD, MANUAL_AC_MAX_PROPORTIONAL_THRESHOLD, 1.0, 0.0);
    } else if (tempSetPercent < MANUAL_HEAT_FULL_DUTY_THRESHOLD) { // >50% to < 90%
        // Heater proportional: 0% at 50%, 100% at 90%
        manualHeatDutyCycle = map(tempSetPercent, MANUAL_HEAT_MIN_PROPORTIONAL_THRESHOLD, MANUAL_HEAT_FULL_DUTY_THRESHOLD, 0.0, 1.0);
    } else { // >= 90%
        manualHeatDutyCycle = 1.0; // 100% Heater
    }
    manualAcDutyCycle = constrain(manualAcDutyCycle, 0.0, 1.0);
    manualHeatDutyCycle = constrain(manualHeatDutyCycle, 0.0, 1.0);

    // --- Implement Slow PWM ---
    if (now - lastPwmUpdateTime >= PWM_INTERVAL_MS) {
        lastPwmUpdateTime = now; // Reset timer for the next cycle
        // Decide state for the *next* cycle based on duty cycle
        currentAcPwmState = (random(100) < (manualAcDutyCycle * 100.0));
        currentHeaterPwmState = (random(100) < (manualHeatDutyCycle * 100.0));
         Serial.printf("Manual PWM Update: AC Duty: %.2f -> State: %s | Heat Duty: %.2f -> State: %s\n",
              manualAcDutyCycle, currentAcPwmState ? "ON" : "OFF",
              manualHeatDutyCycle, currentHeaterPwmState ? "ON" : "OFF");
    }
    // If AC and Heater happen to both be on (only possible near 50% setpoint if ranges overlap slightly), prioritize AC (cooling usually wins)
    // Or, ensure ranges don't overlap in constants. Current logic allows both to be non-zero between 50% and slightly above. Let's allow it.

    targetACState = currentAcPwmState;
    targetHeaterValveState = currentHeaterPwmState;
}

// Handles Auto Mode Logic
void handleAutoMode() {
    unsigned long now = millis();

    // Only allow changes every AUTO_UPDATE_INTERVAL_MS
    if (now - lastAutoUpdateTime < AUTO_UPDATE_INTERVAL_MS) {
        return; // Not time to update yet
    }
    lastAutoUpdateTime = now;

    float tempDifference = interiorTempC - setTempC;

    Serial.printf("Auto Mode Update: Temp Diff: %.2f C (Int: %.1f, Set: %.1f)\n", tempDifference, interiorTempC, setTempC);

    // Determine target states based on temperature difference and hysteresis
    if (tempDifference > TEMP_HYSTERESIS) { // Too warm, need cooling
        targetACState = true;
        targetHeaterValveState = false;
        currentHeatingMode = COOL;

        // Fan speed proportional to difference (Example: more diff = higher speed)
        // Scale difference (e.g., 1C to 5C+) to Low/Med/High
        if (tempDifference > 15.0) targetFanSpeed = FAN_HIGH;
        else if (tempDifference > 2.0) targetFanSpeed = FAN_MED;
        else targetFanSpeed = FAN_LOW;
        Serial.printf("Auto: Cooling needed. Fan: %d\n", targetFanSpeed);

    } else if (tempDifference < -TEMP_HYSTERESIS) { // Too cold, need heating
        targetACState = false;
        targetHeaterValveState = true;
        currentHeatingMode = HEAT;

        // Fan speed proportional to difference
        float absDiff = abs(tempDifference);
        if (absDiff > 15.0) targetFanSpeed = FAN_HIGH;
        else if (absDiff > 2.0) targetFanSpeed = FAN_MED;
        else targetFanSpeed = FAN_LOW;
         Serial.printf("Auto: Heating needed. Fan: %d\n", targetFanSpeed);

    } else { // Within hysteresis band - maintain temp
        targetFanSpeed = FAN_LOW; // Keep air circulating gently

        //Modulate Heat or AC based on which direction we have approached from
        if(currentHeatingMode==COOL){
          if (tempDifference > 0) { // Slightly warm
              targetACState = true; 
              targetHeaterValveState = false;
              Serial.println("Auto: Maintaining (Slightly Warm) -> AC ON, HEATER OFF, Fan LOW");
          } else { // Slightly cool
              targetACState = false;
              targetHeaterValveState = false; 
              Serial.println("Auto: Maintaining (Slightly Cool/OK) -> AC OFF, HEATER OFF, Fan LOW");
          }
        }
        if(currentHeatingMode==HEAT){
          if (tempDifference > 0) { // Slightly warm
              targetACState = false; 
              targetHeaterValveState = false;
              Serial.println("Auto: Maintaining (Slightly Warm) -> AC OFF, HEATER OFF, Fan LOW");
          } else { // Slightly cool
              targetACState = false;
              targetHeaterValveState = true; 
              Serial.println("Auto: Maintaining (Slightly Cool/OK) -> AC OFF, HEATER ON, Fan LOW");
          }
        }

        if(currentHeatingMode==IDLE){
          targetACState = false; 
          targetHeaterValveState = false;
          targetFanSpeed = FAN_LOW;
          Serial.println("Auto: About right -> AC OFF, Heater OFF, Fan LOW");
        }


        // Modulate heating/cooling slightly based on which side of the setpoint we are
        if (tempDifference > 0) { // Slightly warm
             targetACState = true; // Short cycle AC? Or just leave off? Let's try ON.
             targetHeaterValveState = false;
             Serial.println("Auto: Maintaining (Slightly Warm) -> AC ON, Fan LOW");
        } else { // Slightly cool or spot on
             targetACState = false;
             targetHeaterValveState = true; // Short cycle Heater? Or just leave off? Let's try ON.
             Serial.println("Auto: Maintaining (Slightly Cool/OK) -> Heater ON, Fan LOW");
        }
        // More sophisticated: Could implement finer PWM here, but sticking to ON/OFF based on prompt
    }
}

// Handles Defrost Mode Logic
void handleDefrostMode() {
    Serial.println("Defrost Mode Active");

    if (autoModeSwitchActive) { // If Auto switch is also ON
        // Force full defrost: High Fan, Heater ON, AC ON (for dehumidification)
        targetFanSpeed = FAN_HIGH;
        targetACState = true;
        targetHeaterValveState = true;
        Serial.println("Defrost (Auto Base): Fan HIGH, AC ON, Heater ON");
    } else { // Manual mode was selected before Defrost
        // Use manual temperature set logic for AC/Heater
        handleManualMode(); // Calculates targetACState, targetHeaterValveState based on setTempC

        // User controls fan speed, but OFF is not allowed
        targetFanSpeed = setFanSpeed;
        if (targetFanSpeed == FAN_OFF) {
            targetFanSpeed = FAN_LOW; // Force at least low fan
            Serial.println("Defrost (Manual Base): Fan forced to LOW (was OFF)");
        }
        Serial.printf("Defrost (Manual Base): Fan %d (User Set), AC/Heater based on Set Temp\n", targetFanSpeed);
    }
}

// Final update of physical outputs based on target states
void updateOutputs() {
    // Update Fan Speed (ensures mutual exclusivity)
    updateFanOutputs(targetFanSpeed);

    // Update AC and Heater Valve
    // Respect PWM state if in Manual mode, otherwise use direct target state from Auto/Defrost
    bool finalACState = (currentMode == MANUAL) ? currentAcPwmState : targetACState;
    bool finalHeaterState = (currentMode == MANUAL) ? currentHeaterPwmState : targetHeaterValveState;

    digitalWrite(AC_PIN, finalACState ? HIGH : LOW);
    digitalWrite(HEATER_VALVE_PIN, finalHeaterState ? HIGH : LOW);

    Serial.printf("Outputs - Mode: %d, Fan Speed: %d, AC: %s, Heater: %s\n",
                  currentMode, targetFanSpeed, finalACState ? "ON" : "OFF", finalHeaterState ? "ON" : "OFF");
    Serial.println("--------------------");
}


// --- RTOS Tasks ---

// Task for Web Server and OTA updates (Core 0)
void webTask(void *pvParameters) {
    Serial.println("Web Task started on Core 0");

    // Configure static IP
    if (!WiFi.softAPConfig(localIP, gateway, subnet)) {
        Serial.println("AP Static IP Configuration Failed!");
    }

    // Start AP
    Serial.printf("Starting AP '%s'...\n", WIFI_SSID);
    if (!WiFi.softAP(WIFI_SSID, WIFI_PASS)) {
        Serial.println("AP Start Failed!");
        // Consider error handling here - maybe restart?
        vTaskDelete(NULL); // End task if AP fails
    }

    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // OTA Setup
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    // ArduinoOTA.setPassword(OTA_PASSWORD); // Uncomment if using password

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";
        Serial.println("Start updating " + type);
        // Optional: Stop climate control during update?
        // digitalWrite(AC_PIN, LOW); ... etc.
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();

    // Web Server Routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/update", HTTP_GET, handleUpdate); // Page with upload form
    server.on("/doUpdate", HTTP_POST, handleDoUpdate_Finish, handleDoUpdate_Upload); // OTA Upload handler

    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println("HTTP server started");

    while (1) {
        server.handleClient(); // Handle web requests
        ArduinoOTA.handle();  // Handle OTA requests
        vTaskDelay(pdMS_TO_TICKS(WEB_LOOP_DELAY_MS)); // Small delay
    }
}

// Task for Climate Control Logic (Core 1)
void controlTask(void *pvParameters) {
    Serial.println("Control Task started on Core 1");

    // Initialize output pins
    pinMode(AC_PIN, OUTPUT);
    pinMode(HEATER_VALVE_PIN, OUTPUT);
    pinMode(FAN_LOW_PIN, OUTPUT);
    pinMode(FAN_MED_PIN, OUTPUT);
    pinMode(FAN_HIGH_PIN, OUTPUT);

    digitalWrite(AC_PIN, LOW);
    digitalWrite(HEATER_VALVE_PIN, LOW);
    digitalWrite(FAN_LOW_PIN, LOW);
    digitalWrite(FAN_MED_PIN, LOW);
    digitalWrite(FAN_HIGH_PIN, LOW);

    // Initialize input pins
    pinMode(DEFROST_SWITCH_PIN, INPUT_PULLUP);
    pinMode(AUTO_MODE_SWITCH_PIN, INPUT_PULLUP);

    // Analog Inputs - pinMode is not needed for analogRead on ESP32

    // Configure ADC width (optional but good practice)
    analogReadResolution(12); // 12-bit ADC (0-4095)

    lastPwmUpdateTime = millis();
    lastAutoUpdateTime = millis();

    //Determine initial Heating Mode
    readInputs();
    if(interiorTempC < (setTempC-2)) currentHeatingMode = HEAT;
    if(interiorTempC > (setTempC+2)) currentHeatingMode = COOL;
    

    while (1) {
        // 1. Read all sensor inputs
        readInputs();

        // 2. Determine the operating mode
        determineMode();

        // 3. Execute logic based on the current mode
        switch (currentMode) {
            case MANUAL:
                handleManualMode();
                break;
            case AUTO:
                handleAutoMode();
                break;
            case DEFROST:
                handleDefrostMode();
                break;
        }

        // 4. Update the physical outputs
        updateOutputs();

        // 5. Delay before next loop iteration
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_DELAY_MS));
    }
}

// --- Web Server Handlers ---

// HTML Page Content (single string)
// Uses placeholders like %%PLACEHOLDER%% which get replaced dynamically
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Climate Control</title>
    <style>
        body { font-family: sans-serif; padding: 15px; background-color: #f4f4f4; }
        .container { max-width: 600px; margin: auto; background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        h1, h2 { text-align: center; color: #333; }
        .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px; margin-top: 20px; }
        .status-item { background-color: #e9e9e9; padding: 15px; border-radius: 5px; }
        .status-item strong { display: block; margin-bottom: 5px; color: #555; }
        .status-item span { font-size: 1.1em; color: #000; }
        .output-status span { font-weight: bold; }
        .on { color: #28a745; } /* Green */
        .off { color: #dc3545; } /* Red */
        .mode-manual { color: #007bff; } /* Blue */
        .mode-auto { color: #ffc107; } /* Yellow/Orange */
        .mode-defrost { color: #17a2b8; } /* Teal */
        .error { background-color: #f8d7da; color: #721c24; padding: 10px; border: 1px solid #f5c6cb; border-radius: 5px; margin-top: 15px; text-align: center; }
        .footer { text-align: center; margin-top: 25px; font-size: 0.9em; color: #777; }
        .footer a { color: #007bff; text-decoration: none; }
        /* Simple progress bar style */
        #progress { width: 100%; background-color: #ddd; border-radius: 5px; margin-top: 10px; display: none; }
        #bar { width: 0%; height: 20px; background-color: #4CAF50; border-radius: 5px; text-align: center; line-height: 20px; color: white; }

        /* Upload Form Styles */
        .upload-form { margin-top: 20px; padding: 15px; border: 1px dashed #ccc; border-radius: 5px; background-color: #f9f9f9;}
        .upload-form input[type=file] { display: block; margin-bottom: 10px; }
        .upload-form input[type=submit] { background-color: #007bff; color: white; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; }
        .upload-form input[type=submit]:hover { background-color: #0056b3; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Climate Control Status</h1>
        <div class="status-item"><strong>Device IP:</strong> <span style="color: #6c757d;">%%IP_ADDRESS%%</span></div>

        <div id="error-container">
             %%ERROR_MESSAGE%%
        </div>

        <h2>Current State</h2>
        <div class="status-grid">
            <div class="status-item"><strong>Mode:</strong> <span class="mode-%%MODE_CLASS%%">%%MODE%%</span></div>
            <div class="status-item"><strong>Interior Temp:</strong> <span>%%INT_TEMP_C%% °C</span></div>
            <div class="status-item"><strong>Set Temp:</strong> <span>%%SET_TEMP_C%% °C</span></div>
            <div class="status-item"><strong>Set Fan Speed:</strong> <span>%%SET_FAN_SPEED%%</span></div>
            <div class="status-item"><strong>Heating Mode:</strong> <span>%%HEATING_MODE%%</span></div>
        </div>

        <h2>Outputs</h2>
        <div class="status-grid output-status">
            <div class="status-item"><strong>Fan Speed:</strong> <span>%%OUT_FAN_SPEED%%</span></div>
            <div class="status-item"><strong>A/C:</strong> <span class="%%AC_STATE_CLASS%%">%%AC_STATE%%</span></div>
            <div class="status-item"><strong>Heater Valve:</strong> <span class="%%HEAT_STATE_CLASS%%">%%HEAT_STATE%%</span></div>
        </div>

         <h2>Raw Sensor Data</h2>
        <div class="status-grid">
            <div class="status-item"><strong>Set Fan ADC:</strong> <span>%%RAW_FAN%%</span></div>
            <div class="status-item"><strong>Set Temp ADC:</strong> <span>%%RAW_TEMP%%</span></div>
            <div class="status-item"><strong>PT100 ADC:</strong> <span>%%RAW_PT100%%</span></div>
        </div>

        <h2>System</h2>
        <div class="status-grid">
             <div class="status-item"><strong>Defrost Sw:</strong> <span class="%%DEFROST_SW_CLASS%%">%%DEFROST_SW%%</span></div>
             <div class="status-item"><strong>Auto Sw:</strong> <span class="%%AUTO_SW_CLASS%%">%%AUTO_SW%%</span></div>
        </div>


        <div class="footer">
            <a href="/update">Firmware Update (OTA)</a>
        </div>
    </div>
</body>
</html>
)rawliteral";


// HTML for OTA Update page
const char UPDATE_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Firmware Update</title>
     <style>
        body { font-family: sans-serif; padding: 15px; background-color: #f4f4f4; }
        .container { max-width: 600px; margin: auto; background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        h1 { text-align: center; color: #333; }
        .upload-form { margin-top: 20px; padding: 15px; border: 1px dashed #ccc; border-radius: 5px; background-color: #f9f9f9;}
        .upload-form input[type=file] { display: block; margin-bottom: 10px; }
        .upload-form input[type=submit] { background-color: #007bff; color: white; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; }
        .upload-form input[type=submit]:hover { background-color: #0056b3; }
        #progress { width: 100%; background-color: #ddd; border-radius: 5px; margin-top: 10px; display: none; }
        #bar { width: 0%; height: 20px; background-color: #4CAF50; border-radius: 5px; text-align: center; line-height: 20px; color: white; }
        .message { margin-top: 15px; padding: 10px; border-radius: 5px; text-align: center; }
        .success { background-color: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
        .error { background-color: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        .footer { text-align: center; margin-top: 25px; font-size: 0.9em; color: #777; }
        .footer a { color: #007bff; text-decoration: none; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Firmware Update (OTA)</h1>
        <div class="upload-form">
            <form id="upload_form" method="POST" action="/doUpdate" enctype="multipart/form-data">
                <input type="file" name="update" accept=".bin,.bin.gz">
                <input type="submit" value="Upload Firmware">
            </form>
        </div>
        <div id="progress">
            <div id="bar">0%</div>
        </div>
        <div id="message_div" class="message" style="display: none;"></div>

        <div class="footer">
             <a href="/">Back to Status</a>
        </div>
    </div>

    <script>
        const form = document.getElementById('upload_form');
        const progressBar = document.getElementById('progress');
        const bar = document.getElementById('bar');
        const messageDiv = document.getElementById('message_div');

        form.addEventListener('submit', function(e) {
            e.preventDefault();
            const formData = new FormData(form);
            const fileInput = form.querySelector('input[type="file"]');

            if (!fileInput.files || fileInput.files.length === 0) {
                showMessage('Please select a firmware file (.bin).', 'error');
                return;
            }

            const file = fileInput.files[0];
            if (!file.name.endsWith('.bin') && !file.name.endsWith('.bin.gz')) {
                 showMessage('Invalid file type. Please upload a .bin or .bin.gz file.', 'error');
                 return;
            }

            formData.append('update', file);

            const xhr = new XMLHttpRequest();

            xhr.upload.addEventListener('progress', function(e) {
                if (e.lengthComputable) {
                    const percentComplete = Math.round((e.loaded / e.total) * 100);
                    progressBar.style.display = 'block';
                    bar.style.width = percentComplete + '%';
                    bar.textContent = percentComplete + '%';
                }
            }, false);

            xhr.addEventListener('load', function(e) {
                 bar.style.width = '100%'; // Ensure it reaches 100% visually
                 bar.textContent = '100%';
                if (xhr.status === 200) {
                     showMessage('Update successful! Device is rebooting...', 'success');
                     // Optionally redirect or disable form
                     setTimeout(() => { window.location.href = '/'; }, 5000); // Redirect after 5s
                } else {
                    showMessage('Update failed! Server responded with: ' + xhr.status + ' ' + xhr.responseText, 'error');
                    progressBar.style.display = 'none'; // Hide progress on error
                }
            });

            xhr.addEventListener('error', function(e) {
                showMessage('Update failed! Connection error.', 'error');
                 progressBar.style.display = 'none';
            });

             xhr.addEventListener('abort', function(e) {
                showMessage('Update aborted.', 'error');
                 progressBar.style.display = 'none';
            });

            messageDiv.style.display = 'none'; // Hide previous messages
            xhr.open('POST', '/doUpdate');
            xhr.send(formData);
        });

        function showMessage(msg, type) {
            messageDiv.textContent = msg;
            messageDiv.className = 'message ' + type; // Set class for styling
            messageDiv.style.display = 'block';
        }
    </script>
</body>
</html>
)rawliteral";

// Helper to get Fan Speed Name
String getFanSpeedName(int speed) {
    switch (speed) {
        case FAN_OFF: return "OFF";
        case FAN_LOW: return "Low";
        case FAN_MED: return "Medium";
        case FAN_HIGH: return "High";
        default: return "Unknown";
    }
}

// Helper to get Heating Mode Name
String getHeatingModeName(int Hmode) {
    switch (Hmode) {
        case COOL: return "COOL";
        case IDLE: return "IDLE";
        case HEAT: return "HEAT";
        default: return "Unknown";
    }
}

// Function to replace placeholders in the HTML
String processor(const String& var) {
    // Status Page Placeholders
    if (var == "IP_ADDRESS") return WiFi.softAPIP().toString();
    if (var == "MODE") {
        switch (currentMode) {
            case MANUAL: return "Manual";
            case AUTO: return "Auto";
            case DEFROST: return "Defrost";
            default: return "Unknown";
        }
    }
     if (var == "MODE_CLASS") {
        switch (currentMode) {
            case MANUAL: return "manual";
            case AUTO: return "auto";
            case DEFROST: return "defrost";
            default: return "unknown";
        }
    }
    if (var == "INT_TEMP_C") return pt100Error ? "--" : String(interiorTempC, 1);
    if (var == "SET_TEMP_C") return String(setTempC, 1);
    if (var == "SET_FAN_SPEED") return getFanSpeedName(setFanSpeed);
    if (var == "HEATING_MODE") return getHeatingModeName(currentHeatingMode);



    // Output Status
    bool actualACState = digitalRead(AC_PIN) == HIGH;
    bool actualHeaterState = digitalRead(HEATER_VALVE_PIN) == HIGH;
    if (var == "OUT_FAN_SPEED") return getFanSpeedName(targetFanSpeed); // Show the target fan speed
    if (var == "AC_STATE") return actualACState ? "ON" : "OFF";
    if (var == "AC_STATE_CLASS") return actualACState ? "on" : "off";
    if (var == "HEAT_STATE") return actualHeaterState ? "ON" : "OFF";
    if (var == "HEAT_STATE_CLASS") return actualHeaterState ? "on" : "off";

    // Raw ADC Values
    if (var == "RAW_FAN") return String(rawFanSet);
    if (var == "RAW_TEMP") return String(rawTempSet);
    if (var == "RAW_PT100") return String(rawInteriorTemp);

    // Switches
     if (var == "DEFROST_SW") return defrostSwitchActive ? "ON" : "OFF";
     if (var == "DEFROST_SW_CLASS") return defrostSwitchActive ? "on" : "off"; // Use ON/OFF styles
     if (var == "AUTO_SW") return autoModeSwitchActive ? "ON" : "OFF";
     if (var == "AUTO_SW_CLASS") return autoModeSwitchActive ? "on" : "off"; // Use ON/OFF styles

    // Error Message
    if (var == "ERROR_MESSAGE") {
        if (pt100Error || !errorMessage.isEmpty()) {
            return "<div class='error'>" + errorMessage + (pt100Error ? " (PT100 Error)" : "") + "</div>";
        }
        return ""; // No error, return empty string
    }

    return String(); // Return empty string for unknown placeholders
}

void handleRoot() {
    Serial.println("Serving / page");
    String html = HTML_CONTENT; // Load template from PROGMEM
    // Replace all placeholders - needs careful implementation if HTML_CONTENT is large
    // A simpler approach might be needed for very large pages, but this works for moderate size
    html.replace("%%IP_ADDRESS%%", processor("IP_ADDRESS"));
    html.replace("%%MODE%%", processor("MODE"));
    html.replace("%%MODE_CLASS%%", processor("MODE_CLASS"));
    html.replace("%%HEATING_MODE%%", processor("HEATING_MODE"));
    html.replace("%%INT_TEMP_C%%", processor("INT_TEMP_C"));
    html.replace("%%SET_TEMP_C%%", processor("SET_TEMP_C"));
    html.replace("%%SET_FAN_SPEED%%", processor("SET_FAN_SPEED"));
    html.replace("%%OUT_FAN_SPEED%%", processor("OUT_FAN_SPEED"));
    html.replace("%%AC_STATE%%", processor("AC_STATE"));
    html.replace("%%AC_STATE_CLASS%%", processor("AC_STATE_CLASS"));
    html.replace("%%HEAT_STATE%%", processor("HEAT_STATE"));
    html.replace("%%HEAT_STATE_CLASS%%", processor("HEAT_STATE_CLASS"));
    html.replace("%%RAW_FAN%%", processor("RAW_FAN"));
    html.replace("%%RAW_TEMP%%", processor("RAW_TEMP"));
    html.replace("%%RAW_PT100%%", processor("RAW_PT100"));
    html.replace("%%DEFROST_SW%%", processor("DEFROST_SW"));
    html.replace("%%DEFROST_SW_CLASS%%", processor("DEFROST_SW_CLASS"));
    html.replace("%%AUTO_SW%%", processor("AUTO_SW"));
    html.replace("%%AUTO_SW_CLASS%%", processor("AUTO_SW_CLASS"));
    html.replace("%%ERROR_MESSAGE%%", processor("ERROR_MESSAGE"));

    server.send(200, "text/html", html);
}


void handleUpdate() {
    Serial.println("Serving /update page");
    server.send(200, "text/html", UPDATE_PAGE);
}

// Handler for the file upload
void handleDoUpdate_Upload() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // Start update, size may not be known
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
         // Provide feedback during upload (optional, can slow down)
         // Serial.printf("Upload Progress: %d%%\r", (Update.progress() * 100) / Update.size());
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { // Apply the update = true
            Serial.printf("Update Success: %u bytes\n", upload.totalSize);
            // Response sent by handleDoUpdate_Finish
        } else {
            Update.printError(Serial);
            // Response sent by handleDoUpdate_Finish
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
         Serial.println("Update Aborted");
         Update.end(false); // Do not apply aborted update
         // Response sent by handleDoUpdate_Finish
    }
}

// Handler after the upload finishes or fails
void handleDoUpdate_Finish() {
    if (Update.hasError()) {
         server.send(500, "text/plain", "Update Failed! Error: " + String(Update.getError()));
         Serial.println("Update failed!");
    } else if (Update.isFinished()) {
        server.send(200, "text/plain", "Update Successful! Rebooting...");
        Serial.println("Update successful! Rebooting...");
        ESP.restart();
    } else {
         // This might happen if the request ends before UPLOAD_FILE_END status
         server.send(400, "text/plain", "Update Incomplete or Unknown State.");
         Serial.println("Update finish handler called in unexpected state.");
    }
}


void handleNotFound() {
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
    Serial.println("Sent 404 for: " + server.uri());
}

// --- Arduino Setup & Loop ---
void setup() {
    Serial.begin(115200);
    Serial.println("\n\nESP32 Climate Controller Booting...");

    // Create RTOS Tasks
    // Core 0: Network related tasks (lower priority can be ok)
    xTaskCreatePinnedToCore(
        webTask,          // Task function
        "WebTask",        // Name of task
        10000,            // Stack size of task
        NULL,             // Parameter of the task
        1,                // Priority of the task (1 is reasonable)
        &webTaskHandle,   // Task handle
        0);               // Core No. (0)

    // Core 1: Real-time control (higher priority recommended)
    xTaskCreatePinnedToCore(
        controlTask,      // Task function
        "ControlTask",    // Name of task
        10000,            // Stack size of task
        NULL,             // Parameter of the task
        2,                // Priority of the task (Higher than Web Task)
        &controlTaskHandle,// Task handle
        1);               // Core No. (1)

    Serial.println("RTOS Tasks Created.");
    // setup() completes here, tasks run independently.
}

void loop() {
    // Empty. The RTOS tasks handle everything.
    vTaskSuspend(NULL); // Suspend the loop() task itself
}