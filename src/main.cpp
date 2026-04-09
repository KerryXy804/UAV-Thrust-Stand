#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"

// --- CALIBRATION FACTORS ---
const float CAL_THRUST = 73.77;
const float CAL_PUSH_LEFT   = 104.680247642 ;
const float CAL_PULL_RIGHT   = 110;
const float TORQUE_ARM_M = 0.06317685;
const float GRAMS_TO_N   = 0.00981;

// --- CONFIGURATION ---
#define INA228_ADDR      0x45
#define MOTOR_POLE_PAIRS 7

// --- PINS (Renamed to avoid macro conflicts) ---
const int PIN_HX_SCK   = 12;
const int PIN_LC_THRUST = 4;
const int PIN_LC_PUSH   = 16;
const int PIN_LC_PULL   = 17;
const int PIN_I2C_SDA   = 8;
const int PIN_I2C_SCL   = 9;
const int PIN_ESC       = 18;
const int PIN_RPM       = 35;

// --- ESC via LEDC (Legacy v2.x API) ---
// Note: In v2.x, we write to a CHANNEL, not the GPIO pin directly.
#define ESC_LEDC_CHANNEL    0
#define ESC_LEDC_FREQ_HZ    50
#define ESC_LEDC_RESOLUTION 14   
#define ESC_PWM_MIN         819  // 1000us @ 14-bit (50Hz)
#define ESC_PWM_MAX         1638 // 2000us @ 14-bit (50Hz)

// --- OBJECTS ---
HX711 scaleThrust, scalePush, scalePull;
bool inaOnline = false;

// --- STATE ---
volatile unsigned long lastPulseTimeMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile bool newPulseAvailable = false;

unsigned long lastStream  = 0;
float currentRpm      = 0;
int   currentThrottle = 1000;

// --- ISR ---
void IRAM_ATTR handleRpmPulse() {
    unsigned long now = micros();
    // Unsigned math naturally handles the 70-minute micros() overflow
    pulseIntervalMicros = now - lastPulseTimeMicros; 
    lastPulseTimeMicros = now;
    newPulseAvailable = true;
}

// --- ESC HELPER ---
void escWriteMicroseconds(int us) {
    us = constrain(us, 1000, 2000);
    uint32_t ticks = map(us, 1000, 2000, ESC_PWM_MIN, ESC_PWM_MAX);
    // Write to the CHANNEL, which is linked to the PIN in setup
    ledcWrite(ESC_LEDC_CHANNEL, ticks); 
}

// --- I2C HELPERS (Library-Free INA228) ---
uint16_t readReg16(uint8_t reg) {
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    
    // Explicit casts to resolve "ambiguous overload" error
    Wire.requestFrom((uint16_t)INA228_ADDR, (uint8_t)2);
    
    if (Wire.available() == 2) return (Wire.read() << 8) | Wire.read();
    return 0;
}

int32_t readReg24(uint8_t reg) {
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    
    // Explicit casts to resolve "ambiguous overload" error
    Wire.requestFrom((uint16_t)INA228_ADDR, (uint8_t)3);

    if (Wire.available() == 3) {
        uint32_t val = (uint32_t)Wire.read() << 16 | (uint32_t)Wire.read() << 8 | Wire.read();
        if (val & 0x800000) val |= 0xFF000000; // Sign extension
        return (int32_t)val;
    }
    return 0;
}

// --- SAFE TARE HELPER ---
bool safeTare(HX711 &scale, const char* name, unsigned long timeout_ms = 3000) {
    Serial.printf("  Taring %s... ", name);
    unsigned long start = millis();
    while (!scale.is_ready()) {
        if (millis() - start > timeout_ms) {
            Serial.printf("TIMEOUT!\n");
            return false;
        }
        delay(10);
    }
    scale.tare();
    Serial.printf("OK\n");
    return true;
}

void setup() {
    Serial.begin(921600);
    
    // USB CDC Wait for ESP32-S3
    unsigned long bootWait = millis();
    while (!Serial && millis() - bootWait < 3000);
    delay(500);

    Serial.println("\n=== BOOT START ===");

    // 1. I2C Initialization
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    uint16_t id = readReg16(0x3F);
    if (id == 0x2280 || id == 0x2281) {
        inaOnline = true;
        Serial.println("[1/5] INA228 detected.");
    } else {
        Serial.printf("[1/5] INA228 not found (ID: 0x%04X).\n", id);
    }

    // 2. Load Cell Initialization
    scaleThrust.begin(PIN_LC_THRUST, PIN_HX_SCK);
    scalePush.begin(PIN_LC_PUSH,   PIN_HX_SCK);
    scalePull.begin(PIN_LC_PULL,   PIN_HX_SCK);
    
    scaleThrust.set_scale(CAL_THRUST);
    scalePush.set_scale(CAL_PUSH_LEFT);
    scalePull.set_scale(CAL_PULL_RIGHT);

    Serial.println("[2/5] Taring scales...");
    safeTare(scaleThrust, "THRUST");
    safeTare(scalePush,   "PUSH");
    safeTare(scalePull,   "PULL");

    // 3. ESC Initialization (Legacy API)
    Serial.println("[3/5] Initializing ESC PWM...");
    ledcSetup(ESC_LEDC_CHANNEL, ESC_LEDC_FREQ_HZ, ESC_LEDC_RESOLUTION);
    ledcAttachPin(PIN_ESC, ESC_LEDC_CHANNEL);
    escWriteMicroseconds(1000);

    // 4. RPM Initialization
    Serial.println("[4/5] Attaching RPM interrupt...");
    pinMode(PIN_RPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), handleRpmPulse, RISING);

    Serial.println("=== BOOT COMPLETE ===");
    Serial.println("Time_ms,Thrust_g,TorquePush_Nm,TorquePull_Nm,Throttle_PWM,Push_g,Pull_g,Volts,Amps,RPM");
    
    lastStream = millis();
}

void loop() {
    // Serial Command Parser
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input == "Z") {
            safeTare(scaleThrust, "THRUST");
            safeTare(scalePush, "PUSH");
            safeTare(scalePull, "PULL");
        } else {
            int val = input.toInt();
            if (val >= 1000 && val <= 2000) {
                currentThrottle = val;
                escWriteMicroseconds(val);
            }
        }
    }

    // Scale Sampling (Non-blocking)
    static float t = 0, p = 0, l = 0;
    if (scaleThrust.is_ready()) t = scaleThrust.get_units(1);
    if (scalePush.is_ready())   p = scalePush.get_units(1);
    if (scalePull.is_ready())   l = scalePull.get_units(1);

    // Data Streaming @ 50Hz
    unsigned long now = millis();
    if (now - lastStream >= 20) {
        
        // --- RPM PERIOD CALCULATION ---
        // 1. Safely grab the volatile variables
        noInterrupts();
        unsigned long interval = pulseIntervalMicros;
        unsigned long lastPulse = lastPulseTimeMicros;
        bool hasNew = newPulseAvailable;
        newPulseAvailable = false; 
        interrupts();

        // 2. Calculate RPM if a new interval was recorded
        if (hasNew && interval > 0) {
            // 60,000,000 microseconds in a minute
            currentRpm = 60000000.0 / (interval * MOTOR_POLE_PAIRS);
        }

        // 3. Zero-RPM Timeout 
        // If the motor stops, interrupts stop firing, and the RPM would freeze 
        // at its last value. If no pulse is seen for 200ms (200,000 us), force to 0.
        if (micros() - lastPulse > 200000) {
            currentRpm = 0;
        }
        // ------------------------------

        // Torque calculation (Direct Math)
        float tPush = (abs(p) > 0.5) ? (p * GRAMS_TO_N * TORQUE_ARM_M) : 0.0f;
        float tPull = (abs(l) > 0.5) ? (-l * GRAMS_TO_N * TORQUE_ARM_M) : 0.0f;

        // INA228 Power Reading
        float v = 0, a = 0;
        if (inaOnline) {
            v = (readReg24(0x05) >> 4) * 0.0001953125f;
            a = ((readReg24(0x04) >> 4) * 0.0000003125f) / 0.0002f; // Assuming 0.2mOhm shunt
        }

        // CSV Output
        Serial.printf("%lu,%.2f,%.4f,%.4f,%d,%.2f,%.2f,%.2f,%.3f,%.0f\n",
                      now, -t, tPush, tPull, currentThrottle, p, l, v, a, currentRpm);
        
        lastStream = now;
    }
}