#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "motors.h"
#include "robot_config.h"

namespace {
constexpr uint16_t MIN_TARGET_RANGE_MM = 40;
constexpr uint16_t MAX_TARGET_RANGE_MM = 1800;
constexpr uint16_t SAMPLE_INTERVAL_MS = 50;
constexpr uint8_t REQUIRED_HITS = 3;
constexpr uint8_t REQUIRED_MISSES = 3;
constexpr uint8_t SCAN_SPEED = 100;

VL53L0X tof;
Motors motors;

bool targetCandidate = false;
bool targetLocked = false;
uint8_t hitCount = 0;
uint8_t missCount = 0;
uint16_t closestRangeMm = MAX_TARGET_RANGE_MM;
uint32_t firstHitAt = 0;
uint32_t lastHitAt = 0;
uint32_t lastSampleAt = 0;

void clearCandidate() {
    targetCandidate = false;
    hitCount = 0;
    missCount = 0;
    closestRangeMm = MAX_TARGET_RANGE_MM;
}

void beginScan() {
    clearCandidate();
    lastSampleAt = millis();
    motors.setSpeed(-SCAN_SPEED, SCAN_SPEED);
    Serial.println(F("Buscando el pilar: giro lento iniciado."));
}

void centerOnTarget(uint32_t now) {
    const uint32_t centerAt = firstHitAt + (lastHitAt - firstHitAt) / 2;
    const uint32_t correctionMs = now - centerAt;

    targetLocked = true;
    motors.stop();
    delay(150);
    motors.setSpeed(SCAN_SPEED, -SCAN_SPEED);
    delay(correctionMs);
    motors.stop();

    Serial.print(F("Pilar detectado. Distancia minima: "));
    Serial.print(closestRangeMm);
    Serial.print(F(" mm. Correccion aproximada: "));
    Serial.print(correctionMs);
    Serial.println(F(" ms. Robot detenido apuntando al centro."));
}

void sampleTarget() {
    if (targetLocked) {
        return;
    }

    const uint32_t now = millis();
    if (now - lastSampleAt < SAMPLE_INTERVAL_MS) {
        return;
    }
    lastSampleAt = now;

    const uint16_t rangeMm = tof.readRangeContinuousMillimeters();
    const bool timedOut = tof.timeoutOccurred();
    if (timedOut) {
        Serial.println(F("Aviso: timeout leyendo VL53L0X; muestra descartada."));
    }

    const bool hit = !timedOut &&
                     rangeMm >= MIN_TARGET_RANGE_MM &&
                     rangeMm <= MAX_TARGET_RANGE_MM;

    if (hit) {
        if (!targetCandidate) {
            targetCandidate = true;
            firstHitAt = now;
            closestRangeMm = rangeMm;
        }
        lastHitAt = now;
        if (rangeMm < closestRangeMm) {
            closestRangeMm = rangeMm;
        }
        if (hitCount < REQUIRED_HITS) {
            ++hitCount;
        }
        missCount = 0;
        return;
    }

    if (!targetCandidate) {
        return;
    }

    if (missCount < REQUIRED_MISSES) {
        ++missCount;
    }
    if (missCount < REQUIRED_MISSES) {
        return;
    }

    if (hitCount >= REQUIRED_HITS) {
        centerOnTarget(now);
    } else {
        Serial.println(F("Objeto no confirmado; se reanuda la busqueda."));
        beginScan();
    }
}
} // namespace

void setup() {
    Serial.begin(115200);
    Wire.begin();
    motors.begin();
    motors.stop();

    tof.setTimeout(100);
    if (!tof.init()) {
        Serial.println(F("ERROR: no se encontro el VL53L0X. Robot detenido."));
        while (true) {
            motors.stop();
            delay(1000);
        }
    }

    tof.startContinuous(SAMPLE_INTERVAL_MS);
    beginScan();
}

void loop() {
    sampleTarget();
}
