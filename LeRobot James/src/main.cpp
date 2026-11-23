#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <math.h>

#include "motors.h"
#include "robot_config.h"
#include "runlog.h"

// Busca un objeto de TARGET_WIDTH_MM de diámetro girando en el lugar, y guarda
// en la EEPROM lo que pasó para verlo después por USB.
//
// Uso:
//   1. Subir el programa con el USB (VIN desconectado).
//   2. Desenchufar el USB y dar energía por VIN: a los START_DELAY_MS arranca la misión.
//   3. Para repetir la prueba sin USB: apretar RESET. Vuelve a contar y arranca otra misión.
//   4. Volver a enchufar el USB (VIN desconectado) y abrir el monitor: imprime los datos
//      guardados (las 2 últimas corridas). OJO: hay que mandar una tecla durante la
//      cuenta regresiva para quedarse en MODO USB; si no, arranca una misión nueva.
//
// Detección: mientras gira, detecta los bordes del objeto por saltos de distancia
// y cuenta cuántas medidas duró. Lo compara con las que debería durar un objeto
// de TARGET_WIDTH_MM a esa distancia:
//
//   N = θ / Δθ
//   θ  = 2·atan(w / 2D)    ángulo que ocupa el objeto (D = distancia + desfase del sensor)
//   Δθ = ω · T             grados que gira el robot entre dos medidas
//
// Si coincide (± tolerancia), vuelve al centro del objeto, avanza o retrocede hasta
// quedar a APPROACH_TARGET_MM y mueve el servo ~90° y de vuelta.

namespace {
// --- Objetivo ---
constexpr float TARGET_WIDTH_MM = 50.0f;       // diámetro del objeto a buscar (base de 5 cm)

// --- Giro (MEDIR: girar 360° con SCAN_PWM, cronometrar t, ω = 360 / t) ---
constexpr int SCAN_PWM = 150;
constexpr float TURN_RATE_DEG_PER_S = 21.0f;   // aprox.: una vuelta en ~17 s (cronometrado a ojo, falta medir bien)

// --- Geometría del robot (MEDIR) ---
constexpr float SENSOR_OFFSET_MM = 100.0f;     // del eje de giro al sensor

// --- Cono del VL53L0X (CALIBRAR) ---
// 0 = solo geometría. El cono hace que el objeto dure más medidas de las que da la fórmula.
// Calibrar con una corrida: CONE_DEG = medidas reales × DEG_PER_SAMPLE − ángulo geométrico.
constexpr float CONE_DEG = 8.0f;               // de la prueba del 9/11: base a ~68 cm duró 20-26 medidas (40/s)

// --- Frecuencia del sensor ---
// 20 ms es el tiempo de medición mínimo del VL53L0X (modo rápido). Con 33 ms / 40 ms
// el objeto de 5 cm duraba muy pocas medidas y a 30-40 cm quedaba como "ruido".
// (El ruido que se vio una vez venía de apuntar a una pantalla, no de este modo.)
constexpr uint32_t TIMING_BUDGET_US = 20000;
constexpr uint16_t SAMPLE_INTERVAL_MS = 50;    // 20 medidas por segundo (antes 25 ms = 40/s)

// --- Detección ---
constexpr uint16_t MIN_RANGE_MM = 40;
constexpr uint16_t MAX_RANGE_MM = 1200;        // más allá se considera "nada"
constexpr uint16_t EDGE_JUMP_MM = 100;         // salto mínimo para considerar un borde
constexpr uint8_t SAMPLE_TOLERANCE = 2;        // ± medidas aceptadas
constexpr uint8_t MIN_SAMPLES = 3;             // menos que esto se toma como ruido

// --- Misión ---
constexpr uint16_t START_DELAY_MS = 3000;      // espera antes de arrancar (y ventana para modo USB)
constexpr uint8_t SEARCH_TURNS = 3;            // vueltas máximas antes de rendirse
constexpr uint32_t MAX_SEARCH_MS = SEARCH_TURNS * 360000UL / (uint32_t)TURN_RATE_DEG_PER_S;
constexpr uint16_t LOG_UPDATE_MS = 2000;       // cada cuánto actualiza duración y medidas en la EEPROM

// --- Centrado (con el sensor, girando lento) ---
// 1) vuelve lento cruzando el objeto y mide cuánto tarda; 2) gira para el otro lado y,
// al volver a ver el borde, sigue la mitad de ese tiempo. Como las dos cosas se hacen a
// la misma velocidad, se cancelan la inercia del giro rápido, el cono y la distancia.
// Con PWM 100 el robot no llegaba a moverse: cada giro lento arranca con un empujón
// (CENTER_KICK_PWM durante CENTER_KICK_MS) para vencer la fricción y sigue a CENTER_PWM.
constexpr int CENTER_PWM = 120;                // lento, pero que se mueva
constexpr int CENTER_KICK_PWM = 180;
constexpr uint16_t CENTER_KICK_MS = 80;
constexpr uint16_t CENTER_PAUSE_MS = 250;      // quieto entre un giro y otro
constexpr uint16_t CENTER_BAND_MM = 60;        // "veo el objeto" = lectura <= distancia + esto
constexpr uint8_t CENTER_CONFIRM = 2;          // lecturas seguidas para aceptar un borde
constexpr uint16_t CENTER_TIMEOUT_MS = 3000;   // por cada giro lento
constexpr uint8_t CENTER_MIN_SAMPLES = 3;      // un cruce con menos lecturas no vale
constexpr uint8_t CENTER_TRIES = 2;            // intentos de cruce (alternando el sentido)
constexpr uint16_t CENTER_AIM_TOL_MM = 30;     // "quedó apuntando" = distancia final ± esto
// CALIBRAR: si al final queda pasado del centro, subir; si queda corto, bajar (puede ser negativo).
constexpr int16_t CENTER_COAST_MS = 0;

// --- Acercamiento (después de centrarse) ---
// Avanza (o retrocede si quedó muy cerca) en línea recta mirando el sensor, hasta que la
// distancia SENSOR -> objeto sea APPROACH_TARGET_MM. Sin encoders: si el robot se desvía
// y el objeto sale del cono, lo detecta por el salto de distancia y se detiene.
constexpr uint16_t APPROACH_TARGET_MM = 400;   // 40 cm, medidos desde el sensor
constexpr uint16_t APPROACH_TOL_MM = 20;       // aceptado: 38-42 cm
// CALIBRAR: lo que sigue andando después de frenar (si queda más cerca de 40 cm, subir).
constexpr uint16_t APPROACH_COAST_MM = 30;
// CALIBRAR: si al avanzar se va hacia un lado, bajar el PWM de la rueda contraria.
constexpr int APPROACH_PWM_LEFT = 140;
constexpr int APPROACH_PWM_RIGHT = 140;
constexpr int APPROACH_KICK_PWM = 180;
constexpr uint16_t APPROACH_KICK_MS = 80;
constexpr uint16_t APPROACH_TIMEOUT_MS = 10000;  // por cada tramo
constexpr uint8_t APPROACH_TRIES = 3;          // tramos (el 2º y 3º corrigen lo que se pasó)
constexpr uint8_t APPROACH_LOST_CONFIRM = 2;   // lecturas seguidas "lejos" = lo perdió

// --- Servo continuo ---
// Un servo continuo no sabe en qué ángulo está: se controla velocidad y tiempo.
// Antes de la misión se lo deja a mano en la posición inicial. "90°" = girar SERVO_GO_MS.
// CALIBRAR con 's' en MODO USB:
//   - si no queda quieto con SERVO_STOP_US, ajustar SERVO_STOP_US (típico 1480-1520);
//   - si gira para el lado equivocado, cambiar el signo de SERVO_SPEED_US;
//   - si se pasa o no llega a 90°, ajustar SERVO_GO_MS; si no vuelve igual, SERVO_BACK_MS.
constexpr uint16_t SERVO_STOP_US = 1500;
constexpr int16_t SERVO_SPEED_US = 200;        // diferencia con STOP (más = más rápido, máx ~500)
constexpr uint16_t SERVO_GO_MS = 400;
constexpr uint16_t SERVO_BACK_MS = 400;
constexpr uint16_t SERVO_HOLD_MS = 500;        // quieto en los 90° antes de volver

// Si el ruido de los motores traba el I2C, Wire corta a los 25 ms en vez de colgarse.
constexpr uint32_t I2C_TIMEOUT_US = 25000;

constexpr uint32_t SERIAL_BAUD = 9600;         // aguanta mejor el ruido que 115200

// Arranque suave: la velocidad sube de 0 al valor pedido en RAMP_MS.
// Evita el pico de corriente que puede disparar la protección de la batería.
constexpr uint16_t RAMP_MS = 300;
constexpr uint8_t RAMP_STEPS = 15;

constexpr float DEG_PER_SAMPLE = TURN_RATE_DEG_PER_S * SAMPLE_INTERVAL_MS / 1000.0f;

enum Mode : uint8_t { MODE_USB, MODE_MISSION, MODE_DONE };

VL53L0X tof;
Motors motors;
Servo servo;

Mode mode = MODE_USB;
bool showReadings = true;
uint32_t lastSampleAt = 0;
uint32_t missionStartAt = 0;
uint32_t lastLogUpdateAt = 0;
uint16_t readings = 0;

bool hasPrevious = false;
uint16_t previousRangeMm = 0;

bool inSegment = false;
uint16_t segmentSamples = 0;
uint32_t segmentRangeSum = 0;
uint16_t segmentRefMm = 0;
uint32_t segmentFirstAt = 0;
uint32_t segmentLastAt = 0;

// ---------- Sensor y geometría ----------

bool startSensor() {
    tof.setTimeout(100);
    if (!tof.init()) {
        return false;
    }
    tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
    tof.startContinuous(SAMPLE_INTERVAL_MS);
    return true;
}

// Ángulo (grados) que ocupa el objeto visto desde el eje de giro.
float targetSpanDeg(uint16_t distanceMm) {
    const float d = distanceMm + SENSOR_OFFSET_MM;
    return 2.0f * atan(TARGET_WIDTH_MM / (2.0f * d)) * 180.0f / PI + CONE_DEG;
}

// Medidas consecutivas que debería durar el objeto a esa distancia.
float expectedSamples(uint16_t distanceMm) {
    return targetSpanDeg(distanceMm) / DEG_PER_SAMPLE;
}

uint16_t missionDs() {
    return (millis() - missionStartAt) / 100;
}

// ---------- Impresión ----------

void printPadded(uint32_t value, uint8_t width) {
    uint8_t digits = 1;
    for (uint32_t v = value; v >= 10; v /= 10) {
        ++digits;
    }
    while (digits++ < width) {
        Serial.print(' ');
    }
    Serial.print(value);
}

void printSeconds(uint16_t ds, uint8_t width) {
    printPadded(ds / 10, width);
    Serial.print('.');
    Serial.print(ds % 10);
    Serial.print(F(" s"));
}

void printStatus(uint8_t status) {
    switch (status) {
        case RUN_FOUND:        Serial.print(F("OBJETO ENCONTRADO, centrado, a 40 cm y servo movido")); break;
        case RUN_NOT_FOUND:    Serial.print(F("no lo encontro")); break;
        case RUN_NOT_CENTERED: Serial.print(F("lo encontro, pero NO pudo centrarse (no lo cruzo lento)")); break;
        case RUN_NOT_APPROACHED: Serial.print(F("se centro, pero NO pudo quedar a 40 cm (servo sin mover)")); break;
        default:            Serial.print(F("incompleta (se corto la energia o se reinicio)")); break;
    }
}

void printReport() {
    Serial.println();
    Serial.println(F("================ DATOS GUARDADOS ================"));
    bool any = false;
    for (uint8_t age = 0; age < runlog::SLOTS; ++age) {
        RunHeader h;
        if (!runlog::readHeader(age, h)) {
            continue;
        }
        any = true;
        Serial.print(F("Corrida #"));
        Serial.print(h.runId);
        Serial.println(age == 0 ? F("  (la mas reciente)") : F(""));
        Serial.print(F("  resultado: "));
        printStatus(h.status);
        Serial.println();
        Serial.print(F("  duracion"));
        printSeconds(h.durationDs, 3);
        Serial.print(F(" | "));
        Serial.print(h.readings);
        Serial.println(F(" medidas"));
        if (h.status == RUN_FOUND || h.status == RUN_NOT_CENTERED ||
            h.status == RUN_NOT_APPROACHED) {
            Serial.print(F("  centrado: objeto a "));
            Serial.print(h.targetMm);
            Serial.print(F(" mm | cruce lento: "));
            Serial.print(h.centerSamples);
            Serial.print(F(" lecturas, "));
            Serial.print(h.centerHalfMs * 2);
            Serial.print(F(" ms | intentos: "));
            Serial.println(h.centerTries);
        }
        if (h.status == RUN_FOUND || h.status == RUN_NOT_APPROACHED) {
            Serial.print(F("  despues de centrarse apunta a "));
            Serial.print(h.finalMm);
            Serial.print(F(" mm -> "));
            const uint16_t diff = h.finalMm > h.targetMm ? h.finalMm - h.targetMm
                                                         : h.targetMm - h.finalMm;
            Serial.println(diff <= CENTER_AIM_TOL_MM ? F("QUEDO APUNTANDO AL OBJETO")
                                                     : F("NO quedo apuntando al objeto"));
            Serial.print(F("  acercamiento: quedo a "));
            Serial.print(h.approachMm);
            Serial.print(F(" mm (objetivo "));
            Serial.print(APPROACH_TARGET_MM);
            Serial.println(F(" mm)"));
        }

        Serial.print(F("  objetos vistos: "));
        Serial.println(h.objectCount);
        if (h.objectCount == 0) {
            continue;
        }
        Serial.println(F("      #   tiempo  distancia  medidas  esperadas  resultado"));
        for (uint8_t i = 0; i < h.objectCount; ++i) {
            ObjectRecord o;
            if (!runlog::readObject(age, i, o)) {
                break;
            }
            printPadded(i + 1, 7);
            printSeconds(o.timeDs, 5);
            printPadded(o.distanceMm, 8);
            Serial.print(F(" mm"));
            printPadded(o.samples, 9);
            Serial.print(F("       "));
            Serial.print(expectedSamples(o.distanceMm), 1);
            Serial.println(o.result == OBJ_TARGET ? F("   OBJETO") : F("   otro"));
        }
    }
    if (!any) {
        Serial.println(F("(no hay corridas guardadas)"));
    }
    Serial.println(F("================================================="));
}

void printUsbHelp() {
    Serial.println();
    Serial.println(F("MODO USB: motores apagados, no se graba nada."));
    Serial.println(F("  r = ver datos guardados"));
    Serial.println(F("  b = borrar datos guardados"));
    Serial.println(F("  n = mostrar / ocultar distancias"));
    Serial.println(F("  m = arrancar la mision ahora"));
    Serial.println(F("  s = probar el servo (ida ~90 grados y vuelta)"));
}

// ---------- Motores ----------

// Lleva los motores de 0 a (left, right) de a poco, en 'ms'.
void rampTo(int left, int right, uint16_t ms = RAMP_MS) {
    for (uint8_t i = 1; i <= RAMP_STEPS; ++i) {
        motors.setSpeed((long)left * i / RAMP_STEPS, (long)right * i / RAMP_STEPS);
        delay(ms / RAMP_STEPS);
    }
}

// ---------- Lectura y centrado ----------

// Espera la próxima medida del sensor. Devuelve 0 si la lectura no sirve.
uint16_t readOnce() {
    const uint16_t rangeMm = tof.readRangeContinuousMillimeters();
    if (Wire.getWireTimeoutFlag()) {
        // El ruido trabó el I2C: reiniciar el sensor.
        Wire.clearWireTimeoutFlag();
        startSensor();
        return 0;
    }
    if (tof.timeoutOccurred()) {
        return 0;
    }
    if (showReadings) {
        Serial.println(rangeMm);
    }
    if (mode == MODE_MISSION && readings < UINT16_MAX) {
        ++readings;
    }
    return rangeMm;
}

// Sigue girando (ya en marcha) hasta un borde del objeto: entering = empieza a verlo,
// !entering = deja de verlo. Devuelve millis() del borde, o 0 si se acabó el tiempo.
uint8_t edgeSeen = 0;  // lecturas que vieron el objeto desde el último reset

uint32_t turnUntilEdge(bool entering, uint16_t thresholdMm) {
    const uint32_t start = millis();
    uint8_t streak = 0;
    while (millis() - start < CENTER_TIMEOUT_MS) {
        const uint16_t r = readOnce();
        if (r == 0) {
            continue;
        }
        const bool seesObject = r >= MIN_RANGE_MM && r <= thresholdMm;
        if (seesObject && edgeSeen < 255) {
            ++edgeSeen;
        }
        streak = (seesObject == entering) ? streak + 1 : 0;
        if (streak >= CENTER_CONFIRM) {
            return millis();
        }
    }
    return 0;
}

// Giro lento con empujón inicial. dir = +1 sentido del barrido, -1 el contrario.
void turnSlow(int8_t dir) {
    motors.setSpeed(-dir * CENTER_KICK_PWM, dir * CENTER_KICK_PWM);
    delay(CENTER_KICK_MS);
    motors.setSpeed(-dir * CENTER_PWM, dir * CENTER_PWM);
}

// Promedio de 'count' lecturas válidas (descarta "fuera de rango"). 0 si ninguna sirve.
uint16_t readAverage(uint8_t count) {
    uint32_t sum = 0;
    uint8_t n = 0;
    for (uint8_t i = 0; i < count; ++i) {
        const uint16_t r = readOnce();
        if (r >= MIN_RANGE_MM && r <= MAX_RANGE_MM) {
            sum += r;
            ++n;
        }
    }
    return n ? sum / n : 0;
}

struct CenterResult {
    bool ok;
    uint16_t halfMs;
    uint16_t finalMm;
    uint8_t samples;  // lecturas que vieron el objeto en el último cruce
    uint8_t tries;    // cruces intentados
};

// Centra el robot en el objeto que está a objMm, mirando con el sensor.
// Se llama justo después de pasar el objeto girando en el sentido del barrido.
CenterResult centerByEdges(uint16_t objMm) {
    CenterResult res = {false, 0, 0, 0, 0};
    const uint16_t threshold = objMm + CENTER_BAND_MM;

    for (uint8_t attempt = 0; attempt < CENTER_TRIES; ++attempt) {
        // Primero hacia atrás (ya lo pasamos); si falla, para el otro lado.
        const int8_t dir = (attempt % 2 == 0) ? -1 : 1;
        res.tries = attempt + 1;

        motors.stop();
        delay(CENTER_PAUSE_MS);

        // 1) Cruzar el objeto entero, lento, y medir cuánto tarda.
        edgeSeen = 0;
        turnSlow(dir);
        const uint32_t tIn = turnUntilEdge(true, threshold);
        const uint32_t tOut = tIn ? turnUntilEdge(false, threshold) : 0;
        motors.stop();
        res.samples = edgeSeen;
        if (tOut == 0 || edgeSeen < CENTER_MIN_SAMPLES) {
            continue;  // no lo cruzó de verdad (no se movió, o solo rozó el borde)
        }
        res.halfMs = (tOut - tIn) / 2;
        delay(CENTER_PAUSE_MS);

        // 2) Girar para el otro lado: al volver a ver el borde, seguir la mitad del tiempo.
        turnSlow(-dir);
        if (turnUntilEdge(true, threshold) == 0) {
            motors.stop();
            continue;
        }
        // El borde se confirma una medida tarde (CENTER_CONFIRM = 2): descontar esa medida.
        const int32_t runMs = (int32_t)res.halfMs - SAMPLE_INTERVAL_MS - CENTER_COAST_MS;
        if (runMs > 0) {
            delay(runMs);
        }
        motors.stop();
        delay(CENTER_PAUSE_MS);

        // 3) Medir hacia dónde quedó apuntando (sin "fuera de rango").
        res.finalMm = readAverage(4);
        res.ok = true;
        return res;
    }
    motors.stop();
    return res;
}

// ---------- Acercamiento ----------

// Arranca en línea recta con empujón. dir = +1 adelante, -1 atrás.
void driveStraight(int8_t dir) {
    motors.setSpeed(dir * APPROACH_KICK_PWM, dir * APPROACH_KICK_PWM);
    delay(APPROACH_KICK_MS);
    motors.setSpeed(dir * APPROACH_PWM_LEFT, dir * APPROACH_PWM_RIGHT);
}

struct ApproachResult {
    bool ok;
    uint16_t finalMm;  // distancia al terminar (0 = no pudo medir)
};

// Con el robot ya centrado, avanza o retrocede hasta quedar a APPROACH_TARGET_MM.
ApproachResult approachTarget() {
    ApproachResult res = {false, readAverage(4)};
    for (uint8_t attempt = 0; attempt < APPROACH_TRIES && res.finalMm != 0; ++attempt) {
        const int16_t error = (int16_t)res.finalMm - (int16_t)APPROACH_TARGET_MM;
        if (abs(error) <= APPROACH_TOL_MM) {
            res.ok = true;
            return res;
        }
        const int8_t dir = error > 0 ? 1 : -1;  // lejos: adelante; muy cerca: atrás
        // Frena un poco antes, por lo que sigue andando después de cortar los motores.
        const uint16_t stopAtMm = dir > 0 ? APPROACH_TARGET_MM + APPROACH_COAST_MM
                                          : APPROACH_TARGET_MM - APPROACH_COAST_MM;

        driveStraight(dir);
        const uint32_t start = millis();
        uint16_t lastGood = res.finalMm;
        uint8_t lostStreak = 0;
        bool lost = false;
        while (millis() - start < APPROACH_TIMEOUT_MS) {
            const uint16_t r = readOnce();
            if (r < MIN_RANGE_MM) {
                continue;  // lectura inválida (0) o basura
            }
            // Salto hacia lejos (o "fuera de rango"): el objeto salió del cono.
            if (r >= lastGood + EDGE_JUMP_MM) {
                if (++lostStreak >= APPROACH_LOST_CONFIRM) {
                    lost = true;
                    break;
                }
                continue;
            }
            lostStreak = 0;
            lastGood = r;
            if ((dir > 0 && r <= stopAtMm) || (dir < 0 && r >= stopAtMm)) {
                break;
            }
        }
        motors.stop();
        delay(CENTER_PAUSE_MS);
        res.finalMm = readAverage(4);
        if (lost) {
            return res;  // se desvió: no seguir a ciegas
        }
    }
    res.ok = res.finalMm != 0 &&
             abs((int16_t)res.finalMm - (int16_t)APPROACH_TARGET_MM) <= APPROACH_TOL_MM;
    return res;
}

// ---------- Servo ----------

// Ida ~90°, pausa y vuelta. Solo manda pulsos mientras se mueve (detach al final),
// así un STOP mal calibrado no lo deja girando despacio.
void servoSwing() {
    servo.attach(SERVO_PIN);
    servo.writeMicroseconds(SERVO_STOP_US + SERVO_SPEED_US);
    delay(SERVO_GO_MS);
    servo.writeMicroseconds(SERVO_STOP_US);
    delay(SERVO_HOLD_MS);
    servo.writeMicroseconds(SERVO_STOP_US - SERVO_SPEED_US);
    delay(SERVO_BACK_MS);
    servo.writeMicroseconds(SERVO_STOP_US);
    delay(100);
    servo.detach();
}

// ---------- Misión ----------

void startMission() {
    runlog::startRun();
    readings = 0;
    hasPrevious = false;
    inSegment = false;
    mode = MODE_MISSION;

    Serial.println(F("MISION: buscando el objeto. Los motores arrancan."));
    Serial.flush();  // que el texto salga limpio antes del ruido de los motores

    rampTo(-SCAN_PWM, SCAN_PWM);
    missionStartAt = millis();
    lastLogUpdateAt = missionStartAt;
    lastSampleAt = missionStartAt;
}

// LED L: prendido = todo bien; parpadeo rápido = no se centró;
// parpadeo lento = no quedó a 40 cm; apagado = no lo encontró.
uint16_t blinkPeriodMs = 0;  // 0 = no parpadea

void endMission(RunStatus status, uint16_t centerHalfMs = 0, uint16_t targetMm = 0,
                uint16_t finalMm = 0, uint8_t centerSamples = 0, uint8_t centerTries = 0,
                uint16_t approachMm = 0) {
    motors.stop();
    mode = MODE_DONE;
    digitalWrite(LED_BUILTIN, status == RUN_FOUND ? HIGH : LOW);
    blinkPeriodMs = status == RUN_NOT_CENTERED     ? 150
                    : status == RUN_NOT_APPROACHED ? 600
                                                   : 0;
    runlog::finish(status, missionDs(), readings, centerHalfMs, targetMm, finalMm,
                   centerSamples, centerTries, approachMm);
    Serial.print(F("MISION TERMINADA: "));
    printStatus(status);
    Serial.println();
}

// Se llama cuando el objeto terminó (salto hacia arriba de la distancia).
void evaluateSegment() {
    inSegment = false;
    if (segmentSamples < MIN_SAMPLES) {
        return;  // ruido: no se guarda
    }

    const uint16_t avgRangeMm = segmentRangeSum / segmentSamples;
    const float diff = fabs(segmentSamples - expectedSamples(avgRangeMm));
    const bool match = diff <= SAMPLE_TOLERANCE;

    // Si es el objeto, primero frenar, centrar, acercarse y mover el servo;
    // grabar en la EEPROM lleva unos ms.
    CenterResult center = {false, 0, 0, 0, 0};
    ApproachResult approach = {false, 0};
    if (match) {
        digitalWrite(LED_BUILTIN, HIGH);  // LED L prendido: lo encontró
        center = centerByEdges(avgRangeMm);
        if (center.ok) {
            approach = approachTarget();
            if (approach.ok) {
                servoSwing();
            }
        }
    }

    ObjectRecord obj;
    obj.timeDs = (segmentLastAt - missionStartAt) / 100;
    obj.distanceMm = avgRangeMm;
    obj.samples = segmentSamples > 255 ? 255 : segmentSamples;
    obj.result = match ? OBJ_TARGET : OBJ_OTHER;
    runlog::addObject(obj);

    if (match) {
        const RunStatus status = !center.ok    ? RUN_NOT_CENTERED
                                 : !approach.ok ? RUN_NOT_APPROACHED
                                                : RUN_FOUND;
        endMission(status, center.halfMs, avgRangeMm, center.finalMm, center.samples,
                   center.tries, approach.finalMm);
    }
}

// Detección de bordes con una medida ya válida.
void detect(uint16_t rangeMm, uint32_t now) {
    if (rangeMm < MIN_RANGE_MM) {
        return;  // demasiado cerca o lectura basura
    }
    if (rangeMm > MAX_RANGE_MM) {
        rangeMm = MAX_RANGE_MM;  // "nada" cuenta como fondo lejano
    }

    if (!hasPrevious) {
        hasPrevious = true;
        previousRangeMm = rangeMm;
        return;
    }

    if (!inSegment) {
        // Borde de entrada: la distancia baja de golpe.
        if (rangeMm < MAX_RANGE_MM && previousRangeMm >= rangeMm + EDGE_JUMP_MM) {
            inSegment = true;
            segmentSamples = 1;
            segmentRangeSum = rangeMm;
            segmentRefMm = rangeMm;
            segmentFirstAt = now;
            segmentLastAt = now;
        }
    } else if (rangeMm >= segmentRefMm + EDGE_JUMP_MM) {
        // Borde de salida: la distancia sube de golpe.
        evaluateSegment();
    } else if (rangeMm + EDGE_JUMP_MM <= segmentRefMm) {
        // Apareció algo todavía más cerca: empieza un objeto nuevo.
        segmentSamples = 1;
        segmentRangeSum = rangeMm;
        segmentRefMm = rangeMm;
        segmentFirstAt = now;
        segmentLastAt = now;
    } else if (segmentSamples < UINT16_MAX) {
        ++segmentSamples;
        segmentRangeSum += rangeMm;
        segmentLastAt = now;
    }

    previousRangeMm = rangeMm;
}

void sample() {
    const uint32_t now = millis();
    if (now - lastSampleAt < SAMPLE_INTERVAL_MS) {
        return;
    }
    lastSampleAt = now;

    const uint16_t rangeMm = readOnce();
    if (rangeMm != 0 && mode == MODE_MISSION) {
        detect(rangeMm, now);
    }
}

void handleCommand(char c) {
    switch (c) {
        case 'r': case 'R':
            printReport();
            break;
        case 'b': case 'B':
            runlog::clear();
            Serial.println(F("Datos borrados."));
            break;
        case 'n': case 'N':
            showReadings = !showReadings;
            break;
        case 'm': case 'M':
            if (mode != MODE_MISSION) {
                startMission();
            }
            break;
        case 's': case 'S':
            if (mode != MODE_MISSION) {
                Serial.println(F("Servo: ida y vuelta."));
                servoSwing();
            }
            break;
        default:
            if (mode == MODE_USB) {
                printUsbHelp();
            }
            break;
    }
}

// Cuenta regresiva: si llega cualquier tecla por el Serial, devuelve true (modo USB).
bool waitForUsbKey() {
    Serial.print(F("Envia cualquier tecla en "));
    Serial.print(START_DELAY_MS / 1000);
    Serial.println(F(" s para quedarte en MODO USB (si no, arranca la mision)."));
    const uint32_t start = millis();
    uint8_t lastShown = 0;
    while (millis() - start < START_DELAY_MS) {
        if (Serial.available()) {
            while (Serial.available()) {
                Serial.read();
            }
            return true;
        }
        const uint8_t left = (START_DELAY_MS - (millis() - start) + 999) / 1000;  // redondeo hacia arriba
        if (left != lastShown) {
            lastShown = left;
            Serial.print(left);
            Serial.print(F("... "));
        }
    }
    Serial.println();
    return false;
}
} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    Wire.begin();
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);  // no colgarse si el ruido traba el bus
    motors.begin();
    motors.stop();
    runlog::begin();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    printReport();

    if (!startSensor()) {
        Serial.println(F("ERROR: no encuentro el sensor VL53L0X"));
        while (true) {
            motors.stop();
            delay(1000);
        }
    }

    // Siempre arranca la misión salvo que llegue una tecla en la cuenta regresiva:
    // así, apretando RESET (sin USB) se repite la prueba. La corrida nueva pisa la más
    // vieja de la EEPROM; la anterior queda como segunda en el informe.
    if (waitForUsbKey()) {
        runlog::markRead();
        mode = MODE_USB;
        printUsbHelp();
    } else {
        startMission();
    }
}

void loop() {
    if (Serial.available()) {
        handleCommand(Serial.read());
    }

    sample();

    if (mode == MODE_MISSION) {
        const uint32_t now = millis();
        if (now - missionStartAt >= MAX_SEARCH_MS) {
            endMission(RUN_NOT_FOUND);
        } else if (now - lastLogUpdateAt >= LOG_UPDATE_MS) {
            lastLogUpdateAt = now;
            runlog::update(missionDs(), readings);
        }
    } else if (blinkPeriodMs) {
        digitalWrite(LED_BUILTIN, (millis() / blinkPeriodMs) % 2);
    }
}
