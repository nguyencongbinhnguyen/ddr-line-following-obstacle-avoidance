// ============================================================================
// LYAPUNOV + FTC + STA
// Serial format: angle#error#state#mode#curve#seq
// ============================================================================

#include <math.h>
#include <Wire.h>
#include <stdlib.h>
#include "RobotTypes.h"

// ============================================================================
// 1. THAM SO MO HINH
// ============================================================================

constexpr float RADIUS = 0.06f;
constexpr float MASS = 11.0f;
constexpr float HALF_TRACK = 0.18f;
constexpr float B = 0.30f;
constexpr float LOOKAHEAD = 0.40f;
constexpr float IZ = 0.322f;
constexpr float Bf = 0.01543f;

constexpr float V_TARGET_LINE = 0.35f;
constexpr float V_TARGET_AVOID = 0.30f;
constexpr float V_TARGET_REJOIN = 0.30f;

constexpr float BETA1 = 6.0f;
constexpr float BETA2 = 0.8f;
constexpr float K2_OUTER = 2.0f;
constexpr float KE_ERROR = 0.0f;

// ============================================================================
// 2. TRANG THAI PHIEN BAM LINE
// ============================================================================

unsigned long sessionStartMs = 0;
unsigned int lineLossCount = 0;

bool wasEnabled = false;
bool haveControlOutput = false;

// ============================================================================
// 4. HAM PHU TRO
// ============================================================================

inline float clip(float x, float lo, float hi) {
    return fminf(hi, fmaxf(lo, x));
}

inline float signum(float x) {
    return (x > 0) - (x < 0);
}

// ============================================================================
// 5. XAC DINH HUONG MA SAT
// ============================================================================

struct FrictionDirection {
    int moving = 0;

    void reset() {
        moving = 0;
    }

    float get(float omega, float tau) {
        if (omega > 0.20f) moving = 1;
        else if (omega < -0.20f) moving = -1;
        else if (fabsf(omega) < 0.10f) moving = 0;

        if (moving) return (float)moving;
        return fabsf(tau) > 0.001f ? signum(tau) : 0;
    }
};

// ============================================================================
// 6. GIOI HAN HE SO SCALE ALPHA
// ============================================================================

inline bool constrainAlpha(float base, float slope, float &lo, float &hi) {
    if (fabsf(slope) < 1e-8f) return fabsf(base) <= 70;

    float a = (-70 - base) / slope;
    float b = (70 - base) / slope;

    lo = fmaxf(lo, fminf(a, b));
    hi = fminf(hi, fmaxf(a, b));

    return lo <= hi;
}

float maxScaleToFit(float baseL, float deltaL,
                    float baseR, float deltaR,
                    float limit)
{
    float s = 1.0f;

    if (deltaL > 1e-6f) {
        s = min(s, (limit - baseL) / deltaL);
    }
    else if (deltaL < -1e-6f) {
        s = min(s, (-limit - baseL) / deltaL);
    }

    if (deltaR > 1e-6f) {
        s = min(s, (limit - baseR) / deltaR);
    }
    else if (deltaR < -1e-6f) {
        s = min(s, (-limit - baseR) / deltaR);
    }

    return constrain(s, 0.0f, 1.0f);
}

// ============================================================================
// 7. CHUYEN MO-MEN -> PWM
// ============================================================================

/*inline PwmCommand torqueToPWM(float tauL, float tauR, float omegaL, float omegaR, float dirL, float dirR) {
    PwmCommand p = {};

    if (!isfinite(tauL) || !isfinite(tauR) || !isfinite(omegaL) || !isfinite(omegaR)) return p;

    float baseL = 4.15f * omegaL + 21.5f * dirL;
    float baseR = 4.39f * omegaR + 17.4f * dirR;

    float lo = 0;
    float hi = 1;

    if (!constrainAlpha(baseL, 103 * tauL, lo, hi) || !constrainAlpha(baseR, 103 * tauR, lo, hi)) return p;

    p.feasible = true;
    p.alpha = hi;

    p.cmdL = hi * tauL;
    p.cmdR = hi * tauR;

    p.left = clip(baseL + 103 * p.cmdL, -70, 70);
    p.right = clip(baseR + 103 * p.cmdR, -70, 70);

    return p;
}
*/
inline PwmCommand torqueToPWM(
    float tauL,
    float tauR,
    float omegaL,
    float omegaR,
    float dirL,
    float dirR
) {
    PwmCommand p = {};

    if (!isfinite(tauL) || !isfinite(tauR) ||
        !isfinite(omegaL) || !isfinite(omegaR)) {
        return p;
    }

    constexpr float K_TAU_PWM = 103.0f;
    constexpr float PWM_LIMIT = 70.0f;

    // =========================================================
    // 1. PWM BU TOC DO + MA SAT
    // =========================================================

    float baseL = 4.15f * omegaL + 21.5f * dirL;
    float baseR = 4.39f * omegaR + 17.4f * dirR;

    // Neu rieng base da vuot gioi han thi gioi han lai
    baseL = clip(baseL, -PWM_LIMIT, PWM_LIMIT);
    baseR = clip(baseR, -PWM_LIMIT, PWM_LIMIT);


    // =========================================================
    // 2. KHOI PHUC uv, uw TU tauL, tauR
    //
    // tauL = (uv - uw)/2
    // tauR = (uv + uw)/2
    //
    // => uv = tauL + tauR
    // => uw = tauR - tauL
    // =========================================================

    float uv = tauL + tauR;
    float uw = tauR - tauL;


    // =========================================================
    // 3. UU TIEN MO-MEN QUAY uw
    // =========================================================

    float tauYawL = -0.5f * uw;
    float tauYawR =  0.5f * uw;

    float yawPwmL = K_TAU_PWM * tauYawL;
    float yawPwmR = K_TAU_PWM * tauYawR;

    float loYaw = 0.0f;
    float hiYaw = 1.0f;

    if (!constrainAlpha(baseL, yawPwmL, loYaw, hiYaw) ||
        !constrainAlpha(baseR, yawPwmR, loYaw, hiYaw)) {

        return p;
    }

    // Lay he so yaw lon nhat ma PWM van nam trong +-70
    float alphaYaw = clip(hiYaw, 0.0f, 1.0f);

    float cmdYawL = baseL + alphaYaw * yawPwmL;
    float cmdYawR = baseR + alphaYaw * yawPwmR;


    // =========================================================
    // 4. PHAN PWM CON LAI MOI CAP CHO uv
    // =========================================================

    float tauVL = 0.5f * uv;
    float tauVR = 0.5f * uv;

    float vPwmL = K_TAU_PWM * tauVL;
    float vPwmR = K_TAU_PWM * tauVR;

    float loV = 0.0f;
    float hiV = 1.0f;

    if (!constrainAlpha(cmdYawL, vPwmL, loV, hiV) ||
        !constrainAlpha(cmdYawR, vPwmR, loV, hiV)) {

        return p;
    }

    // Phan uv duoc phep giu lai
    float alphaV = clip(hiV, 0.0f, 1.0f);


    // =========================================================
    // 5. PWM CUOI
    // =========================================================

    float finalL =
        cmdYawL +
        alphaV * vPwmL;

    float finalR =
        cmdYawR +
        alphaV * vPwmR;

    p.left  = clip(finalL, -PWM_LIMIT, PWM_LIMIT);
    p.right = clip(finalR, -PWM_LIMIT, PWM_LIMIT);


    // =========================================================
    // 6. MO-MEN THUC TE SAU KHI PHAN PHOI
    // =========================================================

    float uvApplied = alphaV * uv;
    float uwApplied = alphaYaw * uw;

    p.cmdL =
        0.5f * (uvApplied - uwApplied);

    p.cmdR =
        0.5f * (uvApplied + uwApplied);


    // =========================================================
    // 7. THONG TIN TRANG THAI
    // =========================================================

    // Alpha tam thoi dung de log muc uv duoc giu lai
    p.alpha = alphaV;

    // QUAN TRONG:
    // Da tim duoc PWM hop le thi feasible = true.
    // Khong duoc dung alphaYaw < 1 de bao fault.
    p.feasible = true;

    return p;
}
// ============================================================================
// 8. BO DIEU KHIEN LYAPUNOV + FTC + STA
// ============================================================================
//
// fault:
// 0 = OK
// 1 = invalid data / geometry / timing
// 3 = no feasible common torque scale
// ============================================================================

struct Controller {
    float w1 = 0;
    float w2 = 0;

    float wd = 0;
    float wdd = 0;

    bool initialized = false;

    float curveFilt = 0.0f;
    bool curveInit = false;

    FrictionDirection frictionL;
    FrictionDirection frictionR;

    void reset() {
        w1 = 0;
        w2 = 0;
        wd = 0;
        wdd = 0;

        initialized = false;

        curveFilt = 0.0f;
        curveInit = false;

        frictionL.reset();
        frictionR.reset();
    }

    Output step(float e, float theta, float v, float yaw, float curve, float omegaL, float omegaR, float vTarget, float dt) {
        Output out = {};

        // ====================================================================
        // 8.1. KIEM TRA DU LIEU
        // ====================================================================

        if (!isfinite(e) || !isfinite(theta) || !isfinite(v) || !isfinite(yaw) || !isfinite(curve) || !isfinite(omegaL) || !isfinite(omegaR) || !isfinite(vTarget) || !isfinite(dt) || dt <= 0 || dt > 0.02f) {
            out.fault = 1;
            reset();
            return out;
        }

        // ====================================================================
        // 8.2. GIOI HAN VA LOC DO CONG
        // ====================================================================

        curve = clip(curve, -0.6f, 0.6f);

        if (!curveInit) {
            curveFilt = 0.0f;
            curveInit = true;
        }

        curveFilt += 0.05f * (curve - curveFilt);
        curve = curveFilt;

        if (fabsf(theta) >= 1.4f || fabsf(curve * e) >= 1) {
            out.fault = 1;
            reset();
            return out;
        }

        // ====================================================================
        // 8.3. TRANG THAI CAN BANG THEO DO CONG
        // ====================================================================

        float thetaEq = -asinf(LOOKAHEAD * curve);
        float yawEq = curve * vTarget / sqrtf(1 - LOOKAHEAD * LOOKAHEAD * curve * curve);

        // ====================================================================
        // 8.4. LYAPUNOV TANG TREN
        // ====================================================================

        float e2 = theta - thetaEq;
        float cs = cosf(theta);
        float tn = tanf(theta);

        if (fabsf(cs) < 0.05f) cs = (cs >= 0) ? 0.05f : -0.05f;

        float aa = BETA1 * e * tn + BETA2 * curve * e2 / cs;
        float bb = -BETA1 * LOOKAHEAD * e + BETA1 * e * e * tn + BETA2 * e2 + BETA2 * curve * e * e2 / cs;
        float dd = -aa * vTarget + bb * yawEq;

        // ====================================================================
        // 8.5. VAN TOC GOC DAT x4d = wd
        // ====================================================================

        float targetRaw = yawEq - dd * bb / (bb * bb + 0.01f) - K2_OUTER * bb + KE_ERROR * e;
        float target = clip(targetRaw, -0.95f, 0.95f);

        if (!initialized) {
            wd = clip(yaw, -0.95f, 0.95f);
            wdd = 0.0f;
            initialized = true;
        }

        float previous = wd;
        wd += clip(target - wd, -6.0f * dt, 6.0f * dt);

        // ====================================================================
        // 8.6. DAO HAM VAN TOC GOC DAT x4dd = wdd
        // ====================================================================

        wdd = clip(wdd + (dt / 0.02f) * ((wd - previous) / dt - wdd), -8.0f, 8.0f);

        // ====================================================================
        // 8.7. SAI SO VAN TOC TANG DUOI
        // ====================================================================

        float z1 = v - vTarget;
        float z2 = yaw - wd;

        out.z2 = z2;
        out.thetaEq = thetaEq;
        out.e2 = e2;
        out.bb = bb;

        out.targetRaw = targetRaw;
        out.target = target;

        // ====================================================================
        // 8.8. FINITE-TIME CONTROL
        // ====================================================================

        float ft1 = 5 * signum(z1) * powf(fabsf(z1), 0.8f);
        float ft2 = 6 * signum(z2) * powf(fabsf(z2), 0.8f);

        // ====================================================================
        // 8.9. SUPER-TWISTING ALGORITHM
        // ====================================================================

        float robust1 = 1.8f * sqrtf(fabsf(z1)) * signum(z1);
        float robust2 = 2.2f * sqrtf(fabsf(z2)) * signum(z2);

        // ====================================================================
        // 8.10. CAP NHAT BIEN TICH PHAN STA
        // ====================================================================

        float next1 = w1 + dt * 1.54f * clip((z1 + dt * (-ft1 - robust1 - w1)) / (1.54f * dt * dt), -1, 1);
        float next2 = w2 + dt * 1.54f * clip((z2 + dt * (-ft2 - robust2 - w2)) / (1.54f * dt * dt), -1, 1);

        // ====================================================================
        // 8.11. DONG LUC HOC ROBOT
        // ====================================================================

        float cw = RADIUS * (IZ + MASS * B * B) / HALF_TRACK;
        out.uv = 2 * Bf * v / RADIUS - MASS * RADIUS * B * yaw * yaw - MASS * RADIUS * (ft1 + robust1 + next1);
        out.uw = 2 * Bf * HALF_TRACK * yaw / RADIUS + RADIUS * MASS * B * v * yaw / HALF_TRACK + cw * (wdd - ft2 - robust2 - next2);

        // ====================================================================
        // 8.12. CHUYEN uv, uw -> tauL, tauR
        // ====================================================================

        out.tauL = (out.uv - out.uw) / 2;
        out.tauR = (out.uv + out.uw) / 2;

        out.wd = wd;
        out.wdd = wdd;

        if (!isfinite(out.tauL) || !isfinite(out.tauR)) {
            out.fault = 1;
            reset();
            return out;
        }

        // ====================================================================
        // 8.13. XAC DINH HUONG MA SAT
        // ====================================================================

        float dirL = frictionL.get(omegaL, out.tauL);
        float dirR = frictionR.get(omegaR, out.tauR);

        // ====================================================================
        // 8.14. CHUYEN MO-MEN -> PWM
        // ====================================================================

        PwmCommand p = torqueToPWM(out.tauL, out.tauR, omegaL, omegaR, dirL, dirR);

        if (!p.feasible) {
            out.fault = 3;
            reset();
            return out;
        }

        out.alpha = p.alpha;
        out.cmdL = p.cmdL;
        out.cmdR = p.cmdR;

        out.pwmL = p.left;
        out.pwmR = p.right;

        // ====================================================================
        // 8.15. ANTI-WINDUP CHO STA
        // ====================================================================

        if (out.alpha >= 0.999999f) {
            w1 = next1;
            w2 = next2;
        }

        return out;
    }
};

// ============================================================================
// 9. KHAI BAO PHAN CUNG
// ============================================================================

const int A_L = 18;
const int B_L = 19;

const int A_R = 2;
const int B_R = 3;

const int EN_L = 5;
const int L1 = 22;
const int L2 = 24;

const int EN_R = 6;
const int R1 = 26;
const int R2 = 28;

// ============================================================================
// 10. THAM SO ENCODER
// ============================================================================

const float CPR = 1200.0f;
const float GEAR = 1.0f;

// ============================================================================
// 11. QUY UOC DAU
// ============================================================================

const float ERROR_SIGN = 1;
const float ANGLE_SIGN = 1;
const float CURVE_SIGN = 1;
const float GYRO_SIGN = 1;

const float GYRO_BIAS_DPS = -0.27f;

// ============================================================================
// 12. CHU KY DIEU KHIEN
// ============================================================================

const unsigned long CONTROL_US = 10000;
const unsigned long SPEED_US = 10000;
const unsigned long TIMEOUT_MS = 500;

// ============================================================================
// 13. BIEN TOAN CUC
// ============================================================================

volatile long countL = 0;
volatile long countR = 0;

unsigned long lastControl = 0;
unsigned long lastSpeed = 0;
unsigned long lastPacket = 0;
unsigned long lastLog = 0;

float omegaL = 0;
float omegaR = 0;

float angle = 0;
float error = 0;
float curve = 0;

int state = 0;
int receivedMode = 0;
bool rejoinActive = false;
int previousMode = 0;

// Thoi diem bat dau REJOIN
unsigned long rejoinStartMs = 0;

// Thoi diem ANG bat dau on dinh duoi nguong thoat
unsigned long rejoinStableStartMs = 0;

// REJOIN phai ton tai toi thieu 500 ms
constexpr unsigned long REJOIN_MIN_MS = 500;

// ANG phai < 20 deg lien tuc 150 ms moi duoc thoat
constexpr unsigned long REJOIN_STABLE_MS = 150;

constexpr float REJOIN_EXIT_ANGLE_DEG = 35.0f;

long receivedSeq = 0;
long lastAcceptedSeq = -1;

float vLog = 0.0f;
float yawLog = 0.0f;
float z2Log = 0.0f;

float pwmLLog = 0.0f;
float pwmRLog = 0.0f;

bool packetSeen = false;
bool latched = false;

int faultCode = 0;

char rx[100];
unsigned int used = 0;
bool overflow = false;

Controller controller;
Output uLog = {};

// ============================================================================
// 14. NGAT ENCODER
// ============================================================================

void encLA() {
    countL += (digitalRead(A_L) == digitalRead(B_L)) ? -1 : 1;
}

void encLB() {
    countL += (digitalRead(A_L) != digitalRead(B_L)) ? -1 : 1;
}

void encRA() {
    countR += (digitalRead(A_R) == digitalRead(B_R)) ? 1 : -1;
}

void encRB() {
    countR += (digitalRead(A_R) != digitalRead(B_R)) ? 1 : -1;
}

// ============================================================================
// 15. DUNG DONG CO
// ============================================================================

void stopMotors() {
    analogWrite(EN_L, 0);
    analogWrite(EN_R, 0);

    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);

    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
}

// ============================================================================
// 16. DIEU KHIEN PWM DONG CO
// ============================================================================

void setMotorPWM(int en, int in1, int in2, bool left, float signedPWM) {
    int pwm = (int)roundf(clip(fabsf(signedPWM), 0, 70));

    analogWrite(en, 0);

    if (pwm == 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return;
    }

    bool forward = signedPWM > 0;

    digitalWrite(in1, (forward == left) ? LOW : HIGH);
    digitalWrite(in2, (forward == left) ? HIGH : LOW);

    analogWrite(en, pwm);
}

void drive(float left, float right) {
    setMotorPWM(EN_L, L1, L2, true, left);
    setMotorPWM(EN_R, R1, R2, false, right);
}

// ============================================================================
// 17. NHAN VA PHAN TICH GOI DU LIEU CAMERA
// ============================================================================
//
// Format:
// @angle#error#state#mode#curve#seq
//
// Vi du:
// @-2#0.05#1#0#-0.34#125
// ============================================================================

void parsePacket() {
    char *p = rx;
    char *end;

    float angleDeg = strtod(p, &end);
    if (end == p || *end != '#' || !isfinite(angleDeg)) return;
    p = end + 1;

    float errorValue = strtod(p, &end);
    if (end == p || *end != '#' || !isfinite(errorValue)) return;
    p = end + 1;

    long stateValue = strtol(p, &end, 10);
    if (end == p || *end != '#' || stateValue < 0) return;
    p = end + 1;

    long modeValue = strtol(p, &end, 10);
    if (end == p || *end != '#' || modeValue < 0) return;
    p = end + 1;

    float curveValue = strtod(p, &end);
    if (end == p || *end != '#' || !isfinite(curveValue)) return;
    p = end + 1;

    long seqValue = strtol(p, &end, 10);
    if (end == p || *end != '\0' || seqValue < 0) return;

    if (packetSeen &&
        (millis() - lastPacket <= TIMEOUT_MS) &&
        lastAcceptedSeq >= 0 &&
        seqValue <= lastAcceptedSeq) {
        return;
    }
    angle = ANGLE_SIGN * angleDeg * PI / 180.0f;
    error = ERROR_SIGN * errorValue;
    state = (int)stateValue;
    receivedMode = (int)modeValue;
    curve = CURVE_SIGN * curveValue;
    receivedSeq = seqValue;
    lastAcceptedSeq = seqValue;

    packetSeen = true;
    lastPacket = millis();

}

// ============================================================================
// 18. RESET PHIEN DIEU KHIEN
// ============================================================================

void resetTrackingSession(unsigned long nowUs) {
    stopMotors();
    controller.reset();

    noInterrupts();
    countL = 0;
    countR = 0;
    interrupts();

    omegaL = 0.0f;
    omegaR = 0.0f;

    yawLog = 0.0f;
    z2Log = 0.0f;

    pwmLLog = 0.0f;
    pwmRLog = 0.0f;

    uLog = {};

    haveControlOutput = false;
    lastControl = nowUs;
    lastSpeed = nowUs;
}

// ============================================================================
// 19. DOC SERIAL KHONG CHAN
// ============================================================================

void readSerial() {
    static char current[100];
    static unsigned int currentUsed = 0;
    static bool currentOverflow = false;

    char latest[100];
    bool haveLatest = false;

    // Doc HET du lieu dang co trong UART
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n') {
            if (!currentOverflow && currentUsed > 0) {
                current[currentUsed] = '\0';

                // Ghi de packet cu -> chi giu packet moi nhat
                strncpy(latest, current, sizeof(latest) - 1);
                latest[sizeof(latest) - 1] = '\0';

                haveLatest = true;
            }

            currentUsed = 0;
            currentOverflow = false;
        }
        else if (c != '\r') {
            if (!currentOverflow) {
                if (currentUsed < sizeof(current) - 1) {
                    current[currentUsed++] = c;
                }
                else {
                    currentOverflow = true;
                }
            }
        }
    }

    // Chi parse packet hoan chinh MOI NHAT
    if (haveLatest) {
        strncpy(rx, latest, sizeof(rx) - 1);
        rx[sizeof(rx) - 1] = '\0';

        parsePacket();
    }
}

// ============================================================================
// 20. DOC GYRO MPU6050
// ============================================================================

bool readYaw(float &yaw) {
    Wire.beginTransmission(0x68);
    Wire.write(0x47);

    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true) != 2) return false;

    int16_t raw = (int16_t)(((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read());

    yaw = GYRO_SIGN * (raw / 131.0f - GYRO_BIAS_DPS) * PI / 180.0f;

    return true;
}

// ============================================================================
// 21. SETUP
// ============================================================================

void setup() {
    pinMode(EN_L, OUTPUT);
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);

    pinMode(EN_R, OUTPUT);
    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);

    stopMotors();

    pinMode(A_L, INPUT_PULLUP);
    pinMode(B_L, INPUT_PULLUP);

    pinMode(A_R, INPUT_PULLUP);
    pinMode(B_R, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(A_L), encLA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(B_L), encLB, CHANGE);

    attachInterrupt(digitalPinToInterrupt(A_R), encRA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(B_R), encRB, CHANGE);

    Serial.begin(115200);
    Wire.begin();

#if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true);
#endif

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);

    if (Wire.endTransmission(true) != 0) {
        latched = true;
        faultCode = 4;
    }

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0);

    if (Wire.endTransmission(true) != 0) {
        latched = true;
        faultCode = 4;
    }

    lastControl = micros();
    lastSpeed = lastControl;
}

// ============================================================================
// 22. LOOP
// ============================================================================

void loop() {
    // ========================================================================
    // 22.1. NHAN DU LIEU CAMERA
    // ========================================================================

    readSerial();

    unsigned long now = micros();
    unsigned long nowMs = millis();

    // ========================================================================
    // 22.2. TINH TOC DO GOC BANH XE TU ENCODER
    // ========================================================================

    unsigned long elapsed = now - lastSpeed;

    if (elapsed >= SPEED_US) {
        noInterrupts();

        long nl = countL;
        long nr = countR;

        countL = 0;
        countR = 0;

        interrupts();

        lastSpeed = now;

        float dt = elapsed * 1e-6f;

        omegaL = nl * 2 * PI / (CPR * GEAR * dt);
        omegaR = nr * 2 * PI / (CPR * GEAR * dt);
    }

    // ========================================================================
    // 22.3. KIEM TRA SERIAL TIMEOUT
    // ========================================================================

    bool serialAlive = packetSeen && (nowMs - lastPacket <= TIMEOUT_MS);

    if (!serialAlive) {
        if (wasEnabled) {
            lineLossCount++;

            unsigned long runTime = nowMs - sessionStartMs;

            Serial.print("LINE_LOST RUN=");
            Serial.print(runTime);
            Serial.print("ms COUNT=");
            Serial.println(lineLossCount);
        }

        wasEnabled = false;

        stopMotors();
        controller.reset();
        haveControlOutput = false;

        return;
    }

    // ========================================================================
    // 22.4. KIEM TRA CO CHO PHEP DIEU KHIEN KHONG
    // ========================================================================

    bool lineAvailable = state > 0;
    bool enabled = lineAvailable && !latched;

    // ========================================================================
    // 22.5. BAT DAU MOT PHIEN DIEU KHIEN MOI
    // ========================================================================

    if (enabled && !wasEnabled) {

        sessionStartMs = nowMs;

        resetTrackingSession(now);

        if (!latched) {
            faultCode = 0;
        }

        Serial.print("LINE_FOUND T=");
        Serial.println(sessionStartMs);
    }

    // ========================================================================
    // 22.6. VUA MAT LINE
    // ========================================================================

    if (!enabled && wasEnabled) {

        lineLossCount++;

        unsigned long runTime = nowMs - sessionStartMs;

        Serial.print("LINE_LOST RUN=");
        Serial.print(runTime);

        Serial.print("ms COUNT=");
        Serial.println(lineLossCount);
    }

    wasEnabled = enabled;

    // ========================================================================
    // 22.7. STATE = 0 -> DUNG XE
    // ========================================================================

    if (state == 0) {

        stopMotors();

        controller.reset();

        haveControlOutput = false;

        return;
    }

    // ========================================================================
    // 22.8. FAULT / KHONG ENABLE -> DUNG XE
    // ========================================================================

    if (!enabled) {

        stopMotors();

        controller.reset();

        haveControlOutput = false;

        return;
    }
    // Phat hien chuyen AVOID -> LINE
    if (previousMode == 1 && receivedMode == 0) {
        rejoinActive = true;
        rejoinStartMs = nowMs;
        rejoinStableStartMs = 0;    
    }

    previousMode = receivedMode;

    // ========================================================================
    // 22.9. CHON VAN TOC DAT THEO MODE
    // ========================================================================

    /*float vTarget =
        (receivedMode == 0)
        ? V_TARGET_LINE
        : V_TARGET_AVOID;*/
    // ========================================================================
    // 22.9. CHON VAN TOC DAT THEO TRANG THAI
    // ========================================================================

    float vTarget;

    float absAngleDeg = fabsf(angle) * 180.0f / PI;


    // ========================================================================
    // A. TRANH VAT CAN
    // ========================================================================

    if (receivedMode == 1) {

        vTarget = V_TARGET_AVOID;      // 0.30 m/s
    }


    // ========================================================================
    // B. BAM LINE
    // ========================================================================

    else {

        // ====================================================================
        // B1. VUA TU AVOID QUAY LAI LINE
        // ====================================================================

        if (rejoinActive) {

            vTarget = V_TARGET_REJOIN;     // 0.22 m/s


            // ================================================================
            // DIEU KIEN THOAT REJOIN
            // ================================================================

            // Khong cho thoat REJOIN trong 500 ms dau
            if (nowMs - rejoinStartMs >= REJOIN_MIN_MS) {

                // ANG da nho
                if (absAngleDeg < REJOIN_EXIT_ANGLE_DEG) {

                    // Bat dau tinh thoi gian on dinh
                    if (rejoinStableStartMs == 0) {
                        rejoinStableStartMs = nowMs;
                    }

                    // ANG < 20 deg lien tuc 150 ms
                    else if (nowMs - rejoinStableStartMs >= REJOIN_STABLE_MS) {

                        rejoinActive = false;
                        rejoinStableStartMs = 0;

                        // Sau khi thoat REJOIN, tra ve toc do bam line
                        vTarget = V_TARGET_LINE;
                    }
                }
                else {

                    // ANG lon lai -> tinh lai thoi gian on dinh
                    rejoinStableStartMs = 0;
                }
            }
            else {

                rejoinStableStartMs = 0;
            }
        }


        // ====================================================================
        // B2. BAM LINE BINH THUONG
        // ====================================================================

        else {

            vTarget = V_TARGET_LINE;       // 0.35 m/s
        }
    }
    // ========================================================================
    // 22.10. VONG DIEU KHIEN 100 Hz
    // ========================================================================

    if (now - lastControl >= CONTROL_US) {
        float dt = (now - lastControl) * 1e-6f;

        lastControl = now;

        float yaw = 0;

        if (!readYaw(yaw)) {
            latched = true;
            faultCode = 4;

            stopMotors();
            controller.reset();
            haveControlOutput = false;
        }
        else {
            float v = RADIUS * (omegaL + omegaR) / 2;
            vLog = v;

            Output u = controller.step(error, angle, v, yaw, curve, omegaL, omegaR, vTarget, dt);

            if (u.fault) {
                faultCode = u.fault;

                stopMotors();
                controller.reset();
                haveControlOutput = false;
            }
            else {
                drive(u.pwmL, u.pwmR);

                yawLog = yaw;
                z2Log = u.z2;

                pwmLLog = u.pwmL;
                pwmRLog = u.pwmR;

                uLog = u;

                haveControlOutput = true;
            }
        }
    }

    // ========================================================================
    // 22.11. TELEMETRY
    // ========================================================================

    if (enabled && haveControlOutput && nowMs - lastLog >= 200 && Serial.availableForWrite() >= 32) {
        lastLog = nowMs;

        Serial.print(" T=");
        Serial.print(nowMs - sessionStartMs);

        Serial.print(" ERR=");
        Serial.print(error, 3);

        Serial.print(" ANG=");
        Serial.print(angle * 180.0f / PI, 1);

        Serial.print(" TH_EQ=");
        Serial.print(uLog.thetaEq * 180.0f / PI, 1);

        Serial.print(" CUR=");
        Serial.print(curve, 3);

        Serial.print(" MODE=");
        Serial.print(receivedMode);

        Serial.print(" V=");
        Serial.print(vLog, 3);

        Serial.print(" VSP=");
        Serial.print(vTarget, 3);

        //Serial.print(" RJ=");
        //Serial.print(rejoinActive ? 1 : 0);

        //Serial.print(" TRAW=");
        //Serial.print(uLog.targetRaw, 3);

        //Serial.print(" TGT=");
        //Serial.print(uLog.target, 3);

        Serial.print(" WD=");
        Serial.print(uLog.wd, 3);

        Serial.print(" YAW=");
        Serial.print(yawLog, 3);

        //Serial.print(" Z2=");
        //Serial.print(z2Log, 3);

        //Serial.print(" A=");
        //Serial.print(uLog.alpha, 3);

        //Serial.print(" PL=");
        //Serial.print(pwmLLog, 1);

        //Serial.print(" PR=");
        //Serial.print(pwmRLog, 1);

        Serial.println();
    }
}
