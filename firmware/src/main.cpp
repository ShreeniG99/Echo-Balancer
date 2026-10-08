// Echo Balancer -- Wokwi firmware: NIS confidence gate on an ESP32-S3.
//
// Runs the Kalman filter + NIS + NORMAL/CAUTIOUS/HALT gate (and the tilt
// baseline gate) at 200 Hz. The "robot" is a virtual one (echo::VirtualRobot:
// nominal linear plant driven by the LQR motor command) because Wokwi has no
// physics. Faults are injected three ways:
//   * MPU6050 sliders (real I2C part): the accel/gyro DEVIATION from the
//     boot-time rest pose is added to psi_acc / gyro -> sensor faults.
//   * KY-040 knob: wheel-slip offset on the encoder (theta_enc).
//   * Serial commands: p push, g gyro-bias step, a accel-noise x k,
//     m payload shift (true plant != nominal), r reset faults.
// Mode outputs: green/yellow/red LEDs, buzzer in HALT. Serial prints NIS.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "echo_core.hpp"

// ---- Pin map (matches firmware/diagram.json) ------------------------------
static const int PIN_SDA = 8, PIN_SCL = 9;  // ESP32-S3 default I2C0
static const int PIN_LED_NORMAL = 4, PIN_LED_CAUTIOUS = 5, PIN_LED_HALT = 6;
static const int PIN_BUZZER = 7;
static const int PIN_ENC_CLK = 15, PIN_ENC_DT = 16;

// ---- MPU6050 (the MPU6050 half of the GY-87) ------------------------------
static const uint8_t MPU_ADDR = 0x68, MPU_PWR_MGMT_1 = 0x6B, MPU_ACCEL_XOUT_H = 0x3B;
static const double ACCEL_LSB_PER_G = 16384.0;   // +/-2 g range (power-on default)
static const double GYRO_LSB_PER_DPS = 131.0;    // +/-250 deg/s range (power-on default)
static const double DEG2RAD = 0.017453292519943295;
static const int CALIB_SAMPLES = 100;

// ---- Run-time parameters that are firmware-only (not physics) -------------
static const uint32_t SERIAL_BAUD = 115200;
static const int PRINT_EVERY_TICKS = 10;         // 20 Hz telemetry
static const uint32_t BUZZER_HALF_PERIOD_MS = 125;
static const uint64_t RNG_SEED = 1;
static const double ENC_COUNT_TO_RAD = 6.283185307179586 / ECHO_ENCODER_CPR;

static echo::Sim sim(RNG_SEED);
static bool mpu_ok = false;
static double ref_tilt = 0.0, ref_gyro = 0.0;     // rest-pose references
static volatile int32_t enc_count = 0;
static bool gyro_fault = false, accel_fault = false;

static void IRAM_ATTR onEncClk() { enc_count += digitalRead(PIN_ENC_DT) ? 1 : -1; }

static bool readMpu(double &tilt, double &gyro_y) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; ++i) { const uint8_t hi = Wire.read(); v[i] = (int16_t)((hi << 8) | Wire.read()); }
  const double ax = v[0] / ACCEL_LSB_PER_G, az = v[2] / ACCEL_LSB_PER_G;
  tilt = atan2(ax, az);                              // CLAUDE.md section 7: psi_acc = atan2(a_x, a_z)
  gyro_y = (v[5] / GYRO_LSB_PER_DPS) * DEG2RAD;      // pitch rate about the wheel axle (y)
  return true;
}

static void calibrate() {
  double st = 0, sg = 0;
  int ok = 0;
  for (int i = 0; i < CALIB_SAMPLES; ++i) {
    double t, g;
    if (readMpu(t, g)) { st += t; sg += g; ++ok; }
    delay(2);
  }
  mpu_ok = ok > 0;
  if (mpu_ok) { ref_tilt = st / ok; ref_gyro = sg / ok; }
}

static void handleSerial() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'p': sim.robot.push(); Serial.println("# push: psi_dot kick"); break;
      case 'g': gyro_fault = !gyro_fault; Serial.printf("# gyro bias fault %s\n", gyro_fault ? "ON" : "off"); break;
      case 'a': accel_fault = !accel_fault; Serial.printf("# accel noise fault %s\n", accel_fault ? "ON" : "off"); break;
      case 'm': sim.faults.payload_shifted = !sim.faults.payload_shifted;
                Serial.printf("# payload shift %s\n", sim.faults.payload_shifted ? "ON" : "off"); break;
      case 'r': gyro_fault = accel_fault = false; sim.faults.payload_shifted = false;
                noInterrupts(); enc_count = 0; interrupts(); Serial.println("# faults cleared"); break;
      default: break;
    }
  }
}

static void showMode(echo::Mode m, uint32_t now_ms) {
  digitalWrite(PIN_LED_NORMAL, m == echo::Mode::NORMAL);
  digitalWrite(PIN_LED_CAUTIOUS, m == echo::Mode::CAUTIOUS);
  digitalWrite(PIN_LED_HALT, m == echo::Mode::HALT);
  // HALT: beep flag. Pulsing the pin every tick makes an audible tone on the Wokwi buzzer.
  static bool lvl = false;
  if (m == echo::Mode::HALT && ((now_ms / BUZZER_HALF_PERIOD_MS) & 1)) { lvl = !lvl; digitalWrite(PIN_BUZZER, lvl); }
  else digitalWrite(PIN_BUZZER, LOW);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_LED_NORMAL, OUTPUT); pinMode(PIN_LED_CAUTIOUS, OUTPUT); pinMode(PIN_LED_HALT, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP); pinMode(PIN_ENC_DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), onEncClk, FALLING);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(MPU_ADDR); Wire.write(MPU_PWR_MGMT_1); Wire.write(0); Wire.endTransmission();
  calibrate();
  Serial.printf("# Echo Balancer NIS gate | MPU6050 %s | tau1=%.0f tau2=%.0f N=%d\n",
                mpu_ok ? "ok" : "NOT FOUND (slider faults disabled)", ECHO_TAU1, ECHO_TAU2, ECHO_GATE_N);
  Serial.println("# keys: p push | g gyro bias | a accel noise | m payload | r reset; MPU sliders + knob also inject faults");
  Serial.println("t_s,nis,eps,nis_mode,tilt_mode,psi_deg,psi_hat_deg,u_V,fallen");
}

void loop() {
  static uint32_t next_us = micros();
  static uint32_t tick = 0;
  const uint32_t period_us = (uint32_t)(ECHO_DT * 1e6);
  if ((int32_t)(micros() - next_us) < 0) return;
  next_us += period_us;

  handleSerial();

  // Real-sensor deviations from the rest pose become injected faults.
  double tilt = ref_tilt, gy = ref_gyro;
  if (mpu_ok) readMpu(tilt, gy);
  noInterrupts(); const int32_t cnt = enc_count; interrupts();
  sim.faults.accel_offset = tilt - ref_tilt;
  sim.faults.gyro_bias = (gy - ref_gyro) + (gyro_fault ? ECHO_GYRO_BIAS_STEP : 0.0);
  sim.faults.accel_noise_mult = accel_fault ? ECHO_ACCEL_NOISE_MULT : 1.0;
  sim.faults.enc_offset = cnt * ENC_COUNT_TO_RAD;

  const echo::Mode before = sim.nis_gate.mode;
  sim.tick();
  const echo::Mode m = sim.nis_gate.mode;
  showMode(m, millis());
  if (m != before) Serial.printf("# t=%.2fs mode %s -> %s (eps=%.0f)\n", tick * ECHO_DT, echo::mode_name(before), echo::mode_name(m), sim.epsilon);

  if (tick % PRINT_EVERY_TICKS == 0)
    Serial.printf("%.2f,%.2f,%.0f,%s,%s,%.3f,%.3f,%.2f,%d\n", tick * ECHO_DT, sim.nis, sim.epsilon,
                  echo::mode_name(m), echo::mode_name(sim.tilt_gate.mode),
                  sim.robot.x[1] / DEG2RAD, sim.kf.x[1] / DEG2RAD, sim.u_prev, (int)sim.fallen);
  ++tick;
}
