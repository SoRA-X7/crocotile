#include "HapticSystem.h"

#include "Arduino.h"
#include "HardwareSerial.h"
#include "common/base_classes/FOCMotor.h"
#include "utils.h"

#define PCAADDR 0x70

#define FOC_PID_P 4
#define FOC_PID_I 0
#define FOC_PID_D 0.04
#define FOC_PID_OUTPUT_RAMP 10000
#define FOC_PID_LIMIT 10

#define FOC_VOLTAGE_LIMIT 5

void log(const char* msg) { Serial.println(msg); };

HapticSystem::HapticSystem(const HapticSystemConfig& cfg,
                           const HapticMotorCalibration& calib) {
  direction = cfg.direction;
  sen_pca_id = cfg.sen_pca_id;
  sen_i2c_scl = cfg.sen_i2c_scl;
  sen_i2c_sda = cfg.sen_i2c_sda;
  sensor = MagneticSensorI2C(sen_i2c_addr, sen_res, sen_angle_register_msb,
                             sen_bits_used_msb);
  wire = TwoWire(cfg.sen_i2c_bus);
  motor = BLDCMotor(calib.motor_pole_pairs);
  motor.zero_electric_angle = calib.zero_elec_offset;
  motor.sensor_direction = calib.sensor_direction;
  driver = BLDCDriver6PWM(cfg.drv_uh, cfg.drv_ul, cfg.drv_vh, cfg.drv_vl,
                          cfg.drv_wh, cfg.drv_wl);
}

HapticSystem::~HapticSystem() {}

bool HapticSystem::init() {
  pcaselect();
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
  motor.monitor_downsample = 100;
  motor.useMonitoring(Serial);
  Serial.print(sen_i2c_scl);
  Serial.println(sen_i2c_sda);
  wire.begin(sen_i2c_sda, sen_i2c_scl);
  sensor.init(&wire);
  driver.voltage_power_supply = 5;
  driver.voltage_limit = 5;
  driver.pwm_frequency = 32000;

  if (!driver.init()) return false;

  motor.controller = MotionControlType::torque;
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // Configurable area
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.velocity_limit = 10000;
  motor.PID_velocity.P = FOC_PID_P;
  motor.PID_velocity.I = FOC_PID_I;
  motor.PID_velocity.D = FOC_PID_D;
  motor.PID_velocity.output_ramp = FOC_PID_OUTPUT_RAMP;
  motor.PID_velocity.limit = FOC_PID_LIMIT;

  motor.init();

  //   calibrate();

  delay(500);

  return true;
}

void HapticSystem::loop() {
  motor.loopFOC();
  float strength = 0;
  if (haptic_pattern != NULL && haptic_pattern->bumps != NULL) {
    int min = direction == 0 ? haptic_pattern->x_min : haptic_pattern->y_min;
    int max = direction == 0 ? haptic_pattern->x_max : haptic_pattern->y_max;

    float dt = (angle() - mouseOffset) * 400;  // 400 = sens

    int deadzone = direction == 0 ? 0 : 30;
    if (dt < min - deadzone) {
      strength = 10 * softlin(-(dt - (min - deadzone)) / 10);
    } else if (dt > max + deadzone) {
      strength = -10 * softlin(dt - (max + deadzone) / 10);
    } else {
      for (int i = 0; i < haptic_pattern->bump_count; i++) {
        auto bump = haptic_pattern->bumps[i];
        if (direction != bump.direction) continue;

        float curve = bump.curve;
        int t = dt - bump.dt;
        float s = 0;
        if (-curve < t && t < 0) {
          s += bump.strength * (-t - curve) / curve;
        } else if (0 < t && t < curve) {
          s += bump.strength * (-t + curve) / curve;
        }
        s *= haptic_pattern->intensity;
        strength += s;
      }
    }
  }
  motor.move(strength * 0.2);
  motor.monitor();
}

void HapticSystem::set_pattern(HapticPattern* pat) {
  this->haptic_pattern = pat;
  this->mouseOffset = this->angle();
}

void HapticSystem::revoke_pattern() { this->haptic_pattern = NULL; }

void HapticSystem::revise_position(int x, int y) {
  int t = this->direction == 0 ? x : y;
  float dt = (angle() - mouseOffset) * 400;  // 400 = sens
  float diff = dt - t;
  mouseOffset += diff / 400;
}

float HapticSystem::angle() { return lpf(motor.shaftAngle()); }

// https://github.com/scottbez1/smartknob/blob/4eb988399c3fda6ffd3006772856093dfe9adb86/firmware/src/motor_task.cpp#L346
void HapticSystem::calibrate() {
  // SimpleFOC is supposed to be able to determine this automatically (if you
  // omit params to initFOC), but it seems to have a bug (or I've misconfigured
  // it) that gets both the offset and direction very wrong! So this value is
  // based on experimentation.

  log("\n\n\nStarting calibration, please DO NOT TOUCH MOTOR until complete!");
  delay(1000);

  motor.controller = MotionControlType::angle_openloop;
  motor.pole_pairs = 1;

  motor.zero_electric_angle = 0;
  motor.sensor_direction = Direction::CW;
  motor.initFOC();

  float a = 0;

  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.move(a);

  // #### Determine direction motor rotates relative to angle sensor
  for (uint8_t i = 0; i < 200; i++) {
    sensor.update();
    motor.move(a);
    delay(1);
  }
  float start_sensor = sensor.getAngle();

  for (; a < 3 * _2PI; a += 0.01) {
    sensor.update();
    motor.move(a);
    delay(1);
  }

  for (uint8_t i = 0; i < 200; i++) {
    sensor.update();
    delay(1);
  }
  float end_sensor = sensor.getAngle();

  motor.voltage_limit = 0;
  motor.move(a);

  log("");

  float movement_angle = fabsf(end_sensor - start_sensor);
  if (movement_angle < radians(30) || movement_angle > radians(180)) {
    snprintf(buf_, sizeof(buf_),
             "ERROR! Unexpected sensor change: start=%.2f end=%.2f",
             start_sensor, end_sensor);
    log(buf_);
    return;
  }

  log("Sensor measures positive for positive motor rotation:");
  if (end_sensor > start_sensor) {
    log("YES, Direction=CW");
    motor.zero_electric_angle = 0;
    motor.sensor_direction = Direction::CW;
    motor.initFOC();
  } else {
    log("NO, Direction=CCW");
    motor.zero_electric_angle = 0;
    motor.sensor_direction = Direction::CCW;
    motor.initFOC();
  }
  snprintf(buf_, sizeof(buf_), "  (start was %.1f, end was %.1f)", start_sensor,
           end_sensor);
  log(buf_);

  // #### Determine pole-pairs
  // Rotate 20 electrical revolutions and measure mechanical angle traveled, to
  // calculate pole-pairs
  uint8_t electrical_revolutions = 20;
  snprintf(buf_, sizeof(buf_), "Going to measure %d electrical revolutions...",
           electrical_revolutions);
  log(buf_);
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.move(a);
  log("Going to electrical zero...");
  float destination = a + _2PI;
  for (; a < destination; a += 0.03) {
    sensor.update();
    motor.move(a);
    delay(1);
  }
  log("pause...");  // Let momentum settle...
  for (uint16_t i = 0; i < 1000; i++) {
    sensor.update();
    delay(1);
  }
  log("Measuring...");

  start_sensor = motor.sensor_direction * sensor.getAngle();
  destination = a + electrical_revolutions * _2PI;
  for (; a < destination; a += 0.03) {
    sensor.update();
    motor.move(a);
    delay(1);
  }
  for (uint16_t i = 0; i < 1000; i++) {
    sensor.update();
    motor.move(a);
    delay(1);
  }
  end_sensor = motor.sensor_direction * sensor.getAngle();
  motor.voltage_limit = 0;
  motor.move(a);

  if (fabsf(motor.shaft_angle - motor.target) > 1 * PI / 180) {
    log("ERROR: motor did not reach target!");
    while (1) {
    }
  }

  float electrical_per_mechanical =
      electrical_revolutions * _2PI / (end_sensor - start_sensor);
  snprintf(buf_, sizeof(buf_),
           "Electrical angle / mechanical angle (i.e. pole pairs) = %.2f",
           electrical_per_mechanical);
  log(buf_);

  if (electrical_per_mechanical < 3 || electrical_per_mechanical > 12) {
    snprintf(buf_, sizeof(buf_),
             "ERROR! Unexpected calculated pole pairs: %.2f",
             electrical_per_mechanical);
    log(buf_);
    return;
  }

  int measured_pole_pairs = (int)round(electrical_per_mechanical);
  snprintf(buf_, sizeof(buf_), "Pole pairs set to %d", measured_pole_pairs);
  log(buf_);

  delay(1000);

  // #### Determine mechanical offset to electrical zero
  // Measure mechanical angle at every electrical zero for several revolutions
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.move(a);
  float offset_x = 0;
  float offset_y = 0;
  float destination1 = (floor(a / _2PI) + measured_pole_pairs / 2.) * _2PI;
  float destination2 = (floor(a / _2PI)) * _2PI;
  for (; a < destination1; a += 0.4) {
    motor.move(a);
    delay(100);
    for (uint8_t i = 0; i < 100; i++) {
      sensor.update();
      delay(1);
    }
    float real_electrical_angle = _normalizeAngle(a);
    float measured_electrical_angle =
        _normalizeAngle((float)(motor.sensor_direction * measured_pole_pairs) *
                            sensor.getMechanicalAngle() -
                        0);

    float offset_angle = measured_electrical_angle - real_electrical_angle;
    offset_x += cosf(offset_angle);
    offset_y += sinf(offset_angle);

    snprintf(buf_, sizeof(buf_), "%.2f, %.2f, %.2f",
             degrees(real_electrical_angle), degrees(measured_electrical_angle),
             degrees(_normalizeAngle(offset_angle)));
    log(buf_);
  }
  for (; a > destination2; a -= 0.4) {
    motor.move(a);
    delay(100);
    for (uint8_t i = 0; i < 100; i++) {
      sensor.update();
      delay(1);
    }
    float real_electrical_angle = _normalizeAngle(a);
    float measured_electrical_angle =
        _normalizeAngle((float)(motor.sensor_direction * measured_pole_pairs) *
                            sensor.getMechanicalAngle() -
                        0);

    float offset_angle = measured_electrical_angle - real_electrical_angle;
    offset_x += cosf(offset_angle);
    offset_y += sinf(offset_angle);

    snprintf(buf_, sizeof(buf_), "%.2f, %.2f, %.2f",
             degrees(real_electrical_angle), degrees(measured_electrical_angle),
             degrees(_normalizeAngle(offset_angle)));
    log(buf_);
  }
  motor.voltage_limit = 0;
  motor.move(a);

  float avg_offset_angle = atan2f(offset_y, offset_x);

  // #### Apply settings
  motor.pole_pairs = measured_pole_pairs;
  motor.zero_electric_angle = avg_offset_angle + _3PI_2;
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.controller = MotionControlType::torque;

  log("");
  log("RESULTS:");
  snprintf(buf_, sizeof(buf_), "  ZERO_ELECTRICAL_OFFSET: %.2f",
           motor.zero_electric_angle);
  log(buf_);
  if (motor.sensor_direction == Direction::CW) {
    log("  FOC_DIRECTION: Direction::CW");
  } else {
    log("  FOC_DIRECTION: Direction::CCW");
  }
  snprintf(buf_, sizeof(buf_), "  MOTOR_POLE_PAIRS: %d", motor.pole_pairs);
  log(buf_);

  //   log("");
  //   log("Saving to persistent configuration...");
  //   PB_MotorCalibration calibration = {
  //       .calibrated = true,
  //       .zero_electrical_offset = motor.zero_electric_angle,
  //       .direction_cw = motor.sensor_direction == Direction::CW,
  //       .pole_pairs = motor.pole_pairs,
  //   };
  //   if (configuration_.setMotorCalibrationAndSave(calibration)) {
  //     log("Success!");
  //   }
};

void HapticSystem::pcaselect() {
#if defined(__AVR_ATmega2560__) || defined(AVR_ATmega1280)
  uint8_t id = sen_pca_id;
  if (id > 7) return;

  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << id);
  Wire.endTransmission();
#endif
}
