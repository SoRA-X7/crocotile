#include <Arduino.h>
#include <ArduinoJson.h>
#include <BleMouse.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include "HapticSystem.h"
#include "MouseDriver.h"

// const HapticSystemConfig xConf{2,45,46,2,5,4,13}; // Mega2560
// const HapticSystemConfig xConf{0, 1, 12, 13, 10, 11, 46, 9};
const HapticSystemConfig xConf{.direction = 0,
                               .sen_pca_id = 1,
                               .sen_i2c_bus = 0,
                               .sen_i2c_scl = 21,
                               .sen_i2c_sda = 20,
                               .drv_uh = 12,
                               .drv_ul = 13,
                               .drv_vh = 10,
                               .drv_vl = 11,
                               .drv_wh = 46,
                               .drv_wl = 9};
const HapticMotorCalibration xCalib{1.90, Direction::CW, 7};
HapticSystem systemX = HapticSystem(xConf, xCalib);

// const HapticSystemConfig yConf{1, 1, 15, 16, 6, 7, 4, 5};
const HapticSystemConfig yConf{.direction = 1,
                               .sen_pca_id = 1,
                               .sen_i2c_bus = 1,
                               .sen_i2c_scl = 39,
                               .sen_i2c_sda = 38,
                               .drv_uh = 15,
                               .drv_ul = 16,
                               .drv_vh = 6,
                               .drv_vl = 7,
                               .drv_wh = 4,
                               .drv_wl = 5};
const HapticMotorCalibration yCalib{2.32, Direction::CCW, 7};
HapticSystem systemY = HapticSystem(yConf, yCalib);
MouseDriver mouse = MouseDriver(400, &systemX, &systemY, 40, 41);

char* pattern_str = NULL;
DynamicJsonDocument pattern_json(8192);
HapticPattern* pattern = NULL;

void cmdP(String arg) {
  if (pattern != NULL) {
    delete pattern;
    pattern = NULL;
  }
  Serial.println("cmd:P");
  strcpy(pattern_str, arg.c_str());
  Serial.println(pattern_str);

  deserializeJson(pattern_json, pattern_str);

  auto ts = pattern_json["bumps"].as<JsonArray>();
  pattern = new HapticPattern(ts.size());
  int i = 0;
  for (JsonObject t : ts) {
    HapticPatternBump b{t["x"], t["y"], t["strength"], t["curveX"],
                        t["curveY"]};
    pattern->bumps[i++] = b;
  }
  pattern->x_neg = pattern_json["xNeg"];
  pattern->x_pos = pattern_json["xPos"];
  pattern->y_neg = pattern_json["yNeg"];
  pattern->y_pos = pattern_json["yPos"];
  pattern->intensity_x = pattern_json["intensityX"];
  pattern->intensity_y = pattern_json["intensityY"];
  systemX.set_pattern(pattern);
  systemY.set_pattern(pattern);
  Serial.println("cmd:P ok");

  return;
}

void cmdN() {
  Serial.println("cmd:N");
  systemX.revoke_pattern();
  systemY.revoke_pattern();
  pattern_json.clear();
  delete pattern;
  pattern = NULL;
  Serial.println("cmd:N ok");
  return;
}

void cmdR(String arg) {
  Serial.println("cmd:R");
  int valX, valY;
  sscanf(arg.c_str(), "%d,%d", &valX, &valY);
  systemX.revise_position(valX, valY);
  systemY.revise_position(valX, valY);
  Serial.println("cmd:R ok");
}

void runCommand() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  if (cmd[0] == 'P') {
    cmdP(cmd.substring(1));
  } else if (cmd[0] == 'N') {
    cmdN();
  } else if (cmd[0] == 'R') {
    cmdR(cmd.substring(1));
  }
}

void mouseLoop(void* params) {
  for (;;) {
    vTaskDelay(10);
    mouse.loop();
  }
}

void setup() {
  pattern_str = (char*)malloc(sizeof(char) * 32768);
  // put your setup code here, to run once:
  Serial.begin(115200);  // Log
                         //   Serial1.begin(115200);  // To mouse
                         //   Wire.begin(20, 21);    // Magnetic Encoder
  Serial.println("Crocotile MCU v0.3.0");
  Serial.print("port tick rate: ");
  Serial.println(portTICK_RATE_MS);

  // cmdr = Commander(Serial, '\n', true);
  // cmdr.add('P', cmdP, "register linear haptic pattern");
  // cmdr.add('N', cmdN, "reset haptic pattern");

  systemX.init();
  systemY.init();

  mouse.init();
  xTaskCreate(mouseLoop, "mouseLoop", 4096, NULL, 3, NULL);
  Serial.println("ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
  // cmdr.run();
  runCommand();
  systemX.loop();
  systemY.loop();
  vTaskDelay(1);
}
