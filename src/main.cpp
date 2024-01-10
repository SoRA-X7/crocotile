#include <Arduino.h>
#include <ArduinoJson.h>
#include <BleMouse.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include "HapticSystem.h"
#include "MouseDriver.h"

// const HapticSystemConfig xConf{2,45,46,2,5,4,13}; // Mega2560
const HapticSystemConfig xConf{1, 4, 2, 15, 0, 19, 18};  // ESP32
HapticSystem systemX = HapticSystem(xConf);
MouseDriver mouse = MouseDriver(400, &systemX);

hw_timer_t* sub_timer = NULL;
TaskHandle_t thp[1];

TimerHandle_t handle;

// Commander cmdr;

char* pattern_str = NULL;
DynamicJsonDocument current_pattern(2048);
HapticPatternBump bumps[32];
int bump_count;

void cmdP(String arg) {
  Serial.println("cmd:P");
  strcpy(pattern_str, arg.c_str());
  Serial.println(pattern_str);
  deserializeJson(current_pattern, pattern_str);
  auto ts = current_pattern["bumps"].as<JsonArray>();
  int i = 0;
  // bumps = (HapticPatternBump*)malloc(sizeof(HapticPatternBump) * ts.size());
  for (JsonObject t : ts) {
    HapticPatternBump b{t["x"], t["strength"]};
    bumps[i++] = b;
  }
  bump_count = i;
  systemX.set_pattern(bumps, bump_count);
  Serial.println("cmd:P ok");

  return;
}

void cmdN() {
  Serial.println("cmd:N");
  current_pattern.clear();
  bump_count = 0;
  systemX.revoke_pattern();
  Serial.println("cmd:N ok");
  return;
}

void runCommand() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  if (cmd[0] == 'P') {
    cmdP(cmd.substring(1));
  } else if (cmd[0] == 'N') {
    cmdN();
  }
}

void mouseLoop(void* params) {
  for (;;) {
    vTaskDelay(10);
    mouse.loop();
  }
}

void setup() {
  pattern_str = (char*)malloc(sizeof(char) * 4096);
  // put your setup code here, to run once:
  Serial.begin(115200);  // Log
                         //   Serial1.begin(115200);  // To mouse
  Wire.begin();          // Magnetic Encoder
  Serial.println("Crocotile MCU v0.2.0");
  Serial.print("port tick rate: ");
  Serial.println(portTICK_RATE_MS);

  // cmdr = Commander(Serial, '\n', true);
  // cmdr.add('P', cmdP, "register linear haptic pattern");
  // cmdr.add('N', cmdN, "reset haptic pattern");

  systemX.init();

  mouse.init();
  xTaskCreate(mouseLoop, "mouseLoop", 4096, NULL, 3, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
  // cmdr.run();
  runCommand();
  systemX.loop();
  vTaskDelay(1);
}
