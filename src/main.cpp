#include <Arduino.h>
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

void mouseLoop(TimerHandle_t pxTimer) {
  mouse.loop();
  // Serial.println("run");
}

// void registerMouseTimer(void* args) {
//   Serial.println("timer init start");
//   mouse.init();
//   sub_timer = timerBegin(3, 80, true);
//   timerAttachInterrupt(sub_timer, &mouseLoop, true);
//   timerAlarmWrite(sub_timer, 5000, true);
//   Serial.println("timer init done");
//   delay(1000);
//   timerAlarmEnable(sub_timer);
// }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);   // Log
  Serial1.begin(115200);  // To mouse
  Wire.begin();           // Magnetic Encoder
  Serial.println("Crocotile MCU v0.1.2");
  Serial.print("port tick rate: ");
  Serial.println(portTICK_RATE_MS);

  // systemX = HapticSystem(xConf);

  systemX.init();

  // xTaskCreatePinnedToCore(registerMouseTimer, "registerMouseTimer", 4096,
  // NULL,
  //                         3, &thp[0], 0);
  // xTaskCreatePinnedToCore(mouseLoop, "mouseLoop", 2048, NULL, 3, &thp[0], 0);
  mouse.init();
  handle = xTimerCreate("mouseLoop", (10 / portTICK_RATE_MS), pdTRUE, (void*)0,
                        mouseLoop);
  if (handle != NULL) {
    xTimerStart(handle, 0);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  systemX.loop();
  vTaskDelay(1);
}
