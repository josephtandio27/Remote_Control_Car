#include <Arduino.h>
#include <LittleFS.h>
#include "SDLogger.h"

QueueHandle_t telemetryQueue = NULL;
#include "SensorSim.h"
#include <Wire.h>
#include "RCModule.h"


// ============================================================
// CONFIGURATION - CORE 0 FOR CONTROL
// ============================================================
TaskHandle_t Core0TaskHandle = NULL;
void core0LoopTask(void *pvParameters) {
  Serial.printf("[Core 0] Real-time control loop started on Core %d\n", xPortGetCoreID());
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // Enforce strict 200Hz execution lock

  for (;;) {
    RC.update();
    runSensorSimulation();

    // Use DelayUntil for high-frequency precision
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Initialize LittleFS utilizing the 16MB partition layout
  if (!LittleFS.begin(true, "/littlefs", 10, "storage"))
  {
    Serial.println("LittleFS Mount Failed!");
  }
  else
  {
    Serial.printf("LittleFS Mounted. Total Space: %.2f MB\n", LittleFS.totalBytes() / 1024.0 / 1024.0);
  }

  // Initialize and verify the 8MB Octal PSRAM status
  if (psramInit())
  {
    Serial.printf("PSRAM Active. Total Size: %.2f MB\n", ESP.getPsramSize() / 1024.0 / 1024.0);
  }
  else
  {
    Serial.println("PSRAM Initialization FAILED!");
  }
  
  // Initialize RCModule (ESCs, IMU, RPM sensors)
  RC.begin();

  // Spawn Core 1 task — swap between SD logging and serial print as needed
  sdLoggerTaskBegin();
  // serialPrintTaskBegin();
  
  // Pin high-speed tasks to Core 0
  xTaskCreatePinnedToCore(
    core0LoopTask,    // Task function
    "Core0LoopTask",  // Name of the task
    8192,             // Stack size (adjust as needed)
    NULL,             // Task input parameter
    5,                // Priority (adjust as needed)
    &Core0TaskHandle, // Task handle
    0                 // Run on Core 0
  );
  
  Serial.println("\n[Setup Complete] Core 0 loop handling real-time simulations");
}

void loop()
{
  // Leave this empty! Core 1's main thread will do nothing,
  // leaving all of Core 1's processing power open for your SDLoggerTask.
  vTaskDelay(pdMS_TO_TICKS(1000));
}