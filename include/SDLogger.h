#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "Telemetry.h"

// Core 0 pushes packets here; Core 1 SD task drains it
extern QueueHandle_t telemetryQueue;
extern SemaphoreHandle_t spiMutex;

// Initializes SD card, creates queue, and spawns the Core 1 logger task
void sdLoggerTaskBegin();

// Spawns a Core 1 task that prints yaw angle (rad) and yaw rate (rad/s) to Serial
void serialPrintTaskBegin();

#endif
