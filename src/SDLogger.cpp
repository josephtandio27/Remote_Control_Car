#include "SDLogger.h"
#include "RCModule.h"
#include "IMU_ICM20948.h"
#include <SPI.h>
#include <SD.h>

SemaphoreHandle_t spiMutex = NULL;

#define SD_CS_PIN   39
#define SD_SCK_PIN  40
#define SD_MOSI_PIN 41
#define SD_MISO_PIN 42

#define QUEUE_SIZE      500   // enough buffer for ~5 s at 200 Hz
#define FLUSH_INTERVAL  1000  // ms between forced flushes

static SPIClass sdSPI(HSPI);
static File logFile;

static String getNextFilename() {
    for (int i = 1; i <= 999; i++) {
        char name[20];
        snprintf(name, sizeof(name), "/log_%03d.bin", i);
        if (!SD.exists(name)) return String(name);
    }
    return "/log_overflow.bin";
}

static bool sdBegin() {
    sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN, sdSPI, 4000000)) {
        Serial.println("[SDLogger] SD card mount failed — check wiring and card format (FAT32)");
        return false;
    }

    Serial.printf("[SDLogger] SD mounted. Card size: %llu MB\n", SD.cardSize() / (1024 * 1024));

    String filename = getNextFilename();
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.printf("[SDLogger] Failed to open %s\n", filename.c_str());
        return false;
    }

    Serial.printf("[SDLogger] Logging to %s\n", filename.c_str());
    return true;
}

static void sdLoggerTask(void* pvParameters) {
    Serial.printf("[SDLogger] Task running on Core %d\n", xPortGetCoreID());

    const int BUFFER_SIZE = 20; // Number of packets to buffer
    static TelemetryPacket localBuffer[BUFFER_SIZE];
    static int bufferIndex = 0;
    uint32_t lastFlush = millis();

    for (;;) {
        TelemetryPacket packet;
        // Keep a short block time to check if a flush is needed
        if (xQueueReceive(telemetryQueue, &packet, pdMS_TO_TICKS(10)) == pdTRUE) {
            localBuffer[bufferIndex++] = packet;
        }

        // Write to SD when buffer fills up or flush interval hits
        if (bufferIndex >= 20 || (millis() - lastFlush > FLUSH_INTERVAL && bufferIndex > 0)) {
            if (logFile) {
                // ------ PROTECT SD WRITE WITH MUTEX ------
                if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
                    logFile.write((const uint8_t*)localBuffer, sizeof(TelemetryPacket) * bufferIndex);
                    logFile.flush();
                    xSemaphoreGive(spiMutex); // Release immediately
                }
                // ------------------------------------------
                bufferIndex = 0;
            }
            lastFlush = millis();
        }
    }
}

void sdLoggerTaskBegin() {
    // Create the Mutex before starting tasks
    spiMutex = xSemaphoreCreateMutex();
    if (spiMutex == NULL) {
        Serial.println("[SDLogger] Failed to create SPI Mutex");
        return;
    }

    telemetryQueue = xQueueCreate(QUEUE_SIZE, sizeof(TelemetryPacket));
    if (telemetryQueue == NULL) {
        Serial.println("[SDLogger] Failed to create queue");
        return;
    }

    if (!sdBegin()) return;

    xTaskCreatePinnedToCore(
        sdLoggerTask,
        "SDLoggerTask",
        8192,
        NULL,
        3,
        NULL,
        1   // Core 1
    );
}

static void serialPrintTask(void* pvParameters) {
    Serial.printf("[SerialPrint] Task running on Core %d\n", xPortGetCoreID());
    for (;;) {
        float yawRad     = RC.getYaw()     * (M_PI / 180.0f);
        float yawRateRad = RC.getYawRate() * (M_PI / 180.0f);
        float velX       = RC.getVelX();
        float velY       = RC.getVelY();
        float accX       = RC.getAccX();
        Serial.printf("[IMU] Yaw: %.3f rad | YawRate: %.3f rad/s | VelX: %.3f | VelY: %.3f | AccX: %.3f\n", yawRad, yawRateRad, velX, velY, accX);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void serialPrintTaskBegin() {
    xTaskCreatePinnedToCore(
        serialPrintTask,
        "SerialPrintTask",
        4096,
        NULL,
        1,
        NULL,
        1   // Core 1
    );
}
