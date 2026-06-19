#include "SensorSim.h"
#include "SDLogger.h"
#include "RCModule.h"

void runSensorSimulation() {
    static TelemetryPacket currentData;

    memset(&currentData, 0, sizeof(TelemetryPacket));

    currentData.timestamp = millis();

    currentData.vel[0] = RC.getVelX();
    currentData.vel[1] = RC.getVelY();
    currentData.yawRate = RC.getYawRate();

    currentData.steerAngle = RC.getSteer();
    currentData.pidYaw = 0.0f;

    currentData.yawDesired = RC.getYawRateDesired();
    currentData.yawRateMax = RC.getYawRateMax();
    currentData.yawRateMin = RC.getYawRateMin();
    currentData.yawRateRef = RC.getYawRateRef();
    currentData.yawRateError = RC.getYawRateError();
    currentData.yawAngle     = RC.getYaw() * DEG_TO_RAD;

    if (telemetryQueue != NULL) {
        // Using 0 ensures it drops immediately if Core 1 is falling behind,
        // preventing memory bloating.
        xQueueSend(telemetryQueue, &currentData, 0);
    }
}