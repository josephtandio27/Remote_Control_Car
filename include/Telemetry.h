#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

// Packed structure to hold all telemetry data efficiently
struct __attribute__((__packed__)) TelemetryPacket {
    uint32_t timestamp;
    float yawRate;     // Z Axis (deg/s)
    float vel[2];      // Velocity X, Y (m/s)
    float steerAngle;
    float pidYaw;
    float yawDesired;
    float yawRateMax;
    float yawRateMin;
    float yawRateRef;
    float yawRateError;
    float yawAngle;     // rad
};

#endif