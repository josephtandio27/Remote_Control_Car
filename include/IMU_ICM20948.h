#ifndef IMU_TRACKER_H
#define IMU_TRACKER_H

#include "ICM_20948.h"
#include <SPI.h>
#include <math.h>

// ============================================================
// CONFIGURATION - IMU (ICM-20948 via SPI)
// ============================================================
// Default Hardware SPI Pins (Can be overridden before inclusion if needed)
// Coordinate Frame w.r.t the chip: Z up (out of the chip), Y toward the longside of the chip, X toward the shorter side of the chip
#ifndef PIN_SPI_SCK
#define PIN_SPI_SCK   13    // SCL
#endif
#ifndef PIN_SPI_MISO
#define PIN_SPI_MISO  12    // ADO
#endif
#ifndef PIN_SPI_MOSI
#define PIN_SPI_MOSI  11    // SDA
#endif
#ifndef PIN_SPI_CS
#define PIN_SPI_CS    10    // NCS
#endif

class ImuTracker {
private:
    ICM_20948_SPI _myICM;
    
    // Filter Settings
    const float _alpha = 0.9;
    const float _zuptThreshold = 0.1;

    // Hardware Constants
    // accX/Y/Z() return milli-g; divide by 1000 to get g
    // gyrX/Y/Z() return DPS directly; no extra scale factor needed
    const float _ACCEL_SCALE_2G = 1000.0f;
    const float _GYRO_SCALE_250DPS = 1.0f;
    const float _G_CONSTANT = 9.80665;
    const int _calibrationSamples = 5000;

    // Calibration Offsets & State
    float _accelX_offset = 0.0f, _accelY_offset = 0.0f, _accelZ_offset = 0.0f;
    float _gyroX_bias = 0.0f, _gyroY_bias = 0.0f, _gyroZ_bias = 0.0f;
    float _accelZ_scale = 16384.0; 
    int _sampleCount = 0;
    bool _isCalibrated = false;

    // State Variables
    unsigned long _lastTime = 0;
    float _pitch = 0.0, _roll = 0.0, _yaw = 0.0;
    float _rollRate = 0.0, _pitchRate = 0.0, _yawRate = 0.0;
    float _velocityX = 0.0, _velocityY = 0.0, _velocityZ = 0.0;
    float _filteredAccX = 0.0, _filteredAccY = 0.0, _filteredAccZ = 0.0;

    // Private helpers for processing tracking chunks
    void processCalibration() {
        if (_sampleCount < _calibrationSamples) {
            _accelX_offset += _myICM.accX();
            _accelY_offset += _myICM.accY();
            _accelZ_offset += _myICM.accZ();
            _gyroX_bias += _myICM.gyrX();
            _gyroY_bias += _myICM.gyrY();
            _gyroZ_bias += _myICM.gyrZ();
            _sampleCount++;
            
            if (_sampleCount % 50 == 0) Serial.print(".");
        } else {
            _accelX_offset /= _calibrationSamples;
            _accelY_offset /= _calibrationSamples;
            _accelZ_offset /= _calibrationSamples;
            _gyroX_bias /= _calibrationSamples;
            _gyroY_bias /= _calibrationSamples;
            _gyroZ_bias /= _calibrationSamples;

            _accelZ_scale = (float)_accelZ_offset;

            float initX = (float)_accelX_offset / _ACCEL_SCALE_2G; 
            float initY = (float)_accelY_offset / _ACCEL_SCALE_2G; 
            float initZ = (float)_accelZ_offset / _accelZ_scale;                    
            
            _pitch = atan2(initX, sqrt(initY * initY + initZ * initZ)) * 180.0 / M_PI;
            _roll = atan2(initY, initZ) * 180.0 / M_PI;
            _yaw = 0.0; 

            _isCalibrated = true;
            _lastTime = micros();
            Serial.println("\n>>> CALIBRATION DONE. TRACKING ACTIVE. <<<");
        }
    }

    void processTracking() {
        unsigned long currentTime = micros();
        float dt = (currentTime - _lastTime) / 1000000.0;
        _lastTime = currentTime;

        // Convert Gyro to degrees per second and cache to rate variables
        float gyroX = (float)(_myICM.gyrX() - _gyroX_bias) / _GYRO_SCALE_250DPS;
        float gyroY = (float)(_myICM.gyrY() - _gyroY_bias) / _GYRO_SCALE_250DPS;
        float gyroZ = (float)(_myICM.gyrZ() - _gyroZ_bias) / _GYRO_SCALE_250DPS;
        _rollRate  = gyroX;
        _pitchRate = gyroY;
        _yawRate   = gyroZ;

        float rawAccX_G = (float)(_myICM.accX() - _accelX_offset) / _ACCEL_SCALE_2G;
        float rawAccY_G = (float)(_myICM.accY() - _accelY_offset) / _ACCEL_SCALE_2G;
        float rawAccZ_G = (float)_myICM.accZ() / _accelZ_scale;

        float pitchAcc = atan2(rawAccX_G, sqrt(rawAccY_G * rawAccY_G + rawAccZ_G * rawAccZ_G)) * 180.0 / M_PI;
        float rollAcc = atan2(rawAccY_G, rawAccZ_G) * 180.0 / M_PI;

        _pitch = (0.98 * (_pitch + gyroY * dt)) + (0.02 * pitchAcc);
        _roll  = (0.98 * (_roll + gyroX * dt)) + (0.02 * rollAcc);
        
        _yaw  += gyroZ * dt; 
        if (_yaw > 180.0) _yaw -= 360.0;
        else if (_yaw < -180.0) _yaw += 360.0;

        float pitchRad = _pitch * M_PI / 180.0;
        float rollRad  = _roll * M_PI / 180.0;

        float accX_pure = (rawAccX_G - sin(pitchRad)) * _G_CONSTANT;
        float accY_pure = (rawAccY_G - sin(rollRad) * cos(pitchRad)) * _G_CONSTANT;
        float accZ_pure = (rawAccZ_G - cos(rollRad) * cos(pitchRad)) * _G_CONSTANT;

        _filteredAccX = (_alpha * accX_pure) + ((1.0 - _alpha) * _filteredAccX);
        _filteredAccY = (_alpha * accY_pure) + ((1.0 - _alpha) * _filteredAccY);
        _filteredAccZ = (_alpha * accZ_pure) + ((1.0 - _alpha) * _filteredAccZ);

        applyZupt(_filteredAccX, _velocityX, dt);
        applyZupt(_filteredAccY, _velocityY, dt);
        applyZupt(_filteredAccZ, _velocityZ, dt);
    }

    void applyZupt(float &filteredAcc, float &velocity, float dt) {
        if (abs(filteredAcc) < _zuptThreshold) {
            filteredAcc = 0.0;
            velocity *= 0.75; 
            if (abs(velocity) < 0.01) velocity = 0.0;
        } else {
            velocity += filteredAcc * dt;
        }
    }

public:
    void begin() {
        SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS); 

        bool initialized = false;
        while (!initialized) {
            _myICM.begin(PIN_SPI_CS, SPI, 4000000); 
            if (_myICM.status == ICM_20948_Stat_Ok) {
                initialized = true;
            } else {
                Serial.print("Sensor error: "); Serial.println(_myICM.status);
                delay(2000);
            }
        }

        _myICM.swReset();
        delay(100);
        _myICM.sleep(false);
        _myICM.lowPower(false);
        delay(100);

        ICM_20948_fss_t myFSS;
        myFSS.a = gpm2;    
        myFSS.g = dps250;  
        _myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

        ICM_20948_dlpcfg_t myDLPcfg;
        myDLPcfg.a = acc_d246bw_n265bw; 
        myDLPcfg.g = gyr_d196bw6_n229bw8; 
        _myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
        _myICM.enableDLPF((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), true);

        Serial.println(">>> CALIBRATING: PLACE CHIP FLAT AND COMPLETELY STILL! <<<");
        // Serial.println("Timestamp_ms,Roll,Pitch,Yaw,GyroX_dps,GyroY_dps,GyroZ_dps,VelX_ms,VelY_ms,VelZ_ms,AccX_ms2,AccY_ms2,AccZ_ms2");
        
        _lastTime = micros();
    }

    void update() {
        if (_myICM.dataReady()) {
            _myICM.getAGMT(); 
            
            if (!_isCalibrated) {
                processCalibration();
            } else {
                processTracking();
            }
        }
    }

    void printData() {
        float gyroX = (float)(_myICM.gyrX() - _gyroX_bias) / _GYRO_SCALE_250DPS;
        float gyroY = (float)(_myICM.gyrY() - _gyroY_bias) / _GYRO_SCALE_250DPS;
        float gyroZ = (float)(_myICM.gyrZ() - _gyroZ_bias) / _GYRO_SCALE_250DPS;

        Serial.print(millis());          Serial.print(",");
        Serial.print(_roll, 2);          Serial.print(",");
        Serial.print(_pitch, 2);         Serial.print(",");
        Serial.print(_yaw, 2);           Serial.print(",");
        Serial.print(gyroX, 2);          Serial.print(",");
        Serial.print(gyroY, 2);          Serial.print(",");
        Serial.print(gyroZ, 2);          Serial.print(",");
        Serial.print(_velocityX, 4);     Serial.print(",");
        Serial.print(_velocityY, 4);     Serial.print(",");
        Serial.print(_velocityZ, 4);     Serial.print(",");
        Serial.print(_filteredAccX, 4);  Serial.print(",");
        Serial.print(_filteredAccY, 4);  Serial.print(",");
        Serial.println(_filteredAccZ, 4); 
    }

    // Wipes state variables and initiates a hot recalibration
    void reset() {
        Serial.println("\n>>> IMU RESET TRIGGERED: RE-CALIBRATING NOW... <<<");
        Serial.println(">>> KEEP CHIP COMPLETELY STILL! <<<");

        // Clear calibration accumulation pools
        _accelX_offset = 0;
        _accelY_offset = 0;
        _accelZ_offset = 0;
        _gyroX_bias = 0;
        _gyroY_bias = 0;
        _gyroZ_bias = 0;
        _accelZ_scale = 16384.0;
        _sampleCount = 0;

        // Reset tracking integration registers
        _pitch = 0.0;
        _roll = 0.0;
        _yaw = 0.0;
        _velocityX = 0.0;
        _velocityY = 0.0;
        _velocityZ = 0.0;
        _filteredAccX = 0.0;
        _filteredAccY = 0.0;
        _filteredAccZ = 0.0;
        _rollRate = 0.0;
        _pitchRate = 0.0;
        _yawRate = 0.0;

        // Force internal state machine back to step 1
        _isCalibrated = false;
        _lastTime = micros();
    }

    // --- GETTERS ---
    bool isCalibrated() const     { return _isCalibrated; }
    
    // Orientation Angles (deg)
    float getRoll() const         { return _roll; }
    float getPitch() const        { return _pitch; }
    float getYaw() const          { return _yaw; }
    
    // Angular Rates (deg/s)
    float getRollRate() const     { return _rollRate; }
    float getPitchRate() const    { return _pitchRate; }
    float getYawRate() const      { return _yawRate; }

    // Velocities (m/s)
    float getVelocityX() const    { return _velocityX; }
    float getVelocityY() const    { return _velocityY; }
    float getVelocityZ() const    { return _velocityZ; }

    // Gravity compensated, filtered Acceleration (m/s^2)
    float getAccX() const         { return _filteredAccX; }
    float getAccY() const         { return _filteredAccY; }
    float getAccZ() const         { return _filteredAccZ; }
};

#endif // IMU_TRACKER_H