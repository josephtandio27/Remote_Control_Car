#ifndef RCMODULE_H
#define RCMODULE_H

#include <Arduino.h>
#include <SPI.h>
#include <ICM_20948.h>

class RCModule {
public:
  RCModule();
  void begin();
  void update();

  // Telemetry getters
  uint16_t getThrottle() const;
  float getSteer() const;
  void getRPM(float out[4]) const;

  float getRoll() const;
  float getPitch() const;
  float getYaw() const;
  float getRollRate() const;
  float getPitchRate() const;
  float getYawRate() const;
  float getVelX() const;
  float getVelY() const;
  float getVelZ() const;
  float getAccX() const;
  float getAccY() const;
  float getAccZ() const;

  float getYawRateDesired() const;
  float getYawRateMax() const;
  float getYawRateMin() const;
  float getYawRateRef() const;
  float getYawRateError() const;
  float pidRPM();

private:
  friend void isr_chA();
  friend void isr_chB();

  void updateRPM();
  void calcRPM(float deltaTime);
};

extern RCModule RC;

#endif
