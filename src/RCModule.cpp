#include "RCModule.h"
#include "RC_PID.h"
#include "driver/ledc.h"
#include "IMU_ICM20948.h"
#include "SDLogger.h"

// ============================================================
// CONFIGURATION - RC & ESC
// ============================================================
#define SWAP_RC_CHANNELS 0
#define RC_CH_A_PIN 15
#define RC_CH_B_PIN 16

#define ESC_FL_PIN 5
#define ESC_FR_PIN 6
#define ESC_RL_PIN 7
#define ESC_RR_PIN 8
#define SERVO_STEER_PIN 14

#define ESC_FL_CH 0
#define ESC_FR_CH 1
#define ESC_RL_CH 2
#define ESC_RR_CH 3
#define STEER_CH 4

#define P_RC 3.5f
#define I_RC 1.0f
#define D_RC 1.0f

const int AS5600_PIN[4] = {1, 2, 3, 4};
const char *WHEEL_LABEL[4] = {"FL", "FR", "RL", "RR"};

// ============================================================
// PWM CONSTANTS
// ============================================================
#define PWM_MIN 1000
#define PWM_CENTER 1500
#define PWM_MAX 2000
#define DEADBAND 30
#define ESC_FREQ 50
#define ESC_RES 13

// ============================================================
// RC INPUT STATE
// ============================================================
portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t riseA = 0, riseB = 0;
volatile uint16_t pwA = PWM_CENTER;
volatile uint16_t pwB = PWM_CENTER;

// ============================================================
// RPM STATE
// ============================================================
const int MAX_ADC_VALUE = 4095;
int lastRawValue[4] = {0, 0, 0, 0};
float cumulativeDeltaRaw[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long prevPrintMillis = 0;
float pidOutput = 0.0f;
PIDController pidController(P_RC, I_RC, D_RC);

// Internal instance pointer for ISRs
static RCModule *s_instance = nullptr;

// ============================================================
// ISR
// ============================================================
void IRAM_ATTR isr_chA()
{
  if (digitalRead(RC_CH_A_PIN))
  {
    riseA = micros();
  }
  else
  {
    uint32_t pw = micros() - riseA;
    if (pw >= 800 && pw <= 2200)
    {
      portENTER_CRITICAL_ISR(&rcMux);
      pwA = (uint16_t)pw;
      portEXIT_CRITICAL_ISR(&rcMux);
    }
  }
}

void IRAM_ATTR isr_chB()
{
  if (digitalRead(RC_CH_B_PIN))
  {
    riseB = micros();
  }
  else
  {
    uint32_t pw = micros() - riseB;
    if (pw >= 800 && pw <= 2200)
    {
      portENTER_CRITICAL_ISR(&rcMux);
      pwB = (uint16_t)pw;
      portEXIT_CRITICAL_ISR(&rcMux);
    }
  }
}

// ============================================================
// HELPERS - RC & ESC
// ============================================================
uint16_t readChA()
{
  portENTER_CRITICAL(&rcMux);
  uint16_t v = pwA;
  portEXIT_CRITICAL(&rcMux);
  return v;
}

uint16_t readChB()
{
  portENTER_CRITICAL(&rcMux);
  uint16_t v = pwB;
  portEXIT_CRITICAL(&rcMux);
  return v;
}

uint16_t applyDeadband(uint16_t pwmValue)
{
  int delta = (int)pwmValue - PWM_CENTER;
  if (abs(delta) < DEADBAND)
    return PWM_CENTER;
  return (uint16_t)constrain(pwmValue, PWM_MIN, PWM_MAX);
}

uint32_t usToDuty(uint16_t us)
{
  return (uint32_t)((us * 8191) / 20000);
}

void writeOutputChannel(int channel, uint16_t pwm)
{
  ledcWrite(channel, usToDuty(pwm));
}

// Sets all four ESC duties then latches them together so all motors update
// on the same LEDC timer overflow instead of sequentially.
void writeAllESC(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr)
{
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ESC_FL_CH, usToDuty(fl));
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ESC_FR_CH, usToDuty(fr));
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ESC_RL_CH, usToDuty(rl));
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ESC_RR_CH, usToDuty(rr));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ESC_FL_CH);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ESC_FR_CH);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ESC_RL_CH);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ESC_RR_CH);
}

float pwmToAngle(uint16_t pwm)
{
  // Maps 1000-2000 µs PWM to -17° to +17° servo angle
  return (pwm - 1500.0f) * (17.0f / 500.0f);
}

// ============================================================
// HELPERS - RPM
// ============================================================
void RCModule::updateRPM()
{
  for (int i = 0; i < 4; i++)
  {
    int currentRaw = analogRead(AS5600_PIN[i]);
    int deltaRaw = currentRaw - lastRawValue[i];
    if (deltaRaw > MAX_ADC_VALUE / 2)
      deltaRaw -= (MAX_ADC_VALUE + 1);
    else if (deltaRaw < -(MAX_ADC_VALUE / 2))
      deltaRaw += (MAX_ADC_VALUE + 1);
    cumulativeDeltaRaw[i] += deltaRaw;
    lastRawValue[i] = currentRaw;
  }
}

void RCModule::calcRPM(float deltaTime)
{
  for (int i = 0; i < 4; i++)
  {
    float rev = cumulativeDeltaRaw[i] / (float)(MAX_ADC_VALUE + 1);
    rpm[i] = (rev / deltaTime) * 60.0f;
    cumulativeDeltaRaw[i] = 0;
  }
}

// ============================================================
// IMU ICM20948
// ============================================================
ImuTracker icm20948;

// ============================================================
// RCModule methods (thin wrapper around the static logic)
// ============================================================
RCModule RC;

RCModule::RCModule() {}

void RCModule::begin()
{
  s_instance = this;

  delay(500);

  // ===== IMU =====
  icm20948.begin();

  // ===== RC Pins =====
  pinMode(RC_CH_A_PIN, INPUT_PULLDOWN);
  pinMode(RC_CH_B_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(RC_CH_A_PIN), isr_chA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH_B_PIN), isr_chB, CHANGE);

  // ===== ESC + Servo (LEDC) =====
  ledcSetup(ESC_FL_CH, ESC_FREQ, ESC_RES);
  ledcSetup(ESC_FR_CH, ESC_FREQ, ESC_RES);
  ledcSetup(ESC_RL_CH, ESC_FREQ, ESC_RES);
  ledcSetup(ESC_RR_CH, ESC_FREQ, ESC_RES);
  ledcAttachPin(ESC_FL_PIN, ESC_FL_CH);
  ledcAttachPin(ESC_FR_PIN, ESC_FR_CH);
  ledcAttachPin(ESC_RL_PIN, ESC_RL_CH);
  ledcAttachPin(ESC_RR_PIN, ESC_RR_CH);

  // Steering
  ledcSetup(STEER_CH, ESC_FREQ, ESC_RES);
  ledcAttachPin(SERVO_STEER_PIN, STEER_CH);

  writeOutputChannel(ESC_FL_CH, PWM_CENTER);
  writeOutputChannel(ESC_FR_CH, PWM_CENTER);
  writeOutputChannel(ESC_RL_CH, PWM_CENTER);
  writeOutputChannel(ESC_RR_CH, PWM_CENTER);
  writeOutputChannel(STEER_CH, PWM_CENTER);

  delay(2000); // ESC arming

  // ===== RPM Sensors (AS5600 via ADC) =====
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < 4; i++)
  {
    pinMode(AS5600_PIN[i], INPUT);
    lastRawValue[i] = analogRead(AS5600_PIN[i]);
  }
}

void RCModule::update()
{
  unsigned long now = millis();

  // // ===== Serial command =====
  // if (Serial.available())
  // {
  //   char c = Serial.read();
  //   if (c == 'r' || c == 'R')
  //     icm20948.reset();
  // }

  // ===== RC -> ESC + Servo =====
  uint16_t snapA = readChA();
  uint16_t snapB = readChB();

  uint16_t rawThrottle, rawSteer;
  if (SWAP_RC_CHANNELS == 1)
  {
    rawThrottle = snapB;
    rawSteer = snapA;
  }
  else
  {
    rawThrottle = snapA;
    rawSteer = snapB;
  }

  uint16_t finalThrottle = applyDeadband(rawThrottle);
  uint16_t finalSteer = applyDeadband(rawSteer);

  // ===== IMU update (~100Hz) =====
  if (spiMutex != NULL) {
    // Wait up to 1 millisecond for the SD card to yield the SPI bus
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      icm20948.update(); 
      xSemaphoreGive(spiMutex); // Release immediately
    }
  } else {
    icm20948.update(); // Fallback if mutex hasn't initialized yet
  }
  
  pidOutput = pidRPM();

  // Only apply torque vectoring when actively driving; PID correction at neutral
  // would oppose itself and can cause runaway if velocity estimation has noise.
  int correction = (finalThrottle == PWM_CENTER) ? 0 : (int)pidOutput;
  uint16_t pwmL = (uint16_t)constrain((int)finalThrottle - correction, PWM_MIN, PWM_MAX);
  uint16_t pwmR = (uint16_t)constrain((int)finalThrottle + correction, PWM_MIN, PWM_MAX);
  writeAllESC(pwmL, pwmR, pwmL, pwmR);
  writeOutputChannel(STEER_CH, finalSteer);
}

uint16_t RCModule::getThrottle() const
{
  return readChA();
}

float RCModule::getSteer() const
{
  return -pwmToAngle(readChB());
}

void RCModule::getRPM(float out[4]) const
{
  for (int i = 0; i < 4; i++)
    out[i] = rpm[i];
}

float RCModule::getRoll() const {
  return icm20948.getRoll();
}

float RCModule::getPitch() const {
  return icm20948.getPitch();
}

float RCModule::getYaw() const {
  return icm20948.getYaw();
}

float RCModule::getRollRate() const {
  return icm20948.getRollRate();
}

float RCModule::getPitchRate() const {
  return icm20948.getPitchRate();
}

float RCModule::getYawRate() const {
  return icm20948.getYawRate();
}

float RCModule::getVelX() const {
  return icm20948.getVelocityX();
}

float RCModule::getVelY() const {
  return icm20948.getVelocityY();
}

float RCModule::getVelZ() const {
  return icm20948.getVelocityZ();
}

float RCModule::getAccX() const {
  return icm20948.getAccX(); // Fixed from getAccelerationX
}

float RCModule::getAccY() const {
  return icm20948.getAccY();
}

float RCModule::getAccZ() const {
  return icm20948.getAccZ();
}

// ==========Torque vectoring===================
float RCModule::getYawRateDesired() const
{
  float steerRad = pwmToAngle(applyDeadband(readChB())) * DEG_TO_RAD;
  return icm20948.getVelocityY() * steerRad;
}

float RCModule::getYawRateMax() const
{
  float velX = fabs(icm20948.getVelocityY());
  if (velX < 0.01f)
    return 0.0f;
  return (0.8f * 0.4f * 9.81f) / velX;
}

float RCModule::getYawRateMin() const
{
  return -getYawRateMax();
}

float RCModule::getYawRateRef() const
{
  float desired = getYawRateDesired();
  float maxVal = getYawRateMax();
  float minVal = getYawRateMin();
  if (desired > maxVal)
    return maxVal;
  if (desired < minVal)
    return minVal;
  return desired;
}

float RCModule::getYawRateError() const
{
  return getYawRateRef() - (icm20948.getYawRate() * DEG_TO_RAD);
}

// TODO
float RCModule::pidRPM()
{
  float yawRateErr = getYawRateError();
  // float yawRateErr = 0;
  return pidController.compute(yawRateErr);
}
