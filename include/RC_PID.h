#include <esp32-hal.h>

class PIDController {
  public:
    double kp, ki, kd;
    double lastError = 0;
    double integral = 0;
    unsigned long lastTime = 0;

    PIDController(double p, double i, double d) : kp(p), ki(i), kd(d) {}

    double compute(double error) {
      unsigned long now = micros();
      double dt = (double)(now - lastTime) / 1000000.0; // Time change in seconds
      
      if (dt <= 0) return 0; // Prevent division by zero
      
      // Proportional
      double Pout = kp * error;

      // Integral
      integral += error * dt;
      double Iout = ki * integral;

      // Derivative
      double derivative = (error - lastError) / dt;
      double Dout = kd * derivative;

      // Calculate total output
      double output = Pout + Iout + Dout;

      // Save states for next iteration
      lastError = error;
      lastTime = now;

      return output;
    }
};