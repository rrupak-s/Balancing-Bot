//  ~ PID implementations using Tustin's discretization ~
//  2020/11/9

#ifndef _PID_H_
#define _PID_H_

class PID
{
  private:
    bool filter_flag = false;
    float tau; // Derivative LPF time constant
    float max_output = 0.0;
    float min_output = 0.0;

    float last_error = 0.0;
    float last_i_term = 0.0;
    float last_d_term = 0.0;

    inline int getSign(float x)
    {
      return (x > 0) - (x < 0);
    }

  public:
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    float output;

    PID() {}

    PID(float kp_, float ki_, float kd_, float tau_)
    {
      kp = kp_;
      ki = ki_;
      kd = kd_;
      tau = tau_;
      filter_flag = true;
    }

    PID(float kp_, float ki_, float kd_)
    {
      kp = kp_;
      ki = ki_;
      kd = kd_;
    }

    void setOutputLimits(float max, float min)
    {
      max_output = max;
      min_output = min;
    }

    void setGains(float kp_, float ki_, float kd_)
    {
      kp = kp_;
      ki = ki_;
      kd = kd_;
    }

    void setTau(float tau_)
    {
      tau = tau_;
    }

    void reset()
    {
      // Forget things
      last_error = 0.0;
      last_i_term = 0.0;
      last_d_term = 0.0;
      output = 0;
    }

    float update(float error, float dt)
    {

      float p_term, i_term, d_term;
      p_term = kp * error;
      i_term = 0.5 * ki * dt * (error + last_error) + last_i_term;

      if (!filter_flag)
      {
        d_term = 2 * kd * (error - last_error) / dt;
      }
      else
      {
        d_term = 2 * kd * (error - last_error) / (2 * tau + dt) +
                 last_d_term * (2 * tau - dt) / (2 * tau + dt);
      }

      output = p_term + i_term + d_term;

      // Output saturation
      if (abs(output) > max_output)
      {
        output = getSign(output) * max_output;
      }
      else if (abs(output) < min_output)
      {
        output = getSign(output) * min_output;
      }

      // Terms in the memory
      last_error = error;
      last_i_term = i_term;
      last_d_term = d_term;

      return output;
    }
}; // PID

#endif // pid.h
