#ifndef _MOTOR_H_
#define _MOTOR_H_

class Motor
{
  public:
    Motor(uint8_t d1, uint8_t d2, uint8_t pwm);
    void drive(float pwm);

  private:
    uint8_t pin_dir_1;
    uint8_t pin_dir_2;
    uint8_t pin_pwm;

};

Motor::Motor(uint8_t d1, uint8_t d2, uint8_t pwm)
{
  pin_dir_1 = d1;
  pin_dir_2 = d2;
  pin_pwm = pwm;

  pinMode(pin_dir_1, OUTPUT);
  pinMode(pin_dir_2, OUTPUT);
  pinMode(pin_pwm, OUTPUT);
}

void Motor::drive(float pwm)
{
  if (pwm < 0)
  {
    digitalWrite(pin_dir_1, HIGH);
    digitalWrite(pin_dir_2, LOW);
    pwm = pwm * -1;
  }
  else
  {
    digitalWrite(pin_dir_2, HIGH);
    digitalWrite(pin_dir_1, LOW);
  }
  analogWrite(pin_pwm, pwm);
}

#endif
