int motor_pin = 10; // -> Operation Mode
int right_pin_roll = 3; // -> IN A (Hardware PWM)
int left_pin_roll = 4; // -> IN B
// int extend_pin = 3; // -> IN A (Hardware PWM)
// int retract_pin = 4; // -> IN B


int move_left_manual = 8;
int move_right_manual = 9;

// It's definitely this one, Blayze

typedef enum
{
  MOTOR_OFF = (0b00),
  MOTOR_EXTEND = (0b01),
  MOTOR_RETRACT = (0b10),
  MOTOR_BRAKE = (0b11),
} direction_t;

void set_motor(direction_t dir)
{
  static direction_t last_direction = MOTOR_OFF;

  // If flipping directions,
  if (((dir == MOTOR_EXTEND) && (last_dir == MOTOR_RETRACT)) || ((dir == MOTOR_RETRACT) && (last_dir == MOTOR_EXTEND))
  {
    // Allow the motor to stop for a short period of time
    digitalWrite(right_pin_roll, LOW);
    digitalWrite(left_pin_roll, LOW);
    delay(100);
  }

  switch (dir)
  {
    case MOTOR_OFF:
    {
      digitalWrite(right_pin_roll, LOW);
      digitalWrite(left_pin_roll, LOW);
      break;
    }

    case MOTOR_EXTEND:
    {
      digitalWrite(right_pin_roll, HIGH);
      digitalWrite(left_pin_roll, LOW);
      break;
    }

    case MOTOR_RETRACT:
    {
      digitalWrite(right_pin_roll, LOW);
      digitalWrite(left_pin_roll, HIGH);
      break;
    }

    case MOTOR_BRAKE:
    {
      digitalWrite(right_pin_roll, HIGH);
      digitalWrite(left_pin_roll, HIGH);
      break;
    }
  }
}

void setup()
{
  // Set Motor pin to low to configure IN/IN mode
  digitalWrite(motor_pin, LOW);

  // Set INA / INB so the motor is off
  digitalWrite(right_pin_roll, LOW);
  digitalWrite(left_pin_roll, LOW);

  // Set Motor, Extend, and Retract as outputs
  pinMode(motor_pin, OUTPUT);
  pinMode(right_pin_roll, OUTPUT);
  pinMode(left_pin_roll, OUTPUT);

  pinMode(move_left_manual, INPUT);
  pinMode(move_right_manual, INPUT);
}

void loop()
{
  // Extend motor
  //set_motor(MOTOR_EXTEND);
  //delay(900);

  // Retract motor
  //set_motor(MOTOR_RETRACT);
  //delay(900);

  int extend = digitalRead(move_left_manual);
  int retract = digitalRead(move_right_manual);

  if (extend == LOW)
  {
    set_motor(MOTOR_EXTEND);
  }
  else if (retract == LOW)
  {
    set_motor(MOTOR_RETRACT);
  }
  else
  {
    set_motor(MOTOR_OFF);
  }
}
