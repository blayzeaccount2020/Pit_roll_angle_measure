int motor_pin = 10; // -> Operation Mode
int extend_pin = 3; // -> IN A (Hardware PWM)
int retract_pin = 4; // -> IN B

int extend_sense = 8;
int retract_sense = 9;

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
    digitalWrite(extend_pin, LOW);
    digitalWrite(retract_pin, LOW);
    delay(100);
  }

  switch (dir)
  {
    case MOTOR_OFF:
    {
      digitalWrite(extend_pin, LOW);
      digitalWrite(retract_pin, LOW);
      break;
    }

    case MOTOR_EXTEND:
    {
      digitalWrite(extend_pin, HIGH);
      digitalWrite(retract_pin, LOW);
      break;
    }

    case MOTOR_RETRACT:
    {
      digitalWrite(extend_pin, LOW);
      digitalWrite(retract_pin, HIGH);
      break;
    }

    case MOTOR_BRAKE:
    {
      digitalWrite(extend_pin, HIGH);
      digitalWrite(retract_pin, HIGH);
      break;
    }
  }
}

void setup()
{
  // Set Motor pin to low to configure IN/IN mode
  digitalWrite(motor_pin, LOW);

  // Set INA / INB so the motor is off
  digitalWrite(extend_pin, LOW);
  digitalWrite(retract_pin, LOW);

  // Set Motor, Extend, and Retract as outputs
  pinMode(motor_pin, OUTPUT);
  pinMode(extend_pin, OUTPUT);
  pinMode(retract_pin, OUTPUT);

  pinMode(extend_sense, INPUT);
  pinMode(retract_sense, INPUT);
}

void loop()
{
  // Extend motor
  //set_motor(MOTOR_EXTEND);
  //delay(900);

  // Retract motor
  //set_motor(MOTOR_RETRACT);
  //delay(900);

  int extend = digitalRead(extend_sense);
  int retract = digitalRead(retract_sense);

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
