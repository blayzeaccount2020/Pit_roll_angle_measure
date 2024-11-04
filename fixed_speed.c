int motor_pin = 10; // -> Operation Mode
int extend_pin = 3; // -> IN A (Hardware PWM)
int retract_pin = 4; // -> IN B

typedef enum
{
	MOTOR_OFF = (0b00),
	MOTOR_EXTEND = (0b01),
	MOTOR_RETRACT = (0b10),
	MOTOR_BRAKE = (0b11),
} direction_t;

typedef struct __attribute__((packed))
{
	union
	{
		struct
		{
			int extend: 1;
			int retract: 1;
			int: 6;
		};
		struct
		{
			direction_t value: 2;
			direction_t: 6;
		};
	};
} motor_t;

motor_t motor = {0};

void set_motor()
{
	switch (motor.value)
	{
		case MOTOR_OFF:
		{
			// Motor has been stopped => do nothing
			break;
		}

		case MOTOR_EXTEND:
		{
			// Switch motor direction
			motor.value = MOTOR_RETRACT;

			// motor.value = 0b10
			// motor.extend = 0
			// motor.retract = 1

			break;
		}

		case MOTOR_RETRACT:
		{
			// Switch motor direction
			motor.value = MOTOR_EXTEND;

			// motor.value = 0b01
			// motor.extend = 1
			// motor.retract = 0
			break;
		}

		case MOTOR_BRAKE:
		{
			// Motor has been stopped => do nothing
			break;
		}
	}

	digitalWrite(extend_pin, motor.extend);
	digitalWrite(retract_pin, motor.retract);
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

	// Start motor movement
	motor.value = MOTOR_EXTEND;
}

void loop()
{
	// Extend motor
	set_motor();
	delayMilliseconds(900);

	// Gracefully stop motor
	motor.value = MOTOR_BRAKE;
	set_motor();
	delayMilliseconds(100);

	// Retract motor
	set_motor();
	delayMilliseconds(900);

	// Gracefully stop motor
	motor.value = MOTOR_BRAKE;
	set_motor();
	delayMilliseconds(100);
}