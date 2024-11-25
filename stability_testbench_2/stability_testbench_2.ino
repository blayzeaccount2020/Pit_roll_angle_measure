#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>

Adafruit_MPU6050 mpu;
float roll_angle=0;
float pitch_angle=0;
float motor_roll_angle=90;
float motor_pitch_angle=90;
float accel_mag=0;
int pitch_angle_plus=0;
int pitch_angle_minus=0;
int roll_angle_plus=0;
int roll_angle_minus=0;


float roll_angle_highpass=0;
float roll_angle_highpass1=0;
float roll_angle_highpass_filter_out=0;
float pitch_angle_highpass=0;
float pitch_angle_highpass1=0;
float pitch_angle_highpass_filter_out=0;
float roll_angle_lowpass=0;
float roll_angle_lowpass_filter_out=0;
float pitch_angle_lowpass=0;
float pitch_angle_lowpass_filter_out=0;
float bias_x =0.581396;
float bias_y =-.113946;
float bias_z =-0.924158;
float cor_matrix_row1[3] = {0.993176,-0.000012,-0.002005};
float cor_matrix_row2[3] = {-0.000012, 0.996603,0.001797};
float cor_matrix_row3[3] = {-0.002005,0.001797,0.972668};
float gyro_offset_x=0;
float gyro_offset_y=0;
float gyro_offset_z=0;
float corrected_x_acceleration;
float corrected_y_acceleration;
float corrected_z_acceleration;
float tau = 1/(2*PI*1);
volatile int count=0;
int past_count=0;
int sample_number=50;
int loop_delay=50;
float mag_yz_plane=0;
float sample_rate=0;
float time_delay=0;

float roll_angle_original=0;
float pitch_angle_original=0;

float past_time=0;
float current_time=0;

int left_pin_roll=4; //these may need to be switched around in the if statements below. 
int right_pin_roll=3;

int motor_pin = 10;
int pwmpin2 = 11;

int zero_point_set=7;
int mem_address = 0; //address where info is supposed to be stored
int i;

float past_time_count=0;
float current_time_count=0;

int linear_actuator_input = 2;

//int read_left_pin=8;
//int read_right_pin=9;
int engage_centering = 5;
int manual_override_pin = 6;
int move_left_manual = 8;
int move_right_manual = 9; //the code is configured that extending the actuator makes it go right, and retracting makes it go left. This may need to be reversed.

int analogPin = A0;

float relative_angle1=0;
float relative_angle2=0;

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
  if (((dir == MOTOR_EXTEND) && (last_direction == MOTOR_RETRACT)) || ((dir == MOTOR_RETRACT) && (last_direction == MOTOR_EXTEND)))
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

void setup() {
// Set Motor pin to low to configure IN/IN mode
digitalWrite(motor_pin, LOW);

// Set INA / INB so the motor is off
digitalWrite(right_pin_roll, LOW);
digitalWrite(left_pin_roll, LOW);
  
pinMode(left_pin_roll, OUTPUT);
pinMode(right_pin_roll, OUTPUT);
pinMode(motor_pin,OUTPUT);
pinMode(manual_override_pin, INPUT);
pinMode(move_left_manual, INPUT);
pinMode(move_right_manual, INPUT);
//pinMode(read_left_pin, INPUT);
//pinMode(read_right_pin, INPUT);
 Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

 
    for(i = 0; (i < EEPROM.length())&&(EEPROM.read(i)==255); i+=1){
    }
      mem_address = i;
    if(mem_address >= EEPROM.length()-1){
      mem_address = 0;
    }

    count = (EEPROM.read(mem_address)*100)+(EEPROM.read(mem_address+1));

    attachInterrupt(digitalPinToInterrupt(linear_actuator_input),countSteps,RISING);

}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //correct the acceleration values using correction factors obtained using magneto acceleromter calibration software.
  corrected_x_acceleration = (cor_matrix_row1[0]*(a.acceleration.x-bias_x))+(cor_matrix_row1[1]*(a.acceleration.y-bias_y))+(cor_matrix_row1[2]*(a.acceleration.z-bias_z));
  corrected_y_acceleration = (cor_matrix_row2[0]*(a.acceleration.x-bias_x))+(cor_matrix_row2[1]*(a.acceleration.y-bias_y))+(cor_matrix_row2[2]*(a.acceleration.z-bias_z));
  corrected_z_acceleration = (cor_matrix_row3[0]*(a.acceleration.x-bias_x))+(cor_matrix_row3[1]*(a.acceleration.y-bias_y))+(cor_matrix_row3[2]*(a.acceleration.z-bias_z));

  //obtain the angle of chip relative to gravity using gravity vector and trigonometry.
  roll_angle_lowpass = atan(corrected_y_acceleration/corrected_z_acceleration)*(180/PI);
  mag_yz_plane=corrected_z_acceleration/(cos(atan(corrected_y_acceleration/corrected_z_acceleration)));
  pitch_angle_lowpass = -atan(corrected_x_acceleration/mag_yz_plane)*(180/PI);
  
  //Obtain angles by integrating rates provided by internal gyroscope.
  current_time=millis();
  time_delay=current_time-past_time;
  roll_angle_highpass = roll_angle_highpass+(g.gyro.x)*(time_delay)*.001*(180/PI);
  pitch_angle_highpass = pitch_angle_highpass+(g.gyro.y)*(time_delay)*.001*(180/PI);
  past_time=current_time;
  sample_rate=1.0/(time_delay*.001);
  //feed angle data direct from gyroscope into highpass filters meant to filter out the offset integration error.
  roll_angle_highpass_filter_out = (((tau*sample_rate)/((tau*sample_rate)+1))*roll_angle_highpass_filter_out)-(((tau*sample_rate)/((tau*sample_rate)+1))*(roll_angle_highpass1-roll_angle_highpass));
  pitch_angle_highpass_filter_out = (((tau*sample_rate)/((tau*sample_rate)+1))*pitch_angle_highpass_filter_out)-(((tau*sample_rate)/((tau*sample_rate)+1))*(pitch_angle_highpass1-pitch_angle_highpass));
  roll_angle_highpass1=roll_angle_highpass;
  pitch_angle_highpass1=pitch_angle_highpass;
  //feed the angles obtained from gravity vector into a lowpass filter.
  roll_angle_lowpass_filter_out=(roll_angle_lowpass/((tau*100)+1))+(((tau*100)/(((tau*100)+1)))*roll_angle_lowpass_filter_out);
  pitch_angle_lowpass_filter_out=(pitch_angle_lowpass/((tau*100)+1))+(((tau*100)/(((tau*100)+1)))*pitch_angle_lowpass_filter_out);

  //add 90 degrees to angles to orient the numbers for the servo motors.
  roll_angle=roll_angle_lowpass_filter_out + roll_angle_highpass_filter_out+90;
  pitch_angle=pitch_angle_lowpass_filter_out + pitch_angle_highpass_filter_out+90;
  Serial.println(roll_angle);
  Serial.println(pitch_angle);
  Serial.println("end");
  digitalWrite(pwmpin1,LOW);
if(digitalRead(engage_centering)==HIGH){
    if(digitalRead(manual_override_pin)==HIGH){
      //set switch values to increase or decreae pitch and roll to keep be level.
      if(roll_angle<=89){
         digitalWrite(left_pin_roll, HIGH);
         digitalWrite(right_pin_roll, LOW);
        // relative_angle1=100-roll_angle;
        // analogWrite(pwmpin1,map((3.7*relative_angle1)/((0.041*relative_angle1)+1), 0, 90, 0, 255));
      }else{
        if(roll_angle>=91){
         digitalWrite(left_pin_roll, LOW);
         digitalWrite(right_pin_roll, HIGH);
        // relative_angle2=roll_angle-80;
        // analogWrite(pwmpin1,map((3.7*relative_angle2)/((0.041*relative_angle2)+1), 0, 90, 0, 255));
        }else {
         digitalWrite(left_pin_roll, LOW);
         digitalWrite(right_pin_roll, LOW);
         //analogWrite(pwmpin1,map(0, 0, 90, 0, 255));
        }
      }
    }else{
      if(digitalRead(move_left_manual)==LOW){
         digitalWrite(left_pin_roll, HIGH);
         digitalWrite(right_pin_roll, LOW);
        // relative_angle1=100-roll_angle;
        // analogWrite(pwmpin1,map((3.7*relative_angle1)/((0.041*relative_angle1)+1), 0, 90, 0, 255));
      }
      else{
         digitalWrite(left_pin_roll, LOW);
         digitalWrite(right_pin_roll, LOW);
         //analogWrite(pwmpin1,map((3.7*relative_angle1)/((0.041*relative_angle1)+1), 0, 90, 0, 255));
      }
      if(digitalRead(move_right_manual)==LOW){
         digitalWrite(left_pin_roll, LOW);
         digitalWrite(right_pin_roll, HIGH);
         //relative_angle2=roll_angle-80; //investigate
        // analogWrite(pwmpin1,map((3.7*relative_angle2)/((0.041*relative_angle2)+1), 0, 90, 0, 255));
      }
      else{
         digitalWrite(left_pin_roll, LOW);
         digitalWrite(right_pin_roll, LOW);
        // analogWrite(pwmpin1,map((3.7*relative_angle1)/((0.041*relative_angle1)+1), 0, 90, 0, 255));
    }
  }
}
else{
   if(count>295){                                      //295 and 285 are the counts when the linear actuator is near the middle of the its stroke. These values may need to be fine tuned later.
     digitalWrite(left_pin_roll, HIGH);
     digitalWrite(right_pin_roll, LOW);
     //analogWrite(pwmpin1,map(45, 0, 90, 0, 255));
    }else{
      if(count<285){
       digitalWrite(left_pin_roll, LOW);
      digitalWrite(right_pin_roll, HIGH);
     // analogWrite(pwmpin1,map(45, 0, 90, 0, 255));
      }else{
      digitalWrite(left_pin_roll, LOW);
      digitalWrite(right_pin_roll, LOW);
     // analogWrite(pwmpin1,map(0, 0, 90, 0, 255));
      }
   }
  }
//  if(analogRead(analogPin) >= 210){
  //  if(count>295){
 //    digitalWrite(left_pin_roll, HIGH);
 //    digitalWrite(right_pin_roll, LOW);
  //   analogWrite(pwmpin1,map(45, 0, 90, 0, 255));
  //  }else{
   //   if(count<285){
   //    digitalWrite(left_pin_roll, LOW);
   //   digitalWrite(right_pin_roll, HIGH);
  //    analogWrite(pwmpin1,map(45, 0, 90, 0, 255));
   //   }else{
   //   digitalWrite(left_pin_roll, LOW);
   //   digitalWrite(right_pin_roll, LOW);
  //    analogWrite(pwmpin1,map(0, 0, 90, 0, 255));
 //     }
  //  }
 // }
  //this code makes sure that the count variable is added to memory every time it is changed. 
  if(count != past_count){
        past_count = count;
        EEPROM.update(mem_address,(count/100));
        EEPROM.update(mem_address+1,count-((count/100)*100));
    if(mem_address != 0){
       EEPROM.update(mem_address-1,255);
    }
    mem_address+=1;
    if(mem_address >= (EEPROM.length()-1)){
       mem_address = 0;
    }
  }

  //This code makes sure that the count variable can be reset if it ever becomes too far off. 
  //if(digitalRead(zero_point_set) == HIGH){
  //  count = 0;
  //}
  //these print statements print out important parameters that allow the code to be troubleshooted if needed. 
  Serial.println("memory address");
  Serial.println(mem_address);
  Serial.println("count");
  Serial.println(count);
  Serial.println("current_time-past_time");
  Serial.println(current_time-past_time);
  Serial.println("current_time");
  Serial.println(current_time);

  delay(loop_delay);
}

void countSteps(void) {
    current_time_count=millis();
   if(current_time_count-past_time_count >= 9.0){
     if(digitalRead(left_pin_roll)==HIGH){
       count--;
     }
     if(digitalRead(right_pin_roll)==HIGH){
       count++;
     }
    past_time_count = current_time_count;
  }
}
