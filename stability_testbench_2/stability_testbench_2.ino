#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo roll;
Servo pitch;
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
int count=0;
int sample_number=50;
int loop_delay=50;
float mag_yz_plane=0;
float sample_rate=0;
float time_delay=0;

float roll_angle_original=0;
float pitch_angle_original=0;

float past_time=0;
float current_time=0;

int left_pin_roll=2;
int right_pin_roll=3;
int left_pin_pitch=4;
int right_pin_pitch=5;

void setup() {
 roll.attach(8);
 pitch.attach(9);
 //setting angle
 roll.write(motor_roll_angle);
 pitch.write(motor_pitch_angle);
 delay(1000);
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

  //print statements for testing purposes. Only should be uncommented for testing.
  //Serial.print("roll_angle_unfiltered:");
  //Serial.print(roll_angle_lowpass_filter_out + roll_angle_highpass_filter_out+90);
  //Serial.print(",");
  //Serial.print("pitch_angle_comp_filtered:");
  //Serial.print(pitch_angle_lowpass_filter_out + pitch_angle_highpass_filter_out+90);
  //Serial.print(",");
  //Serial.print("motor_pitch_angle:");
  //Serial.print(motor_pitch_angle);
  //Serial.print(",");
  //Serial.print("motor_roll_angle:");
  //Serial.print(motor_roll_angle);
  //Serial.print(",");
  //Serial.print("Sample_Rate:");
  //Serial.println(sample_rate);
  //Serial.print(",");
  //Serial.print("pitch_angle_comp_filtered:");
  //Serial.println(pitch_angle_lowpass_filter_out + pitch_angle_highpass_filter_out);

  //add 90 degrees to angles to orient the numbers for the servo motors.
  roll_angle=roll_angle_lowpass_filter_out + roll_angle_highpass_filter_out+90;
  pitch_angle=pitch_angle_lowpass_filter_out + pitch_angle_highpass_filter_out+90;
  //set switch values to increase or decreae pitch and roll to keep be level.
  if(roll_angle<=89){
     roll_angle_plus=1;
     roll_angle_minus=0;
  }else{
    if(roll_angle>=91){
     roll_angle_plus=0;
     roll_angle_minus=1;
    }else {
     roll_angle_plus=0;
     roll_angle_minus=0;
    }
  }

  if(pitch_angle<=89){
     pitch_angle_plus=0;
     pitch_angle_minus=1;
  }else{
    if(pitch_angle>=91){
     pitch_angle_plus=1;
     pitch_angle_minus=0;
    }else {
     pitch_angle_plus=0;
     pitch_angle_minus=0;
    }
  }

  //adjust the angles of the pitch and roll motors.
  if((roll_angle_plus == 1)&&(roll_angle_minus == 0)){
    motor_roll_angle = motor_roll_angle + 1;
  }
  if((roll_angle_plus == 0)&&(roll_angle_minus == 1)){
    motor_roll_angle = motor_roll_angle - 1;
  }
    if((roll_angle_plus == 0)&&(roll_angle_minus == 0)){
    motor_roll_angle = motor_roll_angle;
  }
  

  if((pitch_angle_plus == 1)&&(pitch_angle_minus == 0)){
    motor_pitch_angle = motor_pitch_angle + 1;
  }
  if((pitch_angle_plus == 0)&&(pitch_angle_minus == 1)){
    motor_pitch_angle = motor_pitch_angle - 1;
  }
    if((pitch_angle_plus == 0)&&(pitch_angle_minus == 0)){
    motor_pitch_angle = motor_pitch_angle;
  }
  //clamp motor angles to in between 0 and 180 degrees.
  if(motor_roll_angle>180){
    motor_roll_angle=180;
  }
  if(motor_roll_angle<0){
    motor_roll_angle=0;
  }
  if(motor_pitch_angle>180){
    motor_pitch_angle=180;
  }
  if(motor_pitch_angle<0){
    motor_pitch_angle=0;
  }
  roll.write(motor_roll_angle);
  pitch.write(motor_pitch_angle);

  //add delay to keep loop stable. 
  delay(loop_delay);
}
