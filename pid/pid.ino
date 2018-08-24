///////////////////////////////////////////////////////////////////////////////////////
//THIS IS A DEMO SOFTWARE JUST FOR EXPERIMENT PURPOER IN A NONCOMERTIAL ACTIVITY
//Version: 1.0 (AUG, 2016)

//Gyro - Arduino UNO R3
//VCC  -  5V
//GND  -  GND
//SDA  -  A4
//SCL  -  A5
//INT - port-2

#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>


// GLOBAL VARIABLES
// orientation
int offset_samples = 1000;
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch, angle_roll;
float angle_pitch_output, angle_roll_output;
int temp;

// motor
Servo ServoMotor;
int pino_motor = 9;
int pwm;
int pwm_max = 70;
int pwm_min = 52;

// parameter identification
//int count = 0;
//int inc = 1;

// pid controller
double Setpoint, Input, Output;
double Kp = 0.03, Ki = 0.02, Kd = 0.02; //posição 1
double Offset = 57; //implementar mudança dinamica do offset
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

unsigned long loop_timer;

void setup() {
  Serial.begin(115200);
  //while (!Serial)
  
  Serial.println("Setting MPU registers");
  Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050
  
  // calculates mpu offset
  Serial.println("Calculating MPU offset");
  for (int cal_int = 0; cal_int < offset_samples ; cal_int ++) {       //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }
  gyro_x_cal /= offset_samples;
  gyro_y_cal /= offset_samples;
  gyro_z_cal /= offset_samples;
  delay(1000); // conferir

  Serial.println("Setting Motor");
  ServoMotor.attach(pino_motor);
  ServoMotor.write(1000); // não lembro o por que
  delay(5000);
  pwm = 50;

  //PID
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100);

  Serial.println("Go!");
  loop_timer = micros();

}

void loop() {
  read_mpu_6050_data();
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

  //Calibrar automaticamente
  angle_pitch_acc -= 1.1;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= -2.73;                                               //Accelerometer calibration value for roll

  if (set_gyro_angles) {                                               //If the IMU is already started
    angle_pitch = angle_pitch * 0.998 + angle_pitch_acc * 0.002;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.998 + angle_roll_acc * 0.002;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else {                                                               //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  //Filtro causando atraso muito grande
  //angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  //angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  Input = angle_pitch;
  myPID.Compute();

  //PID1
  //Serial.println(Output);
  //Serial.println(Output);
  pwm = Offset + (int)Output;
  if (pwm < pwm_min) {
    ServoMotor.write(pwm_min);
    Serial.println(pwm_min);
  } else if (pwm > pwm_max) {
    ServoMotor.write(pwm_max);
    Serial.println(pwm_max);
  } else {
    ServoMotor.write(pwm);
    Serial.println(pwm);
  }

  /*
      if(count<1500){
      ServoMotor.write(pwm);
      count++;
      }else{
      Serial.print(pwm);
      Serial.print(' ');
      Serial.println(angle_pitch);
      if(pwm > 100) inc = -1;
      pwm += inc;
      count = 0;
      }
  */
  while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();//Reset the loop timer



}




void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();
}


void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}














