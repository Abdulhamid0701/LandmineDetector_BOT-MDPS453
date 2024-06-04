#include <Arduino.h>

#include <L298N.h>
// #include <MPU6050_Light.h>
//  #include <MPU6050.h>
//  #include <KalmanFilter.h>
#include "Wire.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <PID_DRIVE.h>
#include <MPU6500_WE.h>
#define MPU6500_ADDR 0x68

// Pin definition
#define INL1 22
#define INL2 24
#define ENL 8

#define INR1 26
#define INR2 28
#define ENR 7

const int PIN_ENCOD_A_MOTOR_LEFT = 30; // A channel for encoder of left motor
const int PIN_ENCOD_B_MOTOR_LEFT = 2;  // B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3; // A channel for encoder of right motor
const int PIN_ENCOD_B_MOTOR_RIGHT = 4; // B channel for encoder of right motor

volatile float pos_left = 0;  // Left motor encoder position
volatile float pos_right = 0; // Right motor encoder position

// int left_V_ros;
// int right_V_ros;

int scenario;
int prev_cmd;
int curr_cmd = 0;

/// @brief Obstacle Avoidance Shit
float motion_cmd = 0;
float current_millis_avoid;
float prev_millis_avoid = 0;
float finished_rot_millis = 0;
float started_fwd_millis = 0;
int delwa2ty_fwd = 0;
int delwa2ty_fwd_oula = 1;
int delwa2ty_fwd_tanya = 0;
int Akher_lafa = 0;
float started_2nd_millis = 0;
float millis_before_cmd = 0;
float millis_before_cmd2 = 0;
int finished_my_shit_flag = 1;

bool rot_finish_flag = 0;

float left_V_ros_recieved;
float right_V_ros_recieved;

float PWM_left;
float PWM_right;

const int cpr = 520;

float dt = 0.1;

float V_Max = 15; // max speed for motor in rad/s (15)

float current_micros;
float prev_micros;

int cc;

float current_basmag;
float prev_basmag = 0;

float kalYawRate = 0; // Variable to store filtered yaw rate

volatile long prevEncoderCountLeft = 0;  // previous count for calculating speed
volatile long prevEncoderCountRight = 0; // previous count for calculating speed

float angularSpeedLeft;
float angularSpeedRight;

// KalmanFilter kalmanZ(0.001, 0.003, 0.03); // Initialize Kalman filter for gyro Z-axis
//  Create one motor instance
L298N motor_right(ENR, INR1, INR2);
L298N motor_left(ENL, INL1, INL2);

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

void encoderLeftMotor()
{
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT))
    pos_left--;
  else
    pos_left++;
}

// Right motor encoder counter
void encoderRightMotor()
{
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT))
    pos_right--;
  else
    pos_right++;
}

ros::NodeHandle nh;

void callBackFunctionMotorLeft(const std_msgs::Float32 &left_V_ros)
{
  left_V_ros_recieved = left_V_ros.data;
}

void callBackFunctionMotorRight(const std_msgs::Float32 &right_V_ros)
{
  right_V_ros_recieved = right_V_ros.data;
}

void callBack_CMD(const std_msgs::Int32 &CMDD)
{
  motion_cmd = CMDD.data;
}

void generate_setpoints()
{
  err_theta = abs(theta_sp - angleZ);

  if (angleZ >= 0)
  {
    if (abs(0 - angleZ) < ang_tol)
    {
      vright_target = target_fwd_speed;
      vleft_target = target_fwd_speed;
    }
    else
    {
      vright_target = target_fwd_speed - kp_angular * err_theta;
      vleft_target = target_fwd_speed + kp_angular * err_theta;
    }
  }
  else
  {
    if (abs(0 - angleZ) < ang_tol)
    {
      vright_target = target_fwd_speed;
      vleft_target = target_fwd_speed;
    }
    else
    {
      vright_target = target_fwd_speed + kp_angular * err_theta;
      vleft_target = target_fwd_speed - kp_angular * err_theta;
    }
  }

  if (err_theta > 40)
  {
    vright_target = vright_target * 0.5;
    vleft_target = vleft_target * 0.5;
  }
}

void rotate_45_counter_clk()
{
  theta_sp = 45;

  err_theta = (theta_sp - angleZ);

  if (abs(err_theta) <= 0.6)
  {
    rot_finish_flag = 1;
    tot_err_vl = 0;
    tot_err_vl = 0;
    prev_err_vl = 0;
    prev_err_vr = 0;
    prev_err_theta = 0;
    tot_err_theta = 0;
    err_theta = 0;
    err_vl = 0;
    err_vr = 0;
    angleZ = 0;
    finished_rot_millis = millis();
    delwa2ty_fwd = 1;
    // myMPU6500.autoOffsets();
    scenario = 0;
    motor_left.stop();
    motor_right.stop();
    // Serial.println("doneeeeeeeeeee");
    // delay(100);
    // return;
  }
  else if (err_theta > 0)
  {
    // Serial.println(err_theta);

    vright_target = -1;
    // 4 - kp_angular * abs(err_theta);
    vleft_target = 3;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
  }
  else
  {
    // Serial.println(err_theta);-

    // Serial.println(err_theta);
    vright_target = 3;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
    vleft_target = -1;
    // 4 - kp_angular * abs(err_theta);
  }

  prev_err_theta = err_theta;
  tot_err_theta += err_theta;
}

void rotate_180_counter_clk()
{
  theta_sp = 90;

  err_theta = (theta_sp - angleZ);

  if (abs(err_theta) <= 0.6)
  {
    rot_finish_flag = 1;
    tot_err_vl = 0;
    tot_err_vl = 0;
    prev_err_vl = 0;
    prev_err_vr = 0;
    prev_err_theta = 0;
    tot_err_theta = 0;
    err_theta = 0;
    err_vl = 0;
    err_vr = 0;
    angleZ = 0;
    finished_rot_millis = millis();
    delwa2ty_fwd = 1;
    // myMPU6500.autoOffsets();
    scenario = 0;
    motor_left.stop();
    motor_right.stop();
    // Serial.println("doneeeeeeeeeee");
    // delay(100);
    // return;
  }
  else if (err_theta > 0)
  {
    // Serial.println(err_theta);

    vright_target = -1;
    // 4 - kp_angular * abs(err_theta);
    vleft_target = 3;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
  }
  else
  {
    // Serial.println(err_theta);-

    // Serial.println(err_theta);
    vright_target = 3;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
    vleft_target = -1;
    // 4 - kp_angular * abs(err_theta);
  }

  prev_err_theta = err_theta;
  tot_err_theta += err_theta;
}

void rotate_45_clk()
{
  theta_sp = -45;

  err_theta = (theta_sp - angleZ);

  if (abs(err_theta) <= 0.6)
  {
    rot_finish_flag = 1;
    tot_err_vl = 0;
    tot_err_vl = 0;
    prev_err_vl = 0;
    prev_err_vr = 0;
    prev_err_theta = 0;
    tot_err_theta = 0;
    err_theta = 0;
    err_vl = 0;
    err_vr = 0;
    angleZ = 0;
    finished_rot_millis = millis() ;
    delwa2ty_fwd = 1;
    // myMPU6500.autoOffsets();
    scenario = 0;
    motor_left.stop();
    motor_right.stop();
    prev_basmag = current_basmag;
    // Serial.println("doneeeeeeeeeee");
    // delay(100);
    // return;
  }
  else if (err_theta > 0)
  {
    // Serial.println(err_theta);

    vright_target = -1;
    // 4 - kp_angular * abs(err_theta);
    vleft_target = 3;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
  }
  else
  {
    // Serial.println(err_theta);-

    // Serial.println(err_theta);
    vright_target = 3;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
    vleft_target = -1;
    // 4 - kp_angular * abs(err_theta);
  }

  prev_err_theta = err_theta;
  tot_err_theta += err_theta;
}

void rotate_to_sp(float sp)
{
  // theta_sp = -45;

  err_theta = (theta_sp - angleZ);

  if (abs(err_theta) <= 0.7)
  {
    rot_finish_flag = 1;
    tot_err_vl = 0;
    tot_err_vl = 0;
    prev_err_vl = 0;
    prev_err_vr = 0;
    prev_err_theta = 0;
    tot_err_theta = 0;
    err_theta = 0;
    err_vl = 0;
    err_vr = 0;
    angleZ = 0;
    finished_rot_millis = millis();
    delwa2ty_fwd = 1;
    // myMPU6500.autoOffsets();
    scenario = 0;
    motor_left.stop();
    motor_right.stop();
    prev_basmag = current_basmag;
    // Serial.println("doneeeeeeeeeee");
    // delay(100);
    // return;
  }
  else if (err_theta > 0)
  {
    // Serial.println(err_theta);

    vright_target = -1.2;
    // 4 - kp_angular * abs(err_theta);
    vleft_target = 3.6;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
  }
  else
  {
    // Serial.println(err_theta);-

    // Serial.println(err_theta);
    vright_target = 3.6;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
    vleft_target = -1.2;
    // 4 - kp_angular * abs(err_theta);
  }

  prev_err_theta = err_theta;
  tot_err_theta += err_theta;
}


void rotate_180_clk()
{
  theta_sp = -90;

  err_theta = (theta_sp - angleZ);

  if (abs(err_theta) <= 0.6)
  {
    rot_finish_flag = 1;
    tot_err_vl = 0;
    tot_err_vl = 0;
    prev_err_vl = 0;
    prev_err_vr = 0;
    prev_err_theta = 0;
    tot_err_theta = 0;
    err_theta = 0;
    err_vl = 0;
    err_vr = 0;
    angleZ = 0;
    finished_rot_millis = millis();
    delwa2ty_fwd = 1;
    // myMPU6500.autoOffsets();
    scenario = 0;
    motor_left.stop();
    motor_right.stop();
    prev_basmag = current_basmag;
    // Serial.println("doneeeeeeeeeee");
    // delay(100);
    // return;
  }
  else if (err_theta > 0)
  {
    // Serial.println(err_theta);

    vright_target = -1;
    // 4 - kp_angular * abs(err_theta);
    vleft_target = 3;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
  }
  else
  {
    // Serial.println(err_theta);-

    // Serial.println(err_theta);
    vright_target = 3;
    //+ kp_angular * abs(err_theta) + (err_theta - prev_err_theta)*kd_angular + ki_angular*tot_err_theta;
    vleft_target = -1;
    // 4 - kp_angular * abs(err_theta);
  }

  prev_err_theta = err_theta;
  tot_err_theta += err_theta;
}

void PID_PWM()
{
  tNow = micros();

  err_vr = vright_target - angularSpeedRight;
  err_vl = vleft_target - angularSpeedLeft;

  vright = kp_vr * err_vr + (kd_vr * ((err_vr - prev_err_vr) / (tNow - tPrev))) + ki_vr * tot_err_vr;
  vleft = kp_vl * err_vl + (kd_vl * ((err_vl - prev_err_vl) / (tNow - tPrev))) + ki_vl * tot_err_vl;

  if (rot_finish_flag == 0)
  {
    vright = vright_target;
    vleft = vleft_target;
  }

  tPrev = tNow;
  tot_err_vr += err_vr;
  tot_err_vl += err_vl;
  prev_err_vr = err_vr;
  prev_err_vl = err_vl;
}

void move_mots()
{
  // int right_speed = (vright / V_Max) * 255;
  // int left_speed = (vleft / V_Max) * 255;

  // map PID output to velocities achievable by the motor
  int right_speed = map(abs(vright), 0, V_Max, 20, 255);
  int left_speed = map(abs(vleft), 0, V_Max, 20, 255);

  motor_right.setSpeed(abs(right_speed));
  motor_left.setSpeed(abs(left_speed));

  right_speed = vright;
  left_speed = vleft;

  if (right_speed > 0)
  {
    motor_right.forward();
  }
  else if (right_speed < 0)
  {
    motor_right.backward();
  }
  else if (abs(right_speed) <= 1)
  {
    motor_right.setSpeed(0);
    motor_right.stop();
  }

  if (left_speed > 0)
  {
    motor_left.forward();
  }
  else if (left_speed < 0)
  {
    motor_left.backward();
  }
  else if (abs(left_speed) <= 1)
  {
    motor_left.setSpeed(0);
    motor_left.stop();
  }

  if (scenario == 0)
  {
    motor_left.stop();
    motor_right.stop();
  }
}

ros::Subscriber<std_msgs::Int32> cmdSUB("COMMAND", &callBack_CMD);

std_msgs::Float32 right_encoder;
ros::Publisher right_encoder_Publisher("encoder_right", &right_encoder);

std_msgs::Float32 left_encoder;
ros::Publisher left_encoder_Publisher("encoder_left", &left_encoder);

std_msgs::Float32 MPU_reading;
ros::Publisher MPU_Publisher("MPU", &MPU_reading);

std_msgs::Int32 ROTTFLAG;
ros::Publisher ROT_Publisher("rotation_flag", &ROTTFLAG);

// MPU6050 mpu(Wire);

void motion_scenario_planner()
{ 
  
  // Landmark turning Planner 
  if (motion_cmd == 2)
  {
    current_millis_avoid = millis();

    // Rotate 90 left
    if ((current_millis_avoid - millis_before_cmd2 > 3000) && rot_finish_flag == 0)
    {
      theta_sp = 90;
      scenario = 11;
      finished_my_shit_flag = 0;
    }
    else
    {
      scenario = 0;
      prev_millis_avoid = current_millis_avoid;
      finished_my_shit_flag = 0;
    }
  }


  // Obstacle Avoidance Planner 
  if (motion_cmd == 3) //  avoid obstacle left
  {
    current_millis_avoid = millis();// - millis_before_cmd;
    // finished_rot_millis = finished_rot_millis - millis_before_cmd;

    // Rotate 45 left
    if ((current_millis_avoid - millis_before_cmd> 3000) && (current_millis_avoid - millis_before_cmd< 5000) && rot_finish_flag == 0)
    {
      theta_sp = 45;
      scenario = 10;
      finished_my_shit_flag = 0;
    }

    // Move FWD a While
    else if ((current_millis_avoid - finished_rot_millis > 1000) && delwa2ty_fwd == 1 && Akher_lafa == 0 && delwa2ty_fwd_oula == 1)
    {
      target_fwd_speed = 4.1;
      scenario = 1;
      started_fwd_millis = millis();// - millis_before_cmd;
      if (started_fwd_millis - finished_rot_millis >= 3000)
      {
        delwa2ty_fwd_oula = 0;
        delwa2ty_fwd = 0;
        started_2nd_millis = millis();// - millis_before_cmd;
        finished_rot_millis = 0;
      }
      finished_my_shit_flag = 0;
      // delwa2ty_fwd = 0;
    }

    // Rotate 90 Right
    else if ((current_millis_avoid - started_2nd_millis > 1000) && rot_finish_flag == 0 && delwa2ty_fwd_oula == 0 && delwa2ty_fwd == 0 && delwa2ty_fwd_tanya == 0)
    {
      theta_sp = -95;
      scenario = 10;
      finished_my_shit_flag = 0;
    }

    // Move FWD a While
    else if ((current_millis_avoid - finished_rot_millis > 1000) && delwa2ty_fwd_oula == 0 && finished_rot_millis != 0 && delwa2ty_fwd_tanya == 0)
    {
      target_fwd_speed = 4;
      scenario = 1;
      Akher_lafa = 1;
      started_fwd_millis = millis();//- millis_before_cmd;
      if (started_fwd_millis - finished_rot_millis >= 2800)
      {
        delwa2ty_fwd_oula = 1;
        delwa2ty_fwd_tanya = 1;
      }
      Akher_lafa = 1;
      finished_my_shit_flag = 0;
      // delwa2ty_fwd = 0;
    }

    // Rotate 45 Left
    else if ((current_millis_avoid - finished_rot_millis > 3000) && rot_finish_flag == 0 && Akher_lafa == 1 && delwa2ty_fwd_tanya == 1)
    {
      theta_sp = 45;
      scenario = 11;
      // scenario = 3;
      prev_millis_avoid = current_millis_avoid;
      finished_my_shit_flag = 0;
    }

    // Motion Done
    else
    {
      scenario = 0;
      finished_my_shit_flag = 0;
    }
  }

  // Moving FWD Planner 
  else if (motion_cmd == 1)
  {
    target_fwd_speed = 3.3;
    scenario = 1;
    finished_my_shit_flag = 1;
  }

  // Stopping the robot 
  else if (motion_cmd == 0)
  {
    scenario = 0;
    finished_my_shit_flag = 1;
  }

}

void setup()
{
  // Serial.begin(9600);
  Wire.begin();

  scenario = 0;

  if (!myMPU6500.init())
  {
    // Serial.println("MPU6500 does not respond");
  }
  else
  {
    // Serial.println("MPU6500 is connected");
  }
  // Serial.println("Position your MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  // Serial.println("Calibration done!");

  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);

  pinMode(INR1, OUTPUT);
  pinMode(INR2, OUTPUT);
  pinMode(ENR, OUTPUT);
  pinMode(INL1, OUTPUT);
  pinMode(INL2, OUTPUT);
  pinMode(ENL, OUTPUT);

  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH); // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH); // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), encoderRightMotor, RISING);

  // byte status = mpu.begin();
  // mpu.calcOffsets(true, true); // gyro and accelero

  nh.getHardware()->setBaud(9600);
  nh.initNode();

  nh.advertise(right_encoder_Publisher);
  nh.advertise(left_encoder_Publisher);
  nh.advertise(MPU_Publisher);

  nh.subscribe(cmdSUB);
  nh.advertise(ROT_Publisher);

  prev_micros = 0;
}

unsigned long delayTime1 = 4000;
unsigned long delayTime2 = 8000;

unsigned long current_micross;
unsigned long prev_micross = 0;
unsigned long prev_micross1 = 0;
unsigned long prev_micross2 = 0;

void loop()
{
  nh.spinOnce();

  // mpu.update();

  xyzFloat gyro = myMPU6500.getGyrValues();

  target_fwd_speed = 2;
  curr_cmd = motion_cmd;

  if (curr_cmd != prev_cmd && curr_cmd == 3)
  {
   millis_before_cmd = millis();
  }

  if (curr_cmd != prev_cmd && curr_cmd == 2)
  {
   millis_before_cmd2 = millis();
  }
  // Serial.println(scenario);


  motion_scenario_planner(); // Sends back scenario

  if (scenario == 1)
  {
    // Move forward
    // target_fwd_speed = 3.2;
    theta_sp = 0;
    rot_finish_flag = 0;
    generate_setpoints();
  }
  else if (scenario == 0)
  {
    // Stop robot
    target_fwd_speed = 0;
    motor_left.stop();
    motor_right.stop();
    tot_err_vl = 0;
    tot_err_vl = 0;
    prev_err_vl = 0;
    prev_err_vr = 0;
    err_theta = 0;
    err_vl = 0;
    err_vr = 0;
    // rot_finish_flag = 0;
  }
  else if (scenario == 2 && rot_finish_flag == 0)
  {
    // rotate 180 degrees clockwise
    rotate_180_clk();
    // target_fwd_speed = 4;
    /// theta_sp = -90;
    // rot_finish_flag = 0;
    // generate_setpoints();
  }
  else if (scenario == 2 && rot_finish_flag == 1)
  {
    // Move forward
    target_fwd_speed = 3.8;
    theta_sp = 0;
    generate_setpoints();
  }
  else if (scenario == 3)
  {
    // rotate 180 degrees anticlockwise
    rotate_45_counter_clk();
  }
  else if (scenario == 3 && rot_finish_flag == 1)
  {
    target_fwd_speed = 0;
    motor_left.stop();
    motor_right.stop();
    angleZ = 0;
    tot_err_vl = 0;
    tot_err_vl = 0;
    prev_err_vl = 0;
    prev_err_vr = 0;
  }
  else if (scenario == 10)
  {
    rotate_to_sp(theta_sp);
  }
  else if (scenario == 11)
  {
    rotate_to_sp(theta_sp);
    if (rot_finish_flag == 1)
    {
      finished_my_shit_flag = 1;
      rot_finish_flag = 0;
    }
  }
  else
  {
    motor_left.stop();
    motor_right.stop();
  }

  // Correct the PWM values using the velocity PID controller
  PID_PWM();

  // Move the motors accordingly
  move_mots();

  // Update sensors data (encoders & IMU)
  current_micros = micros();

  float dtt = (current_micros - prr) * 1e-6;
  prr = current_micros;

  angleZ += gyro.z * dtt;

  // if (angleZ > 360 || angleZ < -360)
  //{
  // angleZ = 0;
  //}
  angularSpeedLeft = (pos_left - prevEncoderCountLeft) * (2 * PI / cpr) / (0.0100);
  angularSpeedRight = ((pos_right - prevEncoderCountRight) * (2 * PI / cpr) / (0.0100));

  right_encoder.data = angularSpeedRight;
  left_encoder.data = angularSpeedLeft;
  MPU_reading.data = gyro.z;

  // Publiseh sensor data to the pythone node
  right_encoder_Publisher.publish(&right_encoder);
  left_encoder_Publisher.publish(&left_encoder);
  MPU_Publisher.publish(&MPU_reading);

  ROTTFLAG.data = finished_my_shit_flag;
  ROT_Publisher.publish(&ROTTFLAG);

  if (current_micros - prev_micris > 0.2e6)
  {
    // Serial.print("Angular speed left (rad/s): ");
    // Serial.println(right_encoder.data);
    // Serial.print("Angular speed right (rad/s): ");
    // Serial.println(angularSpeedRight);
    // Serial.print("IMU Yaw Orientation (Degs): ");
    // Serial.println(angleZ);
    // prev_micris = current_micros;
    // Serial.println(ROTTFLAG.data);
  }
  prevEncoderCountLeft = pos_left;
  prevEncoderCountRight = pos_right;

  prev_cmd = curr_cmd;

  delay(10);
}