// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// -----------------------------------------------------------------------------

#include <Arduino.h>

// Robot control header files
#include <MPU6050_tockn.h>
#include "Servo_STS3032.h"
#include <SimpleFOC.h>

// WiFi control data transmission header files
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include <FS.h>
#include "basic_web.h"
#include "robot.h"
#include <wifi_robot.h>
#include "esp_adc_cal.h"

#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 90
#define SERVOMAX 565

#define SERVO_R 0
#define SERVO_L 15

/************ Instance Definitions *************/

void lqr_balance_loop();
void jump_loop();
void leg_loop();
void yaw_loop();
void web_loop();
void yaw_angle_addup();
void basicWebCallback(void);
void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
//void adc_calibration_init();
//void bat_check();
void moveServoRestricted(uint8_t channel, long stsVal);


// Motor instances
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);

// Encoder instances
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// Pwm Driver instance
Adafruit_PWMServoDriver board = Adafruit_PWMServoDriver(0x40, I2Cone);

// PID Controller instances
/*
PIDController pid_angle(           1,  0, 0, 100000,   8);
PIDController pid_gyro(         0.06,  0, 0, 100000,   8);
PIDController pid_distance(      0.5,  0, 0, 100000,   8);
PIDController pid_speed(         0.7,  0, 0, 100000,   8);
PIDController pid_yaw_angle(     1.0,  0, 0, 100000,   8);
PIDController pid_yaw_gyro(     0.04,  0, 0, 100000,   8);
PIDController pid_lqr_u(           1, 15, 0, 100000,   8);
PIDController pid_zeropoint(   0.002,  0, 0, 100000,   4);
PIDController pid_roll_angle(      8,  0, 0, 100000, 450);
*/

PIDController pid_angle(           1,  0, 0, 100000,   8);
PIDController pid_gyro(         0.06,  0, 0, 100000,   8);
PIDController pid_distance(      0,  0, 0, 100000,   8);
PIDController pid_speed(         0,  0, 0, 100000,   8);
PIDController pid_yaw_angle(     1.0,  0, 0, 100000,   8);
PIDController pid_yaw_gyro(     0.04,  0, 0, 100000,   8);
PIDController pid_lqr_u(           1, 15, 0, 100000,   8);
PIDController pid_zeropoint(   0,  0, 0, 100000,   4);
PIDController pid_roll_angle(      8,  0, 0, 100000, 450);

// Low-pass filter instances
LowPassFilter lpf_joyy(0.2);
LowPassFilter lpf_zeropoint(0.1);
LowPassFilter lpf_roll(0.3);

// Commander communication instance
Commander command = Commander(Serial);

void StabAngle(char* cmd)     { command.pid(&pid_angle, cmd);     }
void StabGyro(char* cmd)      { command.pid(&pid_gyro, cmd);      }
void StabDistance(char* cmd)  { command.pid(&pid_distance, cmd);  }
void StabSpeed(char* cmd)     { command.pid(&pid_speed, cmd);     }
void StabYawAngle(char* cmd)  { command.pid(&pid_yaw_angle, cmd); }
void StabYawGyro(char* cmd)   { command.pid(&pid_yaw_gyro, cmd);  }
void lpfJoyy(char* cmd)       { command.lpf(&lpf_joyy, cmd);      }
void StabLqrU(char* cmd)      { command.pid(&pid_lqr_u, cmd);     }
void StabZeropoint(char* cmd) { command.pid(&pid_zeropoint, cmd); }
void lpfZeropoint(char* cmd)  { command.lpf(&lpf_zeropoint, cmd); }
void StabRollAngle(char* cmd) { command.pid(&pid_roll_angle, cmd);}
void lpfRoll(char* cmd)       { command.lpf(&lpf_roll, cmd);      }

//void Stabtest_zeropoint(char* cmd) { command.pid(&test_zeropoint, cmd); }

// WebServer instances
WebServer webserver; // server object
WebSocketsServer websocket = WebSocketsServer(81); // Define a webSocket server to handle messages sent by clients
RobotProtocol rp(20);
int joystick_value[2];

// STS Servo instance
//SMS_STS sms_sts;

// MPU6050 instance
MPU6050 mpu6050(I2Ctwo);

/************ Parameter Definitions *************/
#define pi 3.1415927

// LQR self-balancing controller parameters
float LQR_angle = 0;
float LQR_gyro  = 0;
float LQR_speed = 0;
float LQR_distance = 0;
float angle_control   = 0;
float gyro_control    = 0;
float speed_control   = 0;
float distance_control = 0;
float LQR_u = 0;
float angle_zeropoint = 8.2;
float distance_zeropoint = -256.0;       // Wheel position zero-point offset (-256 is an impossible position value, used as an unrefreshed flag)

// YAW axis control data
float YAW_gyro = 0;
float YAW_angle = 0;
float YAW_angle_last = 0;
float YAW_angle_total = 0;
float YAW_angle_zero_point = -10;
float YAW_output = 0;

// Leg servo control data
/*byte ID[2];
s16 Position[2];
u16 Speed[2];
byte ACC[2];*/

// Logic processing flags
float robot_speed = 0;          // Record current wheel speed
float robot_speed_last = 0;     // Record wheel speed of the previous moment
int wrobot_move_stop_flag = 0;  // Record joystick stop flag
int jump_flag = 0;              // Jump period flag
float leg_position_add = 0;     // Roll axis balance control variable
int uncontrolable = 0;          // Body tilt angle too large causing loss of control

// Voltage detection
/*uint16_t bat_check_num = 0;
int BAT_PIN = 35;    // select the input pin for the ADC
static esp_adc_cal_characteristics_t adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_7;     
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// Battery level display LED
#define LED_BAT 13*/

void setup() {
  Serial.begin(115200); // Communication serial
  //Serial2.begin(1000000); // Leg STS servo

  // WiFi initialization
  delay(3000);
  WiFi_SetAP();
  //set_sta_wifi();      // ESP-01S STA mode connect to WiFi network
  webserver.begin();
  webserver.on("/", HTTP_GET, basicWebCallback);
  websocket.begin();
  websocket.onEvent(webSocketEventCallback);

  // Servo initialization
  // Servo effective travel 450
  // Left servo [2048+12+50,2048+12+450]
  // Right servo (implied) [2048-12-50,2048-12-450]
  /*sms_sts.pSerial = &Serial2;
  ID[0] = 1;
  ID[1] = 2;
  ACC[0] = 30;
  ACC[1] = 30;
  Speed[0] = 300;
  Speed[1] = 300;  
  Position[0] = 2148;
  Position[1] = 1948;*/
  // Servo (ID1/ID2) runs to respective Position at max speed V=2400 steps/sec, acceleration A=50(50*100 steps/sec^2)
  //sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

  // Voltage detection
  //adc_calibration_init();
  //adc1_config_width(width);
  //adc1_config_channel_atten(channel, atten);
  //esp_adc_cal_characterize(unit, atten, width, 0, &adc_chars);

  // Battery level display LED
  //pinMode(LED_BAT,OUTPUT);
  
  // Encoder settings
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL); 
  sensor1.init(&I2Ctwo);
  sensor2.init(&I2Cone);

  board.begin();
  board.setPWMFreq(60);

  // MPU6050 settings
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  // Link motor object and encoder object
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // Velocity loop PID parameters
  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 1;
  motor1.PID_velocity.D = 0;

  motor2.PID_velocity.P = 0.05;
  motor2.PID_velocity.I = 1;
  motor2.PID_velocity.D = 0;

  // Driver settings
  motor1.voltage_sensor_align = 6;
  motor2.voltage_sensor_align = 6;
  driver1.voltage_power_supply = 12;
  driver2.voltage_power_supply = 12;
  driver1.init();
  driver2.init();

  // Link motor object and driver object
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;   
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;
  
  // Monitor related settings
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);
  // Motor initialization
  motor1.init();
  motor1.initFOC(); 
  motor2.init();
  motor2.initFOC();

  // Map motors to commander
    command.add('A', StabAngle, (char *) "pid angle");
    command.add('B', StabGyro, (char *) "pid gyro");
    command.add('C', StabDistance, (char *) "pid distance");
    command.add('D', StabSpeed, (char *) "pid speed");
    command.add('E', StabYawAngle, (char *) "pid yaw angle");
    command.add('F', StabYawGyro, (char *) "pid yaw gyro");
    command.add('G', lpfJoyy, (char *) "lpf joyy");
    command.add('H', StabLqrU, (char *) "pid lqr u");
    command.add('I', StabZeropoint, (char *) "pid zeropoint");
    command.add('J', lpfZeropoint, (char *) "lpf zeropoint");
    command.add('K', StabRollAngle, (char *) "pid roll angle");
    command.add('L', lpfRoll, (char *) "lpf roll");

    //command.add('M', Stabtest_zeropoint, "test_zeropoint");

  delay(500);
}

void loop() {
  //bat_check();        // Voltage detection
  web_loop();         // Web data update
  mpu6050.update();   // IMU data update
  lqr_balance_loop(); // LQR self-balancing control
  yaw_loop();         // Yaw axis steering control
  leg_loop();         // Leg motion control
  
  // Assign self-balancing calculation output torque to motors
  motor1.target = (-0.5)*(LQR_u + YAW_output);
  motor2.target = (-0.5)*(LQR_u - YAW_output);

  // Turn off output after falling/losing control
  if( abs(LQR_angle) > 25.0f  )
  {
    uncontrolable = 1;
  }
  if( uncontrolable != 0 ) // Delay recovery after being picked up
  {
    if( abs(LQR_angle) < 10.0f  )
    {
      uncontrolable++;
    }
    if( uncontrolable > 200 ) // Delay time of 200 program cycles
    {
      uncontrolable = 0;
    }
  }
  
  // Stop output (remote stop or loss of control due to excessive angle)
  if(wrobot.go==0 || uncontrolable!=0)
  {
    motor1.target = 0;
    motor2.target = 0;
    leg_position_add = 0;
  }
  
  // Record previous remote control data
  wrobot.dir_last  = wrobot.dir;
  wrobot.joyx_last = wrobot.joyx;
  wrobot.joyy_last = wrobot.joyy;
  
  // Iteratively calculate FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();
  
  // Set wheel motor output
  motor1.move();
  motor2.move();
  
  command.run();
  }

// LQR self-balancing control
void lqr_balance_loop(){
  // LQR balance formula. In practice, to facilitate tuning, the formula is decomposed into 4 P controls, using PIDController method for real-time tuning in commander.
  // QR_u = LQR_k1*(LQR_angle - angle_zeropoint) + LQR_k2*LQR_gyro + LQR_k3*(LQR_distance - distance_zeropoint) + LQR_k4*LQR_speed;

  // Negative value is given because with current wiring, positive torque moves it backwards
  LQR_distance  = (-0.5) *(motor1.shaft_angle + motor2.shaft_angle);
  LQR_speed     = (-0.5) *(motor1.shaft_velocity + motor2.shaft_velocity);
  LQR_angle = (float)mpu6050.getAngleY();
  LQR_gyro  = (float)mpu6050.getGyroY(); 
  //Serial.println(LQR_distance); 

  // Calculate self-balancing output
  angle_control     = pid_angle(LQR_angle - angle_zeropoint);
  gyro_control      = pid_gyro(LQR_gyro);

  // Motion detail optimization processing
  if(wrobot.joyy != 0) // Processing when there is forward/backward motion command
  {
    distance_zeropoint = LQR_distance; // Displacement zero-point reset
    pid_lqr_u.error_prev = 0;         // Clear output integral
  }

  if( (wrobot.joyx_last!=0 && wrobot.joyx==0) || (wrobot.joyy_last!=0 && wrobot.joyy==0) ) // Spot parking processing when motion command resets to zero
  {
    wrobot_move_stop_flag = 1;
  }
  if( (wrobot_move_stop_flag==1) && (abs(LQR_speed)<0.5) )
  {
    distance_zeropoint = LQR_distance; // Displacement zero-point reset
    wrobot_move_stop_flag = 0;
  }

  if( abs(LQR_speed)>15 ) // Spot parking processing when pushed quickly
  {
    distance_zeropoint = LQR_distance; // Displacement zero-point reset
  }

  // Calculate displacement control output
  distance_control  = pid_distance(LQR_distance - distance_zeropoint);
  speed_control     = pid_speed(LQR_speed- 0.1*lpf_joyy(wrobot.joyy) );

  // Wheel lift-off detection
  robot_speed_last = robot_speed; // Record two consecutive wheel speeds
  robot_speed = LQR_speed;
  if( abs(robot_speed-robot_speed_last) > 10 || abs(robot_speed) > 50 || (jump_flag != 0))  // If wheel angular velocity/acceleration is too large or in recovery period after jump, assume wheel lift-off and require special handling
  {
    distance_zeropoint = LQR_distance;    // Displacement zero-point reset
    LQR_u = angle_control + gyro_control; // In case of wheel lift-off, do not output wheel component; otherwise, output full balance torque in normal state
    pid_lqr_u.error_prev = 0; // Clear output integral
  }
  else
  {
    LQR_u = angle_control + gyro_control + distance_control + speed_control; 
  }
  
  // Trigger conditions: No RC signal input, wheel displacement control intervenes normally, not in post-jump recovery period
  if( abs(LQR_u)<5 && wrobot.joyy == 0 && abs(distance_control)<4 && (jump_flag == 0))
  {
    
    LQR_u = pid_lqr_u(LQR_u); // Compensate for small torque nonlinearity
    //Serial.println(LQR_u);
    angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control)); // Center of gravity adaptation
  }
  else
  {
    pid_lqr_u.error_prev = 0; // Clear output integral
  }

  // Balance control parameter adaptation
  if(wrobot.height < 50)
  {
    pid_speed.P = 0.7;
  }
  else if(wrobot.height < 64)
  {
    pid_speed.P = 0.6;
  }
  else
  {
    pid_speed.P = 0.5;
  }
}

// Leg motion control
void leg_loop(){

  jump_loop(); // Descomentar se jump_loop estiver ativo
  
  if(jump_flag == 0) // Not in jump state
  {
    // Body height adaptive control
    float roll_angle  = (float)mpu6050.getAngleX() + 2.0;
    
    // Calcula o ajuste do PID (assumindo que as funções PID e LPF existem)
    float current_leg_add = pid_roll_angle(lpf_roll(roll_angle));

    // --- CÁLCULO ORIGINAL ---
    // Mantemos a matemática original para preservar o comportamento do PID de equilíbrio
    long targetPosR = 2048 + 12 + 8.4*(wrobot.height-32) - current_leg_add;
    long targetPosL = 2048 - 12 - 8.4*(wrobot.height-32) - current_leg_add;

    // --- LIMITES DE SOFTWARE (Lógica STS) ---
    if( targetPosR < 2110 ) targetPosR = 2110;
    else if( targetPosR > 2510 ) targetPosR = 2510;

    if( targetPosL < 1586 ) targetPosL = 1586;
    else if( targetPosL > 1986 ) targetPosL = 1986;

    // --- MOVIMENTO ---
    // Passamos o valor direto para a Direita
    moveServoRestricted(SERVO_R, targetPosR);
    
    // TRUQUE MATEMÁTICO PARA A ESQUERDA:
    // O valor da esquerda diminui quando agacha. Para o "map" funcionar igual nas duas pernas,
    // invertemos o valor logicamente aqui: (4096 - valor).
    // Assim, 1796 vira 2300, e o cálculo de ângulo fica idêntico para os dois lados.
    moveServoRestricted(SERVO_L, 4096 - targetPosL); 
  }

  /*jump_loop();
  if(jump_flag == 0) // Not in jump state
  {
    // Body height adaptive control
    ACC[0] = 8;
    ACC[1] = 8;
    Speed[0] = 200;
    Speed[1] = 200;
    float roll_angle  = (float)mpu6050.getAngleX() + 2.0;
    //leg_position_add += pid_roll_angle(roll_angle);
    leg_position_add = pid_roll_angle(lpf_roll(roll_angle));//test
    Position[0] = 2048 + 12 + 8.4*(wrobot.height-32) - leg_position_add;
    Position[1] = 2048 - 12 - 8.4*(wrobot.height-32) - leg_position_add;
    if( Position[0]<2110 )
      Position[0]=2110;
    else if( Position[0]>2510 )
      Position[0]=2510;
    if( Position[1]<1586 )
      Position[1]=1586;
    else if( Position[1]>1986 )
      Position[1]=1986;
    sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
  }*/ 
}

// Jump control
void jump_loop(){
  if( (wrobot.dir_last == 5) && (wrobot.dir == 4) && (jump_flag == 0) )
  {
      long jumpPosR = 2048 + 12 + 8.4*(80-32);
      long jumpPosL = 2048 - 12 - 8.4*(80-32); // Valor baixo (ex: 1600)

      moveServoRestricted(SERVO_R, jumpPosR);
      moveServoRestricted(SERVO_L, 4096 - jumpPosL); // Inverte para virar valor alto (ex: 2496)

      jump_flag = 1;
  }
  
  if( jump_flag > 0 )
  {
    jump_flag++;
    
    if( (jump_flag > 30) && (jump_flag < 35) )
    {
      long landPosR = 2048 + 12 + 8.4*(40-32);
      long landPosL = 2048 - 12 - 8.4*(40-32);

      moveServoRestricted(SERVO_R, landPosR);
      moveServoRestricted(SERVO_L, 4096 - landPosL);

      jump_flag = 40;
    }
    
    if(jump_flag > 200)
    {
      jump_flag = 0; 
    }
  }

  /*if( (wrobot.dir_last == 5) && (wrobot.dir == 4) && (jump_flag == 0) )
  {
      ACC[0] = 0;
      ACC[1] = 0;
      Speed[0] = 0;
      Speed[1] = 0;
      Position[0] = 2048 + 12 + 8.4*(80-32);
      Position[1] = 2048 - 12 - 8.4*(80-32);
      sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

      jump_flag = 1;
  }
  if( jump_flag > 0 )
  {
    jump_flag++;
    if( (jump_flag > 30) && (jump_flag < 35) )
    {
      ACC[0] = 0;
      ACC[1] = 0;
      Speed[0] = 0;
      Speed[1] = 0;
      Position[0] = 2048 + 12 + 8.4*(40-32);
      Position[1] = 2048 - 12 - 8.4*(40-32);
      sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

      jump_flag = 40;
    }
    if(jump_flag > 200)
    {
      jump_flag = 0; // Ready to jump again
    }
  }*/
}

// Nova função auxiliar ajustada
void moveServoRestricted(uint8_t channel, long stsValNormalized) {
  // 1. Mapeamento Lógico:
  // Agora recebemos sempre valores "positivos" (acima de 2048) graças à inversão feita na chamada.
  // 2048 é o zero (em pé), 2600 é o máximo (agachado/45 graus).
  int angle = map(stsValNormalized, 2048, 2600, 0, 45); // Ajustei o range para começar do centro exato

  // 2. Trava de Segurança
  angle = constrain(angle, 0, 45);

  // 3. Conversão para Pulso PWM (PCA9685)
  int pulse;

  if(channel == SERVO_R) {
    // Perna Direita: 0 graus = SERVOMIN, 45 graus = SERVOMAX (ou proporcional)
    pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  } else {
    // Perna Esquerda (Montada espelhada):
    // 0 graus físicos deve ser 180 graus no motor (SERVOMAX)
    // 45 graus físicos deve ser 135 graus no motor
    // Portanto, invertemos o ângulo no mapeamento PWM
    pulse = map(170 - angle, 0, 180, SERVOMIN, SERVOMAX);
  }

  board.setPWM(channel, 0, pulse);
}

// Yaw axis steering control
void yaw_loop(){
  //YAW_output = 0.03*(YAW_Kp*YAW_angle_total + YAW_Kd*YAW_gyro);
  yaw_angle_addup();
  
  YAW_angle_total += wrobot.joyx*0.002;
  float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
  float yaw_gyro_control  = pid_yaw_gyro(YAW_gyro);
  YAW_output = yaw_angle_control + yaw_gyro_control;  
}

// Web data update
void web_loop(){
  webserver.handleClient();
  websocket.loop();
  rp.spinOnce(); // Update control info returned from web
}

// Yaw axis angle accumulation function
void yaw_angle_addup() {
  YAW_angle  = (float)mpu6050.getAngleZ();;
  YAW_gyro   = (float)mpu6050.getGyroZ();

  if(YAW_angle_zero_point == (-10))
  {
    YAW_angle_zero_point = YAW_angle;
  }

  float yaw_angle_1,yaw_angle_2,yaw_addup_angle;
  if(YAW_angle > YAW_angle_last)
  {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last - 2*PI;
  }
  else
  {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last + 2*PI;
  }

  if(abs(yaw_angle_1)>abs(yaw_angle_2))
  {
    yaw_addup_angle=yaw_angle_2;
  }
  else
  {
    yaw_addup_angle=yaw_angle_1;
  }

  YAW_angle_total = YAW_angle_total + yaw_addup_angle;
  YAW_angle_last = YAW_angle;
}

void basicWebCallback(void)
{
  webserver.send(300, "text/html", basic_web);
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  if(type == WStype_TEXT)
  {
    String payload_str = String((char*) payload);   
    StaticJsonDocument<300> doc;  
    DeserializationError error = deserializeJson(doc, payload_str);

    String mode_str = doc["mode"];
    if(mode_str == "basic")
    {
      rp.parseBasic(doc);
    }
  }
}

// Voltage detection initialization
/*void adc_calibration_init()
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}*/

// Voltage detection
/*void bat_check()
{
  if(bat_check_num > 1000)
  {
    // Voltage reading
    uint32_t sum = 0;
    sum= analogRead(BAT_PIN);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(sum, &adc_chars);
    double battery=(voltage*3.97)/1000.0;

    Serial.println(battery);
    // Battery level display
    if(battery>7.8)
      digitalWrite(LED_BAT,HIGH);
    else
      digitalWrite(LED_BAT,LOW);

    bat_check_num = 0;
  }
  else
    bat_check_num++;
}*/