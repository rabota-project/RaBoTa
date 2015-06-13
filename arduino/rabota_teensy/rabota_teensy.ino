// connect motor controller pins to Arduino digital pins
// motor one
#define m1_EN_PIN     0   // digital pin
#define m1_IN1_PIN    1   // digital pin
#define m1_IN2_PIN    2   // digital pin
#define m1_D2_PIN     23  // pwm pin
#define m1_enc_a_PIN  3   // interupt pin
#define m1_enc_b_PIN  4   // interupt pin

// motor two
#define m2_EN_PIN     5   // digital pin
#define m2_IN1_PIN    6   // digital pin
#define m2_IN2_PIN    7   // digital pin
#define m2_D2_PIN     22  // pwm pin
#define m2_enc_a_PIN  11  // interupt pin
#define m2_enc_b_PIN  12  // interupt pin

// motor 3    -  conected to the L293 board
#define m3_IN1_PIN    13  // digital pin
#define m3_IN2_PIN    14  // digital pin
#define m3_D2_PIN     21  // pwm pin
#define m3_enc_a_PIN  15  // interupt pin
#define m3_enc_b_PIN  16  // interupt pin

// motor 4    -  conected to the L293 board
#define m4_IN1_PIN    24  // digital pin
#define m4_IN2_PIN    19  // digital pin
#define m4_D2_PIN     20  // pwm pin
#define m4_enc_a_PIN  17  // interupt pin
#define m4_enc_b_PIN  18   // interupt pin

// motor 5    -  conected to the L293 board
#define m5_IN1_PIN    25  // digital pin
#define m5_IN2_PIN    26  // digital pin
#define m5_D2_PIN     9   // pwm pin
#define m5_enc_a_PIN  27  // interupt pin
#define m5_enc_b_PIN  28  // interupt pin

// motor 6    -  conected to the L293 board
#define m6_IN1_PIN    29  // digital pin
#define m6_IN2_PIN    30  // digital pin
#define m6_D2_PIN     10  // pwm pin
#define m6_enc_a_PIN  31  // interupt pin
#define m6_enc_b_PIN  32  // interupt pin

// joint 5 micro switch
#define m5_MS         8

#include <Encoder.h>
#include <PID_v1.h>
#include <Bounce.h>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

const int number_of_motors=5;
//gear ratios
const double g1 = 2*3.14159/20/4/127.1/5;
const double g2 = 2*3.14159/20/4/127.1/4;
const double g3 = 2*3.14159/20/4/127.1/4;
const double g4 = 2*3.14159/20/4/65.5/(23.0/12.0);
const double g5 = 2*3.14159/20/4/65.5/(23.0/12.0);


Bounce pushbutton = Bounce(m5_MS, 2);  // 10 ms debounce


ros::NodeHandle nh;

Encoder m1_Enc(m1_enc_a_PIN,m1_enc_b_PIN);
Encoder m2_Enc(m2_enc_a_PIN,m2_enc_b_PIN);
Encoder m3_Enc(m3_enc_a_PIN,m3_enc_b_PIN);
Encoder m4_Enc(m4_enc_a_PIN,m4_enc_b_PIN);
Encoder m5_Enc(m5_enc_a_PIN,m5_enc_b_PIN);

//Define Variables we'll be connecting to
double m1_Setpoint, m1_Input, m1_Output;
double m2_Setpoint, m2_Input, m2_Output;
double m3_Setpoint, m3_Input, m3_Output;
double m4_Setpoint, m4_Input, m4_Output;
double m5_Setpoint, m5_Input, m5_Output;

//Specify the PID and initial tuning parameters
PID m1_PID(&m1_Input, &m1_Output, &m1_Setpoint,5,0,0, DIRECT); // pid was 4255 0 0 
PID m2_PID(&m2_Input, &m2_Output, &m2_Setpoint,5,0,0, DIRECT);
PID m3_PID(&m3_Input, &m3_Output, &m3_Setpoint,5,0,0, DIRECT);
PID m4_PID(&m4_Input, &m4_Output, &m4_Setpoint,5,0,0, DIRECT);
PID m5_PID(&m5_Input, &m5_Output, &m5_Setpoint,5,0,0, DIRECT);

void messageCb( const std_msgs::Float64MultiArray& joint_command_msg){
  m1_Setpoint = (1.0/g1) * ( joint_command_msg.data[0] );
  m2_Setpoint = (1.0/g2) * ( joint_command_msg.data[1] );
  m3_Setpoint = (1.0/g3) * ( joint_command_msg.data[1] + joint_command_msg.data[2] );
  m4_Setpoint = (1.0/g4) * ( joint_command_msg.data[1] + joint_command_msg.data[2] + joint_command_msg.data[3] + joint_command_msg.data[4] );
  m5_Setpoint = (1.0/g5) * ( joint_command_msg.data[1] + joint_command_msg.data[2] + joint_command_msg.data[3] - joint_command_msg.data[4] );
}
ros::Subscriber<std_msgs::Float64MultiArray> sub("rabota/arduino/motors_command", &messageCb );

float pos[number_of_motors]; 
float vel[number_of_motors];
float eff[number_of_motors];

std_msgs::Float64MultiArray motors_state;
ros::Publisher motors_state_pub("rabota/arduino/motors_state", &motors_state);


void setup()
{
  pinMode(m5_MS, INPUT_PULLUP);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(motors_state_pub);

  motors_state.data_length = number_of_motors;
  motors_state.data = pos;
  
  m1_Input = 0;
  m1_Setpoint = 0;//2;
  m1_PID.SetMode(AUTOMATIC);
  m1_PID.SetSampleTime(15);
  m1_PID.SetOutputLimits(-255,255);
  
  m2_Input = 0;
  m2_Setpoint = 0;
  m2_PID.SetMode(AUTOMATIC);
  m2_PID.SetSampleTime(15);
  m2_PID.SetOutputLimits(-255,255);
   
  m3_Input = 0;
  m3_Setpoint = 0; 
  m3_PID.SetMode(AUTOMATIC);
  m3_PID.SetSampleTime(15);
  m3_PID.SetOutputLimits(-255,255);
     
  m4_Input = 0; 
  m4_Setpoint = 0;
  m4_PID.SetMode(AUTOMATIC);
  m4_PID.SetSampleTime(15);
  m4_PID.SetOutputLimits(-255,255);
       
  m5_Input = 0;
  m5_Setpoint = 0;
  m5_PID.SetMode(AUTOMATIC);
  m5_PID.SetSampleTime(15);
  m5_PID.SetOutputLimits(-255,255);
  
  // set all the motor control pins to outputs
  // motor 1
  pinMode(m1_EN_PIN, OUTPUT);
  pinMode(m1_IN1_PIN, OUTPUT);
  pinMode(m1_IN2_PIN, OUTPUT);
  pinMode(m1_D2_PIN, OUTPUT);
  // motor 2
  pinMode(m2_EN_PIN, OUTPUT);
  pinMode(m2_IN1_PIN, OUTPUT);
  pinMode(m2_IN2_PIN, OUTPUT);
  pinMode(m2_D2_PIN, OUTPUT);
  // motor 3
  pinMode(m3_IN1_PIN, OUTPUT);
  pinMode(m3_IN2_PIN, OUTPUT);
  pinMode(m3_D2_PIN, OUTPUT);
  // motor 4
  pinMode(m4_IN1_PIN, OUTPUT);
  pinMode(m4_IN2_PIN, OUTPUT);
  pinMode(m4_D2_PIN, OUTPUT);
  // motor 5
  pinMode(m5_IN1_PIN, OUTPUT);
  pinMode(m5_IN2_PIN, OUTPUT);
  pinMode(m5_D2_PIN, OUTPUT);
    
  digitalWrite(m1_EN_PIN, HIGH);
  digitalWrite(m1_IN1_PIN, LOW);
  digitalWrite(m1_IN2_PIN, LOW);
    
  digitalWrite(m2_EN_PIN, HIGH);
  digitalWrite(m2_IN1_PIN, LOW);
  digitalWrite(m2_IN2_PIN, LOW);
      
  digitalWrite(m3_IN1_PIN, LOW);
  digitalWrite(m3_IN2_PIN, LOW);
  
  digitalWrite(m4_IN1_PIN, LOW);
  digitalWrite(m4_IN2_PIN, LOW);
    
  digitalWrite(m5_IN1_PIN, LOW);
  digitalWrite(m5_IN2_PIN, LOW);
  
  //zero_joint_45();
  zero_joint_1();
  zero_joint_2();
  zero_joint_3();
  //zero_joint_4();

  Serial.begin (9600);
}

void loop()
{
  nh.spinOnce();

  m1_Input = m1_Enc.read(); 
  m1_PID.Compute();

  m2_Input = m2_Enc.read(); 
  m2_PID.Compute();

  m3_Input = m3_Enc.read(); 
  m3_PID.Compute();

  m4_Input = m4_Enc.read();
  m4_PID.Compute();

  m5_Input = m5_Enc.read();
  m5_PID.Compute();

  pos[0] = g1 * m1_Input;
  pos[1] = g2 * m2_Input;
  pos[2] = -g2*m2_Input + g3*m3_Input;
  pos[3] = -g3*m3_Input + ( g4*m4_Input + g5*m5_Input)/2;
  pos[4] = (g4*m4_Input - g5*m5_Input)/2;
  
  motors_state.data = pos;
  
  //Serial.print("motor 1: Setpoint=");  Serial.print(m1_Setpoint); Serial.print("     Input=");  Serial.print(m1_Input);  Serial.print("     Output=");  Serial.println(m1_Output); //angles
  //Serial.print("motor 2: Setpoint=");  Serial.print(m2_Setpoint); Serial.print("     Input=");  Serial.print(m2_Input);  Serial.print("     Output=");  Serial.println(m2_Output); //angles
  //Serial.print("motor 3: Setpoint=");  Serial.print(m3_Setpoint); Serial.print("     Input=");  Serial.print(m3_Input);  Serial.print("     Output=");  Serial.println(m3_Output); //angles
  //Serial.print("motor 4: Setpoint=");  Serial.print(m4_Setpoint); Serial.print("     Input=");  Serial.print(m4_Input);  Serial.print("     Output=");  Serial.println(m4_Output); //angles
  //Serial.print("motor 5: Setpoint=");  Serial.print(m5_Setpoint); Serial.print("     Input=");  Serial.print(m5_Input);  Serial.print("     Output=");  Serial.println(m5_Output); //angles
  //Serial.print("Motors Input= [ ");  Serial.print(m1_Input);   Serial.print("  ");  Serial.print(m2_Input);  Serial.print("  ");  Serial.print(m3_Input);  Serial.print("  ");  Serial.print(m4_Input);  Serial.print("  ");  Serial.print(m5_Input);  Serial.println("]");  
  
  command_motor_1(m1_Output);  
  command_motor_2(m2_Output);
  command_motor_3(m3_Output);  
  command_motor_4(m4_Output);
  command_motor_5(m5_Output);  

  motors_state_pub.publish( &motors_state);
  delay(5);
}
//-----------------------------------------------------------------FUNCTIONS
void command_motor_1(double m1_output){
  if (m1_output<0) {
      digitalWrite(m1_EN_PIN, HIGH);
      digitalWrite(m1_IN1_PIN, LOW);
      digitalWrite(m1_IN2_PIN, HIGH);
      analogWrite(m1_D2_PIN, -m1_output);
  }
  else {
      digitalWrite(m1_EN_PIN, HIGH);
      digitalWrite(m1_IN1_PIN,HIGH );
      digitalWrite(m1_IN2_PIN, LOW);
      analogWrite(m1_D2_PIN, m1_output);
  }
}

void command_motor_2(double m2_output){
  if (m2_output<0) {
      digitalWrite(m2_IN1_PIN, LOW);
      digitalWrite(m2_IN2_PIN, HIGH);
      analogWrite(m2_D2_PIN, -m2_output);
  }
  else {
      digitalWrite(m2_IN1_PIN,HIGH );
      digitalWrite(m2_IN2_PIN, LOW);  
      analogWrite(m2_D2_PIN, m2_output);
  }
}

void command_motor_3(double m3_output){
  if (m3_output<0) {
      digitalWrite(m3_IN1_PIN, LOW);
      digitalWrite(m3_IN2_PIN, HIGH);
      analogWrite(m3_D2_PIN, -m3_output);
  }
  else {
      digitalWrite(m3_IN1_PIN,HIGH );
      digitalWrite(m3_IN2_PIN, LOW);  
      analogWrite(m3_D2_PIN, m3_output);
  }
}

void command_motor_4(double m4_output){
  if (m4_output<0) {
      digitalWrite(m4_IN1_PIN, LOW);
      digitalWrite(m4_IN2_PIN, HIGH);
      analogWrite(m4_D2_PIN, -m4_output);
  }
  else {
      digitalWrite(m4_IN1_PIN,HIGH );
      digitalWrite(m4_IN2_PIN, LOW);  
      analogWrite(m4_D2_PIN, m4_output);
  }
}

void command_motor_5(double m5_output){
  if (m5_output<0) {
      digitalWrite(m5_IN1_PIN, LOW);
      digitalWrite(m5_IN2_PIN, HIGH);
      analogWrite(m5_D2_PIN, -m5_output);
  }
  else {
      digitalWrite(m5_IN1_PIN,HIGH );
      digitalWrite(m5_IN2_PIN, LOW);  
      analogWrite(m5_D2_PIN, m5_output);
  }
}

void zero_joint_1() 
{
  command_motor_1(250);
  int diff = m1_Enc.read();
  delay(200);
  while (diff!=m1_Enc.read()) {
    diff = m1_Enc.read();
    Serial.println(diff);
    delay(5);
  }
  //command_motor_1(0);
  m1_Enc.write(21889);
}

void zero_joint_2() 
{
  command_motor_2(-250);
  int diff = m2_Enc.read();
  delay(200);
  while (diff!=m2_Enc.read()) {
    diff = m2_Enc.read();
    Serial.println(diff);
    delay(5);
  }
  //command_motor_2(0);
  m2_Enc.write(-4100);
}

void zero_joint_3() 
{
  command_motor_3(-250);
  int diff = m3_Enc.read();
  delay(400);
  while (diff!=m3_Enc.read()) {
    diff = m3_Enc.read();
    Serial.println(diff);
    delay(5);
  }
  //command_motor_3(0);
  m3_Enc.write(-15822);
}

void zero_joint_45() 
{
  // zero's joint 5
  command_motor_4(-250);
  command_motor_5(250);
  while (1) {                             
    if (pushbutton.update()) {
      if (pushbutton.fallingEdge()) {
       command_motor_4(0);
       command_motor_5(0);
       m4_Enc.write(  837 );
       m5_Enc.write( -837 );
       break;
      }
    }
  }
  
  // zero's joint 4
  int diff4 = m4_Enc.read();
  int diff5 = m5_Enc.read();
  command_motor_4(250);
  command_motor_5(250);
  delay(200);
  while ( (diff4!=m4_Enc.read()) && (diff5!=m5_Enc.read()) ) {
    diff4 = m4_Enc.read();
    diff5 = m5_Enc.read();
    delay(5);
  }
  command_motor_4(0);
  command_motor_5(0);
  m4_Enc.write( -650 );
  m5_Enc.write( -650 );  
}



