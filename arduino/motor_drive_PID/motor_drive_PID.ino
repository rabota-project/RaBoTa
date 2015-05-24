// connect motor controller pins to Arduino digital pins
// motor one
#define m1_EN_PIN   51 //4
#define m1_IN1_PIN  52 //5
#define m1_IN2_PIN  53 //6
#define m1_D2_PIN   9

// motor two
#define m2_EN_PIN   45 
#define m2_IN1_PIN  46
#define m2_IN2_PIN  47
#define m2_D2_PIN   8

// motor 3    -  conected to the L293 board
#define m3_IN1_PIN  40
#define m3_IN2_PIN  41
#define m3_D2_PIN   7


#include <Encoder.h>
#include <PID_v1.h>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

const int number_of_motors=3;

ros::NodeHandle nh;

Encoder m1_Enc(3,2);
Encoder m2_Enc(18,19);
Encoder m3_Enc(20,21);

//Define Variables we'll be connecting to
double m1_Setpoint, m1_Input, m1_Output;
double m2_Setpoint, m2_Input, m2_Output;
double m3_Setpoint, m3_Input, m3_Output;

//Specify the PID and initial tuning parameters
PID m1_PID(&m1_Input, &m1_Output, &m1_Setpoint,255,0,0, DIRECT);
PID m2_PID(&m2_Input, &m2_Output, &m2_Setpoint,255,0,0, DIRECT);
PID m3_PID(&m3_Input, &m3_Output, &m3_Setpoint,255,0,0, DIRECT);

// define the commands subscriber
void messageCb( const std_msgs::Float64MultiArray& motor_command_msg){
  m1_Setpoint = motor_command_msg.data[0];
  m2_Setpoint = motor_command_msg.data[1];
  m3_Setpoint = motor_command_msg.data[2];
}
ros::Subscriber<std_msgs::Float64MultiArray> sub("rabota/arduino/motors_command", &messageCb );

float pos[number_of_motors]; 
float vel[number_of_motors];
float eff[number_of_motors];

std_msgs::Float64MultiArray motors_state;
ros::Publisher motors_state_pub("rabota/arduino/motors_state", &motors_state);


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(motors_state_pub);

  motors_state.data_length = number_of_motors;
  motors_state.data = pos;
  
  //initialize the variables we're linked to
  m1_Input = m1_Enc.read()*360/2554;
  m1_Setpoint = 0;
  //turn the PID on
  m1_PID.SetMode(AUTOMATIC);
  m1_PID.SetSampleTime(15);
  m1_PID.SetOutputLimits(-255,255);
  
  m2_Input = m2_Enc.read()*360/7600;
  m2_Setpoint = 0;
  m2_PID.SetMode(AUTOMATIC);
  m2_PID.SetSampleTime(15);
  m2_PID.SetOutputLimits(-255,255);
   
  m3_Input = m3_Enc.read()*360/7600;
  m3_Setpoint = 0;
  m3_PID.SetMode(AUTOMATIC);
  m3_PID.SetSampleTime(15);
  m3_PID.SetOutputLimits(-255,255);
  
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
  pinMode(m2_EN_PIN, OUTPUT);
  pinMode(m2_IN1_PIN, OUTPUT);
  pinMode(m2_IN2_PIN, OUTPUT);
  pinMode(m2_D2_PIN, OUTPUT);
    
  digitalWrite(m1_EN_PIN, HIGH);
  digitalWrite(m1_IN1_PIN, LOW);
  digitalWrite(m1_IN2_PIN, LOW);
    
  digitalWrite(m2_EN_PIN, HIGH);
  digitalWrite(m2_IN1_PIN, LOW);
  digitalWrite(m2_IN2_PIN, LOW);
      
  //digitalWrite(m3_EN_PIN, HIGH);
  digitalWrite(m3_IN1_PIN, LOW);
  digitalWrite(m3_IN2_PIN, LOW);
  
 // Serial.begin (9600);
}

void loop()
{
  nh.spinOnce();

  m1_Input = m1_Enc.read()*2*3.14159/2554; //rad
  m1_PID.Compute();

  m2_Input = m2_Enc.read()*2*3.14159/2554; //rad
  m2_PID.Compute();

  m3_Input = m3_Enc.read()*2*3.14159/2554; //rad
  m3_PID.Compute();

  pos[0] = double(m1_Input);
  pos[1] = double(m2_Input);
  pos[2] = double(m3_Input);
  
  eff[0] = m1_Output;
  eff[1] = m2_Output;
  eff[2] = m3_Output;
  
  motors_state.data = pos;
  
  //Serial.print("motor 1: Setpoint=");  Serial.print(m1_Setpoint); Serial.print("     Input=");  Serial.print(m1_Input);  Serial.print("     Output=");  Serial.println(m1_Output); //angles
  //Serial.print("motor 2: Setpoint=");  Serial.print(m2_Setpoint); Serial.print("     Input=");  Serial.print(m2_Input);  Serial.print("     Output=");  Serial.println(m2_Output); //angles
  //Serial.print("motor 3: Setpoint=");  Serial.print(m3_Setpoint); Serial.print("     Input=");  Serial.print(m3_Input);  Serial.print("     Output=");  Serial.println(m3_Output); //angles
  
  //-----------------------------------------------motor 1
  if (m1_Output<0) {
      digitalWrite(m1_EN_PIN, HIGH);
      digitalWrite(m1_IN1_PIN, LOW);
      digitalWrite(m1_IN2_PIN, HIGH);
      analogWrite(m1_D2_PIN, -m1_Output);
  }
  else {
      digitalWrite(m1_EN_PIN, HIGH);
      digitalWrite(m1_IN1_PIN,HIGH );
      digitalWrite(m1_IN2_PIN, LOW);
      analogWrite(m1_D2_PIN, m1_Output);
  }
  
  //-----------------------------------------------motor 2
  if (m2_Output<0) {
      digitalWrite(m2_IN1_PIN, LOW);
      digitalWrite(m2_IN2_PIN, HIGH);
      analogWrite(m2_D2_PIN, -m2_Output);
  }
  else {
      digitalWrite(m2_IN1_PIN,HIGH );
      digitalWrite(m2_IN2_PIN, LOW);  
      analogWrite(m2_D2_PIN, m2_Output);
  }
  
  //-------------------------------------------------motor 3
  if (m3_Output<0) {
      digitalWrite(m3_IN1_PIN, LOW);
      digitalWrite(m3_IN2_PIN, HIGH);
      analogWrite(m3_D2_PIN, -m3_Output);
  }
  else {
      digitalWrite(m3_IN1_PIN,HIGH );
      digitalWrite(m3_IN2_PIN, LOW);  
      analogWrite(m3_D2_PIN, m3_Output);
  }
  
  //-------------------------------------------------
  //pub.publish( &robot_state );
  motors_state_pub.publish( &motors_state);
  delay(5);
}
