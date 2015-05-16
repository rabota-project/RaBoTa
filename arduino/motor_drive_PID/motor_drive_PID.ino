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


#include <Encoder.h>
#include <PID_v1.h>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
ros::NodeHandle nh;

Encoder m1_Enc(3, 2);
Encoder m2_Enc(18, 19);

//Define Variables we'll be connecting to
double m1_Setpoint, m1_Input, m1_Output;
double m2_Setpoint, m2_Input, m2_Output;

//Specify the links and initial tuning parameters
PID m1_PID(&m1_Input, &m1_Output, &m1_Setpoint,200,0,0, DIRECT);
PID m2_PID(&m2_Input, &m2_Output, &m2_Setpoint,150,0,0, DIRECT);

void messageCb( const std_msgs::Float64MultiArray& motor_command_msg){
  m1_Setpoint = motor_command_msg.data[0];
  m2_Setpoint = motor_command_msg.data[1];
  
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("arduino/motor_command", &messageCb );

sensor_msgs::JointState robot_state;
char *a[] = {"motor_1","motor_2"}; 
float pos[2]; 
float vel[2];
float eff[2];

ros::Publisher pub("arduino/motor_states", &robot_state);

//std_msgs::Float64MultiArray check_data;
//ros::Publisher check_pub("arduino/check_data", &check_data);


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
//  nh.advertise(check_pub);

  robot_state.header.frame_id = "motors";
  robot_state.name_length = 2;
  robot_state.velocity_length = 2;
  robot_state.position_length = 2; /// here used for arduino time
  robot_state.effort_length = 2; /// here used for arduino time

  robot_state.name = a;
  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;

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
    
  digitalWrite(m1_EN_PIN, HIGH);
  digitalWrite(m1_IN1_PIN, LOW);
  digitalWrite(m1_IN2_PIN, LOW);
    
  digitalWrite(m2_EN_PIN, HIGH);
  digitalWrite(m2_IN1_PIN, LOW);
  digitalWrite(m2_IN2_PIN, LOW);
  
  //Serial.begin (9600);
}

void loop()
{
  nh.spinOnce();

  m1_Input = m1_Enc.read()*2*3.14159/2554; //rad
  m1_PID.Compute();

  m2_Input = m2_Enc.read()*2*3.14159/2554; //rad
  m2_PID.Compute();

  pos[0] = double(m1_Input);
  pos[1] = double(m2_Input);
  eff[0] = m1_Output;
  eff[1] = m2_Output;
  
  robot_state.header.stamp = nh.now();
  robot_state.position = pos;
  robot_state.effort = eff;

//  check_data.data[0] = m1_Output;

  //Serial.print("motor 1: Setpoint=");  Serial.print(m1_Setpoint); Serial.print("     Input=");  Serial.print(m1_Input);  Serial.print("     Output=");  Serial.println(m1_Output); //angles
  //Serial.print("motor 2: Setpoint=");  Serial.print(m2_Setpoint); Serial.print("     Input=");  Serial.print(m2_Input);  Serial.print("     Output=");  Serial.println(m2_Output); //angles
  
  //motor 1
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
  
  //motor 2
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
  
  pub.publish( &robot_state );
//  check_pub.publish( &check_data );
  delay(5);
}
