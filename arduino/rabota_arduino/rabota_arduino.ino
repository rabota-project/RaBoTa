
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
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

Encoder m1_Enc(3, 2);
Encoder m2_Enc(18, 19);

//Define Variables we'll be connecting to
double m1_Setpoint, m1_Input, m1_Output;
double m2_Setpoint, m2_Input, m2_Output;

//Specify the links and initial tuning parameters
PID m1_PID(&m1_Input, &m1_Output, &m1_Setpoint,200,0,1, DIRECT);
PID m2_PID(&m2_Input, &m2_Output, &m2_Setpoint,200,0,1, DIRECT);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

