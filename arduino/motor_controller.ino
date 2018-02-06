#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <Servo.h>
//////////////////////////////////////////////////////////////////////////////////////
//Motor Pin definition
//Left Motor
#define USE_USBCOM
#define MOTOR_L_1 12
#define MOTOR_L_2 9

//PWM 1 pin 
#define ENA_PIN_1   3


//Right Motor
#define MOTOR_R_1 13 
#define MOTOR_R_2 8

//PWM 2 pin 
#define ENB_PIN_2   11
//#define RESET_PIN 4

//encoder pins
int left_encoder0PinA = 4;
 int left_encoder0PinB = 5;

 int lwheel = 0;
 int left_encoder0PinALast = LOW;
 int n = LOW;

//right encoder
int right_encoder0PinA = 6;
 int right_encoder0PinB = 7;

 int rwheel = 0;
 int right_encoder0PinALast = LOW;
 int m = LOW;


float motor_left_speed = 0;
float motor_right_speed = 0;

ros::NodeHandle  nh;

void left( const std_msgs::Float32& msg1){
   motor_left_speed = msg1.data;
}

void right( const std_msgs::Float32& msg2){
   motor_right_speed = msg2.data;
}


ros::Subscriber<std_msgs::Float32> sub1("left_wheel_speed", left); 
ros::Subscriber<std_msgs::Float32> sub2("right_wheel_speed", right); 

//left encoder
std_msgs::Int64 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);   //self._Left_Encoder = rospy.Publisher('rwheel',Int64,queue_size = 10)

//right encoder
std_msgs::Int64 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);   //self._Right_Encoder = rospy.Publisher('rwheel',Int64,queue_size = 10)


void setup()
{
  
  nh.initNode();
  nh.subscribe(sub1);
nh.subscribe(sub2);
  //Init Serial port with 115200 baud rate
  Serial.begin(57600);  
  
  
  //Setup Motors
  SetupMotors();
 

//left encoder
 pinMode (left_encoder0PinA,INPUT);
   pinMode (left_encoder0PinB,INPUT);
  nh.initNode();
  nh.advertise(lwheelPub);
    
//right encoder
pinMode (right_encoder0PinA,INPUT);
   pinMode (right_encoder0PinB,INPUT);
  nh.initNode();
  nh.advertise(rwheelPub);

 }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Motors() function

void SetupMotors()
{
 
 //Left motor
 pinMode(MOTOR_L_1, OUTPUT);
 pinMode(MOTOR_L_2, OUTPUT);
 

 //Right Motor
    pinMode(MOTOR_R_1, OUTPUT);
    pinMode(MOTOR_R_2, OUTPUT);  
  
}


void loop()
{

    nh.spinOnce();
    
    //Update motor values with corresponding speed and send speed values through serial port
    Update_Motors();


//left encoder 
 n = digitalRead(left_encoder0PinA);
   if ((left_encoder0PinALast == LOW) && (n == HIGH)) {
     if (digitalRead(left_encoder0PinB) == LOW) {
       lwheel--;
     } else {
       lwheel++;
     }
lwheelMsg.data = lwheel;
lwheelPub.publish(&lwheelMsg);
   }
   left_encoder0PinALast = n;


//right encoder
m = digitalRead(right_encoder0PinA);
   if ((right_encoder0PinALast == LOW) && (m == HIGH)) {
     if (digitalRead(right_encoder0PinB) == LOW) {
       rwheel--;
     } else {
       rwheel++;
     }
rwheelMsg.data = rwheel;
rwheelPub.publish(&rwheelMsg);
   }
   right_encoder0PinALast = m;


  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Will update both motors
void Update_Motors()
{
  
  moveRightMotor();
  moveLeftMotor();

 }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Motor running function

// to'g you have to change the pin values below as described - kishore
void moveRightMotor()
{
  if (motor_right_speed>0)
  {
 //right wheel forward (change the pin values accordingly)   - kishore    
 digitalWrite(MOTOR_R_1, LOW);
 digitalWrite(MOTOR_R_2, LOW);
 analogWrite(ENA_PIN_1,-1*motor_right_speed);
    
  }
  else if(motor_right_speed<0)
  {

//right wheel backward (change the pin values accordingly) - kishore
 digitalWrite(MOTOR_R_1, HIGH);
 digitalWrite(MOTOR_R_2, LOW);
 analogWrite(ENA_PIN_1,motor_right_speed);
 
  }
  
  else if(motor_right_speed == 0)
  {

//Right wheel stop (change the pin values accordingly) - kishore
 digitalWrite(MOTOR_R_1,HIGH);
 digitalWrite(MOTOR_R_2,HIGH);
    
    
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveLeftMotor()
{
 if (motor_left_speed > 0)
  {
 //left wheel forward (change the pin values accordingly)   - kishore    
digitalWrite(MOTOR_L_1, HIGH);
digitalWrite(MOTOR_L_2, LOW);
analogWrite(ENB_PIN_2,-1*motor_left_speed);
  }
  else if(motor_left_speed < 0)
  {

//left wheel backward (change the pin values accordingly) - kishore
 digitalWrite(MOTOR_L_1, LOW);
 digitalWrite(MOTOR_L_2, LOW);
 analogWrite(ENB_PIN_2,motor_left_speed);

  }
  else if(motor_left_speed == 0)
  {
//left wheel stop (change the pin values accordingly) - kishore
   digitalWrite(MOTOR_L_1,HIGH);
   digitalWrite(MOTOR_L_2,HIGH);
  
   }  
  
  
}



