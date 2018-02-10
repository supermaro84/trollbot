#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

/* 
** PINS
*/

/* Left motor pins for channel A */
#define LEFT_DIR    12   /* Direction */
#define LEFT_PWM    3    /* PWM */
#define LEFT_BRK    9    /* Brake */

/* Right motor pins for channel B */
#define RIGHT_DIR   13   /* Direction */
#define RIGHT_PWM   11   /* PWM */
#define RIGHT_BRK   8    /* Brake */

/* Left encoder pins */
#define LEFT_ENC_A  4    /* Left encoder pin A */
#define LEFT_ENC_B  5    /* Left encoder pin B */

/* Right encoder pins */
#define RIGHT_ENC_A  6   /* Right encoder pin A */
#define RIGHT_ENC_B  7   /* Right encoder pin B */

/*
** GlOBAL VARIABLES
*/

/* Motor status variables */
float motor_left_speed = 0;
float motor_right_speed = 0;

/* Left encoder variables */
int lwheel = 0;
int left_enc_A_last = LOW;
int n = LOW;

/* Right encoder variables */
int rwheel = 0;
int right_enc_A_last = LOW;
int m = LOW;

/* ROS node handle */
ros::NodeHandle nh;

/* Left wheel speed subscriber */
void left(const std_msgs::Float32& msg1){
   motor_left_speed = msg1.data;
}
ros::Subscriber<std_msgs::Float32> sub1("left_wheel_speed", left); 

/* Right wheel speed subscriber */
void right(const std_msgs::Float32& msg2){
   motor_right_speed = msg2.data;
}
ros::Subscriber<std_msgs::Float32> sub2("right_wheel_speed", right); 

/* Left encoder position publisher */
std_msgs::Int64 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

/* Right encoder position publisher */
std_msgs::Int64 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);


void setup()
{
    /* Initialize node */
    nh.initNode();
    
    /* Subscribe to wheel speed topics */
    nh.subscribe(sub1);
    nh.subscribe(sub2);
    
    /* Publish to wheel position topics */
    nh.advertise(rwheelPub);
    nh.advertise(lwheelPub);
    
    /* Initialize serial port with a baud rate of 57600 */
    Serial.begin(57600);
  
    /* Setup Motors */
    /* Setup left motor */
    pinMode(LEFT_DIR, OUTPUT);
    pinMode(LEFT_BRK, OUTPUT);

    /* Setup right motor */
    pinMode(RIGHT_DIR, OUTPUT);
    pinMode(RIGHT_BRK, OUTPUT);
    
    /* Setup encoders */
    /* Setup left encoder */
    pinMode(LEFT_ENC_A, INPUT);
    pinMode(LEFT_ENC_B, INPUT);

    /* Setup right encoder */
    pinMode(RIGHT_ENC_A, INPUT);
    pinMode(RIGHT_ENC_B, INPUT);
}

void loop()
{
    /* Process callbacks */
    nh.spinOnce();
    
    /* Update motor values with target speed */
    update_motors();

    /* Send encoder positions through serial port */
    read_encoders();
}

/* Update motor speeds */
void update_motors()
{
    move_right_motor();
    move_left_motor();
}

/* Update right motor speed */
void move_right_motor()
{
    if (motor_right_speed > 0)
    {
        /* Right wheel forward */
        digitalWrite(RIGHT_DIR, LOW);
        digitalWrite(RIGHT_BRK, LOW);
        analogWrite(RIGHT_PWM, -motor_right_speed);
    } 
    else if(motor_right_speed < 0)
    {
        /* Right wheel backward */
        digitalWrite(RIGHT_DIR, HIGH);
        digitalWrite(RIGHT_BRK, LOW);
        analogWrite(RIGHT_PWM, motor_right_speed);
    }
    else if(motor_right_speed == 0)
    {
        /* Right wheel stop */
        digitalWrite(RIGHT_DIR, HIGH);
        digitalWrite(RIGHT_BRK, LOW);
        analogWrite(RIGHT_PWM, 0);
    }
}

/* Update left motor speed */
void move_left_motor()
{
    if (motor_left_speed > 0)
    {
        /* Left wheel forward */
        digitalWrite(LEFT_DIR, HIGH);
        digitalWrite(LEFT_BRK, LOW);
        analogWrite(LEFT_PWM, -motor_left_speed);
    }
    else if(motor_left_speed < 0)
    {
        /* Left wheel backward */
        digitalWrite(LEFT_DIR, LOW);
        digitalWrite(LEFT_BRK, LOW);
        analogWrite(LEFT_PWM, motor_left_speed);
    }
    else if(motor_left_speed == 0)
    {
        /* Left wheel stop */
        digitalWrite(LEFT_DIR, HIGH);
        digitalWrite(LEFT_BRK, LOW);
        analogWrite(LEFT_PWM, 0);
    }    
}

/* Read both wheel encoders */
void read_encoders()
{
    read_left_encoder();
    read_right_encoder();
}

/* Read left encoder */
void read_left_encoder()
{
    n = digitalRead(LEFT_ENC_A);
    if ((left_enc_A_last == LOW) && (n == HIGH)) {
        if (digitalRead(LEFT_ENC_B) == LOW) {
            lwheel--;
        } else {
            lwheel++;
        }
        lwheelMsg.data = lwheel;
        lwheelPub.publish(&lwheelMsg);
    }
    left_enc_A_last = n;
}

/* Read right encoder */        
void read_right_encoder()
{
    m = digitalRead(RIGHT_ENC_A);
    if ((right_enc_A_last == LOW) && (m == HIGH)) {
        if (digitalRead(RIGHT_ENC_B) == LOW) {
            rwheel--;
        } else {
            rwheel++;
        }
        rwheelMsg.data = rwheel;
        rwheelPub.publish(&rwheelMsg);
    }
    right_enc_A_last = m;
}
