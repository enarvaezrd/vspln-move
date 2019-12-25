/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

// Arduino – ROS headers

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <Servo.h>
#include <std_msgs/String.h>
Servo myServo;


std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";


//Creating a Nodehandle object
ros::NodeHandle nh;

// Creating a callback for the topic toggle_led, whenever a value come through this topic, this callback will execute
// The callback will toggle the state of LED which is on PIN 13
bool clock_wise=true;
void joystick_handler( const sensor_msgs::Joy& joystick){
        str_msg.data = "start";
        chatter.publish( &str_msg );
  if (joystick.buttons[11]==1.0)
    {
      if (clock_wise)
      {
        digitalWrite(13, HIGH-digitalRead(13)); // blink the led replace with pin
        str_msg.data = "clock";
        chatter.publish( &str_msg );
        delay(200);
        clock_wise=false;
        myServo.write(0); //Clockwise
      }
      else
      {
        digitalWrite(13, HIGH-digitalRead(13)); // blink the led replace with pin
        str_msg.data = "c-clock";
        chatter.publish( &str_msg );
        delay(200);
        clock_wise=true;
        myServo.write(180); //C-Clockwise
      }
        delay(2000);      
    }
        str_msg.data = "ending";
        chatter.publish( &str_msg );
    
}

//Creating a subscriber with a name toggle_led, and its callback
ros::Subscriber<sensor_msgs::Joy> sub("/joy",&joystick_handler);

//Setting PIN 13 as output and initializing ROS node and subscriber object
void setup()
{
  myServo.attach(8);
  //myServo.writeMicroseconds(1500);  // Stop 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

// Spining the node each times to listen from the topic
void loop()
{
  nh.spinOnce();
  delay(1);
}
