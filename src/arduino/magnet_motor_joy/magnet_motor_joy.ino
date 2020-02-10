/*
 * EM and servo-motor control
 * Eduardo Narvaez
 */
 
#include <ros.h>
#include <std_msgs/Int8.h>
#include <Servo.h>
#include <std_msgs/String.h>
#include <string.h>
Servo myServo;


std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);


//Creating a Nodehandle object
ros::NodeHandle nh;
char copy[50];
// Creating a callback for the topic toggle_led, whenever a value come through this topic, this callback will execute
// The callback will toggle the state of LED which is on PIN 13



void joystick_handler( const std_msgs::Int8& joystick){
  if ( joystick.data==0 ||joystick.data==3)
  {
    digitalWrite(10, LOW);
    myServo.write(180); // Opened
    
  }
 /* String str_data= String(joystick.buttons[11]);
  str_data.toCharArray(copy, 50);
  str_msg.data =  copy;
        chatter.publish( &str_msg );
        */
        
  if(joystick.data==1 || joystick.data==2)
  {
      digitalWrite(10, HIGH);
  }
  
  if (joystick.data==2 ||joystick.data==0)
    {
      digitalWrite(13, HIGH); 
      myServo.write(0); //Closed
    }
    else
    {
      digitalWrite(13, LOW); 
      myServo.write(180); // Opened
    } 
    delay(5);
    return;
}


ros::Subscriber<std_msgs::Int8> sub("/robot1/arduino_control/flags_msg",&joystick_handler,1);

void setup()
{
  myServo.attach(8);
  //myServo.writeMicroseconds(1500);  // Stop 
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);
  nh.initNode();
  //nh.advertise(chatter);
  nh.subscribe(sub);
  nh.getHardware()->setBaud(57600);
}

// Spining the node each times to listen from the topic
void loop()
{
  nh.spinOnce();
  delay(10);
}
