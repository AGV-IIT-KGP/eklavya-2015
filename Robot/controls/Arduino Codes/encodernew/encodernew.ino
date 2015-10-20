#define USE_USBCON

#include <Encoder.h>
#include <DueTimer.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>


ros::NodeHandle  nh;

geometry_msgs::Twist str_msg;
ros::Publisher chatter("encoders", &str_msg);


  
Encoder knobLeft(2,3);
Encoder knobRight(4,5);



int myLed = 13;

double positionLeft  = -999;
double positionRight = -999;
 double  left_vel=0;
 double right_vel=0;
double newLeft, newRight;
double x=0,y=0;
//unsigned long t=0;
//unsigned long prev_time=0;


void myHandler(){
  
    
  left_vel=.004987*(newLeft-x);//.0030226
  right_vel=.004987*(newRight-y);//.0030226
   x=newLeft;
    y=newRight;
   /* Serial.print("new left= ");
    Serial.println(newLeft);
    Serial.print("new right= ");
    Serial.println(newRight);
    Serial.print("x= ");
    Serial.println(x);
    Serial.print("y= ");
    Serial.println(y);*/
   if( left_vel<.001&&left_vel>-.001)
   { left_vel=0;}
   if( right_vel<.001&& right_vel>-.001)
   { right_vel=0;}
   
   
   /* Serial.print("Leftvel =l ");
    Serial.print(left_vel,8);//,8
    Serial.print(", Rightvel =r ");
    Serial.print(right_vel,8);//,8
    Serial.println();*/
    
    


	
}


void setup(){

 
  nh.initNode();
  nh.advertise(chatter);
  
  //Serial.begin(57600);
  
  // Serial.println("TwoKnobs Encoder Test:");
Timer3.attachInterrupt(myHandler).start(20000);
	
}



void loop() {
  
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  
  if (newLeft != positionLeft || newRight != positionRight) {
  
    positionLeft = newLeft;
    positionRight = newRight;
  }
  
  
 /* if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }*/
    
  str_msg.linear.x = left_vel;
  
  str_msg.linear.y = right_vel;
  chatter.publish( &str_msg );
  nh.spinOnce();
 // delay(20);
  
  
}
