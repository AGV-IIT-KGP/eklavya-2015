#include <avr/interrupt.h>
#include <Encoder.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define ledPin 13

ros::NodeHandle  nh;

geometry_msgs::Twist str_msg;
ros::Publisher chatter("encoders", &str_msg);




Encoder knobLeft(2,3);
Encoder knobRight(18,19);
//   avoid using pins with LEDs attached

void setup() {
  
  
  nh.initNode();
  nh.advertise(chatter);
  
  
  
  //Serial.begin(9600);
    pinMode(ledPin, OUTPUT);
  //Serial.println("TwoKnobs Encoder Test:");
   noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 3277;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

long positionLeft  = -999;
long positionRight = -999;
 double left_vel=0;
 double right_vel=0;
long newLeft, newRight;
long x=0,y=0;
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{   
  left_vel=.004987*(newLeft-x);
  right_vel=.004987*(newRight-y);
   x=newLeft;
    y=newRight;
   
 
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  // toggle LED pin
  
}

void loop() {
  
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
  
    positionLeft = newLeft;
    positionRight = newRight;
  }
 
  str_msg.linear.x = left_vel;
  
  str_msg.linear.y = right_vel;
  chatter.publish( &str_msg );
  nh.spinOnce();

  
}
