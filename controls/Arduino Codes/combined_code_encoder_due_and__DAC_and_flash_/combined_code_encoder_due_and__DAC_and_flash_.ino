#define USE_USBCON

#include <Encoder.h>
#include <DueTimer.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

long int counter=0;
long int value=0;
int cnt=0;
char input;
int array[10];
int switcher=0;


ros::NodeHandle  nh;


void value_callback(const geometry_msgs::Twist& msg){

analogWrite(DAC0,msg.linear.x);


if((msg.angular.y)==1)
{
   digitalWrite(6,HIGH);
}


else if((msg.angular.y)==0)
{

         counter++;
         
         if(counter==15)
         {
         
           if(switcher==1)
           {
             digitalWrite(6,HIGH);
             switcher=0;
           }
           else if(switcher==0)
           {
             digitalWrite(6,LOW);
             switcher=1;
           }
           counter=0;
         }
}
}



ros::Subscriber<geometry_msgs::Twist> sub("twist_msg" , value_callback );

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


void setup()
{
    
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  
  //Serial.begin(57600);
  
  // Serial.println("TwoKnobs Encoder Test:");
Timer3.attachInterrupt(myHandler).start(20000);
    analogWriteResolution(12);
    pinMode(DAC0,OUTPUT);
    pinMode(6,OUTPUT);
    digitalWrite(13,HIGH);
}

void loop(){
  
     /* else if(input=='@')
        { 
             for(int i=0;i<cnt;i++)
          {
            value+=(array[i]-48)*pow(10,cnt-i-1);
        }
        Serial.println(value);
          analogWrite(DAC0,value);
        value=0;
        cnt=0;
        }
       else if(input=='m')
       {
         digitalWrite(6,HIGH);
       }
       else if(input=='a')
       {
         counter++;
         
         if(counter==15)
         {
         
           if(switcher==1)
           {
             digitalWrite(6,HIGH);
             switcher=0;
           }
           else if(switcher==0)
           {
             digitalWrite(6,LOW);
             switcher=1;
           }
           counter=0;
         }

       }

    }*/





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
  delay(20);
          
        
        
    }
    

