#include <math.h>

void setup()
{
    Serial.begin(9600);
    analogWriteResolution(12);
    pinMode(DAC0,OUTPUT);
    pinMode(6,OUTPUT);
    digitalWrite(13,HIGH);
}
long int counter=0;
long int value=0;
int cnt=0;
char input;
int array[10];
int switcher=0;

void loop(){
  
    while(Serial.available()==1)
    {
      input=Serial.read();  
      
      if (input!='@'&& input!='m' && input!='a'){
        array[cnt]= (int)input;
        cnt++;
      }
      else if(input=='@')
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
    }
          
        
        
    }
    


