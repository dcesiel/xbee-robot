
#include <SoftwareSerial.h>
#include <Servo.h>

int RIGHT = 9;
int LEFT = 10; 

int STOP = 127;

Servo right_side;
Servo left_side;

int idx;
int left_joy_val;
int right_joy_val;

int signalCounter;
char data_char;
String buffer;

void right_pwm(int val){
//Value from 0 to 255, 127 = stopped
  if ((val > 110) && (val < 142))
    val = 127;
  else if (val < 0)
    val = 0;
  else if (val > 255)
    val = 255;
  int micro_seconds = 990;
  micro_seconds += (val * 4);
  right_side.writeMicroseconds(micro_seconds);
  return;
}

void left_pwm(int val){
//Value from 0 to 255, 127 = stopped
  if ((val > 110) && (val < 142))
    val = 127;
  else if (val < 0)
    val = 0;
  else if (val > 255)
    val = 255;
  int micro_seconds = 990;
  micro_seconds += (val * 4);
  left_side.writeMicroseconds(micro_seconds);
  return;
}


void setup(){
    Serial.begin(9600);
    signalCounter = 0;
    right_side.attach(RIGHT);
    left_side.attach(LEFT);
    right_pwm(127);  //Make sure both cims are stopped at initialization
    left_pwm(127);  //Make sure both cims are stopped at initialization
}

int stringIndex(String input, char indexChar){
  int i;
  for (i = 0; i < input.length(); i++)
    if (input[i] == indexChar) return i;
  return -1;
}

void loop(){
    //Handle Serial Data
    
    if (Serial.available() > 0){
      signalCounter = 0;
      data_char = Serial.read();
      if(data_char == '\n'){
        data_char = buffer[0];
        Serial.print(data_char);
        Serial.print(" ");
        Serial.println(signalCounter);
        if (data_char == '@'){
          //Remove @ and space
            buffer = buffer.substring(2);
            idx = stringIndex(buffer, ' ');
            if (idx > 0){
              left_joy_val = buffer.substring(0, idx).toInt();
              right_joy_val = buffer.substring((idx+1), buffer.length()).toInt();
              
              if ((left_joy_val > -50) && (left_joy_val < 280)){
                if ((right_joy_val > -50) && (right_joy_val < 280)){ 
                  //Sanity check
                  left_pwm(left_joy_val);
                  right_pwm(right_joy_val);
                }
              }
            }
        }
        
        
        buffer = ""; 
      }
      else{
        buffer += data_char;
      }
    }
    else{
      if (signalCounter > 10000){
        left_pwm(STOP);
        right_pwm(STOP);
      }
      else
        signalCounter++;
    }
}
