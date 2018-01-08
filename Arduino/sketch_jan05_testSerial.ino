#include <Servo.h>
//#include <ContinuousRotationServo.h>

//  Pins
//  BT VCC to Arduino 5V out. 
//  BT GND to GND
//  Arduino D8 (SS RX) - BT TX no need voltage divider 
//  Arduino D9 (SS TX) - BT RX through a voltage divider (5v to 3.3v)
//
#define PIN_SERVO 5

#include <SoftwareSerial.h>
SoftwareSerial BTserial(8,9); 
// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
Servo LeftServo;
Servo RightServo;
Servo SGServo;

char c=' ';
boolean NL = true;

boolean READ_L = false;
boolean READ_R = false;
boolean READ_F = false;
boolean READ_B = false;

int read_count = 0;

int buf[4] = {0,0,0,0}; //X100

int tF = 100; // moving time
int tB = 100;
boolean left = false;
boolean right = false;

int buf2num(){
  int temp = 0;
  for(int i=0; i<4; i++){
    temp = temp+ buf[i]*pow(10.0, 3-i);
  }
  return temp*100;
}

void print_data(){
    Serial.println();
    Serial.print("Forward move time = ");
    Serial.print(tF);
    Serial.print("backward move time = ");
    Serial.print(tB);
    Serial.println(" ");
}


void turn_left()
{
  Serial.println("turn left");
  LeftServo.write(1300);
  RightServo.write(1300);
  delay(500);
  LeftServo.write(1500);
  RightServo.write(1500);
}

void turn_right()
{
  Serial.println("turn right");
  LeftServo.write(1700);
  RightServo.write(1700);
  delay(500);
  LeftServo.write(1500);
  RightServo.write(1500);
}

void forward(int t)
{
  Serial.println("forward");
  LeftServo.write(1700);
  RightServo.write(1300);
  delay(t);
  LeftServo.write(1500);
  RightServo.write(1500);
}

void backward(int t)
{
  Serial.println("backward");
  LeftServo.write(1300);
  RightServo.write(1700);
  delay(t);
  LeftServo.write(1500);
  RightServo.write(1500);
}

void handup()
{
  for(int pos = 0; pos < 180; pos += 1) // goes from 0 degrees to 180 degrees 
  { // in steps of 1 degree 
    SGServo.write(pos); // tell servo to go to position in variable 'pos' 
    delay(15); // waits 15ms for the servo to reach the position 
  }
}

void handdown()
{
  for(int pos = 180; pos>= 1; pos -= 1) // goes from 0 degrees to 180 degrees 
  { // in steps of 1 degree 
    SGServo.write(pos); // tell servo to go to position in variable 'pos' 
    delay(15); // waits 15ms for the servo to reach the position 
  }
}

void start()
{
  LeftServo.attach(2);
  RightServo.attach(3);
}

void shut()
{
  LeftServo.detach();
  RightServo.detach();
}

void stop_car()
{
  LeftServo.write(1500);
  RightServo.write(1500);
}

void move_car()
{
  start();
  if(left)
  {
    turn_left();
  }
  if(right)
  {
    turn_right();
  }
  if(tF > 0)
  {
    forward(tF);
  }
  if(tB > 0)
  {
    backward(tB);
  }
  stop_car();
  handup();
  handdown();
}

void config_input(char c){
    if (c == 'L'){
      READ_L = true;
    }
    else if (c == 'R'){
      READ_R = true;
    }
    else if (c == 'F'){
      READ_F = true;
    }
    else if (c == 'B'){
      READ_B = true;
    }
    else{
   
      if (READ_F ){
        int n = c - 48;
        buf[read_count] = n;
        read_count = read_count + 1;
        if (read_count == 4){
            READ_F = false;
            read_count = 0;
            tF = buf2num();
        }
      }
      else if(READ_B){
        int n = c - 48;
        buf[read_count] = n;
        read_count = read_count + 1;
        if (read_count == 4){
            READ_B = false;  
            read_count = 0;
            tB = buf2num();
            //print_data();
        }
      }
      else if(READ_L){
        int n = c - 48;
        if (n==1){
          left = true;
        }
        else
        {
          left = false;
        }
        READ_L=false;
      }
      else if(READ_R){
        int n = c - 48;
        if (n==1){
          right = true;
        }
        else
        {
          right = false;
        }
        READ_R=false;
        print_data();
        move_car();
      }
    }
}


void setup() 
{
    Serial.begin(9600);
    Serial.print("Sketch:   ");   Serial.println(__FILE__);
    Serial.print("Uploaded: ");   Serial.println(__DATE__);
    Serial.println(" ");
 
    BTserial.begin(9600);  
    Serial.println("BTserial started at 9600");
    SGServo.attach(PIN_SERVO);
    start();
}
 
void loop()
{
    // Read from the Bluetooth module and send to the Arduino Serial Monitor
    if (BTserial.available())
    {
        c = BTserial.read();
        Serial.write(c);
        config_input(c);
        
    }
 
    // Read from the Serial Monitor and send to the Bluetooth module
    if (Serial.available())
    {
        c = Serial.read();
 
        // do not send line end characters to the HM-10
        if (c!=10 & c!=13 ) 
        {  
             BTserial.write(c);
        }
 
        // Echo the user input to the main window. 
        // If there is a new line print the ">" character.
        if (NL) { Serial.print("\r\n>");  NL = false; }
        Serial.write(c);
        if (c==10) { NL = true; }

        //
    }
    
}
