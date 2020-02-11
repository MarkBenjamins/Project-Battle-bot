
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define M1 2
#define M12 3
#define M2 4
#define M22 9

LiquidCrystal_I2C lcd(0x27, 16, 2);
const int trig_pin=13;
const int echo_pin=12;
const int IR_Sensor=10;
const int IR_Sensor2=11;
long distance,duration;

void setup() {
  // put your setup code here, to run once:
  // Ultra sensor
  pinMode(13,OUTPUT);
  pinMode(12,INPUT);
  // Motors
  pinMode(M1, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M22, OUTPUT);
  // IR sensors
  pinMode(IR_Sensor,INPUT);
  pinMode(IR_Sensor2,INPUT);
  // Initialize the LCD
  lcd.begin (16, 2);
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.setCursor(5,0); //set the cursor to column 0 and line 0
  // Put "bennie" on the display
  lcd.clear();
  lcd.print("bennie");
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13,HIGH);
  delayMicroseconds(20);
  digitalWrite(13,LOW);
  delayMicroseconds(20);
  duration = pulseIn(echo_pin, HIGH); //To receive the reflected signal.
  distance= duration*0.034/2;

  rightForward();
  leftForward();
  
  if(digitalRead(IR_Sensor)==1 || digitalRead(IR_Sensor2)==1)
  {
    rightBackward();
    leftBackward();
    delay(100);
    rightBackward();
    leftForward();
    delay(200);
  }
  if(distance < 10)
  {
    rightBackward();
    leftBackward();
    delay(300);
    rightBackward();
    leftForward();
    delay(200);
  }
}

//Functions to use the motors.
void leftBackward()
{
  digitalWrite(M12, LOW);
  analogWrite(M1, 130);
}
void leftForward()
{
  analogWrite(M12, 100);
  digitalWrite(M1, LOW);
}
void rightBackward()
{
  analogWrite(M2, 130);
  digitalWrite(M22, LOW);
}
void rightForward()
{
  digitalWrite(M2, LOW);
  analogWrite(M22, 100);
}
