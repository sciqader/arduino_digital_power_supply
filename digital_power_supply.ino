//LCD wiring
//GND -> GND
//VCC -> 3.3v
//D0 -> D13
//D1 -> D12
//RES -> D8
//DC -> D9
//CS -> D10


#include "U8glib.h"
#include <Wire.h> 

U8GLIB_SH1106_128X64 u8g(13, 12, 10, 9, 8);

// Code for Available PWM frequency for D3 & D11:
//TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
//
//TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
//
//TCCR2B = TCCR2B & B11111000 | B00000011; // for PWM frequency of 980.39 Hz
//
//TCCR2B = TCCR2B & B11111000 | B00000100; // for PWM frequency of 490.20 Hz (The DEFAULT)
//
//TCCR2B = TCCR2B & B11111000 | B00000101; // for PWM frequency of 245.10 Hz
//
//TCCR2B = TCCR2B & B11111000 | B00000110; // for PWM frequency of 122.55 Hz
//
//TCCR2B = TCCR2B & B11111000 | B00000111; // for PWM frequency of 30.64 Hz

//inputs for the buck converter
int voltage_in = A0;
int PWM_PIN = 11;
//variables
int pwm_value = 100;
float set_voltage = 10;
float set_current = 0.10;
float real_output = 0;
float output_current = 0;
unsigned long previousMillis = 0; 
unsigned long currentMillis = 0;
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
//--------------
//inputs for the rotary select menu
static int pinA = 2; // Our first hardware interrupt pin is digital pin 6
static int pinB = 3; // Our second hardware interrupt pin is digital pin 5
static int selectSwitch = 4; //The select switch for our encoder.
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile uint16_t encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile uint16_t oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
//--------------
// inputs for the screen setting
char* menu[3] = {"vol: ", "cur: ", "return"};
int itemPos[3] = {24, 43, 63};
int subItmPos[3] = {10, 45, 65};
int cursorPos = 0;
int oodOn = 0;
int subMenuOn = 0;
int chVal = 0;
//--------------
//set voltage/current
void setDgt(int item){
  if(item==0){
    if(encoderPos>oldEncPos){
      set_voltage = set_voltage + 1;
      oldEncPos = encoderPos;
    }
    if(encoderPos<oldEncPos){
      set_voltage = set_voltage - 1;
      oldEncPos = encoderPos;
    }
  }
  if(item==1){
    if(encoderPos>oldEncPos){
      set_current = set_current + 1;
      oldEncPos = encoderPos;
    }
    if(encoderPos<oldEncPos){
      set_current = set_current - 1;
      oldEncPos = encoderPos;
    }
  }
}
void setSubDgt(int item){
  if(item==0){
    if(encoderPos>oldEncPos){
      set_voltage = set_voltage + 0.1;
      oldEncPos = encoderPos;
    }
    if(encoderPos<oldEncPos){
      set_voltage = set_voltage - 0.1;
      oldEncPos = encoderPos;
    }
  }
  if(item==1){
    if(encoderPos>oldEncPos){
      set_current = set_current + 0.1;
      oldEncPos = encoderPos;
    }
    if(encoderPos<oldEncPos){
      set_current = set_current - 0.1;
      oldEncPos = encoderPos;
    }
  }
}
//------------
// main screen 
void draw(void) {
  u8g.setFont(u8g_font_unifont);  // select font
  u8g.drawStr(0, 10, "Q-technology ");  // put string of display at position X, Y
  u8g.drawStr(0, 30, "Volt: ");
  u8g.setPrintPos(44, 30);
  u8g.print(real_output, 2);  // display voltage
  u8g.drawStr(90, 30, "V ");
  u8g.drawStr(0, 50, "Crnt: ");
  u8g.setPrintPos(44, 50);
  u8g.print(output_current, 2);  // display voltage
  u8g.drawStr(90, 50, "A ");
}
//------------
//menu screen page
void menuPage(void) {
  u8g.setFont(u8g_font_unifont);  // select font
  u8g.drawStr(5, 10, "**** Menu **** "); // title
  u8g.drawStr(0, itemPos[cursorPos], "-> ");  // display cursor 
  for(int i=0; i<3; i++){
     u8g.drawStr(18, itemPos[i], menu[i]);
  }
  u8g.setPrintPos(54, itemPos[0]);
  u8g.print(set_voltage, 2);  // display set voltage
  u8g.drawStr(100, itemPos[0], "V ");
  u8g.setPrintPos(54, itemPos[1]);
  u8g.print(set_current, 2);  // display set current
  u8g.drawStr(100, itemPos[1], "A ");
  //set voltage
  if(cursorPos==0){
    if(digitalRead(selectSwitch)==0){
      while(digitalRead(selectSwitch)==0);//wait till switch is released
      subMenuOn = 1;
      while(subMenuOn){
        u8g.firstPage();  
        do {
          subMenu(0);
        } while( u8g.nextPage() );
          delay(5); // wait for debounce to get over
      }
    }
   }
   //set current
  if(cursorPos==1){
    if(digitalRead(selectSwitch)==0){
      while(digitalRead(selectSwitch)==0);//wait till switch is released
      subMenuOn = 1;
      while(subMenuOn){
        u8g.firstPage();  
        do {
          subMenu(1);
        } while( u8g.nextPage() );
          delay(5); // wait for debounce to get over
      }
    }
   }
  //exit the menu
  if(cursorPos==2){
    if(digitalRead(selectSwitch)==0){
      while(digitalRead(selectSwitch)==0);//wait till switch is released
      oodOn = 0;
      cursorPos = 0;
    }
  }
}
//-----------------
//sub menu page
void subMenu(int item){
  u8g.setFont(u8g_font_helvB12);
  u8g.drawStr(subItmPos[0], 40, "<-");
  u8g.drawStr(subItmPos[cursorPos], 40, "_");
  if(cursorPos==1)
   if(digitalRead(selectSwitch)==0){//if the sw pushed
      while(digitalRead(selectSwitch)==0);//wait untl released
      chVal = 1;
      while(chVal){
        cursorPos = 0;
        setDgt(item);
        u8g.firstPage();  
        do {
          subSubMenu(item);
        } while( u8g.nextPage() );
          delay(5); // wait for debounce to get over
      }
   }
  if(cursorPos==2)
   if(digitalRead(selectSwitch)==0){
      while(digitalRead(selectSwitch)==0); 
      chVal = 1;
      while(chVal){
        cursorPos = 0;
        setSubDgt(item);
        u8g.firstPage();  
        do {
          subSubMenu(item);
        } while( u8g.nextPage() );
          delay(5); // wait for debounce to get over
      }
   } 
  u8g.setPrintPos(40, 38);
  if(item==0)
    u8g.print(set_voltage, 2);  // display set voltage
  if(item==1)
    u8g.print(set_current, 2);  // display set current
  //exsit the sub menu
  if(cursorPos==0)
    if(digitalRead(selectSwitch)==0){
        while(digitalRead(selectSwitch)==0);
        subMenuOn = 0;
    }
}
//--------------------
//sub submenu page
void subSubMenu(int item){
  u8g.setFont(u8g_font_helvB12);
  u8g.setPrintPos(40, 38);
  if(item==0)
    u8g.print(set_voltage, 2);  // display set voltage
  if(item==1)
    u8g.print(set_current, 2); // display set current
  if(digitalRead(selectSwitch)==0){
      while(digitalRead(selectSwitch)==0);
      chVal = 0;
  }
}
//-------------------
//setup arduino 
void setup() {
  Serial.begin(9600); // for debuging
  pinMode(voltage_in,INPUT);
  pinMode(PWM_PIN,OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000001;      // pin 3 PWM frequency of 31372.55 Hz
  pinMode(pinA, INPUT); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(selectSwitch, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(pinB), PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  currentMillis = millis();
}
//-------------------
//entrupt functions for the rotary
void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
    cursorPos --; // cursor position for the menu page
    if(cursorPos<0)
      cursorPos = 2;
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}
void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
    cursorPos ++; // cursor position for the menu page
    if(cursorPos>2)
      cursorPos = 0;
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}
//-----------------
//read the arduino vcc
long readVcc(){
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result;
  return result;
  }
//-----------------
//read the output voltage
float read_real_voltage(){
  total = 0;
    readIndex = 0;
  while(readIndex<numReadings){
    total = total + analogRead(voltage_in);
    readIndex = readIndex + 1;
    delay(1);
  }
  // calculate the average:
  average = total / numReadings;
  float voltage = average/1024. * readVcc()/1000.;
  voltage = voltage/10 * 46.34; // calculate the output voltage
  //round the value
  float value = (int)(voltage * 10 + .5); 
    return (float)value / 10;
  //return voltage;
}
//-----------------
//main loop
void loop() {

  real_output = read_real_voltage();
  Serial.println(real_output, 2);
  
  // The select switch is pulled high, hence the pin goes low if the switch is pressed. 
  if(digitalRead(selectSwitch)==0)
  {
    while(digitalRead(selectSwitch)==0);
    oodOn = 1;  
    Serial.println("Key Pressed");
  }
  while(oodOn)  
  {
     u8g.firstPage();  
  do {
    menuPage();
  } while( u8g.nextPage() );
    delay(5); // wait for debounce to get over
  }
  
  //regulate the output voltage 
  if (set_voltage > real_output)
  {
    pwm_value = pwm_value - 1;
    pwm_value = constrain(pwm_value, 1, 254);
  }
  if (set_voltage < real_output)
  {
    pwm_value = pwm_value + 1;
    pwm_value = constrain(pwm_value, 1, 254);
  }
  pwm_value = constrain(pwm_value, 1, 254);
  analogWrite(PWM_PIN,pwm_value);
  
  
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );
  
  delay(100);

}
