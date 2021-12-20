/*
  This code was developed by Walid Khaled - Nile University graduating student 2021
  It is used to run Teach-in Robot Proramming
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <MultiStepper.h>
#include <AccelStepper.h>

//Stepper Motors Pin Configuration
AccelStepper stepper1 (AccelStepper::DRIVER, 2, 3);     //PULSE Pin, Direction Pin , Stepper 1
AccelStepper stepper2 (AccelStepper::DRIVER, 4, 5);     //PULSE Pin, Direction Pin , Stepper 2
AccelStepper stepper3 (AccelStepper::DRIVER, 6, 7);     //PULSE Pin, Direction Pin , Stepper 3
AccelStepper stepper4 (AccelStepper::DRIVER, 8, 9);     //PULSE Pin, Direction Pin , Stepper 4
AccelStepper stepper5 (AccelStepper::DRIVER, 10, 11);   //PULSE Pin, Direction Pin , Stepper 5
AccelStepper stepper6 (AccelStepper::DRIVER, 12, 13);   //PULSE Pin, Direction Pin , Stepper 6

MultiStepper steppers; // upt to 10 steppers we can use MultiStepper

//Driver Mode
int DPulsePerRev[] = {400, 400, 400, 400, 800, 400}; //MicroStepping mode 400 is half-step, 800 is quarter-step

//Set Gear Ratios for Joints
float GearRatio[] = {10, 50, 50, (3969/289), 1, (3585/187)}; //(3969/289)=13.73 and (3585/187)=19.17

//Set Belt Ratios for Joints
float BeltRatio[] = {4.2, 1, 1, 3.09, 8.8889, 1};

//Set Positions for Motors
long positions[6]; //Array of desired stepper positions
long sequence[30][8] = {};
int s_step = 0;
int index;
int s_c = 0;
int buttonState;         
int lastButtonState = 0;
char cont_flag = 1;

//Set Speeds for Motors
float n = 120;
float MotorSpeeds[] = {((n/60)*DPulsePerRev[0]), ((n/60)*DPulsePerRev[1]), ((n/60)*DPulsePerRev[2]), ((n/60)*DPulsePerRev[3]), ((n/60)*DPulsePerRev[4]), ((n/60)*DPulsePerRev[5])};
float MotorSpeeds_J[] = {((n*0.5/60)*DPulsePerRev[0]), ((n*0.5/60)*DPulsePerRev[1]), ((n/60)*DPulsePerRev[2]), ((n/60)*DPulsePerRev[3]), ((n/60)*DPulsePerRev[4]), ((n/60)*DPulsePerRev[5])};
int speed_rpm = 60; // 10% after mapping: Min rpm is 60 and Max rpm is 240

//Set Limit Switches 
long LSwitchS[6];
int Lswitch1 = 14;
int Lswitch2 = 15;
int Lswitch3 = 16;
int Lswitch4 = 17;
int Lswitch5 = 18;
int Lswitch6 = 19;

//Set Push Buttons
char PB_J1_Right = 51;
char PB_J1_Left = 50;
char PB_J2_Up = 49;
char PB_J2_Down = 48;
char PB_J3_Up = 47;
char PB_J3_Down = 46;
char PB_J4_Right = 45;
char PB_J4_Left = 44;
char PB_J5_Up = 43;
char PB_J5_Down = 42;
char PB_J6_Right = 41;
char PB_J6_Left = 40;
char PB_G_C = 39;
char PB_G_O = 38;
char PB_Set = 37;
char PB_Reset = 36;
char PB_Play = 35;
char PB_Cont = 34;
char PB_Stop = 33;

//Set KeyPad
#include <Keypad.h>
const byte ROWS = 4; 
const byte COLS = 4; 
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {A12, A13, A14, A15};
byte colPins[COLS] = {A8, A9, A10, A11}; 
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
String inputString;
int inputInt; 

//Set LCD
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
char lcd_flag = 0;

//Set Gripper
#include <Servo.h>
Servo myservo;  
int pos = 0;    

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  inputString.reserve(3);          // maximum number of digit for a number is 3, change if needed
  
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Welcome to AR3!");
  lcd.setCursor(0,1);
  lcd.print("Teach_in Programming");
  lcd.setCursor(4,2);
  lcd.print("WOMMA_GP2021");
  lcd.setCursor(2,3);
  lcd.print("Please Calibrate!");
  
  pinMode(PB_J1_Right, INPUT_PULLUP);
  pinMode(PB_J1_Left, INPUT_PULLUP);
  pinMode(PB_J2_Up, INPUT_PULLUP);
  pinMode(PB_J2_Down, INPUT_PULLUP);
  pinMode(PB_J3_Up, INPUT_PULLUP);
  pinMode(PB_J3_Down, INPUT_PULLUP);
  pinMode(PB_J4_Right, INPUT_PULLUP);
  pinMode(PB_J4_Left, INPUT_PULLUP);
  pinMode(PB_J5_Up, INPUT_PULLUP);
  pinMode(PB_J5_Down, INPUT_PULLUP);
  pinMode(PB_J6_Right, INPUT_PULLUP);
  pinMode(PB_J6_Left, INPUT_PULLUP);
  pinMode(PB_G_O, INPUT_PULLUP);
  pinMode(PB_G_C, INPUT_PULLUP);
  pinMode(PB_Set, INPUT_PULLUP);
  pinMode(PB_Reset, INPUT_PULLUP);
  pinMode(PB_Play, INPUT_PULLUP);
  pinMode(PB_Stop, INPUT_PULLUP);
  pinMode(PB_Cont, INPUT_PULLUP);

  //Add steppers to the MultiStepper Object
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  steppers.addStepper(stepper5);
  steppers.addStepper(stepper6);

  myservo.attach(A6);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  while ((digitalRead(PB_Stop) == HIGH)) {
    J1();
    J2();
    J3();
    J4();
    J5();
    J6();
    if ((digitalRead(PB_G_O) == LOW)) open_gripper();
    if ((digitalRead(PB_G_C) == LOW)) close_gripper();
    if ((digitalRead(PB_Set) == LOW)) set();
    if ((digitalRead(PB_Reset) == LOW)) reset();
    if ((digitalRead(PB_Play) == LOW)) play();
    KeyPad();
    proceed();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calibration() {
  //Configure each Stepper Motor
  stepper1.setMaxSpeed(MotorSpeeds[0]);
  stepper2.setMaxSpeed(MotorSpeeds[1]);
  stepper3.setMaxSpeed(MotorSpeeds[2]);
  stepper4.setMaxSpeed(MotorSpeeds[3]);
  stepper5.setMaxSpeed(MotorSpeeds[4]);
  stepper6.setMaxSpeed(MotorSpeeds[5]);

  //Set Smallest Increment for each Motor, Note Motors Found to Rotate in the Direction of Limit Switches in the Following Sign Convention
  positions[0] = 1;
  positions[1] = 1;
  positions[2] = 0;
  positions[3] = -1;
  positions[4] = 1;
  positions[5] = -1;

  while (true) {
    //Take the Readings of Limit Switches Continuously for Motors except Motor 3
    LSwitchS[0] = digitalRead(Lswitch1);
    LSwitchS[1] = digitalRead(Lswitch2);
    LSwitchS[2] = digitalRead(Lswitch3);
    LSwitchS[3] = digitalRead(Lswitch4);
    LSwitchS[4] = digitalRead(Lswitch5);
    LSwitchS[5] = digitalRead(Lswitch6);
    //If One of Motor Reaches the Limit Switch, Set the Position to Zero
    for (int i = 0; i < 6; i++) {
      if (LSwitchS[i] == 1) {
        positions[i] = 0;
      }
    }
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();

    //Temporarily Zeros after each Step
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);
    stepper5.setCurrentPosition(0);
    stepper6.setCurrentPosition(0);

    //Increment by 1
    positions[0] = positions[0] + 1;
    positions[1] = positions[1] + 1;
    positions[2] = positions[2] - 0;
    positions[3] = positions[3] - 1;
    positions[4] = positions[4] + 1;
    positions[5] = positions[5] - 1;

    //Limit Switches Zeros
    if ((LSwitchS[0] == HIGH) && (LSwitchS[1] == HIGH) && (LSwitchS[3] == HIGH) && (LSwitchS[4] == HIGH) && (LSwitchS[5] == HIGH)){
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(0);
      stepper5.setCurrentPosition(0);
      stepper6.setCurrentPosition(0);
      break;
    }
  }

  delay(2000);

  //Moving to Actual Zeros
  positions[0] = -7733.33;//-7933.33
  positions[1] = -2333.33;
  positions[2] = -3000;
  positions[3] = 7000;//7732.912
  positions[4] = -1975.31;//-2034.57;
  positions[5] = 3600;//3408
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper5.setCurrentPosition(0);
  stepper6.setCurrentPosition(0);
  
  //Motor 3 Calibration and Zero 
  positions[2] = -1;
  while ((digitalRead(Lswitch3) == LOW)) {
    positions[0] = 0; positions[1] = 0; positions[3] = 0; positions[4] = 0; positions[5] = 0;
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    if (digitalRead(Lswitch3) == HIGH) {
      stepper3.setCurrentPosition(0);
      break;
    }
    positions[2] = positions[2] - 1;
  }
  positions[0] = 0; positions[1] = 0; positions[2] = 2777.77; positions[3] = 0; positions[4] = 0; positions[5] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  stepper3.setCurrentPosition(0);

  lcd.clear();lcd.setCursor(2,1);lcd.print("Calibration Done");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void homing() {
  //Configure each Stepper Motor
  stepper1.setMaxSpeed(MotorSpeeds[0]);
  stepper2.setMaxSpeed(MotorSpeeds[1]);
  stepper3.setMaxSpeed(MotorSpeeds[2]);
  stepper4.setMaxSpeed(MotorSpeeds[3]);
  stepper5.setMaxSpeed(MotorSpeeds[4]);
  stepper6.setMaxSpeed(MotorSpeeds[5]);
  positions[0] = 0; positions[1] = 0; positions[2] = 0; positions[3] = 0; positions[4] = 0; positions[5] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); 
    
  lcd.clear();lcd.setCursor(3,1);lcd.print("Home Position");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cg_end_position() {
  //Configure each Stepper Motor
  stepper1.setMaxSpeed(MotorSpeeds[0]);
  stepper2.setMaxSpeed(MotorSpeeds[1]);
  stepper3.setMaxSpeed(MotorSpeeds[2]);
  stepper4.setMaxSpeed(MotorSpeeds[3]);
  stepper5.setMaxSpeed(MotorSpeeds[4]);
  stepper6.setMaxSpeed(MotorSpeeds[5]);
  positions[0] = 0; positions[1] = 0; positions[2] = 5000; positions[3] = 0; positions[4] = 0; positions[5] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  lcd.clear();lcd.setCursor(4,1);lcd.print("End Position");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void J1() {
  stepper1.setMaxSpeed(MotorSpeeds_J[0]);
  while ((digitalRead(PB_J1_Right) == LOW) && (positions[0] <= 7000)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[0] = positions[0] + 5;
    Serial.println(positions[0]);
  }
  while ((digitalRead(PB_J1_Left) == LOW) && (positions[0] >= -7000)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[0] = positions[0] - 5;
    Serial.println(positions[0]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void J2() {
  stepper2.setMaxSpeed(MotorSpeeds_J[1]);
  while ((digitalRead(PB_J2_Up) == LOW) && (positions[1] <= 2300)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[1] = positions[1] + 5;
    Serial.println(positions[1]);
  }
  while ((digitalRead(PB_J2_Down) == LOW) && (positions[1] >= -5000)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[1] = positions[1] - 5;
    Serial.println(positions[1]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void J3() {
  stepper3.setMaxSpeed(MotorSpeeds_J[2]);
  while ((digitalRead(PB_J3_Up) == LOW) && (positions[2] <= 5000)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[2] = positions[2] + 5;
    Serial.println(positions[2]);
  }
  while ((digitalRead(PB_J3_Down) == LOW) && (positions[2] >= -2700)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[2] = positions[2] - 5;
    Serial.println(positions[2]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void J4() {
  stepper4.setMaxSpeed(MotorSpeeds_J[3]);
  while ((digitalRead(PB_J4_Right) == LOW) && (positions[3] <= 6500)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[3] = positions[3] + 5;
    Serial.println(positions[3]);
  }
  while ((digitalRead(PB_J4_Left) == LOW) && (positions[3] >= -6500)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[3] = positions[3] - 5;
    Serial.println(positions[3]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void J5() {    
  while ((digitalRead(PB_J5_Up) == LOW) && (positions[4] <= 1900)) {
    stepper5.setMaxSpeed(MotorSpeeds_J[4]);
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[4] = positions[4] + 1;
    Serial.println(positions[4]);
  }
  while ((digitalRead(PB_J5_Down) == LOW) && (positions[4] >= -1900)) {
    stepper5.setMaxSpeed(MotorSpeeds_J[4]);
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[4] = positions[4] - 1;
    Serial.println(positions[4]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void J6() {    
  stepper6.setMaxSpeed(MotorSpeeds_J[5]);
  while ((digitalRead(PB_J6_Right) == LOW) && (positions[5] >= -3400)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[5] = positions[5] - 5;
    Serial.println(positions[5]);
  }
  while ((digitalRead(PB_J6_Left) == LOW) && (positions[5] <= 3400)) {
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    positions[5] = positions[5] + 5;
    Serial.println(positions[5]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void open_gripper() {
  if (pos!=-1){
    for (pos = 50; pos >= 0; pos -= 1){ 
      myservo.write(pos);              
      delay(10);                       
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void close_gripper() {
  if (pos!=101 && pos!=0){
    for (pos = 0; pos <= 100; pos += 1){
      myservo.write(pos);              
      delay(5);  
    }                        
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set() {
  sequence[s_step][0] = positions[0];
  sequence[s_step][1] = positions[1];
  sequence[s_step][2] = positions[2];
  sequence[s_step][3] = positions[3];
  sequence[s_step][4] = positions[4];
  sequence[s_step][5] = positions[5];
  sequence[s_step][6] = speed_rpm;
  sequence[s_step][7] = pos;

  for (int s = 0; s < 8; s++) {
    Serial.print("Sequence["); Serial.print(s_step); Serial.print("]["); Serial.print(s); Serial.print("]= "); Serial.println(sequence[s_step][s]);
  }
  s_step++;
  Serial.print("Step= "); Serial.println(s_step);
  lcd.clear();lcd.setCursor(4,1);lcd.print("Set Step : ");lcd.print(s_step);
  lcd.setCursor(6,2);lcd.print("Speed=");lcd.print((speed_rpm-40)/2);lcd.print("%");
  delay(500);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void reset() {
  if (s_step > 0)s_step--;
  Serial.println("Reset");Serial.print("Step= "); Serial.println(s_step);
  lcd.clear();lcd.setCursor(2,1);lcd.print("Reset Prev Step");lcd.setCursor(6,2);lcd.print("Step = ");lcd.print(s_step);
  delay(500);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void reset_all() {
  s_step = 0;
  if (pos == -1) close_gripper();
  Serial.println("Reset All");Serial.print("Step= "); Serial.println(s_step);
  lcd.clear();lcd.setCursor(3,1);lcd.print("Program Reset!");
  delay(500);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void play() {
  if (cont_flag == 1){
    homing();
    close_gripper(); //In case of not closing
    delay(2000);
  }

  for (index = s_c; index < s_step; index++){
    s_c++;
    Serial.print("Moving to Step: "); Serial.println(index + 1);
    lcd.clear();lcd.setCursor(2,1);lcd.print("Moving to Step:");lcd.print(index + 1);
    positions[0] = sequence[index][0];
    positions[1] = sequence[index][1];
    positions[2] = sequence[index][2];
    positions[3] = sequence[index][3];
    positions[4] = sequence[index][4];
    positions[5] = sequence[index][5];
    Serial.print("Speed: "); Serial.print(((sequence[index][6])-40)/2); Serial.println("%");
    lcd.setCursor(5,2);lcd.print("Speed:");lcd.print(((sequence[index][6])-40)/2);lcd.print("%");
    stepper1.setMaxSpeed(((sequence[index][6])/60)*DPulsePerRev[0]);
    stepper2.setMaxSpeed(((sequence[index][6])/60)*DPulsePerRev[1]);
    stepper3.setMaxSpeed(((sequence[index][6])/60)*DPulsePerRev[2]);
    stepper4.setMaxSpeed(((sequence[index][6])/60)*DPulsePerRev[3]);
    stepper5.setMaxSpeed(((sequence[index][6])/60)*DPulsePerRev[4]);
    stepper6.setMaxSpeed(((sequence[index][6])/60)*DPulsePerRev[5]);

    steppers.moveTo(positions);
    //steppers.runSpeedToPosition();

    while (true) {
      steppers.run(); // Does not block
      if ((stepper1.distanceToGo() == 0) && (stepper2.distanceToGo() == 0) && (stepper3.distanceToGo() == 0) && (stepper4.distanceToGo() == 0) && (stepper5.distanceToGo() == 0) && (stepper6.distanceToGo() == 0)) {
        break;
      }
      if ((digitalRead(PB_Stop) == LOW)) {
        Serial.println("stop");
        lcd.clear();lcd.setCursor(16,0);lcd.print("Stop");
        index = s_step;
        s_c--;
        break;
      }
    }

    if (sequence[index][7] == -1) {
      delay(100);
      open_gripper();
    }
    else if (sequence[index][7] == 101) {
      delay(100);
      close_gripper();
    }

    if(index==(s_step-1)){
      s_c=0;
      cont_flag = 1;   
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void KeyPad() {
  //Serial.println("Please enter rpm: ");
  char customKey = customKeypad.getKey();
  if (customKey){
    //Serial.println(customKey);
    switch(customKey){
      case 'A':lcd.clear();lcd.setCursor(2,1);lcd.print("Calibration ....");lcd.setCursor(4,2);lcd.print("Please Wait!");calibration();break;
      case 'B':lcd.clear();lcd.setCursor(4,1);lcd.print("Homing ....");lcd.setCursor(4,2);lcd.print("Please Wait!");homing();break;
      case 'C':lcd.clear();lcd.setCursor(3,1);lcd.print("CG End Position");lcd.setCursor(4,2);lcd.print("Please Wait!");cg_end_position();break;
      case 'D':reset_all();break;
    }
    if (customKey >= '0' && customKey <= '9') {     // only act on numeric keys
      inputString += customKey;                     // append new character to input 
      Serial.println(inputString);
      lcd.setCursor(5,3);lcd.print("Speed:");lcd.print(inputString);
    }
    else if (customKey == '#') {
      if (inputString.length() > 0) {
        inputInt = inputString.toInt();             // YOU GOT AN INTEGER NUMBER
        inputString = "";                           // clear input
        if (inputInt >= 10 && inputInt <=100) {
          speed_rpm = inputInt*2+40;                // Mapping
          //Serial.println(speed_rpm);
          lcd.setCursor(5,3);lcd.print("Speed=");lcd.print((speed_rpm-40)/2);lcd.print("%");
        }
      }
    } 
    else if (customKey == '*') {
      inputString = "";                             // clear input
      Serial.println(inputString);
      lcd.clear();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void proceed() {
  buttonState = digitalRead(PB_Cont);  
  if ((buttonState != lastButtonState)&&(index!=s_step)) {
    if ((buttonState == LOW)) {
      Serial.println("proceed");
      index=s_c;
      cont_flag = 0;
      play();
    }
    delay(50);
  }
  lastButtonState = buttonState;// save the current state as the last state, for next time through the loop
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
