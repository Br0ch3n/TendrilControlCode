#include <RunningAverage.h>
#include <MiniTen.h>

// Printing and debug variables
long counter = 0; // counter for when to display data. Too much time printing...
bool pFlag = false; // printing flag tells when to display info...
bool fMotorsAtZero = false; // flag to begin motor zeroing
bool buttonPressed = false;
uint8_t tipEnable;
uint8_t midEnable;
uint8_t baseEnable;

// State variables, flags, and definitions
#define INITIAL           0
#define ZEROING           1
#define RUNNING           2
#define DIFF_THRESH_FALL  4
#define DIFF_THRESH_RISE  4
#define TENSION_THRESH    100
#define MOTOR_COUNT       3

uint8_t state = ZEROING;
bool btConnected = false;

// function prototypes
void printTension();
void zeroingFunction();
void checkForTension();

// Pin assignments
int pwmPins[MOTOR_COUNT]       = { 2, 3, 4};//, 5, 6, 7, 8, 9,10};
int dirOutPins[MOTOR_COUNT]    = {42,43,44};//,45,46,47,48,49,50};
int encAPins[MOTOR_COUNT]      = {25,27,29};//,30,32,34,36,38,40}; // encoder A  (motor 0,1,2 encoder pins switched, I think 24 is burnt)
int encBPins[MOTOR_COUNT]      = {24,26,28};//,31,33,35,37,39,41}; // encoder B
int tSensorPins[MOTOR_COUNT]   = {A0,A1,A2};//,A3,A4,A5,A6,A7,A8};
static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};


// Tension constants for each sensor
float tScales[MOTOR_COUNT]     = {1,1,1};//,1,1,1,1,1,1};
float tIntercepts[MOTOR_COUNT] = {0,0,0};//,0,0,0,0,0,0};
float tMaximums[MOTOR_COUNT]   = {9,9,9};//,9,9,9,9,9,9};

// Array for current desired motor counts
int motorSetCounts[MOTOR_COUNT]     = {0,0,0};//,0,0,0,0,0,0};
int32_t motorCounts[MOTOR_COUNT]    = {0,0,0};//,0,0,0,0,0,0};
float tVals[MOTOR_COUNT]            = {0,0,0};//,0,0,0,0,0,0};
float tReadings[MOTOR_COUNT]        = {0,0,0};//,0,0,0,0,0,0};
float tReadingsPrev[MOTOR_COUNT]    = {0,0,0};//,0,0,0,0,0,0};

// Array for string encoders
int stringEncAPins[MOTOR_COUNT]      = {30,32,34};
int stringEncBPins[MOTOR_COUNT]      = {31,33,35};
int32_t stringEncCounts[MOTOR_COUNT]= {0,0,0};//,0,0,0,0,0,0};

// PD control gains
uint32_t pgain = 350;
uint32_t dgain = 250;

//Declaring tendril object(contains motor information, only used for initialization and safety reasons)
tendril Tendril;
//declaring tendril motors
//DCMotor *motorPointer = Tendril.getMotor(motorNum,PWMpin,dirOutPin,EncAPin,EncBPin,pgain,dgain);
DCMotor *base1 = Tendril.getMotor(1,pwmPins[0],dirOutPins[0],encBPins[0],encAPins[0],pgain,dgain,tSensorPins[0],tScales[0],tIntercepts[0]);
DCMotor *base2 = Tendril.getMotor(2,pwmPins[1],dirOutPins[1],encBPins[1],encAPins[1],pgain,dgain,tSensorPins[1],tScales[1],tIntercepts[1]);
DCMotor *base3 = Tendril.getMotor(3,pwmPins[2],dirOutPins[2],encBPins[2],encAPins[2],pgain,dgain,tSensorPins[2],tScales[2],tIntercepts[2]);
//DCMotor *mid1  = Tendril.getMotor(4,pwmPins[3],dirOutPins[3],encBPins[3],encAPins[3],pgain,dgain,tSensorPins[3],tScales[3],tIntercepts[3]);
//DCMotor *mid2  = Tendril.getMotor(5,pwmPins[4],dirOutPins[4],encBPins[4],encAPins[4],pgain,dgain,tSensorPins[4],tScales[4],tIntercepts[4]);
//DCMotor *mid3  = Tendril.getMotor(6,pwmPins[5],dirOutPins[5],encBPins[5],encAPins[5],pgain,dgain,tSensorPins[5],tScales[5],tIntercepts[5]);
//DCMotor *tip1  = Tendril.getMotor(7,pwmPins[6],dirOutPins[6],encBPins[6],encAPins[6],pgain,dgain,tSensorPins[6],tScales[6],tIntercepts[6]);
//DCMotor *tip2  = Tendril.getMotor(8,pwmPins[7],dirOutPins[7],encBPins[7],encAPins[7],pgain,dgain,tSensorPins[7],tScales[7],tIntercepts[7]);
//DCMotor *tip3  = Tendril.getMotor(9,pwmPins[8],dirOutPins[8],encBPins[8],encAPins[8],pgain,dgain,tSensorPins[8],tScales[8],tIntercepts[8]);

// Alias for the motor pointers for easier coding in places
DCMotor *motorList[MOTOR_COUNT] = {base1, base2, base3};//, mid1, mid2, mid3, tip1, tip2, tip3};

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial1.begin(115200);
  //Serial.println("Motor test!");

  pinMode(22,OUTPUT);
  digitalWrite(22,HIGH);

  pinMode(23,INPUT_PULLUP);
  //pinMode(22,INPUT_PULLUP);
  pinMode(21,INPUT);//_PULLUP);
  pinMode(20,INPUT);

  for (int i = 0; i < MOTOR_COUNT; i++) tReadings[i] = motorList[i]->getTension();

  buttonPressed = false;
  tipEnable = HIGH;
  midEnable = HIGH;
  baseEnable = HIGH;

  for(int i;i <MOTOR_COUNT;i++) {
    pinMode(encBPins[i], INPUT_PULLUP);
    digitalWrite(encBPins[i],HIGH);
    pinMode(encAPins[i], INPUT_PULLUP);
    digitalWrite(encAPins[i],HIGH);
  }
  //attaching interrupts
  attachInterrupt(digitalPinToInterrupt(encAPins[0]), base1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPins[1]), base2Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPins[2]), base3Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stringEncAPins[0]), stringEnc1Interrupt , CHANGE);
  attachInterrupt(digitalPinToInterrupt(stringEncAPins[1]), stringEnc2Interrupt , CHANGE);
  attachInterrupt(digitalPinToInterrupt(stringEncAPins[2]), stringEnc3Interrupt , CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encAPins[6]), tip1Interrupt , CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encAPins[7]), tip2Interrupt , CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encAPins[8]), tip3Interrupt , CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPins[0]), base1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPins[1]), base2Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPins[2]), base3Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stringEncBPins[0]), stringEnc1Interrupt , CHANGE);
  attachInterrupt(digitalPinToInterrupt(stringEncBPins[1]), stringEnc2Interrupt , CHANGE);
  attachInterrupt(digitalPinToInterrupt(stringEncBPins[2]), stringEnc3Interrupt , CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encBPins[6]), tip1Interrupt , CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encBPins[7]), tip2Interrupt , CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encBPins[8]), tip3Interrupt , CHANGE);


  attachInterrupt(digitalPinToInterrupt(23),baseSwitch,CHANGE);
  attachInterrupt(digitalPinToInterrupt(22),midSwitch,CHANGE);
  //attachInterrupt(digitalPinToInterrupt(21),tipSwitch,CHANGE);
  attachInterrupt(digitalPinToInterrupt(20),zeroingButton,FALLING);
}

void loop() {
  // begin loop 
  counter++; // keeps track of 'time' gone by...
  switch (state) {
    case INITIAL:
      //Serial.print(millis()); Serial.print(",");//Serial.println(analogRead(A9));
//      if(buttonPressed) {
//        Serial.println("Am I bouncing?");
//        buttonPressed = false;
//        state = ZEROING;
//      }
      
//      if(!baseEnable){
//        Serial.println("base disabled");
//      }
//      if(!midEnable){
//        Serial.println("mid disabled");
//      }
//      if(!tipEnable){
//        Serial.println("tip disabled");
//      }
//      
//      if (!(counter % 200000)) {
//        Serial.println("Waiting...");
//        pFlag = !pFlag;
//      }
      break;
    case ZEROING:
      buttonPressed = false;
      //if(!fMotorsAtZero) {
      //zeroingFunction(8,0);
      //zeroingFunction(0,8); 
      zeroingFunction(2,0);  // <====================================== Only base sectioned zeroed
      zeroingFunction(0,2); 
      //}
      state = RUNNING;
      break;
    case RUNNING:
//      if(buttonPressed) {
//        state = ZEROING;
//        Serial.print("back to zeroing.....");
//      }
      if (!(counter % 100)) {
        //checkForTension();
        if (btConnected) printTension();
        //Serial.println(motorCounts[6]);
        //pFlag = !pFlag;
      }
      
      // temporary array of errors
      int32_t motorErrors[MOTOR_COUNT] = {0,0,0};//,0,0,0,0,0,0};
      
      //place current motor counts in array
      //obtain errors for each motor
      for(int j=0;j<MOTOR_COUNT;j++){
        motorCounts[j] = motorList[j]->count;
        motorErrors[j] = (int32_t) motorSetCounts[j] - motorCounts[j];
        //if (j == 0) {
          //Serial.print(digitalRead(base1->dirOutpin)); Serial.print(", "); Serial.println(motorErrors[j]);
        //}
        //if (j == 2 || j == 1) {
          //Serial.print(motorCounts[j]); Serial.print(", "); Serial.println(motorErrors[j]);
        //}
        
      }

//      if(!baseEnable){
////        if(pFlag) {
////          //Serial.println("base disabled");
////          pFlag = false;
////        }
//        motorErrors[0] = 0;
//        motorErrors[1] = 0;
//        motorErrors[2] = 0;
//      }
//      if(!midEnable){
////        if(pFlag) {
////          //Serial.println("mid disabled");
////          pFlag = false;
////        }
//        motorErrors[3] = 0;
//        motorErrors[4] = 0;
//        motorErrors[5] = 0;
//      }

//      if(!tipEnable){
//        motorErrors[6] = 0;
//        motorErrors[7] = 0;
//        motorErrors[8] = 0;
//      }
      
      //send errors to control loops
      for (int i = 0; i < MOTOR_COUNT; i++) {
        motorList[i]->update((int32_t)motorErrors[i]);
//        if (i == 3 && abs(motorErrors[i]) > 40) {
//          Serial.print("3: "); Serial.println(motorErrors[i]);
//        }
      }

      break;
    
  }
  

}
////////////////////////////////
//////  Tension checking  //////
////////////////////////////////

void checkForTension() {
  
}


////////////////////////////////
//////  Tension Printing  //////
////////////////////////////////

void printTension() {
//  Serial.println("0\t1\t2\t3\t4\t5\t6\t7\t8");

  if (btConnected) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
  //    tReadings[0] = base1->getTension(); 
      //Serial.print(motorList[i]->getTension()); 
      Serial1.print(motorList[i]->getTension()); 
      //Serial.print(analogRead(tSensorPins[i])); Serial.print("\t");
      //Serial.print(abs(tReadings[i] - tReadingsPrev[i])); 
      if(i<MOTOR_COUNT-1) {
        //Serial.print(",");
        Serial1.print(",");
      }
      //tReadingsPrev[i] = tReadings[i];
    }
    Serial1.print(",");
    for (int i=MOTOR_COUNT;i < 9; i++) {
      Serial1.print("0");
      if(i<8) {
        Serial1.print(",");
      }
    }
    //Serial.println("");
    Serial1.print("\t");
    for(int i = 0; i < MOTOR_COUNT; i++) {
        //Serial.print(motorSetCounts[i]);
        //motorList[i]->count = motorSetCounts[i];
        Serial1.print(motorList[i]->count);
  //      Serial.print(", ");
        //Serial.print(motorSetCounts[i]);
        if(i<MOTOR_COUNT-1){
          //Serial.print(",");
          Serial1.print(",");
        }
    }
    Serial1.print(",");
    for (int i=MOTOR_COUNT;i < 9; i++) {
      Serial1.print("0");
      if(i<8) {
        Serial1.print(",");
      }
    }
    
    Serial1.print("\t");
    for(int i = 0; i < MOTOR_COUNT; i++) {
  //    tReadings[0] = base1->getTension(); 
      //Serial.print(motorList[i]->getTension()); 
      Serial1.print(stringEncCounts[i]); 
      //Serial.print(analogRead(tSensorPins[i])); Serial.print("\t");
      //Serial.print(abs(tReadings[i] - tReadingsPrev[i])); 
      if(i<MOTOR_COUNT-1) {
        //Serial.print(",");
        Serial1.print(",");
      }
      //tReadingsPrev[i] = tReadings[i];
    }
    Serial1.print(",");
    for (int i=MOTOR_COUNT;i < 9; i++) {
      Serial1.print("0");
      if(i<8) {
        Serial1.print(",");
      }
    }
    
    Serial1.print("\t");
    //Serial.println("");
    //Serial.println("Message sent.");
    Serial1.println("");  
  }
  
}

/////////////////////////////////////////
///// ZERO MOTOR COUNTS AND SENSORS /////
/////////////////////////////////////////

void zeroingFunction(int ndxStart, int ndxEnd) {
  bool diffMet = false;
  int32_t curCount = 0;
  int32_t curSetPnt = 0;
  int32_t jumpLgMot = 250;
  int32_t jumpSmMot = 50;
  int32_t zeroCounter = 0;

  int32_t ndxStep = (ndxEnd - ndxStart)/abs(ndxStart - ndxEnd);
  Serial.println("Beginning Zeroing Process...");
  for(int i = ndxStart; i != ndxEnd + ndxStep; i+=ndxStep) {
    Serial.print("Zeroing Motor "); Serial.println(i);
    diffMet = false;
    while(!diffMet) { // unwinding until difference in tensions stops
      tReadingsPrev[i] = analogRead(tSensorPins[i]); // Store previous reading
      curCount = motorList[i]->count;
      if (i < 3) curSetPnt = curCount - jumpLgMot; // if large motor (0-2), use larger step
      else curSetPnt = curCount - jumpSmMot; // smaller motors (3-8), use smaller step
      motorList[i]->update(curSetPnt - curCount);
      while(abs(curSetPnt - curCount) > 8) { 
        // keep going to new set point until error is less than 4 counts
        curCount = motorList[i]->count;
        motorList[i]->update(curSetPnt - curCount);
        //Serial.println(curCount);
      }
      motorList[i]->update(0); // stop motor      
      delay(300);
      tReadings[i] = analogRead(tSensorPins[i]);
      Serial.print("Unwinding, Previous: "); Serial.print(tReadingsPrev[i]);
      Serial.print(", Current: "); Serial.println(tReadings[i]);
      if (abs(tReadings[i] - tReadingsPrev[i]) <= DIFF_THRESH_FALL) { // if change in tension is large enough we've reached the goal 
        diffMet = true;
      }
    }
    diffMet = false;
    while(!diffMet) { // winding until difference in tensions begins again
      tReadingsPrev[i] = analogRead(tSensorPins[i]); // find starting tension
      curCount = motorList[i]->count; // find current motor count
      if (i < 3) curSetPnt = curCount + jumpLgMot; else curSetPnt = curCount + jumpSmMot;
      motorList[i]->update(curSetPnt - curCount); // tell motors to wind to set point
      while(abs(curSetPnt - curCount) > 8) { 
        // continue to set point until within 8 error
        curCount = motorList[i]->count;
        motorList[i]->update(curSetPnt - curCount);
        //Serial.println(curCount);
      }
      motorList[i]->update(0); // stop motor
      delay(300);
      tReadings[i] = analogRead(tSensorPins[i]);
      Serial.print("Winding, Previous: "); Serial.print(tReadingsPrev[i]);
      Serial.print(", Current: "); Serial.println(tReadings[i]);
      if (abs(tReadings[i] - tReadingsPrev[i]) >= DIFF_THRESH_RISE) { 
        // if change in tension is large enough we've reached the goal 
        diffMet = true;
      }
    }
    curCount = motorList[i]->count; // find current motor count
    if (i < 3) curSetPnt = curCount - (uint32_t)(jumpLgMot * 0.75); 
    else curSetPnt = curCount - (uint32_t)(jumpSmMot); // set set point to lower than current count
    motorList[i]->update(curSetPnt - curCount); // tell motors to wind to set point
    while(abs(curSetPnt - curCount) > 8) { // continue to set point until within 8
      curCount = motorList[i]->count;
      motorList[i]->update(curSetPnt - curCount);
    }
    motorList[i]->update(0); // stop motor
    delay(400);
    
  }
  delay(1000);
  for(int i = 8; i >= 0; i--) {
    motorList[i]->count = 0;
    motorList[i]->setTenIntercept(analogRead(tSensorPins[i]));
  }
  //Serial.println("Finished zeroing process");
  for (int i = 0; i < 9; i++) motorList[i]->setTenIntercept(analogRead(tSensorPins[i]));
  fMotorsAtZero = !fMotorsAtZero;
  
  Serial.println("Zeroing complete.");
}

////////////////////////////////
//////RECEIVE MOTOR COUNTS//////
////////////////////////////////

int charCount=0;
String dummy = "";
String inString = "";
void serialEvent1(void){
  char cmd = 'r';
  int i;
  while (Serial1.available() > 0) {
    int inChar = Serial1.read();
    inString += (char)inChar;
    
    // if you get a newline, print the string,
    // then the string's value:
    if ((char)inChar == ','){
      charCount++;
      //Serial.print(count);
    }
    if (inChar == '\n' || inChar == ' ') {
      charCount++;
      for(i=0;i<charCount;i++){
        int index = inString.indexOf(',');
        dummy = inString.substring(0,index);
        //Serial.println(dummy);
        if (i == 0) {
          if(dummy.equals("Connect") && !btConnected) {
            btConnected = true;
            Serial.println("Connect received");
            dummy = "";
            inString = "";
            charCount = 0;
            return;
          } else if(dummy.equals("Disconnect") && btConnected) {
            btConnected = false;
            Serial.println("Disconnect received");
            dummy = "";
            inString = "";
            charCount = 0;
            return;
          } else if(dummy.equals("Zero") && btConnected) {
            Serial.println("Zero received");
            dummy = "";
            inString = "";
            charCount = 0;
            return;
          }
          else if(dummy.equals("Read")) {
            cmd = 'r'; // reading new set counts
          } else if (dummy.equals("Write")) {
            cmd = 'w'; // writing/overwriting current motor counts; 
                       // used for fudging counts
          }
          else return;
        }
        else if (cmd == 'r' && btConnected) {
          motorSetCounts[i-1] = dummy.toInt();
          Serial.print(dummy.toInt());
        } else if (cmd = 'w' && btConnected) {
          motorSetCounts[i-1] = dummy.toInt();
          motorList[i-1]->count = dummy.toInt();
        } else {
          Serial.println("Invlaid command");
        }
        Serial.println("");
        inString = inString.substring(index+1,inString.length());
        dummy = "";       
      }
      inString = "";
      charCount = 0;
    }
  } 
}

////////////////////////////////////
////////INTERRUPT FUNCTIONS/////////
////////////////////////////////////

static uint8_t base1_enc_val = 0;
void base1Interrupt() { // pins 24 & 25, PA15 & PD0
  if(digitalRead(base1->dirOutpin) == HIGH){base1->count--;}
  else{base1->count++;}
  //Serial.println(base1->count);
  
  //base1_enc_val = base1_enc_val << 2;
  //base1_enc_val = base1_enc_val | (((PIOA->PIO_PDSR & 0x8000) >> 14)|((PIOD->PIO_PDSR & 0x01)));
  //base1->count = base1->count + lookup_table[base1_enc_val & 0b1111];
  
}
static uint8_t base2_enc_val = 0;
void base2Interrupt() {// pins 26 & 27, PD1 & PD2
  if(digitalRead(base2->dirOutpin) == HIGH){base2->count--;}
  else{base2->count++;}
  
  //base2_enc_val = base2_enc_val << 2;
  //base2_enc_val = base2_enc_val | (((PIOD->PIO_PDSR & 0x02))|((PIOD->PIO_PDSR & 0x04) >> 2));
  //Serial.println(base2_enc_val);
  //Serial.println(lookup_table[base2_enc_val & 0b1111]);
  //base2->count = base2->count + lookup_table[base2_enc_val & 0b1111];
}
static uint8_t base3_enc_val = 0;
void base3Interrupt() {// pins 28 & 29, PD3 & PD6
  if(digitalRead(base3->dirOutpin) == HIGH){base3->count--;}
  else{base3->count++;}
  //static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};   // kept for quickly switching to smaller motors
  
  //base3_enc_val = base3_enc_val << 2;
  //base3_enc_val = base3_enc_val | (((PIOD->PIO_PDSR & 0x08) >> 2)|((PIOD->PIO_PDSR & 0x40) >> 6));
  //base3->count = base3->count + lookup_table[base3_enc_val & 0b1111];
  //Serial.println(base3_enc_val);
}
void stringEnc1Interrupt() {// pins 30 & 31, PD9 & PA7
  static uint8_t stringEnc1_enc_val = 0;
  stringEnc1_enc_val = stringEnc1_enc_val << 2;
  stringEnc1_enc_val = stringEnc1_enc_val | (((PIND & 0x0200) >> 8)|((PINA & 0x80) >> 7));
  stringEncCounts[0] = stringEncCounts[0] + lookup_table[stringEnc1_enc_val & 0b1111];
}
void stringEnc2Interrupt() {// pins 32 & 33, PD10 & PC1
  static uint8_t stringEnc2_enc_val = 0;
  stringEnc2_enc_val = stringEnc2_enc_val << 2;
  stringEnc2_enc_val = stringEnc2_enc_val | (((PIND & 0x0400) >> 9)|((PINC & 0x02) >> 1));
  stringEncCounts[1] = stringEncCounts[1] + lookup_table[stringEnc2_enc_val & 0b1111];
}
void stringEnc3Interrupt() { // pins 34 & 35 PC2 & PC3
  static uint8_t stringEnc3_enc_val = 0;
  stringEnc3_enc_val = stringEnc3_enc_val << 2;
  stringEnc3_enc_val = stringEnc3_enc_val | (((PINC & 0x04) >> 1)|((PINC & 0x08) >> 3));
  stringEncCounts[2] = stringEncCounts[2] + lookup_table[stringEnc3_enc_val & 0b1111];
}
//void tip1Interrupt() {// pins 36 & 37, PC4 & PC5
//  static uint8_t tip1_enc_val = 0;
//  tip1_enc_val = tip1_enc_val << 2;
//  tip1_enc_val = tip1_enc_val | (((PIOC->PIO_PDSR & 0x10) >> 3)|((PIOC->PIO_PDSR & 0x20) >> 5));
//  tip1->count = tip1->count + lookup_table[tip1_enc_val & 0b1111];
//}
//void tip2Interrupt() {// pins 38 & 39, PC6 & PC7
//  static uint8_t tip2_enc_val = 0;
//  tip2_enc_val = tip2_enc_val << 2;
//  tip2_enc_val = tip2_enc_val | (((PIOC->PIO_PDSR & 0x0040) >> 5)|((PIOC->PIO_PDSR & 0x0080) >> 7));
//  tip2->count = tip2->count + lookup_table[tip2_enc_val & 0b1111];
//}
//void tip3Interrupt() {// pins 40 & 41, PC8 & PC9
//  static uint8_t tip3_enc_val = 0;
//  tip3_enc_val = tip3_enc_val << 2;
//  tip3_enc_val = tip3_enc_val | (((PIOC->PIO_PDSR & 0x100) >> 7)|((PIOC->PIO_PDSR & 0x200) >> 9));
//  tip3->count = tip3->count + lookup_table[tip3_enc_val & 0b1111];
//}

void baseSwitch() {
  if(digitalRead(23) == LOW){baseEnable = HIGH;}
  else{baseEnable=LOW;}
}
void midSwitch() {
  if(digitalRead(22) == LOW){midEnable = HIGH;}
  else{midEnable=LOW;}
}
//void tipSwitch() {
////  if(digitalRead(20) == LOW){tipEnable = HIGH;}
////  else{tipEnable=LOW;}
//}
void zeroingButton() {
  //Serial.println("Am I bouncing?...");
  //buttonPressed = true;
}
