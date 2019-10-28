
#include <TimerOne.h>

#define auto_reset              0
#define start_moving_forward    1
#define start_moving_backward   2
#define keep_moving             3
#define stoping                 4
#define auto_end                5
#define one_frame               6
#define hundred_frames          7
#define loading                 8
#define keep_moving_slow        9
#define test                    10


const int frameRatio = 1600 / 2.55;
const int enabPin = 8;
const int dirPin = 4;
const int stepPin = 5;
const int led = 13;
const int sensor = 2;
const int sensorThreshold = 590;
int lastSensorState = 0;
uint8_t sensorState = 0; //
uint8_t stepState = 0; //

//trigger
uint8_t trigger = 0;
uint8_t triggerCounter = 0;
bool sameState = true;
byte threshold[8];
uint8_t sum_threshold = 0;
uint8_t gate;
uint8_t gatePrevious;
bool gateOpen;
uint8_t shutterCounter = 0;
bool boolGate;
bool boolState = !digitalRead(sensor);



volatile bool home_position = false;
volatile uint8_t state = auto_end;



uint8_t doFullRotation = 0;
/*
   0..255 8bit uint8_t, char, byte, unsinged integer 8 bit,
   -127..127 8 bit int8_t,
   0..65335, 16bit unsigned int, word, uint16_t
   -32,768..-32,767 int, short int16_t
   0..2**32, 32 bit unsigned long, uint32_t
*/
int stepCount = 10000;

void stepperTimer() {
  if (stepCount > 0) {
    if (stepState == 0) {
      stepCount--;
    }
    digitalWrite(stepPin, stepState);
    stepState = !stepState;
  }
}


int homing(void) {

  digitalWrite(enabPin, LOW);
  digitalWrite(dirPin, LOW);
  int val = 0;
  bool gate_closed = true; //boolean related roughly to the shutter in front of the sensor
  bool center_not_found = true; //boolean related precisely to the sensor being at the end of the shutter
  byte gate[8]; //array used to make an average of the values; 0 means the shutter is in front of the sensor
  int sum_gate = 0;

  while (gate_closed == true && center_not_found == true) {
    for (int i = 0; i < 8; i ++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(50);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(50);
      val = !digitalRead(sensor);
      gate[i] = val;
    }
    for (int i = 0; i < 8; i ++) {
      sum_gate += gate[i];
      //Serial.println(gate[i]);
    }
    //Serial.println(sum_gate);

    if (sum_gate == 0) {
      gate_closed = false;
      continue;
    } else {
      sum_gate = 0;
      continue;
    }
  }

  if (gate_closed == false) {
    //val = digitalRead(sensor);
    while (center_not_found == true) {
      //Serial.println("buscando el 8");
      for (int i = 0; i < 8; i ++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(50);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(50);
        val = !digitalRead(sensor);
        gate[i] = val;
      }
      for (int i = 0; i < 8; i ++) {
        sum_gate += gate[i];
        //Serial.println(gate[i]);
      }
      //Serial.println(sum_gate);
      if (sum_gate == 8) {
        center_not_found = false;
        continue;
      } else {
        sum_gate = 0;
        continue;
      }
    }
  }
  digitalWrite(dirPin, HIGH);
  gateOpen = true;
  moveMotorSlow();
}

void load() {
    moveMotorSlow();
}

int moveMotor() {
  oneFrame();
  //serialFrames();
}

int moveMotorSlow() {
  oneFrameSlow();
}

void checkTrigger(){

  if(gateOpen){
   if(triggerCounter < 8){
      threshold[triggerCounter] = digitalRead(sensor);
      sum_threshold += threshold[triggerCounter];
    }
    else{
      triggerCounter = 0;    
    }
    if (sum_threshold == 8){
      sameState = false;
      sum_threshold = 0;
      gateOpen = false;
      shutterCounter += 1;
    }
    else if (sum_threshold > 8){
      sum_threshold = 0;
    }   
  }
  if (!gateOpen){
    triggerCounter = 0;
    if(digitalRead(sensor) == 1){
      gateOpen = true;
      shutterCounter +=1;
    }
  }
}

void triggerSensor(){
  if(digitalRead(sensor) == false){
    boolGate = true;
  }
  else{
    boolGate = false;
  }
  if(boolGate != boolState){
    triggerCounter += 1;
    boolState = boolGate;
  }
  if (triggerCounter == 4){
    Serial.write('0' + 0);
    triggerCounter = 0;
  }
}


void testMove(){
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(500);
  checkTrigger();
  if(shutterCounter == 4){
    //Serial.println("Aqui");
    //Serial.println(shutterCounter);
    delay(1000);
    shutterCounter = 0;
  }
}

void readSensor(){
  Serial.println(digitalRead(sensor));
}
void oneFrame() {
  for (int i = 0; i < frameRatio ; i ++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(200);
  }
}

void oneFrameSlow() {
  for (int i = 0; i < frameRatio ; i ++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}

void oneHundredFrames() {
  for (int i = 0; i < 100; i ++) {
    oneFrame();
  }
}

void serialFrames(){
    
  Serial.write('0' + 0);
  delay(1000);
}

//unsigned long forwardDelay = 0;
//int forwardInterval = 10000

boolean isMoving = false;

void serialTask() {

  switch (state) {
    case auto_reset:
      homing();
      //Serial.println("Homing executed2");
      state = auto_end;
      //Serial.println(state);
      break;

    case start_moving_forward:
      if ( !isMoving) {
        homing();
        isMoving = true;
      } else {
        state = keep_moving;
        isMoving = true;
      }
      digitalWrite(dirPin, LOW);
      delayMicroseconds(10000);
      break;

    case start_moving_backward:
      if ( !isMoving) {
        homing();
        isMoving = true;
      } else {
        state = keep_moving;
        isMoving = true;
      }
      digitalWrite(dirPin, HIGH);
      delayMicroseconds(10000);
      break;

    case keep_moving:
      home_position = false;
      moveMotor();
      //serialFrames();
      state = keep_moving;
      break;

    case keep_moving_slow:
      home_position = false;
      moveMotorSlow();
      state = keep_moving_slow;
      break;

    case stoping:
      digitalWrite(stepPin, LOW);
      digitalWrite(enabPin, HIGH);
      state = auto_end;
      isMoving = false;
      digitalWrite(enabPin, LOW);
      break;

    case auto_end:
      home_position = false;
      break;

    case one_frame:
      //oneFrame();
      oneFrameSlow();
      state = auto_end;
      break;
      
    case loading:
      if ( !isMoving) {
        isMoving = true;
      } else {
        state = keep_moving_slow;
        isMoving = true;
      }
      digitalWrite(dirPin, LOW);
      break;
    case hundred_frames:
      oneHundredFrames();
      state = auto_end;
      break;
    
    case test:
      //testMove();
      readSensor();
      break;
     
  }
}
void stateSwitch() {

  if (Serial.available()) {
    uint8_t code = Serial.read();
    //Serial.println(code);
    switch (code) {
      case 'a':
        state = auto_reset;
        break;
      case 'b':
        state = start_moving_forward;
        break;
      case 'c':
        state = start_moving_backward;
        break;
      case 'd':
        state = stoping;
        break;
      case 'e':
        state = one_frame;
        break;
      case 'f':
        state = hundred_frames;
        break;
      case 'g':
        state = loading;
        break;
      case 'h':
        state = test;
        break;
        
      case '0':
          state = stoping;
          break;
        case '1':
          state = start_moving_forward;
          break;
        case '2':
          state = start_moving_backward;
          break;
        case '4':
          state = auto_reset;
          break;
    }
    //Serial.println(state);

  }
}


void setup() {
  pinMode(enabPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(sensor, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  digitalWrite(enabPin, LOW);
  digitalWrite(led, LOW);
  digitalWrite(dirPin, HIGH);

  state = auto_reset;

  Serial.begin(115200);
  Timer1.initialize(200);
  Timer1.attachInterrupt(triggerSensor);
  //Serial.println("Setup");
}


long  lastTest = 0;
int testInterv = 400;

void loop() {

  stateSwitch();
  serialTask();
}
