#include <Servo.h>

int numberOfServos = 4;
int savedServoPositions[ 100 ][ 4 ];
int numberOfSavedServoPositions = 0;
int currentPlaybackStep = 0;
unsigned long lastPlaybackTimestamp;
int timeBetweenPlaybackSteps = 500;

enum SelectedWorkflow
{
  Sandbox,
  Recording,
  Playback
};

SelectedWorkflow selectedWorkflow = SelectedWorkflow::Sandbox;

struct RobotServo
{
  const byte digitalServoPin;
  const byte analogPotPin;
  const int servoMinimumValue;
  const int servoMaximumValue;
  int servoValue;
  int prevServoValue;
  int potValue;

  RobotServo(int digitalServoPin, int analogPotPin, int servoMinimumValue, int servoMaximumValue) : digitalServoPin(digitalServoPin), analogPotPin(analogPotPin), servoMinimumValue(servoMinimumValue), servoMaximumValue(servoMaximumValue) 
  {}

  void updateServoPosition()
  {
    potValue = analogRead(analogPotPin);
    servoValue = map(potValue, 0, 1023, servoMinimumValue, servoMaximumValue);
    if( servoValue != prevServoValue){
      Serial.println((String)"Value of servo " + digitalServoPin + " was changed to " + servoValue);  
    }
    prevServoValue = servoValue;
  }

  void playbackServo(int servoNumber)
  {
    servoValue = savedServoPositions[currentPlaybackStep][servoNumber];
    Serial.println((String)"Playback: Value of servo " + digitalServoPin + " was changed to " + servoValue + " at step " + currentPlaybackStep);
  }
};

struct WorkflowStateController
{
  const byte digitalButtonPin;
  const byte digitalLedPin;
  bool isActive = false;
  int buttonState = 0;
  int prevButtonState;
  SelectedWorkflow controlledWorkflow;

  WorkflowStateController(byte digitalButtonPin, byte digitalLedPin, SelectedWorkflow controlledWorkflow) : digitalButtonPin(digitalButtonPin), digitalLedPin(digitalLedPin), controlledWorkflow(controlledWorkflow)
  {
    pinMode(digitalLedPin, OUTPUT);
    pinMode(digitalButtonPin, INPUT);
    prevButtonState = digitalRead(digitalButtonPin);
  }

  void updateStatus()
  {
    buttonState = digitalRead(digitalButtonPin);
    if (buttonState == LOW && prevButtonState == HIGH )
    {
      Serial.println((String)"Status of button " + digitalButtonPin + " was changed to " + isActive); 
      if (isActive == false) {
        digitalWrite(digitalLedPin, HIGH);
        isActive = true;
        if (controlledWorkflow == SelectedWorkflow::Recording)
        {
          numberOfSavedServoPositions = 0;
        }
        selectedWorkflow = controlledWorkflow;
      } else {
        digitalWrite(digitalLedPin, LOW);
        isActive = false;
        selectedWorkflow = SelectedWorkflow::Sandbox;
      }
    } 
    prevButtonState = digitalRead(digitalButtonPin);
  }
};

struct PositionSaver
{
  const byte digitalButtonPin;
  const byte digitalLedPin;
  bool isActive = false;
  int buttonState = 0;
  int prevButtonState;

  PositionSaver(byte digitalButtonPin, byte digitalLedPin) : digitalButtonPin(digitalButtonPin), digitalLedPin(digitalLedPin)
  {
    pinMode(digitalLedPin, OUTPUT);
    pinMode(digitalButtonPin, INPUT);
    prevButtonState = digitalRead(digitalButtonPin);
  }

  void saveServoPosition(RobotServo robotServos[], int arraySize)
  {
    buttonState = digitalRead(digitalButtonPin);
    if (buttonState == HIGH)
    {
      digitalWrite(digitalLedPin, HIGH);
    } else {
      digitalWrite(digitalLedPin, LOW);
    }
    if (buttonState == LOW && prevButtonState == HIGH )
    {
      for(int i = 0; i < arraySize; i++){
        savedServoPositions[numberOfSavedServoPositions][i] = robotServos[i].servoValue;
        Serial.println((String)"Saved Position " + savedServoPositions[numberOfSavedServoPositions][i] + " for servo " + i + " at position " + numberOfSavedServoPositions); 
      }
      numberOfSavedServoPositions++;
    } 
    prevButtonState = digitalRead(digitalButtonPin);
  }
};

//*************BEGIN CONFIG*************************
RobotServo robotServoBase{ 2, 1, 180, 0 };
RobotServo robotServoLeft{ 3, 2, 20, 170 };
RobotServo robotServoClaw{ 4, 3, 0, 180 };
RobotServo robotServoRight{ 5, 4, 170, 10 };

RobotServo robotServos[4] = {robotServoBase, robotServoLeft, robotServoClaw, robotServoRight};
Servo servos[4];

WorkflowStateController recordMode{ 10, 11, SelectedWorkflow::Recording};
WorkflowStateController playbackMode{ 6, 7, SelectedWorkflow::Playback};

PositionSaver positionSaver{8, 9};
//**************END CONFIG**********************

void setup() 
{
  Serial.begin(9600);
  for(int i = 0; i < 4; i++){
    servos[i].attach(robotServos[i].digitalServoPin);
  }
}

void loop() 
{
  if(selectedWorkflow == SelectedWorkflow::Playback){
    if(numberOfSavedServoPositions > 0 && millis()-lastPlaybackTimestamp > timeBetweenPlaybackSteps){
      if( currentPlaybackStep >= numberOfSavedServoPositions)
      {
        currentPlaybackStep = 0;
      }
      for(int i = 0; i < sizeof(robotServos)/sizeof(robotServos[0]); i++){
        robotServos[i].playbackServo(i);
        servos[i].write(robotServos[i].servoValue);
      }
      currentPlaybackStep++;
      lastPlaybackTimestamp = millis();
    }
    playbackMode.updateStatus();
  } else if(selectedWorkflow == SelectedWorkflow::Recording){
    positionSaver.saveServoPosition(robotServos, sizeof(robotServos)/sizeof(robotServos[0]));
    for(int i = 0; i < sizeof(robotServos)/sizeof(robotServos[0]); i++){
      robotServos[i].updateServoPosition();
      servos[i].write(robotServos[i].servoValue);
    }
    recordMode.updateStatus();
  } else if(selectedWorkflow == SelectedWorkflow::Sandbox){
    for(int i = 0; i < sizeof(robotServos)/sizeof(robotServos[0]); i++){
      robotServos[i].updateServoPosition();
      servos[i].write(robotServos[i].servoValue);
    }
    recordMode.updateStatus();
    playbackMode.updateStatus();
  }
    
  delay(50); 
}
