
#include <SoftwareSerial.h>

#define INPUT_ACTIVE (1)
#define INPUT_INACTIVE (0)
#define NUMBER_OF_ACTIVE_GPIO_INPUT_PINS (sizeof(activeGpioInputPins)/sizeof(activeGpioInputPins[0]))
#define NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS (sizeof(activeGpioOutputPins)/sizeof(activeGpioOutputPins[0]))
#define INCOMING_DATA_BUFFER_SIZE (20)
#define INCOMING_DATA_COMMAND_SIZE (10)

SoftwareSerial mySerial(7, 8); // RX, TX

static char incomingData[INCOMING_DATA_BUFFER_SIZE];
static char incomingDataCommand[INCOMING_DATA_COMMAND_SIZE];

typedef struct
{
  uint8_t number;
  uint8_t lastState;
} InputPin;

static InputPin activeGpioInputPins[] = {
  { 2, INPUT_INACTIVE },
  { 3, INPUT_INACTIVE },
  { 5, INPUT_INACTIVE },
};

static const char activeGpioOutputPins[] = {
  4,
  6,
  9,
  13
};

void setGpioInputs()
{
  for(char i = 0; i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS; i++)
  {
    pinMode(activeGpioInputPins[i].number, INPUT_PULLUP);
  }
}

void setGpioOutputs()
{
  for(char i = 0; i < NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS; i++)
  {
    pinMode(activeGpioOutputPins[i], OUTPUT);
  }
}

void SendGPIOInputState(uint8_t pinNumber, bool state)
{
//  Serial.print("GPIO:");
//  Serial.print("INPUT:");
//  Serial.print(pinNumber);
//  Serial.print(":");
//  Serial.print(state);
//  
  mySerial.print("GPIO:");
  mySerial.print("INPUT:");
  mySerial.print(pinNumber);
  mySerial.print(":");
  mySerial.print(state);
}

void monitorInputPins()
{
  for(char i = 0; i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS; i++)
  {
     bool currentState = digitalRead(activeGpioInputPins[i].number);
     if(currentState != activeGpioInputPins[i].lastState)
     {
        SendGPIOInputState(activeGpioInputPins[i].number, currentState);
        activeGpioInputPins[i].lastState = currentState;
     }
  }
}

/********** Command Handling **************/
void saveIncomingData()
{
  char incomingDataIndex = 0;
  while(mySerial.available() > 0)
  {
    incomingData[incomingDataIndex] = mySerial.read();
    incomingDataIndex++;
    delay(5);
  }
}

uint8_t getNumberOfParams()
{
  uint8_t paramCounter = 0;
  for(uint8_t i = 0; i < INCOMING_DATA_BUFFER_SIZE; i++)
  {
    if(incomingData[i] == ':')
    {
      Serial.println("Got a param!");
      paramCounter++;
    }
  }
  return paramCounter;
}

void extractCommand()
{
  char currentCharacter;
  
  for(uint8_t commandIndex = 0; commandIndex< INCOMING_DATA_COMMAND_SIZE; commandIndex++)
  {    
    currentCharacter = incomingData[commandIndex];
    if(currentCharacter == ':')
    {
      break;
    }
    incomingDataCommand[commandIndex] = currentCharacter;
  }
}

void clearIncomingDataBuffer()
{
  for(uint8_t i = 0; i < INCOMING_DATA_BUFFER_SIZE; i++)
  {
    incomingData[i] = 0;
  }
}

void clearIncomingDataCommandBuffer()
{
  for(uint8_t i = 0; i < INCOMING_DATA_BUFFER_SIZE; i++)
  {
    incomingDataCommand[i] = 0;
  }
}

void processIncomingData(uint8_t numberOfParams)
{
  
}

void setup()
{
  setGpioInputs();
  setGpioOutputs();
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
}

void loop() // run over and over
{
  monitorInputPins();

  if(mySerial.available())
  {
    saveIncomingData();
    extractCommand();
    uint8_t numberOfParams = getNumberOfParams();
    
    Serial.print("The complete data is: ");
    Serial.println(incomingData);
    Serial.print("The number of params is: ");
    Serial.println(numberOfParams, DEC);
    Serial.print("The command is: ");
    Serial.println(incomingDataCommand);
    clearIncomingDataBuffer();
    clearIncomingDataCommandBuffer();
  }
}

