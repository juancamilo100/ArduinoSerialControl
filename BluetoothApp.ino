
#include <SoftwareSerial.h>
#include <string.h>

#define INPUT_ACTIVE (1)
#define INPUT_INACTIVE (0)
#define NUMBER_OF_ACTIVE_GPIO_INPUT_PINS (sizeof(activeGpioInputPins)/sizeof(activeGpioInputPins[0]))
#define NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS (sizeof(activeGpioOutputPins)/sizeof(activeGpioOutputPins[0]))
#define INCOMING_DATA_BUFFER_SIZE (20)
#define INCOMING_DATA_COMMAND_SIZE (15)
#define COMMAND_SEPARATOR ':'
#define BLE_PAYLOAD_SIZE_MAX (20)

#define PERSONALITY_REQUEST_COMMAND "PERS"

#define GPIO_INTPUT_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:IN:"
#define GPIO_OUTPUT_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:OUT:"
#define PWM_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:PWM:"
#define PWM_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:ADC:"

SoftwareSerial mySerial(7, 8); // RX, TX

static char *personalityRequestCommand = PERSONALITY_REQUEST_COMMAND;

static char incomingData[INCOMING_DATA_BUFFER_SIZE];
static char incomingDataCommand[INCOMING_DATA_COMMAND_SIZE];

typedef struct
{
  uint8_t number;
  uint8_t lastState;
} InputPin;

typedef struct
{
  uint8_t number;
} OutputPin;

static InputPin activeGpioInputPins[] = {
  { 2, INPUT_INACTIVE },
  { 3, INPUT_INACTIVE },
  { 5, INPUT_INACTIVE },
  { 10, INPUT_INACTIVE }
//  { 11, INPUT_INACTIVE }
};

static OutputPin activeGpioOutputPins[] = {
  { 4 },
  { 6 },
  { 9 },
  { 13 },
  { 12 }
};

void setGpioInputs()
{
  for (char i = 0; i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS; i++)
  {
    pinMode(activeGpioInputPins[i].number, INPUT_PULLUP);
  }
}

void setGpioOutputs()
{
  for (char i = 0; i < NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS; i++)
  {
    pinMode(activeGpioOutputPins[i].number, OUTPUT);
  }
}

void SendGPIOInputState(uint8_t pinNumber, bool state)
{
  mySerial.print("GPIO:");
  mySerial.print("INPUT:");
  mySerial.print(pinNumber);
  mySerial.print(":");
  mySerial.print(state);
}

void monitorInputPins()
{
  for (char i = 0; i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS; i++)
  {
    bool currentState = digitalRead(activeGpioInputPins[i].number);
    if (currentState != activeGpioInputPins[i].lastState)
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
  while (mySerial.available() > 0)
  {
    incomingData[incomingDataIndex] = mySerial.read();
    incomingDataIndex++;
    delay(5);
  }
  Serial.print("The complete data is: ");
  Serial.println(incomingData);
}

uint8_t getNumberOfParams()
{
  uint8_t paramCounter = 0;
  for (uint8_t i = 0; i < INCOMING_DATA_BUFFER_SIZE; i++)
  {
    if (incomingData[i] == COMMAND_SEPARATOR)
    {
      Serial.println("Got a param!");
      paramCounter++;
    }
  }
  Serial.print("The number of params is: ");
  Serial.println(paramCounter, DEC);
  return paramCounter;
}

void extractCommand()
{
  char currentCharacter;

  for (uint8_t commandIndex = 0; commandIndex < INCOMING_DATA_COMMAND_SIZE - 1; commandIndex++)
  {
    currentCharacter = incomingData[commandIndex];
    if (currentCharacter == COMMAND_SEPARATOR)
    {
      incomingDataCommand[commandIndex] = '\0';
      Serial.print("The command is: ");
      Serial.println(incomingDataCommand);
      break;
    }
    incomingDataCommand[commandIndex] = currentCharacter;
  }
}

void clearIncomingDataBuffer()
{
  for (uint8_t i = 0; i < INCOMING_DATA_BUFFER_SIZE; i++)
  {
    incomingData[i] = 0;
  }
}

void clearIncomingDataCommandBuffer()
{
  for (uint8_t i = 0; i < INCOMING_DATA_BUFFER_SIZE; i++)
  {
    incomingDataCommand[i] = 0;
  }
}

void ProvideGPIOInputPersonalityDetails()
{
  char byteCount;

  byteCount += mySerial.print(GPIO_INTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
  byteCount += mySerial.print(NUMBER_OF_ACTIVE_GPIO_INPUT_PINS);
  byteCount += mySerial.print(':');
  for (char i = 0; i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS; i++)
  {
    if (byteCount == BLE_PAYLOAD_SIZE_MAX) //When message buffer overflow happens, send the message header again
    {
      byteCount = 0;
      byteCount += mySerial.print(GPIO_OUTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
      byteCount += mySerial.print(NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS);
      byteCount += mySerial.print(':');
    }
    byteCount += mySerial.print(activeGpioInputPins[i].number);
    if ((i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS - 1) && (i != BLE_PAYLOAD_SIZE_MAX - 1))
    {
      byteCount += mySerial.print(',');
    }
    else
    {
      byteCount += mySerial.print("");
    }
  }
}

void ProvideGPIOOutputPersonalityDetails()
{
  char byteCount;

  byteCount += mySerial.print(GPIO_OUTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
  byteCount += mySerial.print(NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS);
  byteCount += mySerial.print(':');
  for (char i = 0; i < NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS; i++)
  {
    if (byteCount == BLE_PAYLOAD_SIZE_MAX) //When message buffer overflow happens, send the message header again
    {
      byteCount = 0;
      byteCount += mySerial.print(GPIO_OUTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
      byteCount += mySerial.print(NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS);
      byteCount += mySerial.print(':');
    }
    byteCount += mySerial.print(activeGpioOutputPins[i].number);
    if ((i < NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS - 1) && (i != BLE_PAYLOAD_SIZE_MAX - 1))
    {
      byteCount += mySerial.print(',');
    }
    else
    {
      byteCount += mySerial.print("");
    }
  }
}

void processIncomingData()
{
  if (!strcmp(personalityRequestCommand, incomingDataCommand))
  {
    Serial.println("The commands are equal");
    ProvideGPIOInputPersonalityDetails();
    delay(10);
    ProvideGPIOOutputPersonalityDetails();
  }
}

void setup()
{
  setGpioInputs();
  setGpioOutputs();
  clearIncomingDataBuffer();
  clearIncomingDataCommandBuffer();

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

  if (mySerial.available())
  {
    saveIncomingData();
    extractCommand();

    processIncomingData();
    clearIncomingDataBuffer();
    clearIncomingDataCommandBuffer();
  }
}

