
#include <SoftwareSerial.h>
#include <string.h>

/* BLE PROTOCOL:
 *  Personality: PERS:PeripheralType:
 *  GPIO Input Update: GPIO:INPUT:PinNumber:State
 *  GPIO Output Update:
 */

#define INPUT_ACTIVE (1)
#define INPUT_INACTIVE (0)
#define NUMBER_OF_ACTIVE_GPIO_INPUT_PINS (sizeof(activeGpioInputPins)/sizeof(activeGpioInputPins[0]))
#define NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS (sizeof(activeGpioOutputPins)/sizeof(activeGpioOutputPins[0]))
#define INCOMING_DATA_BUFFER_SIZE (20)
#define INCOMING_DATA_COMMAND_SIZE (15)
#define COMMAND_SEPARATOR ':'
#define BLE_PAYLOAD_SIZE_MAX (20)

#define PERSONALITY_REQUEST_COMMAND "PERS"
#define DIGITAL_OUTPUT_UPDATE_COMMAND "OUT"
#define DIGITAL_INPUT_UPDATE_COMMAND "OUT"

#define GPIO_INTPUT_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:INP:"
#define GPIO_OUTPUT_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:OUT:"
#define PWM_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:PWM:"
#define PWM_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:ADC:"

SoftwareSerial mySerial(7, 8); // RX, TX

static char *personalityRequestCommand = PERSONALITY_REQUEST_COMMAND;
static char *digitalOutputUpdateCommand = DIGITAL_OUTPUT_UPDATE_COMMAND;

static char incomingData[INCOMING_DATA_BUFFER_SIZE];
static char incomingDataCommand[INCOMING_DATA_COMMAND_SIZE];
static char payloadData[INCOMING_DATA_BUFFER_SIZE];

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
  { 6, INPUT_INACTIVE },
  { 3, INPUT_INACTIVE },
  { 5, INPUT_INACTIVE },
  { 10, INPUT_INACTIVE },
  { 4, INPUT_INACTIVE },
  { 12, INPUT_INACTIVE },

};

static OutputPin activeGpioOutputPins[] = {
  { 2 },
  { 11 },
  { 9 },
  { 13 }
  //  { 12 },
  //  { 11 }
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
  //  Serial.print("The complete data is: ");
  //  Serial.println(incomingData);
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

uint8_t extractPayloadData(uint8_t dataIndex)
{
  char currentCharacter;
  uint8_t indexCounter = 0;

  for (uint8_t index = 0; index < INCOMING_DATA_BUFFER_SIZE - 1; index++)
  {
    currentCharacter = incomingData[index];

    if (indexCounter == dataIndex)
    {
      for (uint8_t targetDataIndex = 0; targetDataIndex < INCOMING_DATA_BUFFER_SIZE - 1; targetDataIndex++)
      {
        currentCharacter = incomingData[index];

        if (currentCharacter == COMMAND_SEPARATOR)
        {
          
          payloadData[targetDataIndex] = '\0';
          
//          Serial.print(atoi(payloadData));
//          Serial.println("");
//
//          Serial.println("Return!");
          
          goto end_nested_loop;
        }
        payloadData[targetDataIndex] = incomingData[index];
        index++;
      }
    }

    if (currentCharacter == COMMAND_SEPARATOR)
    {
      indexCounter++;
    }
  }
end_nested_loop: return atoi(payloadData);
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
  uint8_t byteCount = 0;

  byteCount += mySerial.print(GPIO_INTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
  byteCount += mySerial.print(NUMBER_OF_ACTIVE_GPIO_INPUT_PINS);
  byteCount += mySerial.print(':');
  for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS; i++)
  {
    //    Serial.print("BYTE COUNT: ");
    //    Serial.println(byteCount);
    if (byteCount >= BLE_PAYLOAD_SIZE_MAX) //When message buffer overflow happens, send the message header again
    {
      byteCount = 0;
      byteCount += mySerial.print(GPIO_INTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
      byteCount += mySerial.print(NUMBER_OF_ACTIVE_GPIO_INPUT_PINS);
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
    ProvideGPIOInputPersonalityDetails();
    delay(10);
    ProvideGPIOOutputPersonalityDetails();
  }
  else if (!strcmp(digitalOutputUpdateCommand, incomingDataCommand))
  {
    Serial.print("Got GPIO Output update command with value: ");
    Serial.println(extractPayloadData(1));
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

