
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
#define NUMBER_OF_ACTIVE_ADC_PINS (sizeof(activeAdcPins)/sizeof(activeAdcPins[0]))
#define ADC_VALUE_DELTA (2)
#define INCOMING_DATA_BUFFER_SIZE (20)
#define INCOMING_DATA_COMMAND_SIZE (15)
#define COMMAND_SEPARATOR ':'
#define BLE_PAYLOAD_SIZE_MAX (20)

#define PERSONALITY_REQUEST_COMMAND "PERS"
#define DIGITAL_OUTPUT_UPDATE_COMMAND "OUT"
#define DIGITAL_INPUT_UPDATE_COMMAND "OUT"

#define DIGITAL_OUTPUT_UPDATE_COMMAND_PIN_INDEX (1)
#define DIGITAL_OUTPUT_UPDATE_COMMAND_STATE_INDEX (2)

#define GPIO_INTPUT_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:INP:"
#define GPIO_OUTPUT_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:OUT:"
#define ADC_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:ADC:"
#define PWM_PINS_AVAILABLE_RESPONSE_COMMAND "PERS:PWM:"

SoftwareSerial bleSerial(7, 8); // RX, TX

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

typedef struct
{
  int number;
  char *charNumber;
  int lastValue;
} AdcPin;

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
};

static AdcPin activeAdcPins[] = {
  { A0, "A0", INPUT_INACTIVE },
//  { A1, "A1", INPUT_INACTIVE },
  { A2, "A2", INPUT_INACTIVE }
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
  bleSerial.print("GPIO:");
  bleSerial.print("INPUT:");
  bleSerial.print(pinNumber);
  bleSerial.print(":");
  bleSerial.print(state);
}

void SendAdcInputState(char *pinNumber, int value)
{
  bleSerial.print("ADC:");
  bleSerial.print(pinNumber);
  bleSerial.print(":");
  bleSerial.print(value);
  delay(5);
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

void monitorAdcPins()
{
  for (char i = 0; i < NUMBER_OF_ACTIVE_ADC_PINS; i++)
  {
    int currentValue = analogRead(activeAdcPins[i].number);
    if (abs(currentValue - activeAdcPins[i].lastValue) > ADC_VALUE_DELTA)
    {
      SendAdcInputState(activeAdcPins[i].charNumber, currentValue);
      activeAdcPins[i].lastValue = currentValue;  
    }
  }
}

/********** Command Handling **************/
void saveIncomingData()
{
  char incomingDataIndex = 0;
  while (bleSerial.available() > 0)
  {
    incomingData[incomingDataIndex] = bleSerial.read();
    incomingDataIndex++;
    delay(5);
  }
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

  byteCount += bleSerial.print(GPIO_INTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
  byteCount += bleSerial.print(NUMBER_OF_ACTIVE_GPIO_INPUT_PINS);
  byteCount += bleSerial.print(':');
  for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS; i++)
  {
    if (byteCount >= BLE_PAYLOAD_SIZE_MAX) //When message buffer overflow happens, send the message header again
    {
      byteCount = 0;
      byteCount += bleSerial.print(GPIO_INTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
      byteCount += bleSerial.print(NUMBER_OF_ACTIVE_GPIO_INPUT_PINS);
      byteCount += bleSerial.print(':');
    }
    byteCount += bleSerial.print(activeGpioInputPins[i].number);
    if ((i < NUMBER_OF_ACTIVE_GPIO_INPUT_PINS - 1) && (i != BLE_PAYLOAD_SIZE_MAX - 1))
    {
      byteCount += bleSerial.print(',');
    }
    else
    {
      byteCount += bleSerial.print("");
    }
  }
}

void ProvideGPIOOutputPersonalityDetails()
{
  char byteCount;

  byteCount += bleSerial.print(GPIO_OUTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
  byteCount += bleSerial.print(NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS);
  byteCount += bleSerial.print(':');
  for (char i = 0; i < NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS; i++)
  {
    if (byteCount == BLE_PAYLOAD_SIZE_MAX) //When message buffer overflow happens, send the message header again
    {
      byteCount = 0;
      byteCount += bleSerial.print(GPIO_OUTPUT_PINS_AVAILABLE_RESPONSE_COMMAND);
      byteCount += bleSerial.print(NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS);
      byteCount += bleSerial.print(':');
    }
    
    byteCount += bleSerial.print(activeGpioOutputPins[i].number);
    
    if ((i < NUMBER_OF_ACTIVE_GPIO_OUTPUT_PINS - 1) && (i != BLE_PAYLOAD_SIZE_MAX - 1))
    {
      byteCount += bleSerial.print(',');
    }
    else
    {
      byteCount += bleSerial.print("");
    }
  }
}

void ProvideAdcInputPersonalityDetails()
{
  uint8_t byteCount = 0;

  byteCount += bleSerial.print(ADC_PINS_AVAILABLE_RESPONSE_COMMAND);
  byteCount += bleSerial.print(NUMBER_OF_ACTIVE_ADC_PINS);
  byteCount += bleSerial.print(':');
  for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_ADC_PINS; i++)
  {
    if (byteCount >= BLE_PAYLOAD_SIZE_MAX) //When message buffer overflow happens, send the message header again
    {
      byteCount = 0;
      byteCount += bleSerial.print(ADC_PINS_AVAILABLE_RESPONSE_COMMAND);
      byteCount += bleSerial.print(NUMBER_OF_ACTIVE_ADC_PINS);
      byteCount += bleSerial.print(':');
    }
    
    byteCount += bleSerial.print(activeAdcPins[i].charNumber);
    
    if ((i < NUMBER_OF_ACTIVE_ADC_PINS - 1) && (i != BLE_PAYLOAD_SIZE_MAX - 1))
    {
      byteCount += bleSerial.print(',');
    }
    else
    {
      byteCount += bleSerial.print("");
    }
  }
}

void processIncomingData()
{
  if (!strcmp(personalityRequestCommand, incomingDataCommand)) //Personality command
  {
    ProvideGPIOInputPersonalityDetails();
    delay(10);
    ProvideGPIOOutputPersonalityDetails();
    delay(10);
    ProvideAdcInputPersonalityDetails();
    delay(10);
  }
  else if (!strcmp(digitalOutputUpdateCommand, incomingDataCommand)) // Digital Output Update Command
  {
    uint8_t pinNumber = extractPayloadData(DIGITAL_OUTPUT_UPDATE_COMMAND_PIN_INDEX);
    uint8_t state = extractPayloadData(DIGITAL_OUTPUT_UPDATE_COMMAND_STATE_INDEX);

    digitalWrite(pinNumber, state);
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
  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  bleSerial.begin(9600);
  bleSerial.println("Hello, world?");
}

void loop() // run over and over
{
  monitorInputPins();
  monitorAdcPins();

  if (bleSerial.available())
  {
    saveIncomingData();
    extractCommand();
    processIncomingData();

    clearIncomingDataBuffer();
    clearIncomingDataCommandBuffer();
  }
}

