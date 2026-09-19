#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <string.h>
#include <stdlib.h>

//========================== LCD ==========================
// Try 0x27 first.
// If LCD does not show, try 0x3F.
// For TinkerCAD, try 0x20.
LiquidCrystal_I2C lcd(0x27, 16, 2);

//========================== DHT11 ==========================
// 3-pin DHT11 module:
// VCC  -> 5V
// DATA -> D4
// GND  -> GND
#define DHT_PIN 4
#define DHT_TYPE DHT11

DHT dht(DHT_PIN, DHT_TYPE);

//========================== FUNCTION PROTOTYPES ==========================
void beepButton();
void beepMode();
void beepWaterStart();
void beepWaterStop();
void beepPause();
void beepError();

//========================== AUTO CLASS ==========================
class Auto
{
  public:
    Auto();
    void commands();
    bool checkCommand(char cmd[]);
    unsigned long getTimer();
    int getThreshold();
    const char* getCropName();

  private:
    enum Crops { BEANS = 2, PEPPERS = 3, TOMATOS = 4 };
    Crops _crop;
    int _threshold;
    unsigned long _timer;
    const char* _cropName;
};

Auto::Auto()
{
  _crop = BEANS;
  _threshold = 450;
  _timer = 3000;
  _cropName = "beans";
}

//Display auto mode commands
void Auto::commands()
{
  Serial.println(F("1. beans"));
  Serial.println(F("2. peppers"));
  Serial.println(F("3. tomatos"));
}


bool Auto::checkCommand(char cmd[])
{
  if(strcmp(cmd, "beans") == 0)
  {
    _crop = BEANS;
  }
  else if(strcmp(cmd, "peppers") == 0)
  {
    _crop = PEPPERS;
  }
  else if(strcmp(cmd, "tomatos") == 0 || strcmp(cmd, "tomatoes") == 0)
  {
    _crop = TOMATOS;
  }
  else if(strlen(cmd) == 0)
  {
    return false;
  }
  else
  {
    Serial.println(F("Crop not recognized."));
    commands();
    beepError();
    return false;
  }

  switch(_crop)
  {
    case BEANS:
      _threshold = 450;
      _timer = 3000;
      _cropName = "beans";
      break;

    case PEPPERS:
      _threshold = 500;
      _timer = 4000;
      _cropName = "peppers";
      break;

    case TOMATOS:
      _threshold = 550;
      _timer = 5000;
      _cropName = "tomatos";
      break;
  }

  beepMode();

  Serial.print(F("Threshold: "));
  Serial.println(_threshold);
  Serial.print(F("Water time(ms): "));
  Serial.println(_timer);

  return true;
}

int Auto::getThreshold()
{
  return _threshold;
}

unsigned long Auto::getTimer()
{
  return _timer;
}

const char* Auto::getCropName()
{
  return _cropName;
}

//========================== MANUAL CLASS ==========================
class Manual
{
  public:
    Manual();
    void commands();
    bool checkCommand(char cmd[], char desiredTime[]);
    unsigned long getTimer();
    void setTimer(unsigned long newTimer);

  private:
    enum TimerType { SECONDS, MINUTES, HOURS };
    TimerType timerType;
    unsigned long _timer;
    unsigned long _multiplier;
};

Manual::Manual()
{
  _timer = 0;
  _multiplier = 1000;
}

//Display Manual mode commands
void Manual::commands()
{
  Serial.println(F("1. timer.s=X"));
  Serial.println(F("2. timer.m=X"));
  Serial.println(F("3. timer.h=X"));
  Serial.println(F("4. water"));
}

bool Manual::checkCommand(char cmd[], char desiredTime[])
{
  if(strcmp(cmd, "timer.s=") == 0 || strcmp(cmd, "time.s=") == 0)
  {
    timerType = SECONDS;
    _multiplier = 1000UL;
  }
  else if(strcmp(cmd, "timer.m=") == 0 || strcmp(cmd, "time.m=") == 0)
  {
    timerType = MINUTES;
    _multiplier = 60000UL;
  }
  else if(strcmp(cmd, "timer.h=") == 0 || strcmp(cmd, "time.h=") == 0)
  {
    timerType = HOURS;
    _multiplier = 3600000UL;
  }
  else if(strlen(cmd) == 0)
  {
    return false;
  }
  else
  {
    Serial.println(F("Command not recognized."));
    commands();
    beepError();
    return false;
  }

  if(strlen(desiredTime) == 0)
  {
    Serial.println(F("No time entered."));
    beepError();
    return false;
  }

  for(int i = 0; desiredTime[i] != '\0'; i++)
  {
    if(desiredTime[i] < '0' || desiredTime[i] > '9')
    {
      Serial.println(F("Non numerical value."));
      beepError();
      return false;
    }
  }

  unsigned long timeValue = atol(desiredTime);
  _timer = timeValue * _multiplier;

  beepMode();

  Serial.print(F("Manual timer(ms): "));
  Serial.println(_timer);

  return true;
}

unsigned long Manual::getTimer()
{
  return _timer;
}

void Manual::setTimer(unsigned long newTimer)
{
  _timer = newTimer;
}

//========================== OBJECTS ==========================
Manual manual;
Auto autoObj;

//========================== PIN CONSTANTS ==========================
const int MOIST_SENSOR = A0;

const int MODE_BTN = 2;
const int WATER_BTN = 3;

const int SOLENOID_RELAY = 8;
const int BUZZER_PIN = 9;

const int RELAY_ON = HIGH;
const int RELAY_OFF = LOW;

//========================== TIMERS ==========================
unsigned long currentTime = 0;
unsigned long prevTime = 0;
unsigned long prevSerialTime = 0;
unsigned long waterStartMillis = 0;
unsigned long activeWaterTimer = 0;
unsigned long lastModeButton = 0;
unsigned long lastWaterButton = 0;
unsigned long lastAutoWaterTime = 0;
unsigned long lastDHTReadTime = 0;

const unsigned long DEFAULT_MANUAL_TIMER = 5000;
const unsigned long BUTTON_DELAY = 250;
const unsigned long AUTO_WATER_DELAY = 8000;
const unsigned long DHT_READ_DELAY = 2500;

//========================== SENSORS ==========================
int moisture = 0;
int temperatureC = 0;
int humidity = 0;
bool dhtOK = false;

//========================== SERIAL COMMAND BUFFER ==========================
char inputBuffer[32];
char commandBuffer[32];
byte inputIndex = 0;
unsigned long lastSerialCharTime = 0;
const unsigned long SERIAL_TIMEOUT = 100;

//========================== STATE VARIABLES ==========================
volatile bool running = true;
volatile bool mode = true; // true = AUTO, false = MANUAL
volatile bool buttonWaterRequest = false;
volatile bool modeChangeRequest = false;

bool prevMode = false;
bool startWater = false;
bool valveIsOpen = false;
bool forceWater = false;

//========================== FUNCTION PROTOTYPES ==========================
void readSensors();
void updateLCD();
void clearLCDRow(int row);
void printData();

bool getSerialCommand();
void cleanCommand(char cmd[]);
void processCommand(char cmd[]);
void autoMode(char cmd[]);
void manualMode(char cmd[]);

void waterLogic(bool currentMode);
void sprinklerOn();
void sprinklerOff();

void printMainCommands();

void changeMode();
void waterButton();

void beepButton();
void beepMode();
void beepWaterStart();
void beepWaterStop();
void beepPause();
void beepError();

//========================== SETUP ==========================
void setup()
{
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  dht.begin();

  pinMode(MODE_BTN, INPUT_PULLUP);
  pinMode(WATER_BTN, INPUT_PULLUP);
  pinMode(SOLENOID_RELAY, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(SOLENOID_RELAY, RELAY_OFF);
  noTone(BUZZER_PIN);

  attachInterrupt(digitalPinToInterrupt(MODE_BTN), changeMode, FALLING);
  attachInterrupt(digitalPinToInterrupt(WATER_BTN), waterButton, FALLING);

  lcd.setCursor(0, 0);
  lcd.print(F("Smart Irrigation"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));

  tone(BUZZER_PIN, 1000, 150);
  delay(1000);

  Serial.println(F("Smart Irrigation System"));
  Serial.println(F("DHT11 on D4."));
  printMainCommands();

  prevMode = !mode;
}

//========================== LOOP ==========================
void loop()
{
  currentTime = millis();

  //Check for mode change requests using button
  noInterrupts();
  if(modeChangeRequest && currentTime - lastModeButton > BUTTON_DELAY)
  {
    modeChangeRequest = false;
    lastModeButton = currentTime;

    mode = !mode;
    running = true;
    sprinklerOff();
    startWater = false;
    forceWater = false;

    beepButton();
  }
  interrupts();

  //Check for watering requests using button
  noInterrupts();
  if(buttonWaterRequest && currentTime - lastWaterButton > BUTTON_DELAY)
  {
    buttonWaterRequest = false;
    lastWaterButton = currentTime;

    beepButton();

    if(mode == false)
    {
      if(manual.getTimer() == 0)
      {
        manual.setTimer(DEFAULT_MANUAL_TIMER);
      }

      Serial.println(F("Water button pressed."));
      forceWater = true;
      startWater = true;
    }
    else
    {
      running = !running;
      sprinklerOff();
      startWater = false;
      Serial.println(F("Auto pause/resume."));
      beepPause();
    }
  }
  interrupts();

  if(getSerialCommand())
  {
    processCommand(commandBuffer);
  }

  if(!running)
  {
    sprinklerOff();
    startWater = false;
  }

  //Main program logic
  if(currentTime - prevTime >= 500)
  {
    prevTime = currentTime;

    readSensors();
    updateLCD();

    //Check current mode
    if(mode == true)
    {
      //Check if mode has changed
      if(prevMode != mode)
      {
        sprinklerOff();
        startWater = false;
        Serial.println(F("Auto Mode."));
        autoObj.commands();
        prevMode = mode;
        beepMode();
      }

      if(running && !valveIsOpen && moisture < autoObj.getThreshold())
      {
        if(currentTime - lastAutoWaterTime >= AUTO_WATER_DELAY || lastAutoWaterTime == 0)
        {
          startWater = true;
          forceWater = false;
        }
      }
    }
    else
    {
      //Check if mode has changed
      if(prevMode != mode)
      {
        sprinklerOff();
        startWater = false;
        Serial.println(F("Manual Mode."));
        manual.commands();
        prevMode = mode;
        beepMode();
      }
    }
  }

  //Periodically print data
  printData();

  if(startWater && running)
  {
    waterLogic(mode);
  }
}

//========================== SENSOR FUNCTIONS ==========================
void readSensors()
{
  moisture = analogRead(MOIST_SENSOR);

  if(currentTime - lastDHTReadTime >= DHT_READ_DELAY || lastDHTReadTime == 0)
  {
    lastDHTReadTime = currentTime;

    float tempRead = dht.readTemperature();
    float humidityRead = dht.readHumidity();

    if(isnan(tempRead) || isnan(humidityRead))
    {
      dhtOK = false;
      Serial.println(F("DHT fail"));
    }
    else
    {
      dhtOK = true;
      temperatureC = (int)tempRead;
      humidity = (int)humidityRead;
    }
  }
}

//========================== LCD FUNCTIONS ==========================
void updateLCD()
{
  clearLCDRow(0);
  lcd.setCursor(0, 0);

  if(dhtOK)
  {
    lcd.print(F("T:"));
    lcd.print(temperatureC);
    lcd.print(F("C "));

    lcd.print(F("H:"));
    lcd.print(humidity);
    lcd.print(F("%"));
  }
  else
  {
    lcd.print(F("DHT ERROR"));
  }

  clearLCDRow(1);
  lcd.setCursor(0, 1);

  lcd.print(F("S:"));
  lcd.print(moisture);
  lcd.print(F(" "));

  if(mode == true)
  {
    lcd.print(F("A"));
  }
  else
  {
    lcd.print(F("M"));
  }

  lcd.print(F(" "));

  if(valveIsOpen)
  {
    lcd.print(F("ON"));
  }
  else
  {
    lcd.print(F("OFF"));
  }
}

void clearLCDRow(int row)
{
  lcd.setCursor(0, row);
  lcd.print(F("                "));
  lcd.setCursor(0, row);
}

//========================== SERIAL DATA DISPLAY ==========================
void printData()
{
  if(currentTime - prevSerialTime >= 3000)
  {
    prevSerialTime = currentTime;

    Serial.print(F("Mode: "));

    if(mode == true)
    {
      Serial.print(F("AUTO"));
    }
    else
    {
      Serial.print(F("MANUAL"));
    }

    Serial.print(F(" | Soil: "));
    Serial.print(moisture);

    Serial.print(F(" | Temp: "));
    Serial.print(temperatureC);

    Serial.print(F("C | Hum: "));
    Serial.print(humidity);

    Serial.print(F("% | Valve: "));

    if(valveIsOpen)
    {
      Serial.println(F("ON"));
    }
    else
    {
      Serial.println(F("OFF"));
    }
  }
}

//========================== COMMAND FUNCTIONS ==========================
bool getSerialCommand()
{
  bool commandReady = false;

  while(Serial.available() > 0)
  {
    char c = Serial.read();

    if(c == '\n' || c == '\r')
    {
      if(inputIndex > 0)
      {
        commandReady = true;
        break;
      }
    }
    else
    {
      if(inputIndex < 31)
      {
        inputBuffer[inputIndex] = c;
        inputIndex++;
        lastSerialCharTime = millis();
      }
      else
      {
        inputIndex = 0;
        Serial.println(F("Command too long."));
        beepError();
      }
    }
  }

  if(!commandReady && inputIndex > 0)
  {
    if(millis() - lastSerialCharTime > SERIAL_TIMEOUT && Serial.available() == 0)
    {
      commandReady = true;
    }
  }

  if(commandReady)
  {
    inputBuffer[inputIndex] = '\0';

    strcpy(commandBuffer, inputBuffer);
    cleanCommand(commandBuffer);

    inputIndex = 0;
    inputBuffer[0] = '\0';

    return true;
  }

  return false;
}

void cleanCommand(char cmd[])
{
  for(int i = 0; cmd[i] != '\0'; i++)
  {
    if(cmd[i] >= 'A' && cmd[i] <= 'Z')
    {
      cmd[i] = cmd[i] + 32;
    }
  }

  while(cmd[0] == ' ')
  {
    for(int i = 0; cmd[i] != '\0'; i++)
    {
      cmd[i] = cmd[i + 1];
    }
  }

  int len = strlen(cmd);

  while(len > 0 && cmd[len - 1] == ' ')
  {
    cmd[len - 1] = '\0';
    len--;
  }
}

void processCommand(char cmd[])
{
  Serial.print(F("Cmd: "));
  Serial.println(cmd);

  if(strcmp(cmd, "auto") == 0)
  {
    mode = true;
    running = true;
    beepMode();
    return;
  }
  else if(strcmp(cmd, "manual") == 0)
  {
    mode = false;
    running = true;
    beepMode();
    return;
  }
  else if(strcmp(cmd, "pause") == 0)
  {
    running = false;
    sprinklerOff();
    startWater = false;
    Serial.println(F("Paused."));
    beepPause();
    return;
  }
  else if(strcmp(cmd, "resume") == 0)
  {
    running = true;
    Serial.println(F("Resumed."));
    beepMode();
    return;
  }
  else if(strcmp(cmd, "stop") == 0)
  {
    sprinklerOff();
    startWater = false;
    forceWater = false;
    Serial.println(F("Water stopped."));
    beepWaterStop();
    return;
  }
  else if(strcmp(cmd, "water") == 0)
  {
    if(mode == false && manual.getTimer() == 0)
    {
      manual.setTimer(DEFAULT_MANUAL_TIMER);
    }

    forceWater = true;
    startWater = true;
    Serial.println(F("Water command."));
    beepButton();
    return;
  }
  else if(strcmp(cmd, "help") == 0)
  {
    printMainCommands();
    beepButton();
    return;
  }

  if(mode == true)
  {
    autoMode(cmd);
  }
  else
  {
    manualMode(cmd);
  }
}

void autoMode(char cmd[])
{
  if(autoObj.checkCommand(cmd))
  {
    if(moisture < autoObj.getThreshold())
    {
      startWater = true;
    }
    else
    {
      Serial.println(F("Crop updated."));
    }
  }
}

void manualMode(char cmd[])
{
  char timerCmd[16];
  char desiredTime[16];

  timerCmd[0] = '\0';
  desiredTime[0] = '\0';

  int equalPos = -1;

  for(int i = 0; cmd[i] != '\0'; i++)
  {
    if(cmd[i] == '=')
    {
      equalPos = i;
      break;
    }
  }

  if(equalPos >= 0)
  {
    int i;

    for(i = 0; i <= equalPos && i < 15; i++)
    {
      timerCmd[i] = cmd[i];
    }

    timerCmd[i] = '\0';

    int j = 0;

    for(i = equalPos + 1; cmd[i] != '\0' && j < 15; i++)
    {
      desiredTime[j] = cmd[i];
      j++;
    }

    desiredTime[j] = '\0';
  }
  else
  {
    strcpy(timerCmd, cmd);
  }

  if(manual.checkCommand(timerCmd, desiredTime))
  {
    forceWater = true;
    startWater = true;
  }
}

//========================== WATERING FUNCTION ==========================
void waterLogic(bool currentMode)
{
  if(currentMode == true)
  {
    if(!valveIsOpen)
    {
      if(moisture < autoObj.getThreshold() || forceWater)
      {
        activeWaterTimer = autoObj.getTimer();
        waterStartMillis = currentTime;
        sprinklerOn();
        Serial.println(F("AUTO watering."));
      }
      else
      {
        startWater = false;
      }
    }
    else if(currentTime - waterStartMillis >= activeWaterTimer)
    {
      sprinklerOff();
      startWater = false;
      forceWater = false;
      lastAutoWaterTime = currentTime;
      Serial.println(F("Done watering."));
    }
  }
  else
  {
    if(!valveIsOpen)
    {
      activeWaterTimer = manual.getTimer();

      if(activeWaterTimer == 0)
      {
        activeWaterTimer = DEFAULT_MANUAL_TIMER;
      }

      waterStartMillis = currentTime;
      sprinklerOn();
      Serial.println(F("MANUAL watering."));
    }
    else if(currentTime - waterStartMillis >= activeWaterTimer)
    {
      sprinklerOff();
      startWater = false;
      forceWater = false;
      Serial.println(F("Done watering."));
    }
  }
}

//========================== VALVE FUNCTIONS ==========================
void sprinklerOn()
{
  digitalWrite(SOLENOID_RELAY, RELAY_ON);
  valveIsOpen = true;
  beepWaterStart();
}

void sprinklerOff()
{
  digitalWrite(SOLENOID_RELAY, RELAY_OFF);

  if(valveIsOpen)
  {
    beepWaterStop();
  }

  valveIsOpen = false;
}

//========================== MAIN COMMANDS ==========================
void printMainCommands()
{
  Serial.println();
  Serial.println(F("Main commands:"));
  Serial.println(F("auto / manual"));
  Serial.println(F("beans / peppers / tomatos"));
  Serial.println(F("timer.s=5 / timer.m=1"));
  Serial.println(F("water / stop / pause / resume / help"));
  Serial.println();
}

//========================== BUZZER FUNCTIONS ==========================
void beepButton()
{
  tone(BUZZER_PIN, 1800, 80);
}

void beepMode()
{
  tone(BUZZER_PIN, 1200, 120);
}

void beepWaterStart()
{
  tone(BUZZER_PIN, 2000, 150);
}

void beepWaterStop()
{
  tone(BUZZER_PIN, 700, 120);
}

void beepPause()
{
  tone(BUZZER_PIN, 400, 180);
}

void beepError()
{
  tone(BUZZER_PIN, 250, 250);
}

//========================== INTERRUPTS ==========================
void changeMode()
{
  modeChangeRequest = true;
}

void waterButton()
{
  buttonWaterRequest = true;
}