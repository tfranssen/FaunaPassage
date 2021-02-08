#include <RTCZero.h>
#include "ArduinoLowPower.h"
#include <thingsml_http.h>
#include <Sodaq_R4X.h>
#include <Sodaq_wdt.h>
#include "sodaqConfig.h"
#include <Sodaq_LSM303AGR.h>

Sodaq_LSM303AGR accelerometer;

//Battery vars
#define ADC_AREF 3.3f
#define BATVOLT_R1 4.7f
#define BATVOLT_R2 10.0f
#define BATVOLT_PIN BAT_VOLT

//Sensor pins
#define beamNorthPin 2
#define beamSouthPin 3
#define pirPin 4

/* Create an rtc object */
RTCZero rtc;

//Detection counters
volatile int beamNorthCounter = 0;
volatile int beamSouthCounter = 0;

volatile int pirCounter = 0;

//Direction vars & counters
const long intervaDirection = 8; //Interval between detection south <-> north in sec
volatile int beamNorthSouthCounter = 0;
volatile int beamSouthNorthCounter = 0;
time_t timeNorth = 0;
time_t timeSouth = 0;


//Variables for interrupts
volatile bool beamNorthInterruptFlag = false;
volatile bool beamSouthInterruptFlag = false;
volatile int pirInterruptFlag = false;
volatile bool alarmInterruptFlag = false;

//RTC Settings, we dont use the date so keep it simple
const byte seconds = 0;
const byte minutes = 00;
const byte hours = 00;
const byte day = 01;
const byte month = 01;
const byte year = 21;
long alarmCounter = 0;

int hourInterval = 2; //Interval in between alarms

//Init SenML message
SenMLPack device(DEVICE_URN);
SenMLIntRecord counterBeamNorth("counterBeamNorth", SENML_UNIT_NONE);
SenMLIntRecord counterBeamSouth("counterBeamSouth", SENML_UNIT_NONE);
SenMLIntRecord counterBeamNorthSouth("counterBeamNorthSouth", SENML_UNIT_NONE);
SenMLIntRecord counterBeamSouthNorth("counterBeamSouthNorth", SENML_UNIT_NONE);
SenMLIntRecord counterPir("counterPir", SENML_UNIT_NONE);
SenMLIntRecord battery(SENML_NAME_BATTERY_VOLTAGE, SENML_UNIT_VOLT);
#define BUFFER_SIZE 1024
char buff[BUFFER_SIZE] = {0};

void setup() {
  sodaq_wdt_safe_delay(2000);
  SerialUSB.begin(115200);

  Wire.begin();// I2C for the accelerometer

  // Disable the LSM303AGR
  accelerometer.disableAccelerometer();
  accelerometer.disableMagnetometer();

  //Disable Accelerometer GPS and Modem
  pinMode(LED_BUILTIN,  OUTPUT);
  pinMode(MAG_INT,  OUTPUT);
  pinMode(GPS_ENABLE,  OUTPUT);
  pinMode(SARA_ENABLE,  OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);   // LED low=off, high=on
  digitalWrite(MAG_INT, LOW);       // we need to make this low otherwise this pin on the LSM303AGR starts leaking current
  digitalWrite(GPS_ENABLE, LOW);    // low=poweredoff, high=poweredon
  digitalWrite(SARA_ENABLE, LOW);  // low=poweredoff, high=poweredon

  //Init SenML: Add counter
  device.add(counterPir);
  device.add(counterBeamNorth);
  device.add(counterBeamSouth);
  device.add(counterBeamNorthSouth);
  device.add(counterBeamSouthNorth);
  device.add(battery);

  //Init Modem
  MODEM_STREAM.begin(r4x.getDefaultBaudrate());
  r4x.init(&saraR4xxOnOff, MODEM_STREAM);

  //Initialise status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Initialise sensors and attach it to an interrupt
  //Initialise PIR
  pinMode(pirPin, INPUT);
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(pirPin), pirInterrupt, RISING);

  //Initialise NorthBeam
  pinMode(beamNorthPin, INPUT);
  digitalWrite(beamNorthPin, HIGH); // turn on the pullup
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(beamNorthPin), beamNorthInterrupt, FALLING);

  //Initialise SouthBeam
  pinMode(beamSouthPin, INPUT);
  digitalWrite(beamSouthPin, HIGH); // turn on the pullup
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(beamSouthPin), beamSouthInterrupt, FALLING);

  // Configure EIC to use GCLK1 which uses XOSC32K (RTC Clock)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

  //Set RTC and set timer
  rtc.begin();
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);
  rtc.setAlarmTime(00, 00 , 30);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.attachInterrupt(alarmMatch);

  SerialUSB.println("Setup done");

  //DisableUSB
  SerialUSB.flush();
  SerialUSB.end();
  USBDevice.detach();
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB

  LowPower.sleep(); //Put chip to sleep after setup and wait for Alarm or Trigger interrupt


}

void loop() {


  //Run if Pir interrupt is triggerd
  if (pirInterruptFlag) {
    digitalWrite(LED_BUILTIN, HIGH);
    sodaq_wdt_safe_delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    pirCounter++;
    SerialUSB.println("Pir interrupt. Counter = " + String(pirCounter));
    pirInterruptFlag = false;
  }

  //Run if beamNorth interrupt is triggerd
  if (beamNorthInterruptFlag) {
    digitalWrite(LED_BUILTIN, HIGH);
    sodaq_wdt_safe_delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    beamNorthCounter++;
    SerialUSB.println("beamNorth interrupt. Counter = " + String(beamNorthCounter));
    //Direction
    timeNorth = rtc.getEpoch();
    if ((timeNorth - timeSouth) <= intervaDirection) {
      beamSouthNorthCounter++;
      SerialUSB.println("Traffic South -> North detected. Counter = " + String(beamSouthNorthCounter));
      timeNorth = 0;
      timeSouth = 0;
    }
    beamNorthInterruptFlag = false;
  }

  //Run if beamSouth interrupt is triggerd
  if (beamSouthInterruptFlag) {
    digitalWrite(LED_BUILTIN, HIGH);
    sodaq_wdt_safe_delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    beamSouthCounter++;
    SerialUSB.println("beamSouth interrupt. Counter = " + String(beamSouthCounter));
    //Direction
    timeSouth = rtc.getEpoch();
    if ((timeSouth - timeNorth) <= intervaDirection) {
      beamNorthSouthCounter++;
      SerialUSB.println("Traffic North -> South detected. Counter = " + String(beamNorthSouthCounter));
      timeNorth = 0;
      timeSouth = 0;
    }
    beamSouthInterruptFlag = false;
  }


  //Run if alarm interrupt is triggerd
  if (alarmInterruptFlag) {
    SerialUSB.println("Alarm Interrupt");
    alarmCounter++;
    //Set alarm to new alarmTime
    rtc.setAlarmHours((alarmCounter * hourInterval) % 24);
    //Turn on modem power
    digitalWrite(SARA_ENABLE, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    //Give modem some time to wake-up
    sodaq_wdt_safe_delay(100);
    SerialUSB.println("Send message");

    sendMessage();
    pirCounter = 0;
    beamSouthCounter = 0;
    beamNorthCounter = 0;
    beamNorthSouthCounter = 0;
    beamSouthNorthCounter = 0;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(SARA_ENABLE, LOW);
    alarmInterruptFlag = false;
  }
  LowPower.sleep();
}

void alarmMatch()
{
  alarmInterruptFlag = true;
}

void pirInterrupt()
{
  pirInterruptFlag = true;
}

void beamNorthInterrupt()
{
  beamNorthInterruptFlag = true;
}

void beamSouthInterrupt()
{
  beamSouthInterruptFlag = true;
}

void sendMessage()
{
  SerialUSB.println("Turning on modem...");
  SerialUSB.println("Connecting to network...");

  r4x.on(); //Turn on modem
  isReady = r4x.connect(APN, CURRENT_URAT, CURRENT_MNO_PROFILE, CURRENT_OPERATOR, BAND_MASK_UNCHANGED, NBIOT_BANDMASK); //Connect to network
  SerialUSB.println(isReady ? "Network connected" : "Network connection failed");

  if (isReady) {
    postHTTP();
  }
  r4x.off();//Turn off modem
}

void postHTTP() {
  counterPir.set(pirCounter);
  counterBeamNorth.set(beamNorthCounter);
  counterBeamSouth.set(beamSouthCounter);
  counterBeamNorthSouth.set(beamNorthSouthCounter);
  counterBeamSouthNorth.set(beamSouthNorthCounter);

  battery.set(getBatteryVoltage());
  int len = ThingsML::httpPost(buff, BUFFER_SIZE, DEVICE_KEY, HTTP_HOST, HTTP_PATH, device);
  SerialUSB.println("Sending message...");
  SerialUSB.println(buff);

  uint8_t socketId = r4x.socketCreate(0, TCP);
  r4x.socketConnect(socketId, HTTP_IP, HTTP_PORT);

  r4x.socketWrite(socketId, (uint8_t *) buff, len);
  r4x.socketWaitForRead(socketId);
  SerialUSB.println("Receiving message...");
  int receiveLength = r4x.socketRead(socketId, (uint8_t *) buff, BUFFER_SIZE);

  r4x.socketClose(socketId);

  SerialUSB.print("Message response length: ");
  SerialUSB.println(receiveLength);

  if (receiveLength > 0) {
    buff[receiveLength] = 0; // Null terminate the string
    SerialUSB.println("Message response:");
    SerialUSB.println(buff);
  }
  SerialUSB.println("Message sending finished.");
}

uint16_t getBatteryVoltage()
{
  uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));
  return voltage;
}
