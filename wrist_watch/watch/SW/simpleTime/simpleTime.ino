// Low Power settings
// BOD 1.8v
// BootLoader Yes (UART0)
// CLock Internal 1Mhz
// EEPROM EEPROM retained
// Compiler LTO LTO Disabled
// Varient 328/328PA

//-----------LED Settings-----------
const int row[4] = {3, 4, 5, 6};  // pins for A, B, C, D rows in the LED matrix. see drawing in instructions
const int col[4] = {7, 8, 9, 10}; // pins for 1, 2, 3, 4 columns in the LED matrix
byte rowByte[4] = {0, 0, 0, 0};   // table that stores and indexes the bytes used to display
int rowCount = 0;                 // index used while looping through bytes to display on matrix
int delayTime = 300;              // microseconds. delay time between each row beeing displayed in LED matrix. scanning display

//-----------Sleep Settings-----------
#include "LowPower.h"             //library for deep sleep
unsigned long onTime = 10 * 1000; // time display stays on before going to deep sleep
unsigned long sleepTimer = 0;     // used to time how long the watch has been awake
volatile bool woke = false;       // AF
int pushBtn = 2;                  // pin for push button used to interrupt sleep
bool awakened = false;

//-----------Time Settings-----------
#include <Wire.h> //i2c library
#include "RTClib.h"
RTC_DS3231 rtc;         // name of attached RTC
int timeOrDate = false; // indicates which display option to show. time or date
DateTime now;

//-----------Other Variables-----------
volatile unsigned long prevInterrupt = 0; // used for debouncing push button
void wakeFunction();
void displayRow(int, byte);

void setup()
{

  rtc.begin();
  // do you want to set the time? uncomment this line after first upload and time has been set
  //rtc.adjust(DateTime(2023, 04, 12, 00, 16, 0)); // year, month, day, hour, minute, second

  pinMode(pushBtn, INPUT_PULLUP);
  for (int i = 0; i < 4; i++) // Defines all LEDs pins as OUTPUT
  {
    pinMode(row[i], OUTPUT);
    pinMode(col[i], OUTPUT);
  }

  allOff();
  delay(40);
  attachInterrupt(0, wakeFunction, FALLING);
}

void wakeFunction() // interrupt function when button is pressed
{
  woke = true;              // flags the microcontroller as just woken up
  prevInterrupt = millis(); // saves millis for debouncing
  // goes to main loop
}

void displaytime() // handles one row to display on the matrix at the time
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      int k;
      k = bitRead(rowByte[i], j);

      if (k == 1)
      {
        digitalWrite(row[i], LOW);
        digitalWrite(col[j], HIGH);
        delay(1);
      }
      else if (k = 0)
      {
        digitalWrite(row[i], HIGH);
        digitalWrite(col[j], LOW);
        delay(1);
      }
      digitalWrite(row[i], HIGH);
      digitalWrite(col[j], LOW);
      delay(0.5);
    }
  }
}

void allOff() // turn all IO off before going to sleep to save power
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(row[i], LOW); // rows must be set low to turn off
    digitalWrite(col[i], LOW); // columns must be set high to turn off
  }
}

void loop()
{
  if (woke)
  {
    if (millis() - prevInterrupt > 100) // if button has been pressed for 100ms
    {
      if (!digitalRead(pushBtn))
      {
        awakened = true;          // flag to start led matrix
        timeOrDate = !timeOrDate; // toggle between time and day being shown
        sleepTimer = millis();    // start the sleep countdown
        woke = false;
        now = rtc.now(); // gets the current time from DST
      }
      else
        woke = false;
    }
  }

  if (awakened)
  {
    if (millis() - sleepTimer < onTime) // if display has been on for less than 30 sec
    {
      if (timeOrDate) // loads the current time into bytes to be displayed
      {
        rowByte[0] = now.hour() / 10;   // hour second digit
        rowByte[1] = now.hour() % 10;   // hour first digit
        rowByte[2] = now.minute() / 10; // minute second digit
        rowByte[3] = now.minute() % 10; // minute first digit
      }
      else // loads the day and month into bytes to be displayed
      {
        rowByte[0] = now.day() / 10;   // day first digit
        rowByte[1] = now.day() % 10;   // day second digit
        rowByte[2] = now.month() / 10; // month first digit
        rowByte[3] = now.month() % 10; // month second digit
      }

      displaytime(); // shows one byte at the time on the matrix
    }
    else // after display complete
    {
      timeOrDate = false; // set to false so time is first thing being shown on wakeup
      woke = false;
      awakened = false;
    }
  }

  if (!woke && !awakened)
  {
    allOff();
    delay(30);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // zzzz
  }
}
