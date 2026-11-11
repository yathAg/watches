#include <Wire.h>
#include "RTClib.h"
#include "LowPower.h"

//-----------LED Settings-----------
const int row[4] = {3, 4, 5, 6};       // LED matrix row pins
const int col[4] = {7, 8, 9, 10};      // LED matrix column pins
byte rowByte[4] = {0, 0, 0, 0};        // Stores the digits to display

//-----------RTC & Time Settings-----------
RTC_DS3231 rtc;

//-----------Modes & Sleep Settings-----------
enum Mode { NORMAL, EDIT_HOUR, EDIT_MINUTE };
Mode currentMode = NORMAL;
bool displayTimeMode = true;           // true: show time; false: show date
unsigned long onTime = 10 * 1000;        // Display active time before sleep
unsigned long sleepTimer = 0;

//-----------Button Debounce & Press Handling-----------
const int pushBtn = 2;
const unsigned long debounceDelay = 50;
const unsigned long longPressThreshold = 1000; // 1 second
bool buttonState = HIGH;              // current reading from button (pull-up so HIGH means unpressed)
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long pressStartTime = 0;
bool buttonHandled = false;

//-----------LED Multiplexing (Non-blocking)-----------
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 2; // ms per row update
int currentRowDisplay = 0;

//-----------Edit Mode Variables-----------
int newHour = 0, newMinute = 0;

void setup() {
  rtc.begin();
  // Uncomment and adjust to set the RTC initially:
  // rtc.adjust(DateTime(2023, 04, 12, 0, 16, 0));
  
  pinMode(pushBtn, INPUT_PULLUP);
  for (int i = 0; i < 4; i++) {
    pinMode(row[i], OUTPUT);
    pinMode(col[i], OUTPUT);
  }
  allOff();
  sleepTimer = millis();
}

void loop() {
  // Refresh LED display using non-blocking multiplexing
  updateDisplay();

  // --- Button Handling (Debounce & Press Duration) ---
  int reading = digitalRead(pushBtn);
  unsigned long currentTime = millis();
  
  if (reading != lastButtonState) {
    lastDebounceTime = currentTime;  // reset the debounce timer
  }
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        // Button pressed: record time and mark as not handled
        pressStartTime = currentTime;
        buttonHandled = false;
      } else {
        // Button released: if not already handled, determine press type
        if (!buttonHandled) {
          unsigned long pressDuration = currentTime - pressStartTime;
          handleButtonPress(pressDuration);
          buttonHandled = true;
        }
      }
    }
  }
  lastButtonState = reading;
  
  // Reset sleep timer on any button activity
  if (buttonState == LOW || currentMode != NORMAL) {
    sleepTimer = currentTime;
  }
  
  // Enter low-power sleep if in NORMAL mode and inactive
  if (currentMode == NORMAL && (currentTime - sleepTimer > onTime)) {
    allOff();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

// Called when a button press is completed
void handleButtonPress(unsigned long duration) {
  if (currentMode == NORMAL) {
    if (duration >= longPressThreshold) {
      // Long press: Enter hour edit mode and initialize new values from RTC
      DateTime now = rtc.now();
      newHour = now.hour();
      newMinute = now.minute();
      currentMode = EDIT_HOUR;
    } else {
      // Short press: Toggle between time and date display
      displayTimeMode = !displayTimeMode;
    }
  }
  else if (currentMode == EDIT_HOUR) {
    if (duration >= longPressThreshold) {
      // Long press: Confirm hour and move to minute editing
      currentMode = EDIT_MINUTE;
    } else {
      // Short press: Increment hour (cycles 0-23)
      newHour = (newHour + 1) % 24;
    }
  }
  else if (currentMode == EDIT_MINUTE) {
    if (duration >= longPressThreshold) {
      // Long press: Confirm minute, update RTC, and return to normal mode
      DateTime now = rtc.now();
      DateTime newTime(now.year(), now.month(), now.day(), newHour, newMinute, now.second());
      rtc.adjust(newTime);
      currentMode = NORMAL;
    } else {
      // Short press: Increment minute (cycles 0-59)
      newMinute = (newMinute + 1) % 60;
    }
  }
}

// Non-blocking LED matrix update function
void updateDisplay() {
  // Update the display content based on mode
  if (currentMode == NORMAL) {
    DateTime now = rtc.now();
    if (displayTimeMode) {
      rowByte[0] = now.hour() / 10;
      rowByte[1] = now.hour() % 10;
      rowByte[2] = now.minute() / 10;
      rowByte[3] = now.minute() % 10;
    } else {
      rowByte[0] = now.day() / 10;
      rowByte[1] = now.day() % 10;
      rowByte[2] = now.month() / 10;
      rowByte[3] = now.month() % 10;
    }
  }
  else if (currentMode == EDIT_HOUR) {
    rowByte[0] = newHour / 10;
    rowByte[1] = newHour % 10;
    rowByte[2] = 0;
    rowByte[3] = 0;
  }
  else if (currentMode == EDIT_MINUTE) {
    rowByte[0] = newMinute / 10;
    rowByte[1] = newMinute % 10;
    rowByte[2] = 0;
    rowByte[3] = 0;
  }
  
  // Non-blocking multiplexing: update one row every displayInterval ms
  unsigned long currentTime = millis();
  if (currentTime - lastDisplayUpdate >= displayInterval) {
    // First, turn off all rows and columns
    for (int i = 0; i < 4; i++) {
      digitalWrite(row[i], HIGH);
      digitalWrite(col[i], LOW);
    }
    // For the current row, set columns based on the bit pattern in rowByte
    for (int j = 0; j < 4; j++) {
      int bitVal = bitRead(rowByte[currentRowDisplay], j);
      if (bitVal == 1) {
        digitalWrite(row[currentRowDisplay], LOW);
        digitalWrite(col[j], HIGH);
      } else {
        digitalWrite(row[currentRowDisplay], HIGH);
        digitalWrite(col[j], LOW);
      }
    }
    currentRowDisplay = (currentRowDisplay + 1) % 4;
    lastDisplayUpdate = currentTime;
  }
}

// Turn off all LED outputs (useful before sleep)
void allOff() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(row[i], LOW);
    digitalWrite(col[i], LOW);
  }
}
