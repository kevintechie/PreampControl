// Volume control pins (PGA2311)
#define VOL_SELECT 23
#define VOL_DATA 24
#define VOL_ZCROSS 25
#define VOL_MUTE 26
#define VOL_CLOCK 27

// Input / Output control pins (74HC595)
#define IO_LATCH 7
#define IO_CLOCK 6
#define IO_DATA 13
#define IO_ENABLE 19

// Rotary encoder pins
#define ROT_IO_TOGGLE 9
#define ROT_VOL_MUTE 8
#define ROT1_PINA 2
#define ROT1_PINB 3
#define ROT2_PINA 4
#define ROT2_PINB 5

// LED display pins (MAX7219)
#define DISP_DATA 12
#define DISP_CLOCK 11
#define DISP_LOAD 10

#include <LedControl.h>
#include <rotary.h>
#include <Bounce.h>
#include "PreampControl.h"


// Input / output and volume rotary controllers
Rotary ioSelector = Rotary(ROT1_PINA, ROT1_PINB);
Rotary volumeControl = Rotary(ROT2_PINA, ROT2_PINB);
Bounce ioBounce = Bounce(ROT_IO_TOGGLE, 5);
Bounce volBounce = Bounce(ROT_VOL_MUTE, 5);
volatile int ioCount = 0;
int prevIoCount = 0;
volatile int volCount = 0;
int prevVolCount = 0;

ioModes ioSelectorMode = inputs;

boolean volMute = true;

const byte ioTable[8] = {
  B10000000,
  B01000000,
  B00100000,
  B00010000,
  B00001000,
  B00000100,
  B00000010,
  B00000001
};

const byte volTable[12][2] = {
  {B00000000, B00000000},
  {B10000000, B00000000},
  {B11000000, B00000000},
  {B11100000, B00000000},
  {B11110000, B00000000},
  {B11111000, B00000000},
  {B11111100, B00000000},
  {B11111110, B00000000},
  {B11111111, B00000000},
  {B11111111, B10000000},
  {B11111111, B11000000},
  {B11111111, B11100000}
};

int input = 0;
const int inputMax = 5;
const int inputMin = 0;
int output = 6;
const int outputMax = 7;
const int outputMin = 6;

int volume = 0;
int volMax = 255;
const int volMin = 0;

// LED status display
LedControl statusDisplay = LedControl (12, 11, 10, 1);

void setup() {

  pinMode(IO_LATCH, OUTPUT);
  pinMode(IO_CLOCK, OUTPUT);
  pinMode(IO_DATA, OUTPUT);
  pinMode(IO_ENABLE, OUTPUT);
  digitalWrite(IO_ENABLE, HIGH);

  //Serial.begin(115200);

  PCICR |= (1 << PCIE2) | (1 << PCIE1);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);
  sei();

  pinMode(ROT_IO_TOGGLE, INPUT);
  pinMode(ROT_VOL_MUTE, INPUT);
  digitalWrite(ROT_IO_TOGGLE, HIGH);
  digitalWrite(ROT_VOL_MUTE, HIGH);

  statusDisplay.shutdown(0,false);
  statusDisplay.setIntensity(0,4);
  statusDisplay.clearDisplay(0);

  setIo(input, output);
  digitalWrite(IO_ENABLE, LOW);
  setIoMode(inputs);

  setVolume(0);
  setMute(false);
}

void loop(){
  if(volCount != prevVolCount) {

    if (volMute == false) {
      int volStep = (volCount - prevVolCount) * 6;

      if (volStep > 0) {
        for (int i = 0 ; i < volStep; i++){
          volume++;
          if (volume > volMax)
            volume = volMax;

          setVolume(volume);
        }
      }

      if (volStep < 0) {
        for (int i = 0; i > volStep; i--) {
          volume--;
          if (volume < volMin)
            volume = volMin;

          setVolume(volume);
        }
      }
    }

    prevVolCount = volCount;
  }

  if(ioCount != prevIoCount) {
    if (ioSelectorMode == inputs) {
      input += ioCount - prevIoCount;
      if (input > inputMax)
        input = inputMax;
      if (input < inputMin)
        input = inputMin;
    } 
    else {
      output += ioCount - prevIoCount;
      if (output > outputMax)
        output = outputMax;
      if (output < outputMin)
        output = outputMin;
    }

    setIo(input, output);
    prevIoCount = ioCount;
  }

  if (ioBounce.update()) {
    if (ioBounce.read() == LOW) {
      if (ioSelectorMode == inputs) {
        setIoMode(outputs);
      } 
      else {
        setIoMode(inputs);
      }
    }
  }

  if (volBounce.update()) {
    if (volBounce.read() == LOW) {
      if (volMute == false) {
        setMute(true);
      } 
      else {
        setMute(false);
      }
    }
  }
}

ISR(PCINT2_vect) {
  char result = ioSelector.process();
  char result2 = volumeControl.process();
  if (result) {
    result == DIR_CW ? ioCount ++ : ioCount --;
  }
  if (result2) {
    result2 == DIR_CW ? volCount ++ : volCount--;
  }
}

// This method sends bits to the shift register:
void registerWrite(byte shiftValue ) {

  // turn off the output so the pins don't light up
  // while you're shifting bits:
  digitalWrite(IO_LATCH, LOW);

  // shift the bits out:
  shiftOut(IO_DATA, IO_CLOCK, LSBFIRST, shiftValue);

  // turn on the output so the LEDs can light up:
  digitalWrite(IO_LATCH, HIGH);
}

void setVolume(int newVolume) {

  int vol = 0;

  if (newVolume != 0) {
    vol = map(newVolume, 1, 255, 1, 11); 
  }

  statusDisplay.setRow(0,1,volTable[vol][0]);
  statusDisplay.setRow(0,2,volTable[vol][1]);
}

void setIo(int inputValue, int outputValue) {
  byte ioSettings = ioTable[inputValue] | ioTable[outputValue];
  registerWrite(ioSettings);
  statusDisplay.setRow(0,0,ioSettings);
}

void setIoMode(ioModes mode) {

  switch (mode) {
  case inputs:
    ioSelectorMode = inputs;
    statusDisplay.setLed(0,3,0,true);
    statusDisplay.setLed(0,3,1,false);
    break;
  case outputs:
    ioSelectorMode = outputs;
    statusDisplay.setLed(0,3,0,false);
    statusDisplay.setLed(0,3,1,true);
    break;
  }
}

void setMute(boolean mute) {

  switch (mute) {
  case true:
    setVolume(0);
    volMute = true;
    statusDisplay.setLed(0,3,2,true);
    break;
  case false:
    int vol = 10;
    while (vol <= volume) {
      setVolume(vol);
      delay((vol + (vol / 2))*0.0625);
      vol++;
    }
    volMute = false;
    statusDisplay.setLed(0,3,2,false);
    break;
  }
}









