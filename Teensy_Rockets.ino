/*
  SD card read/write
 
 This example shows how to read and write data to and from an SD card file 	
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11, pin 7 on Teensy with audio board
 ** MISO - pin 12
 ** CLK - pin 13, pin 14 on Teensy with audio board
 ** CS - pin 4, pin 10 on Teensy with audio board
 
 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 	 
 */
 
#include <SD.h>
#include <SPI.h>
#define PINO_BOTAO 24
#define PINO_MICROFONE  0
File myFile;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy audio board: pin 10
// Teensy 3.5 & 3.6 on-board: BUILTIN_SDCARD
// Wiz820+SD board: pin 4
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
const int chipSelect = BUILTIN_SDCARD;

void setup()
{

    pinMode(PINO_BOTAO,INPUT);
    SD.begin(chipSelect);
}

File csvfile;
int val;
int tempo;


void loop()                     
{
  if (not(digitalRead(PINO_BOTAO))) {
    csvfile = SD.open("Micro.csv",FILE_WRITE);
    val = analogRead(PINO_MICROFONE);
    tempo = micros();
    csvfile.print(val);
    csvfile.print(",");
    csvfile.println(tempo);
    csvfile.close();
  }
}


