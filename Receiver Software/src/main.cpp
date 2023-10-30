#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include "ssd1306.h"
#include "driver/gpio.h"

#define ss 5
#define rst 14
#define dio0 2

int iMode = 1;
char sMode[10] = "1";
unsigned long currentTicks = 0;  // current time
unsigned long preTicks = 0;    // previous time that button was pushed
unsigned long delayDebounce = 200;    // Delay to be adheared to for the debouncing
bool leftButton = false;
bool rightButton = false;
bool bModeChanged = true;
int iTotalRSSI = 0;
int iCountRSSI = 0;
float fAveRSSI = 0.0;

void IRAM_ATTR leftButtonPush();
void IRAM_ATTR rightButtonPush();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433e6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(7.8E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setGain(1);

  // put the radio into receive mode
  LoRa.receive();

  pinMode(15, INPUT_PULLUP); // Left button input
  pinMode(0, INPUT_PULLUP); // Right button input
  attachInterrupt(15, leftButtonPush, FALLING);
  attachInterrupt(0, rightButtonPush, FALLING);


  ssd1306_128x64_i2c_init();
  ssd1306_fillScreen(0x00);
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_printFixedN(0,  0, "Mode:", STYLE_NORMAL, FONT_SIZE_4X);
  sprintf(sMode, "%d        ",iMode);
  ssd1306_printFixedN(0,  35, sMode, STYLE_NORMAL, FONT_SIZE_4X);
  //Serial.println(getXtalFrequencyMhz());
  //Serial.println(getCpuFrequencyMhz());
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    iCountRSSI += 1;
    // received a packet
    Serial.print("Received packet '");
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.print(LoRa.packetRssi());
    iTotalRSSI += LoRa.packetRssi();
    Serial.print(" and frequency error of ");
    Serial.println(LoRa.packetFrequencyError());
  }

  if (leftButton) {
    if (iCountRSSI>0) {
      fAveRSSI = float(iTotalRSSI)/float(iCountRSSI);
      Serial.printf("Average RSSI: %.2f\n",fAveRSSI);
      fAveRSSI = 0.0;
      iTotalRSSI = 0;
      iCountRSSI = 0;
    }

    Serial.println("");

    if (iMode == 1) {
      iMode = 60;
      sprintf(sMode, "%d        ",iMode);
      ssd1306_printFixedN(0,  35, sMode, STYLE_NORMAL, FONT_SIZE_4X);
    } else {
      iMode = iMode - 1;
      sprintf(sMode, "%d        ",iMode);
      ssd1306_printFixedN(0,  35, sMode, STYLE_NORMAL, FONT_SIZE_4X);
    }


    leftButton = false;
  }

  if (rightButton) {
    if (iCountRSSI>0) {
      fAveRSSI = float(iTotalRSSI)/float(iCountRSSI);
      Serial.printf("Average RSSI: %.2f\n",fAveRSSI);
      fAveRSSI = 0.0;
      iTotalRSSI = 0;
      iCountRSSI = 0;
    }


    Serial.println("");

    if (iMode == 60) {
      iMode = 1;
      sprintf(sMode, "%d        ",iMode);
      ssd1306_printFixedN(0,  35, sMode, STYLE_NORMAL, FONT_SIZE_4X);
    } else {
      iMode = iMode + 1;
      sprintf(sMode, "%d        ",iMode);
      ssd1306_printFixedN(0,  35, sMode, STYLE_NORMAL, FONT_SIZE_4X);
    }
    rightButton = false;
  }


  if (bModeChanged){
    switch (iMode)
    {
    case 1:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(7.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 2:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(10.4E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 3:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(15.6E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 4:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(20.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 5:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(31.25E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 6:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(41.7E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 7:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(62.5E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 8:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 9:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(250E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 10:
      LoRa.setSpreadingFactor(7);
      LoRa.setSignalBandwidth(500E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 11:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(7.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 12:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(10.4E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 13:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(15.6E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 14:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(20.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 15:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(31.25E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 16:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(41.7E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 17:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(62.5E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 18:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 19:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(250E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 20:
      LoRa.setSpreadingFactor(8);
      LoRa.setSignalBandwidth(500E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 21:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(7.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 22:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(10.4E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 23:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(15.6E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 24:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(20.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 25:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(31.25E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 26:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(41.7E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 27:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(62.5E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 28:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 29:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(250E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 30:
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(500E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 31:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(7.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 32:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(10.4E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 33:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(15.6E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 34:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(20.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 35:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(31.25E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 36:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(41.7E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 37:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(62.5E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 38:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 39:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(250E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 40:
      LoRa.setSpreadingFactor(10);
      LoRa.setSignalBandwidth(500E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 41:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(7.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 42:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(10.4E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 43:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(15.6E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 44:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(20.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 45:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(31.25E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 46:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(41.7E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 47:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(62.5E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 48:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 49:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(250E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 50:
      LoRa.setSpreadingFactor(11);
      LoRa.setSignalBandwidth(500E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 51:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(7.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 52:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(10.4E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 53:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(15.6E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 54:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(20.8E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 55:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(31.25E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 56:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(41.7E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 57:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(62.5E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 58:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 59:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(250E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;

    case 60:
      LoRa.setSpreadingFactor(12);
      LoRa.setSignalBandwidth(500E3);
      LoRa.setCodingRate4(5);
      LoRa.setPreambleLength(8);
      break;
    
    default:
      break;
    }
    
    Serial.print("Mode: ");
    Serial.println(sMode);
    bModeChanged = false;
    LoRa.dumpRegisters(Serial);
  }
}


void IRAM_ATTR leftButtonPush(){
  currentTicks = millis();
  if ((currentTicks-preTicks)>delayDebounce) {
    leftButton = true;
    bModeChanged = true;
  } else {
    //break
  }
  preTicks = currentTicks;
}

void IRAM_ATTR rightButtonPush(){
  currentTicks = millis();
  if ((currentTicks-preTicks)>delayDebounce) {
    rightButton = true;
    bModeChanged = true;
  } else {
    //break
  }
  preTicks = currentTicks;
}
