/***********************************************
 * 0.- Definiciones
 * 
 * b1p0 V 1.0.0
 * 
 * Laboratorio de control digital aplicado (LCDA)
 * Universidad de Concepcion
 * 
 * Integrantes:
 *  Bastian Inostroza   (2025-Nov)
 *  Alonso Garrido      (2026-Mar)
 *  German Inostroza    
 * 
 ***********************************************/
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <SD.h>

/**
 * Teensy Pins
 */
//COMMS
#define WIRE                  Wire          //18 SDA 19 SCL
//GPIO
#define EN1_PIN               22            //Enable pin for MFP42975 
#define EN2_PIN               22            //Enable pin for MFP42975 
#define RED_BUTTON            38            //
//PWM
#define LED_RED               2
#define LED_GREEN             3
#define LED_BLUE              4
//Interrupts
#define IRQ_MONITOR_PIN       21            //IRQ Interrupt 

//Timeouts //in millis
#define IRQ_DEBOUNCE          50            //if from push button 
#define BUTTON_DEBOUNCE       1000          //for push button

/***********************************************
 *  2.- Globales
 * Declaracion de variables globales y funciones 
 * auxiliares por componente.
 * 
 * Indice:
 *  Main sequence
 *  Teensy FSM
 *  Front LED
 *  Comms
 *  Battery monitor(MP2790) and Fuel Gauge(MPF42795)
 * 
 ***********************************************/
// --------------- Main sequence ----------------
uint8_t buttonCounter = 0;
uint32_t lastMillis = 0;
uint32_t lastMillis2 = 0;
uint32_t resetFetMillis = 0;

// Interrupt handler
void ISR_monitor();                       //Interrupt 1 ()
volatile bool IRQFlag_1 = false;
volatile bool IRQFlag_2 = false;

// --------------- Teensy FSM ----------------
enum class TeensyFSM {
    OFF,
    IDLE,
    BUSY,
    ERROR,
    ALERT,
    UNKWNOWN
};
TeensyFSM stateMain = TeensyFSM::OFF;
bool stateChanged = false;

// --------------- Front LED -------------------
struct ledColour{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

ledColour ledBlue = {0, 0, 255};
ledColour ledGreen = {0, 255, 0};
ledColour ledYellow = {255, 255, 0};
ledColour ledRed = {255, 0, 0};
ledColour ledOff = {0, 0, 0};


void toggleLedColour(ledColour colour){
  analogWrite(LED_RED, colour.r);
  analogWrite(LED_GREEN, colour.g);
  analogWrite(LED_BLUE, colour.b);
  return;
}


// ----------------- Comms  ---------------------
#include "../lib/Comms/comms.h"
#include "../lib/RTC/RTCmanager.h"

const int chipSelect = BUILTIN_SDCARD;
char buff[127];
File dataFile;

// --- Battery monitor(MP2790) and Fuel Gauge(MPF42795) ---
// //Main driver
#include "../lib/MP2790/MP2790_driver.h"
#include "../lib/MP2790/MP2790.h"

// //Set up
// #include "../test/mp2790_setup.h"

// //Tests
// #include "../test/mp2790_test.h"

/***********************************************
 * 3.- Funciones
 ***********************************************/

void ISR_monitor(){
  static unsigned long IRQtimeLast_1 = 0;
  unsigned long IRQtime_1 = millis();

  // Interrupt debounce
  if ((IRQtime_1-IRQtimeLast_1) > IRQ_DEBOUNCE) {
    IRQFlag_1 = true;
    // FOR DEBUG // MAY CAUSE WDT TIMEOUTS | CPU PANIC
    // Serial.println("Interrupt detected");      
    stateMain = TeensyFSM::ALERT;
      stateChanged = true;
  }
  // Update the time
  IRQtimeLast_1 = IRQtime_1;
}

void updaterMaster(){
  //IRQ flags checker
  if(IRQFlag_1){
    checkInt();
    checkFaults();
    IRQFlag_1 = false;
    stateMain = TeensyFSM::IDLE;
    stateChanged = true;
  }
  //Hr updater
  static uint32_t nowMillis = millis();
  if (millis() - nowMillis >500){
      mp27.triggerHRScan(0); //update
      nowMillis = millis();
  }
}

/***********************************************
 * 4.- Setup
 ***********************************************/

void setup() { 
    //Initialize serial
    Serial.begin(115200);

    initRTC();

    //Initialize Wire/I2C
    WIRE.begin();
    WIRE.setClock(400000);

    //Initialize SD
    if (!SD.begin(chipSelect)) {
      snprintf(buff, sizeof(buff), "Inicializacion de SD fallida!\n");
      logger(buff);
    }
    dataFile = SD.open("log.csv", FILE_WRITE);
    if (dataFile) {
      // Write header only if the file is empty/new
      if (dataFile.size() == 0) {
        dataFile.println("******************************");
        dataFile.println("********* SD logger **********");
        dataFile.println("******************************");
        dataFile.println("Timestamp - (Tipo) Evento");
      }
      else{
        dataFile.println("");
        dataFile.println("Timestamp - (Tipo) Evento");
      }
    }
    else{
      logger("Error al abrir log.csv\n");
    }

    //Declare GPIO
    pinMode(EN1_PIN, OUTPUT);
    pinMode(RED_BUTTON, INPUT_PULLUP);
    pinMode(IRQ_MONITOR_PIN, INPUT_PULLUP);

    //Declare Interrupts
    attachInterrupt(digitalPinToInterrupt(IRQ_MONITOR_PIN), ISR_monitor, RISING);  
    
    //Initialize GPIO
    digitalWrite(EN1_PIN, HIGH);

    //Check MP2790 connection
    mpStatus = mp27.testConnection();
}

/***********************************************
 * 5.- Loop
 ***********************************************/
void loop() {

  // Master Updater
  updaterMaster();

  // Main FSM
  switch(stateMain){  
    case TeensyFSM::OFF:{
      if(stateChanged){
        toggleLedColour(ledBlue);
        stateChanged = false;
      }
      break;
    }
    case TeensyFSM::IDLE:{
      if(stateChanged){
        toggleLedColour(ledGreen);
        stateChanged = false;
      }
      break;
    }
    case TeensyFSM::BUSY:{
      if(stateChanged){
        toggleLedColour(ledYellow);
        stateChanged = false;
      }
      break;
    }        
    case TeensyFSM::ALERT:{
      if(stateChanged){
        toggleLedColour(ledYellow);
        stateChanged = false;
      }
      break;
    }
    case TeensyFSM::ERROR:{
      if(stateChanged){
        toggleLedColour(ledRed);
        stateChanged = false;
      }
      break;
    }
    default:{
      break;
    }
  }
  return;
}
