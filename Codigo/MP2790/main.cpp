/*
 * I2C MP27xx Read Write
 *
 * Reads the value of each byte of the I2C MP27xx and prints it
 * to the computer.
 * This example code is in the public domain.
 */

#include <Arduino.h>
#include "MP2790.h"

//Global Variables
#define MP_ENABLE             25
#define RED_BUTTON            32
#define XALERT_INPUT          33
#define BUTTON_DEBOUNCE       1000        //in milliseconds

uint16_t mp2790_nCells = 6;               //6 cells
uint16_t mp2790_address = 0x01;           // Default address, can be changed later
bool mpStatus = false;
uint8_t buttonCounter = 0;
uint32_t lastMillis = 0;
uint32_t lastMillis2 = 0;
uint32_t resetFetMillis = 0;

// Interrupt handler
void xAlertISR();
volatile bool xAlertFlag = false;

//Function declarations
void setupMP2790();
void setupValues();
void checkSensors();
void checkInt();
void checkFaults();
void handleInt(bool *intFlags);
void handleFault(bool *faultFlags);
bool millisTracker(uint32_t *trackMillis, uint16_t trackTime);
bool resetFET(bool trigger = false);
void dumpHRData();

// Initialize using MP27XX(i2c_address, number of cells).  
MP2790 mp27 = MP2790(mp2790_address, mp2790_nCells); //empty for default

// Initialize protections
bool intFlags[32];
bool faultFlags[32];
HRDataPacket hrData;

// Enum for Read/Write access permissions
enum class TeensyFSM {
    IDLE,
    BUSY,
    ERROR,
    ALERT,
    UNKWNOWN
};

TeensyFSM mainState = TeensyFSM::BUSY;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 

  //Initialize Wire/I2C
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Wire.setSDA(18);
  Wire.setSCL(19);

  //Check MP2790 connection
  mpStatus = mp27.testConnection();

  //Declare GPIO
  pinMode(MP_ENABLE, OUTPUT);
  pinMode(RED_BUTTON, INPUT_PULLUP);
  pinMode(XALERT_INPUT, INPUT_PULLUP);

  //Declare Interrupts
  attachInterrupt(digitalPinToInterrupt(XALERT_INPUT), xAlertISR, RISING);

  //Initialize GPIO
  digitalWriteFast(MP_ENABLE, HIGH);
}

void loop() {

  if(xAlertFlag){
    checkInt();
    checkFaults();
    xAlertFlag = false;
  }

  static uint32_t nowMillis = millis();
  if (millis() - nowMillis >500){
    mp27.triggerHRScan(0);
    nowMillis = millis();
  }

  if ((digitalReadFast(RED_BUTTON) == LOW) && (millis() - lastMillis > BUTTON_DEBOUNCE)){

    Serial.println("Buttonpress");

    if(buttonCounter == 0){
      Serial.println("Minimal setup");
      mp27.writeAdd(CELLS_CTRL, 0x0005);  //6 cells
      // mp27.writeAdd(INT0_EN, 0x8800);  //EN xalert and Vscan
      // mp27.writeAdd(HR_SCAN0, 0x037f);  //EN Scan ldos+
      // mp27.writeAdd(HR_SCAN1, 0x0001);  //EN cell 1 scan
      // mp27.writeAdd(HR_SCAN2, 0x00A0);  //EN HR NTC1-3

      // //Cell ov
      // mp27.writeAdd(INT0_EN, 0x8801);  //EN int
      // mp27.writeAdd(CELLFT_CTRL, 0x8830);  //EN fun EN fault

      //Cell uv
      // mp27.writeAdd(INT0_EN, 0x8802);  //EN int
      // mp27.writeAdd(CELLFT_CTRL, 0x8806);  //EN fun EN fault
    
      //fet control)
        //barebones dsg (ok)
      // mp27.writeAdd(0x13, 0x0102);  //0x10a all SS off -> 0x102 +sc pre turn on
      // mp27.writeAdd(0x1A, 0x2001);   //fet interrupt

        //dsg ss 0.1v/ms (ok)
      // mp27.writeAdd(0x13, 0x0103);  //0x10b SS -> 0x103 +sc pre turn on
      // mp27.writeAdd(0x1A, 0x2001);   //fet interrupt

      //   //dsg ss 1v/ms (ok)
      mp27.writeAdd(0x13, 0x0103);  //0x10b SS -> 0x103 +s/c pre turn on
      mp27.writeAdd(0x14, 0x68f5);  //0x68f0 def -> 0x68f5 1v/ms
      mp27.writeAdd(0x1A, 0x2001);   //fet interrupt


      buttonCounter++;
    }
    // Serial.print(mp27.readAdd(ADC_SCAN_GO), BIN);
    // mp27.triggerHRScan(1);
    // lastMillis = millis();
    // Serial.println(mp27.readAdd(HR_SCAN0), BIN);
    // Serial.println(mp27.readAdd(ADC_STS), BIN);
    // Serial.println(mp27.readAdd(ADC_CTRL), BIN);
  }

  // // switch(mainState){
  // //   case TeensyFSM::IDLE:{
  // //     break;
  // //   }
  // //   case TeensyFSM::BUSY:{
  // //     break;
  // //   }    
  // //   case TeensyFSM::ERROR:{
  // //     break;
  // //   }
  // //   case TeensyFSM::ALERT:{
  // //     if (resetFET(false)){
  // //       mainState = TeensyFSM::IDLE;
  // //     }
  // //     break;    
  // //   }
  // //   default:{
  // //     break;
  // //   }
  // // }

  // //Setup sequence when button is pressed
  if ((digitalReadFast(RED_BUTTON) == LOW) && (millis() - lastMillis > BUTTON_DEBOUNCE)){
    lastMillis = millis();
    Serial.println("Button pressed");

    if (buttonCounter < 3){
      buttonCounter++;
      return;
    }
  //   // // If cells !=6, configure from scratch
  //   if (buttonCounter < 1){
  //     // if (mp27.readAdd(CELLS_CTRL) != 0x00C5){
  //     Serial.println(mp27.readAdd(CELLS_CTRL));
  //     Serial.println(" initializing MP2790 Setup...");
  //     setupMP2790();
  //     setupValues();
  //     buttonCounter++;
  //     return;
  //   }
  //   // checkSensors();

  //   // Serial.println("Cell value");
  //   // Serial.println(mp27.readAdd(CELLS_CTRL));

    if (mp27.powerStatus() == 0b01000){ //Normal c 
      // resetFET(true);
      checkInt();
      checkFaults();
      Serial.println(mp27.readAdd(MP2790_Reg::FET_TIMEOUT));
      Serial.println(mp27.readAdd(MP2790_Reg::FET_DRIVER_INT_STS));
      //clear interrupt
      // Serial.println(mp27.readAdd(MP2790_Reg::FET_DRIVER_INT_CLEAR));
      //Posibilities: 
      // - Timeout
      // - OC    
      // - SC SS //FT1_CFG
      // mainState = TeensyFSM::ALERT;
      return;
    }
    else if(mp27.powerStatus() == 0b00001){ // safe
      Serial.println("Turn on Fet");
      mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 0);    
      mp27.writeAdd(MP2790_Reg::DRIVER_FAULT_CLR, 1);
      delay(1000);
      mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 3);
      Serial.println(mp27.readAdd(MP2790_Reg::DRIVER_FAULT_STS));
    }
    else if(mp27.powerStatus() == 0b10000){ //Normal a
      Serial.println("Turn off Fet");
      mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 0);
    }    
    else if(mp27.powerStatus() == 0b00100){ //Normal b
      Serial.println("Turn off Fet");
      mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 0);
    }
  }

  // if (millis() - lastMillis2 > 5000){
  //   mp27.powerStatus();
  //   lastMillis2 = millis();
  //   mp27.triggerHRScan(1);
  //   // Serial.println("Digital die temp sts");
  //   // Serial.println(mp27.readAdd(MP2790_Reg::DIE_TEMP_DIG_STS));
  //   // if(mp27.readAdd(MP2790_Reg::DIE_TEMP_DIG_STS)){
  //   //   mp27.writeAdd(MP2790_Reg::DIE_TEMP_DIG_CLEAR, 1);
  //   // }
  //   // Serial.println(mp27.readAdd(MP2790_Reg::DIE_TEMP_DIG_RT_STS));
  //   // checkInt();   
  //   // Serial.println(mp27.readAdd(SCAN_ERROR_STS));
  //   // Serial.println(mp27.readAdd(SCAN_DONE_STS));
  //   // Serial.println(mp27.readAdd(VADC_FSM),HEX);
  //   // Serial.println("self status");
  //   // Serial.println(mp27.readAdd(SELF_TEST_STS_OV));
  //   // Serial.println(mp27.readAdd(SELF_TEST_STS_UV));
  //   // Serial.println(mp27.writeAdd(SELF_TEST_INT_CLEAR,1));
  //   // Serial.println(mp27.writeAdd(FSM_ERROR_INT_CLEAR,1));
  // }
}

void setupMP2790() {

  // Check Address
  // uint16_t addressi2c = mp27.getAddress(); //reads address from object
  uint16_t addressi2c = mp27.readAdd(MP2790_Reg::DEVICE_ADD); //reads address using register
  Serial.print("MP2790 I2C Address: ");
  Serial.println(addressi2c, HEX);
  
  // //Checking MTP status
  // Serial.println("Checking MTP status");
  // uint16_t mtpStatus = mp27.readReg(MP2790_Reg::MTP_STATUS); //by register
  // Serial.print("MTP Status: ");
  // Serial.println(mtpStatus, BIN);
  // Serial.print("MEMORY STATUS: ");
  // Serial.println(mp27.readAdd(MEM_STATUS), BIN); //by address

  // // //Writing new address
  // Serial.println("Writing new address");
  // mp27.setAddress(11);                                                   //writes 11 as device address on the go
  // // mp27.writeReg(MP2790_Reg::DEVICE_ADD, 11);                          //writes 11 to device register, needs to initialize
  // //// Serial.println(mp27.writeAdd(COMM_CFG, 0b00000000101100000000));  //writes 11 to device address, needs to initialize
  // Serial.println("Initializing new address");
  // mp27 = MP2790(11);                                                     //initializes with new address
  // // Re-Check Address
  // Serial.print("MP2790 I2C Address2: ");
  // addressi2c = mp27.readAdd(COMM_CFG);
  // Serial.println(addressi2c, HEX);
  // //Re-Check Connection
  // mp27.testConnection();


  // Storing toNVM page
  // Serial.println("Access code input");
  // mp27.storeAccessCode();
  // Serial.println("Storing NVM page");
  // if (mp27.storeNVM() != true) {
  //   Serial.println("Failed to store NVM page.");
  // } else {
  //   Serial.println("NVM page stored successfully.");
  // }

  // //Checking MTP satus
  // Serial.println("Checking MTP status");
  // uint16_t mtpStatus = mp27.checkMTPStatus();
  // Serial.print("MTP Status: ");
  // Serial.println(mtpStatus, HEX);

}
void setupValues(){
  // // Importing data
  Serial.println("Loading configuration values...");
  mp27.writeAdd(0x00, 0x00C5);   // CELLS_CTRL C3: 4cells ; c5: 6cells
  mp27.writeAdd(0x05, 0x0000);   // ACT_CFG 0x0000: all off 0x000A: all on, direct mode; 0x0000: all off, direct mode  
  mp27.writeAdd(0x06, 0x0022);   // STB_CFG
  mp27.writeAdd(0x07, 0x000D);   // SAFE_CFG
  mp27.writeAdd(0x08, 0x0005);   // RGL_CFG
  mp27.writeAdd(0x09, 0x0500);   // LOAD_CHARGER_CFG
  mp27.writeAdd(0x0B, 0x0000);   // GPIO_OUT
  mp27.writeAdd(0x0C, 0x0444);   // GPIO_CFG
  mp27.writeAdd(0x0D, 0x0061);   // PINS_CFG
  mp27.writeAdd(0x10, 0x4E9C);   // WDT_CFG 0x4E9D: WDT on 0x4E9C: WDT off
  mp27.writeAdd(0x12, 0x0000);   // FET_CTRL
  mp27.writeAdd(0x13, 0x051b);   // FET_MODE //turnon timeount ? 0x071b : 0x051b 0x68F0 //0x070A: SS DSG and CHG disabled
  mp27.writeAdd(0x14, 0x38F0);   // FET_CFG 0x78F0: 12VCP; 0x68F0: 10Vcp 0x38F0: 7Vcp 0x08F0: 5Vcp-low 
  mp27.writeAdd(0x19, 0xCFFF);   // INTO_EN
  mp27.writeAdd(0x1A, 0x2F8E);   // INT1_EN 0x2FFE: normal, 0x2FEE: ow off
  mp27.writeAdd(0x1B, 0x0000);   // INT_TYPE0
  mp27.writeAdd(0x1C, 0x0000);   // INT_TYPE1
  mp27.writeAdd(0x1D, 0x0000);   // INT_TYPE2
  mp27.writeAdd(0x1E, 0x40FF);   // MASK_INTO
  mp27.writeAdd(0x1F, 0x0142);   // MASK_INT1
  mp27.writeAdd(0x23, 0x01BF);   // OCFT_CTRL
  mp27.writeAdd(0x24, 0x2C19);   // DSGOC_LIM
  mp27.writeAdd(0x25, 0x0428);   // DSGOC_DEG
  mp27.writeAdd(0x26, 0x0404);   // CHGOC_DEG
  mp27.writeAdd(0x2A, 0x003F);   // SCFT_CTRL
  mp27.writeAdd(0x2B, 0x0115);   // DSGSC_CFG
  mp27.writeAdd(0x2C, 0x0804);   // CHGSC_CFG
  mp27.writeAdd(0x34, 0x0033);   // PACKFT_CTRL (dead cell) 0x1B33: report latch, 0x1F33: immediate, 0x1A33: disable, 0x0033: Only Vtop OV and UV 
  mp27.writeAdd(0x35, 0x6036);   // CELLFT_CTRL
  mp27.writeAdd(0x36, 0x0AA0);   // CELL_HST
  mp27.writeAdd(0x37, 0x8080);   // PACK_UV_OV
  mp27.writeAdd(0x38, 0x009F);   // CELL_UV
  mp27.writeAdd(0x39, 0x00D7);   // CELL_OV
  mp27.writeAdd(0x3A, 0x03B3);   // PACK_UV
  mp27.writeAdd(0x3B, 0x0500);   // PACK_OV
  mp27.writeAdd(0x3C, 0x0068);   // CELL_DEAD_THR
  mp27.writeAdd(0x3D, 0x0002);   // CELL_MSMT
  mp27.writeAdd(0x44, 0x0000);   // NTC_CLR
  mp27.writeAdd(0x46, 0x000A);   // DIE_CFG
  mp27.writeAdd(0x47, 0xE4A0);   // NTC_CFG 0xE4F5:NTC1-4 0xE4B1: NTC1,3, 0xE4A0: all off
  mp27.writeAdd(0x48, 0x012E);   // NTCC_OTHR_DSG
  mp27.writeAdd(0x49, 0x0294);   // NTCC_UTHR_DSG
  mp27.writeAdd(0x4A, 0x012E);   // NTCC_OTHR_CHG
  mp27.writeAdd(0x4B, 0x8A94);   // NTCC_UTHR_CHG
  mp27.writeAdd(0x4C, 0x80EB);   // NTCM_OTHR
  mp27.writeAdd(0x4D, 0xAAEC);   // DIE_OT
  mp27.writeAdd(0x4E, 0x0000);   // SELF_STS
  mp27.writeAdd(0x55, 0x0501);   // SFT_GO
  mp27.writeAdd(0x56, 0x400F);   // SELF_CFG (Open wire) 0x470F: enabled, 0x430F: disable on Pwr on, 0x410F: full disable, 0x400F: full disable no fault
  mp27.writeAdd(0x58, 0x016D);   // REGIN_UV
  mp27.writeAdd(0x59, 0x00F0);   // V3P3_UV
  mp27.writeAdd(0x5A, 0x0084);   // VDD_UV
  mp27.writeAdd(0x5B, 0x6555);   // SELF_THR
  mp27.writeAdd(0x60, 0x1016);   // FT_REC 0x1016 auto recovery
  mp27.writeAdd(0x61, 0x3DD0);   // FT0_CFG
  mp27.writeAdd(0x62, 0x1000);   // FT1_CFG
  mp27.writeAdd(0x99, 0x0000);   // ADC_CTRL
  mp27.writeAdd(0x9A, 0x5F0A);   // CC_CFG
  mp27.writeAdd(0x9B, 0x4000);   // TRIMG_IPCB
  mp27.writeAdd(0x9C, 0x037F);   // HR_SCANO
  mp27.writeAdd(0x9D, 0x003F);   // HR_SCAN1
  mp27.writeAdd(0x9E, 0x0001);   // HR_SCAN2
  mp27.writeAdd(0xA0, 0x0000);   // SILC_INFO1
  mp27.writeAdd(0xA3, 0x0100);   // COMM_CFG
  mp27.writeAdd(0xA5, 0x0000);   // BAL_LIST
  mp27.writeAdd(0xA6, 0x0000);   // BAL_CTRL
  mp27.writeAdd(0xA7, 0x00F8);   // BAL_CFG
  mp27.writeAdd(0xA8, 0x0A21);   // BAL_THR

}
void checkSensors() {

  //Clear dead cell
  // mp27.writeAdd(MP2790_Reg::CELL_DEAD_DET_CLEAR, 1);

  // Read and print some sensor values
  // Temperature 
  uint16_t dieTemp = mp27.getDieTemperature();
  Serial.print("Die Temperature: ");
  Serial.println(dieTemp);
  
  // Temperature HR
  uint16_t dieTempHR = mp27.getHRDieTemperature();
  Serial.print("Die Temperature HR: ");
  Serial.println(dieTempHR);

  // Temperature NTC
  int32_t ntcreadings[4];
  Serial.print("NTC Temperature: ");
  mp27.getNTCreadings(ntcreadings);
  for (int i = 0; i < 4; i++) {
    Serial.print(ntcreadings[i]);
    Serial.print(" ");
  }
  Serial.println();

  // ADC Readings
  uint16_t adcValues[4];
  mp27.getADCReadings(adcValues);
  Serial.print("ADC Readings: 1v8, 3v3, regin, Vself: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(adcValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  // ADC Readings HR
  // uint16_t adcHRValues[3];
  // mp27.getHRADCVals(adcHRValues);
  //   Serial.print("ADC HR Readings: 1v8, 3v3, regin: ");
  // for (int i = 0; i < 3; i++) {
  //   Serial.print(adcHRValues[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Vtop Readings
  Serial.print("VTop Readings: "); 
  Serial.println(mp27.getVTopReadings());

  // Power state
  mp27.powerStatus();
}
void checkInt(){
  // Interrupts
  Serial.println("Checking Interrupts...");
  if (mp27.getInt(intFlags)){
    Serial.println("Interrupts active");
    handleInt(intFlags);
    return;
  }
  Serial.println("No Interrupts...");
}
void checkFaults(){
  // Faults
  Serial.println("Checking Faults...");
  if (mp27.getFault(faultFlags)){
    Serial.println("Faults active");
    handleFault(faultFlags);
    return;
  }
  Serial.println("No Faults...");
}
//Resets Fet with wait sequence
bool resetFET(bool trigger){

  static uint8_t fetState = 0;
  uint32_t fetTimer = 0;

  switch(fetState){
  // Checks trigger, exits if not being reset
    case 0:{
      if(trigger){
        digitalWriteFast(MP_ENABLE, LOW);
        Serial.println("Reseting Fet");
        fetState = 1;
        fetTimer = millis();
      }
      break;
    }

    // Turns on mp and clr faults, waits timeout
    case 1:{
      if (millisTracker(&fetTimer, 1000)){
        digitalWriteFast(MP_ENABLE, HIGH);
        mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 0);
        mp27.writeAdd(0x60, 0x2000);
        // mp27.writeAdd(0x5F, 0xFFFF); //All fet fault clear
        mp27.writeAdd(MP2790_Reg::DRIVER_FAULT_CLR, 1);
        fetState = 2;
        fetTimer = millis();
      }
      break;
    }
    case 2:{
      if (millisTracker(&fetTimer, 500)){
        digitalWriteFast(MP_ENABLE, HIGH);
        mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 0);
        mp27.writeAdd(0x60, 0x2000);
        // mp27.writeAdd(0x5F, 0xFFFF); //All fet fault clear
        mp27.writeAdd(MP2790_Reg::DRIVER_FAULT_CLR, 1);
        fetState = 3;
        fetTimer = millis();
      }
      break;
    }
    case 3:{
      //Turns on FET
      if (millisTracker(&fetTimer, 500)){
        mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 3);
        // mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 1);
        // Serial.print("driver Fault: ");
        // Serial.println(mp27.readAdd(MP2790_Reg::DRIVER_FAULT_STS));
        // Serial.print("charge pump: ");
        // Serial.println(mp27.readAdd(MP2790_Reg::CP_STS));
        // Serial.print("fet timeout: ");
        // Serial.println(mp27.readAdd(MP2790_Reg::FET_TIMEOUT));
        // Serial.print("charging dsg: ");
        // Serial.println(mp27.readAdd(MP2790_Reg::DSG_DRV_TRANS));
        // Serial.print("dsg on: ");
        // Serial.println(mp27.readAdd(MP2790_Reg::DSG_DRV));
        fetState = 0;
      }
    }
  }
  return (fetState != 0);
}

//Keeps track of millis counter
bool millisTracker(uint32_t *trackMillis, uint16_t trackTime){
  if (millis() - *trackMillis > trackTime){
    *trackMillis = millis();
    return true;
  }
  return false;
}
void handleInt(bool *intFlags) {
  Serial.println("Handling Interrupts...");
  for (int i = 0; i < 32; i++) {
    if (intFlags[i]){ // Skip if no interrupt flag is set
      Serial.print("Interrupt ");
      Serial.println(i);
      switch (i) {
        case 0: break;
        case 1: break;
        case 2: break;
        case 3: break;
        case 4: break;
        case 5: break;
        case 6: break;
        case 7: break;
        case 8: break;
        case 9:{
          Serial.println("State C recovered");
          mp27.clearInt(i);
          break;
        }
        case 10:{
          Serial.println("Mode change detected");
          mp27.powerStatus();
          mp27.clearInt(i);
          break;
        }
        case 11:{
          Serial.println("HR ADC scan completed ");
          hrData = mp27.getHRADCVals();
          dumpHRData();
          mp27.clearInt(i);
          Serial.println(mp27.readAdd(RD_INT1), BIN);
          break;
        }
        case 12: break;
        case 13: break;
        case 14:{
          Serial.println("Current change detected");
          mp27.currentDirection();
          mp27.clearInt(i);
          break;
        }
        case 15: break;
        case 16: break;
        case 17: break;
        case 18: break;
        case 19: break;
        case 20:{
          int openWires = mp27.readAdd(RD_OPENH);  
          for (int i = 0; i < 11; i++) {
            int currentWire = (openWires >> i) & 0x01;
            if (currentWire){
              Serial.print("Cell "); Serial.print(i); Serial.println(" Open");
            }
          }
          break;
        }
        case 21:{          
          int deadCell = mp27.readAdd(RD_CELL_DEAD);  
          for (int i = 0; i < mp2790_nCells; i++) {
            int currentWire = (deadCell >> i) & 0x01;
            if (currentWire){
              Serial.print("Cell "); Serial.print(i); Serial.println(" Dead");
            }
          }
          break;
        }
        case 22: break;
        case 23:{
          if(mp27.readAdd(MP2790_Reg::DIE_TEMP_DIG_STS) == 1 && mp27.readAdd(MP2790_Reg::DIE_TEMP_DIG_RT_STS) == 0){
            mp27.writeAdd(MP2790_Reg::DIE_TEMP_DIG_CLEAR, 1);
            mp27.clearInt(i);
          }
          break;
        }
        case 24: break;
        case 25: {
          mp27.clearInt(i);
          break;
        }
        case 26:{
          mp27.clearInt(i);
          break;
        }
        case 27: break;
        default: Serial.println("Interrupt out of boundaries"); break;
      }
    }
  }
}

void handleFault(bool *faultFlags) {
  Serial.println("Handling Faults...");
  if(faultFlags[0]){
    mp27.clearFault(0);
  }  
  if(faultFlags[1]){
    mp27.clearFault(1);
  }  
  return;
}

void dumpHRData(){

  Serial.println("HRvoltage");
  for (int i = 0; i<10 ; i++){
    Serial.print(hrData.cellHRVoltage[i]);
    Serial.print(" | ");
  }
  Serial.println("");

  Serial.println("HRCurrent");
  for (int i = 0; i<10 ; i++){
    Serial.print(hrData.cellHRCurrent[i]);
    Serial.print(" | ");
  }
  Serial.println("");

  Serial.println("NTCvalue");
  for (int i = 0; i<4 ; i++){
    Serial.print(hrData.ntcHRValues[i]);
    Serial.print(" | ");
  }
  Serial.println("");

  Serial.println("Gpiovalue");
  for (int i = 0; i < 3 ; i++){
    Serial.print(hrData.gpioHRValues[i]);
    Serial.print(" | ");
  }
  Serial.println("");

  Serial.println("ADCvalue");
  for (int i = 0; i < 3 ; i++){
    Serial.print(hrData.adcHRValues[i]);
    Serial.print(" | ");
  }
  Serial.println("");
  
  Serial.println("PackP, Vtop, Itop, DieTemp");
  Serial.print(hrData.packPReadings);
    Serial.print(" | ");
  Serial.print(hrData.vTopReadings);
    Serial.print(" | ");
  Serial.print(hrData.iTopReadings);
    Serial.print(" | ");
  Serial.println(hrData.dieTemp);

}

void xAlertISR(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  // If interrupts come faster than 50ms, assume it's noise and ignore it.
  if (interrupt_time - last_interrupt_time > 50) {
    // This is the *only* thing we do. Set the flag.
    // The main loop() will see this and do the actual work.
    Serial.println("Interrupt detected");
    xAlertFlag = true;
  }
  // Update the time
  last_interrupt_time = interrupt_time;
  
}

void testRun(){
  /*
  // ==========================================
// MP2790 Register Map & Bit Definitions
// ==========================================
// (Add these if not present in your MP2790.h)

#define INT0_EN         0x10
#define INT1_EN         0x11
#define CELLFT_CTRL     0x0B
#define PACKFT_CTRL     0x0C
#define CRNTFT_CTRL     0x0D
#define CELL_OV         0x16
#define CELL_UV         0x17
#define PACK_OV         0x18
#define PACK_UV         0x19
#define COC_CFG         0x1A
#define DOC1_CFG        0x1B
#define DOC2_CFG        0x1C
#define SC_D_CFG        0x1D

// Helper bitmasks for "Minimal" setup
// 0x8000 ensures the Global Interrupt/System bit is active (assuming 0x8000 is needed based on your example)
#define GLOBAL_INT_EN   0x8000 

// ==========================================
// Protection Test Sequence
// ==========================================

void runProtectionTests() {
    // Note: Ensure you reset the device or clear previous faults before switching tests.
  
  // --------------------------------------------------------
  // 1. Cell Over-Voltage (COV) - (From your example)
  // --------------------------------------------------------
   Serial.println("TEST: Cell Over-Voltage (COV)");
  
  // A. Set Threshold (Optional: Default is usually 4.2V-ish)
  // Formula: Val = Voltage / 2.5mV (Verify with your specific datasheet version)
  // Example: 4200mV / 2.5 = 1680 (0x0690)
  mp27.writeAdd(CELL_OV, 0x0690); 

  // B. Enable Interrupts
  // 0x8001 -> Global EN (bit 15) + COV_INT_EN (bit 0)
  mp27.writeAdd(INT0_EN, 0x8001); 

  // C. Enable Fault Protection
  // 0x0020 -> COV_EN (bit 5). (Your example used 0x8830 which enables CUV too)
  // We use 0x8020 to keep Global/Reserved bits high if needed.
  mp27.writeAdd(CELLFT_CTRL, 0x8020); 
  

  // --------------------------------------------------------
  // 2. Cell Under-Voltage (CUV)
  // --------------------------------------------------------
  
  Serial.println("TEST: Cell Under-Voltage (CUV)");

  // A. Set Threshold
  // Example: 2500mV / 2.5 = 1000 (0x03E8)
  mp27.writeAdd(CELL_UV, 0x03E8);

  // B. Enable Interrupts
  // 0x8002 -> Global EN + CUV_INT_EN (bit 1)
  mp27.writeAdd(INT0_EN, 0x8002);

  // C. Enable Fault Protection
  // 0x0010 -> CUV_EN (bit 4)
  mp27.writeAdd(CELLFT_CTRL, 0x8010);
  

  // --------------------------------------------------------
  // 3. Pack Over-Voltage (POV)
  // --------------------------------------------------------
  
  Serial.println("TEST: Pack Over-Voltage (POV)");

  // A. Set Threshold
  // Formula: Val = Voltage / 20mV (Check datasheet multiplier for PACK)
  // Example: 45V = 45000mV / 20 = 2250 (0x08CA)
  mp27.writeAdd(PACK_OV, 0x08CA);

  // B. Enable Interrupts
  // 0x8004 -> Global EN + POV_INT_EN (bit 2)
  mp27.writeAdd(INT0_EN, 0x8004);

  // C. Enable Fault Protection
  // 0x0020 -> POV_EN (bit 5 of PACKFT_CTRL)
  mp27.writeAdd(PACKFT_CTRL, 0x0020);
  

  // --------------------------------------------------------
  // 4. Pack Under-Voltage (PUV)
  // --------------------------------------------------------
  
  Serial.println("TEST: Pack Under-Voltage (PUV)");

  // A. Set Threshold
  // Example: 20V = 20000mV / 20 = 1000 (0x03E8)
  mp27.writeAdd(PACK_UV, 0x03E8);

  // B. Enable Interrupts
  // 0x8008 -> Global EN + PUV_INT_EN (bit 3)
  mp27.writeAdd(INT0_EN, 0x8008);

  // C. Enable Fault Protection
  // 0x0010 -> PUV_EN (bit 4 of PACKFT_CTRL)
  mp27.writeAdd(PACKFT_CTRL, 0x0010);
  

  // --------------------------------------------------------
  // 5. Charge Over-Current (COC)
  // --------------------------------------------------------
  
  Serial.println("TEST: Charge Over-Current (COC)");

  // A. Set Threshold
  // Formula depends on Rsense. Val ~ V_sense / Step
  mp27.writeAdd(COC_CFG, 0x00FF); // Set a low threshold for easy testing

  // B. Enable Interrupts (Note: Current faults are usually in INT1_EN)
  // 0x8001 -> Global EN + COC_INT_EN (bit 0 of INT1)
  mp27.writeAdd(INT1_EN, 0x8001);

  // C. Enable Fault Protection
  // 0x0080 -> COC_EN (bit 7 of CRNTFT_CTRL)
  mp27.writeAdd(CRNTFT_CTRL, 0x0080);
  

  // --------------------------------------------------------
  // 6. Discharge Over-Current 1 (DOC1)
  // --------------------------------------------------------
  
  Serial.println("TEST: Discharge Over-Current 1 (DOC1)");

  // A. Set Threshold
  mp27.writeAdd(DOC1_CFG, 0x00FF); // Set low threshold

  // B. Enable Interrupts
  // 0x8002 -> Global EN + DOC1_INT_EN (bit 1 of INT1)
  mp27.writeAdd(INT1_EN, 0x8002);

  // C. Enable Fault Protection
  // 0x0040 -> DOC1_EN (bit 6 of CRNTFT_CTRL)
  mp27.writeAdd(CRNTFT_CTRL, 0x0040);
  

  // --------------------------------------------------------
  // 7. Short Circuit Discharge (SC_D)
  // --------------------------------------------------------
  
  Serial.println("TEST: Short Circuit Discharge (SC)");

  // A. Set Threshold
  mp27.writeAdd(SC_D_CFG, 0x0050); // Set threshold

  // B. Enable Interrupts
  // 0x8008 -> Global EN + SC_D_INT_EN (bit 3 of INT1)
  mp27.writeAdd(INT1_EN, 0x8008);

  // C. Enable Fault Protection
  // 0x0010 -> SC_D_EN (bit 4 of CRNTFT_CTRL)
  mp27.writeAdd(CRNTFT_CTRL, 0x0010);
  

  // --------------------------------------------------------
  // 8. Discharge Over-Current 2 (DOC2)
  // --------------------------------------------------------
  
  Serial.println("TEST: Discharge Over-Current 2 (DOC2)");

  // A. Set Threshold & Delay
  // Reg: DSGOC_LIM (0x24) 
  // Bits [12:8] = Threshold (5 bits). 
  // Formula: Threshold = Val * Step (Check datasheet, e.g., 2mV/bit or similar)
  // We set a moderate value to distinguish it from DOC1.
  mp27.writeAdd(DSGOC_LIM, 0x0F00); // Example: 0x0F in bits 8-12

  // B. Enable Interrupts
  // Reg: INT0_EN (0x19)
  // Bit 15 (Global) + Bit 4 (OVER_CURR_INT_EN)
  mp27.writeAdd(INT0_EN, 0x8010); 

  // C. Enable Fault Protection
  // Reg: OCFT_CTRL (0x23)
  // Bit 7: OC2_DCHG_FAULT_EN
  // Bit 4: OC2_DCHG_INT_EN (Enable local interrupt generation)
  // 0x0090 = 1001 0000
  mp27.writeAdd(OCFT_CTRL, 0x0090); 
  

  // --------------------------------------------------------
  // 9. Short Circuit Charge (SC_C)
  // --------------------------------------------------------
  
  Serial.println("TEST: Short Circuit Charge (SC_C)");

  // A. Set Threshold
  // Reg: CHGSC_CFG (0x2C)
  // Bits [4:0] = Threshold.
  // We set a minimal threshold to trigger easily with safe current.
  mp27.writeAdd(CHGSC_CFG, 0x0005); 

  // B. Enable Interrupts
  // Reg: INT0_EN (0x19)
  // Bit 15 (Global) + Bit 5 (SHORT_CURR_INT_EN)
  mp27.writeAdd(INT0_EN, 0x8020); 

  // C. Enable Fault Protection
  // Reg: SCFT_CTRL (0x2A)
  // Bit 5: SC_CHG_FAULT_EN
  // Bit 3: SC_CHG_INT_EN
  // 0x0028 = 0010 1000
  mp27.writeAdd(SCFT_CTRL, 0x0028);
  
  //  --------------------------------------------------------
  // 10. Die Over-Temperature (OT_DIE)
  // --------------------------------------------------------
  
  Serial.println("TEST: Die Over-Temperature");

  // A. Set Threshold
  // Reg: DIE_OT (0x4D)
  // Bits [9:0] = Threshold (10 bits).
  // Formula usually: Voltage related to temp. Set low to trigger at room temp for test.
  // Example: Set to approx 25C-30C raw value (Check datasheet graph)
  mp27.writeAdd(DIE_OT, 0x0200); // Arbitrary low value to force trigger

  // B. Enable Interrupts
  // Reg: INT1_EN (0x1A) - Note: Temp faults are often on INT1
  // Bit 7: DIE_TEMP_INT_EN
  mp27.writeAdd(INT1_EN, 0x0080); 
  // Also ensure INT0 Global (Bit 15) is on if strictly required by your setup logic
  mp27.writeAdd(INT0_EN, 0x8000);

  // C. Enable Fault Protection
  // Reg: DIE_CFG (0x46)
  // Bit 3: DIE_TEMP_DIG_FAULT_EN
  // Bit 1: DIE_TEMP_DIG_EN (Enable the sensor itself)
  mp27.writeAdd(DIE_CFG, 0x000A); 
  

  // --------------------------------------------------------
  // 11. NTC Discharge Over-Temperature (OT_DSG)
  // --------------------------------------------------------
  
  Serial.println("TEST: NTC Over-Temp (Discharge)");

  // A. Set Thresholds
  // Reg: NTCC_OTHR_DSG (0x48) - "Over Temp Threshold Discharge"
  // Set to a voltage corresponding to a "Hot" state.
  mp27.writeAdd(NTCC_OTHR_DSG, 0x0150); 

  // B. Enable Interrupts
  // Reg: INT0_EN (0x19)
  // Bit 6: NTC_DSG_INT_EN
  mp27.writeAdd(INT0_EN, 0x8040); 

  // C. Enable Fault Protection
  // Reg: NTC_CFG (0x47)
  // Bit 14: NTC_CELL_DSG_FAULT_EN
  // Bits [0-7]: NTC Enable bits (Enable NTC1 for this test, bit 0)
  mp27.writeAdd(NTC_CFG, 0x4001); 

  // --------------------------------------------------------
  // 13. NTC Charge Over-Temperature (OT_CHG)
  // --------------------------------------------------------
  
  Serial.println("TEST: NTC Over-Temp (Charge)");

  // A. Set Threshold
  // Reg: NTCC_OTHR_CHG (0x4A)
  // Set to a raw value corresponding to "Hot" (e.g., < 1V or depends on NTC curve)
  mp27.writeAdd(NTCC_OTHR_CHG, 0x0150); 

  // B. Enable Interrupts
  // Reg: INT0_EN (0x19)
  // Bit 7: NTC_CHG_INT_EN + Bit 15: Global
  mp27.writeAdd(INT0_EN, 0x8080); 

  // C. Enable Fault Protection
  // Reg: NTC_CFG (0x47)
  // Bit 13: NTC_CELL_CHG_FAULT_EN
  // Bit 0:  NTC1_EN (Assuming NTC1 is used for the test)
  // 0x2001 = 0010 0000 0000 0001
  mp27.writeAdd(NTC_CFG, 0x2001); 
  

  // --------------------------------------------------------
  // 14. NTC Discharge Under-Temperature (UT_DSG / Cold)
  // --------------------------------------------------------
  
  Serial.println("TEST: NTC Under-Temp / Cold (Discharge)");

  // A. Set Threshold
  // Reg: NTCC_UTHR_DSG (0x49)
  // Set to a raw value corresponding to "Cold" (Higher voltage on NTC divider)
  mp27.writeAdd(NTCC_UTHR_DSG, 0x0300); 

  // B. Enable Interrupts
  // Reg: INT0_EN (0x19)
  // Bit 6: NTC_DSG_INT_EN (Shared with OT, usually distinct by status flag)
  mp27.writeAdd(INT0_EN, 0x8040); 

  // C. Enable Fault Protection
  // Reg: NTC_CFG (0x47)
  // Bit 14: NTC_CELL_DSG_FAULT_EN
  // Bit 0:  NTC1_EN
  mp27.writeAdd(NTC_CFG, 0x4001); 
  
  // --------------------------------------------------------
  // 15. Open Wire Detection (OW)
  // --------------------------------------------------------
  
  Serial.println("TEST: Open Wire Detection");
  // To Trigger: Physically disconnect a cell balance wire (e.g., Cell 2).

  // A. Set Thresholds
  // Reg: OPEN_CFG (0x57)
  // Bits [11:8]: Threshold (Voltage drop to detect open)
  // Bits [3:0]:  Check Length/Time
  mp27.writeAdd(OPEN_CFG, 0x0202); 

  // B. Enable Interrupts
  // Reg: INT1_EN (0x1A)
  // Bit 4: OPEN_WIRE_INT_EN
  mp27.writeAdd(INT1_EN, 0x0010); 
  mp27.writeAdd(INT0_EN, 0x8000); // Global Enable

  // C. Enable Fault Protection & Logic
  // Reg: SELF_CFG (0x56)
  // Bit 10: OPEN_WIRE_PON (Power on detection)
  // Bit 9:  OPEN_WIRE_FAULT_EN
  // 0x0600 = 0000 0110 0000 0000
  mp27.writeAdd(SELF_CFG, 0x0600); 
  
  // D. Note: You may need to manually trigger the scan if not automatic:
  // mp27.writeAdd(SFT_GO, 0x0100); // Bit 8: OPEN_WIRE_GO
  

  // --------------------------------------------------------
  // 16. Dead Cell Detection
  // --------------------------------------------------------
  
  Serial.println("TEST: Dead Cell Detection");
  // To Trigger: Drop one cell simulator voltage near 0V or below threshold.

  // A. Set Threshold
  // Reg: CELL_DEAD_THR (0x3C)
  // Bits [6:0]: Threshold (e.g., 2V or similar very low value)
  mp27.writeAdd(CELL_DEAD_THR, 0x0010); 

  // B. Enable Interrupts
  // Reg: INT1_EN (0x1A)
  // Bit 5: CELL_DEAD_INT_EN
  mp27.writeAdd(INT1_EN, 0x0020);
  mp27.writeAdd(INT0_EN, 0x8000);

  // C. Enable Fault Protection
  // Reg: PACKFT_CTRL (0x34)
  // Bit 9: CELL_DEAD_FAULT_EN
  // Bit 8: CELL_DEAD_EN
  // 0x0300 = 0000 0011 0000 0000
  mp27.writeAdd(PACKFT_CTRL, 0x0300); 
  
  */
  


}