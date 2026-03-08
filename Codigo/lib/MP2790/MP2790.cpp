#include "MP2790.h"

// Comms
uint16_t mp2790_nCells = 6;               // 6 cells
uint16_t mp2790_address = 0x01;           // Default address
bool mpStatus = false;   
// Alerts
bool intFlags[32];
bool faultFlags[32];
// Data
mpDataP hrData;      

// Inicializar usando MP2790(celda mp2790, numero de celdas).  
MP2790 mp27 = MP2790(mp2790_address, mp2790_nCells); // sin argumentos: por defecto


/**
 * @brief Lee y reporta el valor de los sensores
 */
void checkSensors() {

  // Temperatura 
  uint16_t dieTemp = mp27.getDieTemperature();
  Serial.print("Temperatura Die [°C *100]: ");
  Serial.println(dieTemp);
  
  // Temperatura HR (Desabilitado, problemas de calibracion)
  // uint16_t dieTempHR = mp27.getHRDieTemperature();
  // Serial.print("Temperature Die HR: ");
  // Serial.println(dieTempHR);

  // Temperatura NTC
  int32_t ntcreadings[4];
  Serial.print("Temperatura NTC [°C *100]: ");
  mp27.getNTCreadings(ntcreadings);
  for (int i = 0; i < 4; i++) {
    Serial.print(ntcreadings[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Lecturas ADC
  uint16_t adcValues[4];
  mp27.getADCReadings(adcValues);
  Serial.println("Lecturas ADC [mV]: 1v8, 3v3, regin, Vself: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(adcValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Vtop
  Serial.print("Lecturas VTop [mV]: "); 
  Serial.println(mp27.getVTopReadings());

  // Estato de poder
  mp27.powerStatus();    

  // Direction de la corriente
  mp27.currentDirection();

  // Lecturas ADC HR
  mp27.triggerHRScan(1);
}

/**
 * @brief Comprueba si hay interrupts 
 */
void checkInt(){
  // Interrupts
  // Serial.println("Checking Interrupts...");
  if (mp27.getInt(intFlags)){
    Serial.println("Interrupts active");
    handleInt(intFlags);
    return;
  }
  Serial.println("No Interrupts...");
}
/**
 * @brief Comprueba si hay fallas 
 */
void checkFaults(){
  // Faults
  // Serial.println("Checking Faults...");
  if (mp27.getFault(faultFlags)){
    Serial.println("Faults active");
    handleFault(faultFlags);
    return;
  }
  Serial.println("No Faults...");
  return;
}

/**
 * @brief Despeja interrupts
 */
void handleInt(bool *_intFlags) {
  // Serial.println("Handling Interrupts...");
  for (int i = 0; i < 32; i++) {
    if (_intFlags[i]){ // Skip if no interrupt flag is set
      Serial.print("Interrupt ");
      Serial.println(i);
      switch (i) {
        case 0:{
          Serial.println("Cell OV dismissed");
          mp27.clearInt(i);
          break;
        }
        case 1: {
          Serial.println("Cell UV dismissed");
          mp27.clearInt(i);
          break;
        }
        case 2: {
          Serial.println("Stack UV dismissed");
          mp27.clearInt(i);
          break;
        }
        case 3: {
          Serial.println("Stack OV dismissed");
          mp27.clearInt(i);
          break;
        }
        case 4: {
          Serial.println("OC dismissed");
          mp27.clearInt(i);
          break;
        }
        case 5: {
          Serial.println("SC dismissed");
          mp27.clearInt(i);
          break;
        }
        case 6:{
          Serial.println("NTC DSG hot/cold dismissed");
          mp27.clearInt(i);
          break;
        }
        case 7:{
          Serial.println("NTC CHG hot/cold dismissed");
          mp27.clearInt(i);
          break;
        }
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
          // if(mp27.readAdd(MP2790_Reg::DIE_TEMP_DIG_STS) == 1 && mp27.readAdd(MP2790_Reg::DIE_TEMP_DIG_RT_STS) == 0){
            Serial.println("PCB die hot clear");
            mp27.writeAdd(MP2790_Reg::DIE_TEMP_DIG_CLEAR, 1);
            mp27.clearInt(i);
          // }
          break;
        }
        case 24: {
          Serial.println("PCB ntc hot clear");
          mp27.clearInt(i);
          break;
        }
        case 25: {
          mp27.clearInt(i);
          break;
        }
        case 26:{
          mp27.clearInt(i);
          break;
        }
        case 27: break;
        default: {
          Serial.println("Interrupt out of boundaries");
          break;
        }
      }
    }
  }
}

/**
 * @brief Despeja fallas
 */
void handleFault(bool *_faultFlags) {

  // Serial.println("Handling Faults...");

  for (int i = 0; i < 32; i++) {
    if (_faultFlags[i]){ // Skip if no fault flag is set
      Serial.print("Fault ");
      Serial.print(i);
      Serial.print(" ");
      mp27.clearFault(i);
    }
  }
  Serial.println("cleared");
  return;
}

/**
 * @brief Volcado de datos ADC HR 
 */
void dumpHRData(){
  Serial.println("HRvoltage mV");
  for (int i = 0; i<10 ; i++){
    Serial.print(hrData.cellHRVoltage[i]);
    Serial.print(" | ");
  }
  Serial.println("");

  Serial.println("HRCurrent mA");
  for (int i = 0; i<10 ; i++){
    Serial.print(hrData.cellHRCurrent[i]);
    Serial.print(" | ");
  }
  Serial.println("");

  Serial.println("NTCvalue [°C *100]");
  for (int i = 0; i<4 ; i++){
    Serial.print(hrData.ntcHRValues[i]);
    Serial.print(" | ");
  }
  Serial.println("");

  // Serial.println("Gpiovalue");
  // for (int i = 0; i < 3 ; i++){
  //   Serial.print(hrData.gpioHRValues[i]);
  //   Serial.print(" | ");
  // }
  // Serial.println("");

  Serial.println("HR ADCvalue [mV]: 1v8 3v3 45v");
  for (int i = 0; i < 3 ; i++){
    Serial.print(hrData.adcHRValues[i]);
    Serial.print(" | ");
  }
  Serial.println("");
  
  Serial.println("PackP [mV], Vtop [mV], Itop [mA]");//, DieTemp");
  Serial.print(hrData.packPReadings);
    Serial.print(" | ");
  Serial.print(hrData.vTopReadings);
    Serial.print(" | ");
  Serial.println(hrData.iTopReadings);
  //   Serial.print(" | ");
  // Serial.println(hrData.dieTemp);

}

/**
 * @brief Interruptor llave de paso 
 */
void fetFlip(){
  if (mp27.powerStatus() == 0b01000){ //Normal c 
    checkInt();
    checkFaults();
  }
  else if(mp27.powerStatus() == 0b00001){ // safe
    Serial.println("Turn on Fet");
    mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 0);    
    mp27.writeAdd(MP2790_Reg::DRIVER_FAULT_CLR, 1);
    // delay(1000);
    mp27.writeAdd(MP2790_Reg::ACTIVE_CTRL, 3);
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

//Keeps track of millis counter
bool millisTracker(uint32_t *trackMillis, uint16_t trackTime){
  if (millis() - *trackMillis > trackTime){
    *trackMillis = millis();
    return true;
  }
  return false;
}
float getSoC(float currentOCV) {
    // Clamp values to table boundaries
    if (currentOCV >= ocv_table[0]) return soc_table[0];
    if (currentOCV <= ocv_table[BATT_TABLE_SIZE - 1]) return soc_table[BATT_TABLE_SIZE - 1];

    // Binary search for descending table
    int low = 0;
    int high = BATT_TABLE_SIZE - 1;
    
    while (high - low > 1) {
        int mid = low + (high - low) / 2;
        // Since OCV is descending, if currentOCV is higher than mid, 
        // the target is in the left half (smaller indices)
        if (ocv_table[mid] < currentOCV) {
            high = mid;
        } else {
            low = mid;
        }
    }

    // Linear Interpolation
    float v0 = ocv_table[low];
    float v1 = ocv_table[high];
    float s0 = soc_table[low];
    float s1 = soc_table[high];

    return s0 + (currentOCV - v0) * (s1 - s0) / (v1 - v0);
}