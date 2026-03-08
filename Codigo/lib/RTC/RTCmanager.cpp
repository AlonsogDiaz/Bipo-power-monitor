
#include "RTCmanager.h"

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void initRTC() {
    // Captura el tiempo interno
    setSyncProvider(getTeensy3Time);

    // Comprueba validez del tiempo
    if (timeStatus() != timeSet || year() < 2025) {    
        logger("RTC invalido o energia perdida");
        if(!Serial){
            Teensy3Clock.set(0);
            logger("RTC reiniciado\n");
        }
        logger("REQUERIDO: Enviar 'T' seguido de tiempo en formato Unix Epoch>\n");
        logger("Ej. T946684800\n");
    } 
    else{
        logger("RTC sincronizado\n");
    }

}


void updateRTC() {
  // If time is invalid, keep prompting until corrected
  if (timeStatus() != timeSet || year() < 2025) {
    if (millis() % 5000 < 10) { // Non-blocking reminder every 5 seconds
       logger("Waiting for valid time input (T+Timestamp)...");
    }
  }

  if (Serial.available()) {
    if (Serial.find("T")) {
      unsigned long pctime = Serial.parseInt();
      if (pctime >= 1609459200) { // Verify timestamp is post-2021
        Teensy3Clock.set(pctime); // Set hardware RTC
        setTime(pctime);          // Set software system time
        logger("Time Updated and Saved to Hardware RTC.");
      }
    }
  }
}