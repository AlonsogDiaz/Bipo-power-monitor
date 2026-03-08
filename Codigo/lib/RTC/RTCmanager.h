#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <Arduino.h>
#include <TimeLib.h>
#include <time.h>
#include "comms.h"

/**
 * @brief Obtiene objeto de tiempo Teensy
 * @return time_t 
 */
time_t getTeensy3Time();

/**
 * @brief Inicializa el RTC y comprueba entrada
 */
void initRTC();

// Call this in loop() to handle serial time setting if the clock is invalid

/**
 * @brief Llamar en loop() para actualizar tiempo si es invalido
 */
void updateRTC();



#endif