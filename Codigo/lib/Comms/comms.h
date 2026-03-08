#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include <cstdint>
#include "RTCmanager.h"

int logger(const char *str);
int logger(char *str);
int logger(String str);

#endif