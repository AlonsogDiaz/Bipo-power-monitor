#include "comms.h"

#include <SD.h>


bool serialOn = true;
bool logOn = true;
extern File dataFile;


// snprintf(buff, sizeof(buff), ...)
/**
 * @brief Loggea informacion a SD y puerto serial
 * 
 * @param str char*, String, sprintf(), snprintf()
 * @return int Numero de bytes escritos
 */
int logger(const char *str){
  size_t writtenBytes = 0;
  char t_buff[30];

  // char *src = "Worldly Wonders";
  // char remainder[127];
  // int limit = sizeof(str);
  // int current_len = strlen(str);
  // int space_left = limit - current_len - 1; // -1 for null terminator

  // if (space_left > 0) {
  //     // Copy only what fits
  //     strncpy((char*)str + current_len, src, space_left);
  //     // str[limit - 1] = '\0'; // Manually ensure null termination

  //     // Check if there is a remainder
  //     if (strlen(src) > space_left) {
  //         strcpy(remainder, src + space_left);
  //         printf("Dest: %s\n", str);
  //         printf("Remainder: %s\n", remainder);
  //     }
  // }


  if (Serial && serialOn){
    // snprintf(t_buff,sizeof(t_buff),"%02d:%02d:%02d_%02d:%02d:%02d ", year(), month(), day(), hour(), minute(), second());
    // writtenBytes = Serial.print(strcat(t_buff, str));
    writtenBytes = Serial.print(str);
  }

  if (dataFile && logOn){
    writtenBytes = dataFile.print(str);
    dataFile.flush();
  }
  
  return (int)writtenBytes;
}

int logger(char *str){
  return logger((const char*)str);
}
int logger(String str){
  return logger((const char*)str.c_str());
}