/**
 * @file MP2790.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef MP2790_H
#define MP2790_H

#include "MP2790_driver.h"

// Comms
extern uint16_t mp2790_nCells;            
extern uint16_t mp2790_address;          
extern bool mpStatus;  
// Alerts
extern bool intFlags[32];
extern bool faultFlags[32];
// Data
extern mpDataP hrData;      

// Initialize using MP27XX(i2c_address, number of cells).  
extern MP2790 mp27;             

//Funciones
//Set up
// void setupMP2790();

//Metodos de funcion
void checkSensors();
void checkInt();
void checkFaults();
void handleInt(bool *_intFlags);
void handleFault(bool *_faultFlags);
void dumpHRData();
void fetFlip();
// bool resetFET(bool trigger = false);
float getSoC(float currentOCV);

//Tests
void mainTests();

//Otros
bool millisTracker(uint32_t *trackMillis, uint16_t trackTime);

/**
 * Battery OCV to SoC Lookup Table
 * Weighted by rate of change. 
 * Size: 100 points
 */

const int BATT_TABLE_SIZE = 100;

// Open Circuit Voltage (Independent Variable)
const float ocv_table[100] = {
    25.540600f, 25.121499f, 24.966139f, 24.859605f, 24.811693f, 24.714992f, 24.693805f, 24.607466f, 24.561117f, 24.502326f,
    24.401402f, 24.383430f, 24.304944f, 24.246295f, 24.186371f, 24.107169f, 24.051111f, 23.996335f, 23.909284f, 23.828249f,
    23.821331f, 23.726635f, 23.653022f, 23.619460f, 23.590834f, 23.515271f, 23.482226f, 23.441481f, 23.414717f, 23.373448f,
    23.336684f, 23.337677f, 23.305334f, 23.280729f, 23.255807f, 23.213323f, 23.217141f, 23.139274f, 23.119792f, 23.080491f,
    23.054720f, 23.003602f, 23.015481f, 22.931901f, 22.942713f, 22.881852f, 22.933317f, 22.851327f, 22.821547f, 22.818582f,
    22.833116f, 22.819040f, 22.776119f, 22.790042f, 22.749232f, 22.706285f, 22.740989f, 22.691188f, 22.652525f, 22.650481f,
    22.666006f, 22.625759f, 22.642672f, 22.617522f, 22.612726f, 22.581251f, 22.578683f, 22.590272f, 22.563408f, 22.504978f,
    22.496883f, 22.483794f, 22.449478f, 22.470251f, 22.418939f, 22.440809f, 22.386496f, 22.415117f, 22.423982f, 22.389854f,
    22.350566f, 22.313063f, 22.284980f, 22.258993f, 22.268622f, 22.191519f, 22.166441f, 22.133173f, 22.116147f, 22.073872f,
    21.992314f, 21.958665f, 21.871287f, 21.846195f, 21.754995f, 21.677128f, 21.570509f, 21.478930f, 21.386750f, 21.269524f
};

// State of Charge (Dependent Variable, 0.0 to 1.0)
const float soc_table[100] = {
    1.000000f, 0.989426f, 0.980356f, 0.971190f, 0.963216f, 0.956544f, 0.948603f, 0.942706f, 0.935924f, 0.926919f,
    0.918030f, 0.911022f, 0.900605f, 0.892660f, 0.883287f, 0.873697f, 0.864915f, 0.855901f, 0.845812f, 0.835331f,
    0.826684f, 0.812122f, 0.799169f, 0.787329f, 0.775145f, 0.763203f, 0.753703f, 0.741962f, 0.732471f, 0.722439f,
    0.712779f, 0.702158f, 0.693922f, 0.683708f, 0.674759f, 0.662655f, 0.654844f, 0.643867f, 0.632741f, 0.618930f,
    0.608212f, 0.593983f, 0.583715f, 0.570825f, 0.562666f, 0.548986f, 0.541400f, 0.528114f, 0.518440f, 0.509751f,
    0.500803f, 0.493450f, 0.485771f, 0.474501f, 0.462626f, 0.453971f, 0.443341f, 0.435431f, 0.424407f, 0.417153f,
    0.406977f, 0.392891f, 0.383055f, 0.374401f, 0.365594f, 0.355619f, 0.347660f, 0.338336f, 0.329848f, 0.315623f,
    0.303799f, 0.292575f, 0.280541f, 0.270610f, 0.262257f, 0.252522f, 0.240007f, 0.230403f, 0.220581f, 0.211863f,
    0.203257f, 0.193916f, 0.179554f, 0.169426f, 0.156604f, 0.147228f, 0.136034f, 0.124094f, 0.113975f, 0.102312f,
    0.092361f, 0.081238f, 0.070812f, 0.059387f, 0.048876f, 0.038090f, 0.027209f, 0.018116f, 0.008493f, 0.000000f
};

#endif //MP2790_H