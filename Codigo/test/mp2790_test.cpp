#include "MP2790.h"
#include "mp2790_test.h"

void mainTests(){
  Serial.println("Minimal setup");
  mp27.writeAdd(CELLS_CTRL, 0x0005);  //6 cells
  mp27.writeAdd(INT0_EN, 0x8800);  //EN xalert and Vscan
  mp27.writeAdd(HR_SCAN0, 0x037f);  //EN Scan ldos+

  //EN cell scan (0x3f: 1 to 6) (0x39: 1,4,5,6) (0x01: 1)
  mp27.writeAdd(HR_SCAN1, 0x0039);  

  //0x01C7: def
  //0x51C7: OC CHG rising int + OC DSG rising int
  mp27.writeAdd(OCFT_CTRL, 0x037f);  //EN Scan ldos+


  // -----------------------------------------------------
  // 0. Function legend
  /*
  // A. Register descrition
  // NAME_REG(0xNN, parent address): 0xNNNN def (default value)
  // 0xMMMM use (used value)
  // Bit X-Y: Configuration bit
  // Bit Z:   0:Configuration bit ; *1:Configuration bit used
  */

  //

  // -----------------------------------------------------
  // 1. OK Cell Over-Voltage  
  /*
  Serial.println("TEST: Cell Over-Voltage (COV)");

  // A. Set Threshold
  // CELL_OV(0x39): 0x00D7 def
  // 0x03D7 use
  // Bit 8-11: Deglitch: BIN (0b0011 0x03XX: 4) times
  // Bit 0-7 : Formula: Volt mV= Val * 19.53 (0xXXD7: 4.199mV)
  mp27.writeAdd(CELL_OV_X, 0x03D7); 

  // B. Enable Alerts Protection
  // CELLFT_CTRL(0x35): 0x0000 def
  // 0x0210 use
  // Bit 10-9 : 0x01 rising 0x0200
  // Bit 5: Enable cell OV fault protection 0x0020
  // Bit 4: Enable cell OV int and fault detection 0x0010
  mp27.writeAdd(CELLFT_CTRL, 0x0210); 

  // C. Enable Interrupts
  // INT0_EN(0x19): 0x0000 def
  // 0x8801 use
  // bit 15: xAlert EN
  // bit 11: HR_ADC scan EN
  // bit 0 : Cell OV
  mp27.writeAdd(INT0_EN, 0x8801); 

  // D. Enable interrupt masking (Hides the INTERRUPT!)
  // MASK_INT0(0x1E): 0x0000 def
  // bit 0 : Cell OV
  // mp27.writeAdd(MASK_INT0, 0x0000); 

  // E. Fault protection & recovery
  // FT0_CFG(0x61): 0x0110 def
  // 0x0510 use
  // Bit 11: *0: manual             ;  1: auto
  // Bit 10:  0: turn off chg       ; *1: turn off both
  // Bit 09: *0: rec if clear & B8  ;  1: rec if clear & voltage drop
  // Bit 08:  0: not rec            ; *1: rec if dsg
  mp27.writeAdd(FT0_CFG, 0x0510);
  */

  //

  // -----------------------------------------------------
  // 2. OK Cell Under-Voltage (CUV)
  /*
  Serial.println("TEST: Cell Under-Voltage (CUV)");
  // A. Set Threshold
  // CELL_UV(0x38): 0x0098 def
  // 0x03A9 use
  // Bit 8-11: Deglitch: BIN (0b0011 0x03XX: 4) times
  // Bit 0-7 : Formula: Volt mV= Val * 19.53 (0xXXA9: 3.301mV)
  mp27.writeAdd(CELL_UV_X, 0x03A9); 

  // B. Enable Alerts Protection
  // CELLFT_CTRL(0x35): 0x0280 def
  // 0x0082 use
  // Bit 8-7: 0x01 rising 0x0080
  // Bit 2: Enable cell OV fault protection 0x0004
  // Bit 1: Enable cell OV int and fault detection 0x0002
  mp27.writeAdd(CELLFT_CTRL, 0x0082); 

  // C. Enable Interrupts
  // INT0_EN(0x19): 0x0000 def
  // 0x8801 use
  // bit 15: xAlert EN
  // bit 11: HR_ADC scan EN
  // bit 1 : Cell UV
  mp27.writeAdd(INT0_EN, 0x8802); 

  // D. Enable interrupt masking (Hides the INTERRUPT!)
  // MASK_INT0(0x1E): 0x0000 def
  // 0x0000 use
  // bit 1 : Cell UV
  // mp27.writeAdd(MASK_INT0, 0x0000); 

  // E. Fault protection & recovery
  // FT0_CFG(0x61): 0x0110 def
  // 0x0150 use
  // Bit 7: *0: manual              ;  1: auto
  // Bit 6:  0: turn off chg        ; *1: turn off both
  // Bit 5: *0: rec if clear & B8   ;  1: rec if clear & voltage drop
  // Bit 4:  0: not rec             ; *1: rec if dsg
  mp27.writeAdd(FT0_CFG, 0x0150);
  */

  //

  // -----------------------------------------------------
  // 3. Vtop Over-Voltage  
  /*
  Serial.println("TEST: Vtop Over-Voltage (TOP_OV)");
  
  // A. Enable Function
  // PACKFT_CTRL(0x34):  def
  // 0x0000 def
  // 0x0010 use
  // Bit  5: VTOP_OV_FAULT_EN_CTRL (0:off ; 1:en)
  // Bit *4: VTOP_OV_EN_CTRL       (0:off ; 1:en)
  mp27.writeAdd(0x34, 0x0010); 

  // B. Enable Interrupts
  // INT0_EN(0x19)
  // 0x0000 def
  // 0x8808 use
  // bit 15: xAlert EN
  // bit 11: HR_ADC scan EN
  // bit 3 : Vtop OV
  mp27.writeAdd(0x19, 0x8808); 
  // INT_TYPE0(0x1B)
  // 0x0000 def
  // 0x0004 use
  // Bit 3-2: VTOP_OV_Int_type: 0b01 Rising
  mp27.writeAdd(0x1B, 0x0004); 

  // C. Threshold
  // PACK_OV (0x3B)
  // def 0x0866
  // use 0x051A
  // Bit 15-12: Deglitch
  // Bit 11-00: Limit (def: 2150|0x0866|42V) (use: 1306|0x051A|25.5V))
  mp27.writeAdd(0x3B, 0x051A); 
  

  // May interfere 
  // // O1.Disable OC  Fault Protection
  // // OCFT_CTRL 0x23
  // // 0x01C7 def
  // // 0x000 use
  // mp27.writeAdd(0x23, 0x0000);

  // // O2. Disable SC Fault Protection
  // // Reg: SCFT_CTRL (0x2A)
  // // 0x0033 def
  // // 0x0000 use
  // mp27.writeAdd(0x2A, 0x0000);
  */

  // -----------------------------------------------------
  // 4. Vtop Under-Voltage  
  /*
  Serial.println("TEST: Vtop Under-Voltage (TOP_UV)");
  
  // A. Enable Function
  // PACKFT_CTRL(0x34)
  // 0x0000 def
  // 0x0001 use
  // Bit  1: VTOP_UV_FAULT_EN_CTRL (0:off ; 1:en)
  // Bit *0: VTOP_UV_EN_CTRL       (0:off ; 1:en)
  mp27.writeAdd(0x34, 0x0001); 

  // B. Enable Interrupts
  // INT0_EN(0x19)
  // 0x0000 def
  // 0x8804 use
  // bit 15: xAlert EN
  // bit 11: HR_ADC scan EN
  // bit 2 : Vtop OV
  mp27.writeAdd(0x19, 0x8804); 
  // INT_TYPE0(0x1B)
  // 0x0000 def
  // 0x0001 use
  // Bit 1-0: VTOP_UV_Int_type: 0b01 Rising
  mp27.writeAdd(0x1B, 0x0001); 

  // C. Threshold
  // PACK_OV (0x3A)
  // def 0x059A
  // use 0x03CD
  // Bit 15-12: Deglitch
  // Bit 11-00: Limit (def: 1434|0x059A|28V) (use: 973|0x03CD|19V)
  mp27.writeAdd(0x3A, 0x03CD); 
  */


  // -----------------------------------------------------
  // 5. NOK Temperature DIE

  // OK Temp die regular

  // NOK Temp die HR
  // mp27.writeAdd(HR_SCAN0, 0x0367); 



  //  ---------------------------------------------------
  // 6. OK Die Over-Temperature (OT_DIE)
  /*
  Serial.println("TEST: Die Over-Temperature");

  // A. Enable
  // Bit 1: Enable
  mp27.writeAdd(0x46, 0x0002); 

  // B. Enable Interrupts
  // Reg: INT1_EN (0x1A) - Note: Temp faults are often on INT1
  // Bit 7: DIE_TEMP_INT_EN
  mp27.writeAdd(0x1A, 0x0080); 
  // Interrupt type
  // 0x0000 def
  // 0x0010 use
  // Bit 4-5: 01 rising
  mp27.writeAdd(0x1D, 0x0010); 

  // C. Set Threshold
  // Reg: DIE_OT (0x4D)
  // 0x6A97 use
  // Bits [15:11] = hysteresis (5 bits/13 hex: 6deg C ) 
  // Bits [9:0] = Threshold (10 bits/297 hex: 45deg C ) 
  mp27.writeAdd(0x4D, 0x6A97);

  // D. Recovery
  // Reg: FT_REC (0x60)
  // 0x0000 def
  // 0x1000 use
  // Bits 12: 0: manual; 1: auto
  mp27.writeAdd(0x60, 0x0000);
  */

  //

  // -----------------------------------------------------
  // 10. OK NTCS
  /*
  // A.Enable NTC NTC_CFG 0x47
  // 0x0470 def
  // 0x0431 use (EN: 1(cell), 3(monitor))
  // Bit 1,3,5,7: for 1,2,3,4:0: 0=cell monitor; 1=pcb monitor
  // Bit 0,2,4,6: Enable NTC 1,2,3,4
  mp27.writeAdd(NTC_CFG, 0x0431);

  // B.Enable NTC HR 
  // 0x0000 def
  // 0x00A0 use (EN: 1,3)
  // Bit 5,6,7,8: Enable ntc hr 1,2,3,4
  mp27.writeAdd(HR_SCAN2, 0x00A0);  
  */
 
  //

  // ----------------------------------------------------
  // 11. OK NTC PCB hot 
  /*
  
  // Serial.println("TEST: NTC PCB HOT");
  // A.Enable NTC_CFG 0x47
  // 0x0470 def
  // 0x0431 use (EN: 1(cell), 3(monitor))
  // Bit 15: Fault EN (PCB HOT))
  // Bit *10: dynamic on
  // Bit 1,3,*5,7: for 1,2,3,4:0: 0=cell monitor; 1=pcb monitor
  // Bit *0,2,*4,6: Enable NTC 1,2,3,4
  mp27.writeAdd(0x47, 0x0431);
  
  // B.Enable PCB hot int 
  // INT1_EN 0x1A
  // 0x0000 def
  // 0x0100 use
  // Bit 8: En ntc pcb hot
  mp27.writeAdd(0x1A, 0x0100);  
  // INT_TYPE2 0x1D
  // 0x0000 def
  // 0x0100 use
  // Bit 6-7: Int type: 0b01 Rising
  mp27.writeAdd(0x1D, 0x0040);  
  
  // C.Threshold PCB hot int 
  // NTCM_OTHR 0x4C
  // 0x0000 def
  // 0x8124 use
  // Bit 15:11 = hysteresis (5 bits/16 dec: 3.125 C ) 
  // Bit 9-0 = threshold (def: 0xEB|23%|57°C)(use: 0x0124|28.5%|50°C)
  mp27.writeAdd(0x4C, 0x8124);  
  */

  //

  // ----------------------------------------------------
  // 12. OK NTC CELL DSG HOT/COLD
  /*
  // Serial.println("TEST: NTC CELL DSG HOT/COLD");

  // A.Enable NTC Cell NTC_CFG 0x47
  // 0x0470 def
  // 0x0431 use (EN: 1(cell), 3(monitor))
  // Bit 14: Fault EN (Cell DSG))
  // Bit *10: dynamic on
  // Bit 1,3,*5,7: for 1,2,3,4:0: 0=cell monitor; 1=pcb monitor
  // Bit *0,2,*4,6: Enable NTC 1,2,3,4
  mp27.writeAdd(0x47, 0x0431);
  
  // B.Enable NTC Cell DSG hot/cold int 
  // INT0_EN 0x19
  // 0x8800 def
  // 0x8840 use
  // Bit *6: En ntc dsg hot/cold
  mp27.writeAdd(0x19, 0x8840);  
  // INT_TYPE0 0x1B
  // 0x0000 def
  // 0x0050 use
  // Bit 7-6: NTC CHG Int type: 0b01 Rising
  // Bit 5-4: NTC DSG Int type: 0b01 Rising
  mp27.writeAdd(0x1B, 0x0050);  
  
  // C. Hysteresis NTC Cell hot/cold int 
  // NTCC_UTHR_CHG 0x4B (For all NTC cell)
  // 0x8A97 def
  // Bit 15:11 = hysteresis (5 bits/177|3.320 C ) 
  mp27.writeAdd(0x4B, 0x8A97);  

  // D.1.Threshold NTC Cell DSG HOT 
  // NTCC_OTHR_DSG 0x48
  // 0x012E def (302 | 29.5% | 49°C)
  // 0x0124 use (292 | 28.5% | 50°C)
  // Bit 9:0 = 10bit threshold 
  mp27.writeAdd(0x48, 0x0124);  

  // D.2.Threshold NTC Cell DSG COLD 
  // NTCC_UTHR_DSG 0x49
  // 0x0294 def (660 | 64.5% | 17°C)
  // 0x0294 use (660 | 64.5% | 17°C)
  // Bit 9:0 = 10bit threshold 
  mp27.writeAdd(0x49, 0x0294);  
  */

  //

  // ----------------------------------------------------
  // 13.OK NTC CELL CHG HOT/COLD
  /*
  // Serial.println("TEST: NTC CELL CHG HOT/COLD");
  
  // A.Enable NTC Cell NTC_CFG 0x47
  // 0x0470 def
  // 0x0431 use (EN: 1(cell), 3(monitor))
  // Bit 13: Fault EN (Cell CHG))
  // Bit *10: dynamic on
  // Bit 1,3,*5,7: for 1,2,3,4:0: 0=cell monitor; 1=pcb monitor
  // Bit *0,2,*4,6: Enable NTC 1,2,3,4
  mp27.writeAdd(0x47, 0x0431);
  
  // B.Enable NTC Cell CHG hot/cold int 
  // INT0_EN 0x19
  // 0x8800 def
  // 0x8880 use
  // Bit *7: En ntc CHG hot/cold
  mp27.writeAdd(0x19, 0x8880);  
  // INT_TYPE0 0x1B
  // 0x0000 def
  // 0x0050 use
  // Bit 7-6: NTC CHG Int type: 0b01 Rising
  // Bit 5-4: NTC DSG Int type: 0b01 Rising
  mp27.writeAdd(0x1B, 0x0050);  
  
  
  // // MIX BELOW!
  // // C. Hysteresis NTC Cell CHG hot/cold 
  // // NTCC_UTHR_CHG 0x4B (For all NTC cell)
  // // 0x8A97 def
  // // Bit 15:11 = hysteresis (5 bits/177|3.320 °C) 
  // // mp27.writeAdd(0x4B, 0x8A97);  

  // D.1.Threshold NTC Cell CHG HOT 
  // NTCC_OTHR_CHG 0x4A
  // 0x012E def (302 | 29.5% | 49°C)
  // 0x0124 use (292 | 28.5% | 50°C)
  // Bit 9:0 = 10bit threshold 
  mp27.writeAdd(0x4A, 0x0124);  

  // D.2.Threshold NTC Cell CHG COLD 
  // NTCC_OTHR_CHG 0x4B
  // 0x0294 def (660 | 64.5% | 17°C)
  // 0x8A94 use (660 | 64.5% | 17°C)
  // Bit 15:11 = hysteresis (5 bits/177|3.320 °C) 
  // Bit 9:0 = 10bit threshold 
  mp27.writeAdd(0x4B, 0x8A94);  
  */

  //

  // --------------------------------------------------------
  // 20. OK Discharge Over-Current 1 (DOC1)
  /*
  // Serial.println("TEST: Discharge Over-Current 1");
  // //OCFT_CTRL 0x23
  // // 0x01C7 def
  // // 0x1049 use
  // // Bit 13-12: Int type (*Rising)
  // // Bit 6: DSG 1 OC EN FAULT
  // // Bit 3: DSG 1 OC EN INT (needs OVER_CURR_INT_EN 0x19 b4)
  // // Bit 0: DSG 1 OC EN
  // mp27.writeAdd(0x23, 0x1049);

  // // EN xalert & Vscan
  // // INT1_EN (0x19)
  // // Bit 4: OC
  // mp27.writeAdd(0x19, 0x8810);  

  // // DSCOC_LIM (0x24))
  // // def 0x2910
  // // use 0x0010
  // // Bit 5   : OC DSG Range (*0:1x2.5mV ; 1:3x2.5mV)
  // // Bit 0-4 : OC DSG LIM: (5bits:0xXX10 = 42.5mV)
  // mp27.writeAdd(0x24, 0x0010);  
  */

  //

  // --------------------------------------------------------
  // // 21. OK Discharge Over-Current 2 (DOC2)
  /*
  // Serial.println("TEST: Discharge Over-Current 2");
  // //OCFT_CTRL 0x23
  // // 0x01C7 def
  // // 0x1092 use
  // // Bit 13-12: Int type (*Rising)
  // // Bit 7: DSG 2 OC EN FAULT
  // // Bit 4: DSG 2 OC EN INT (needs OVER_CURR_INT_EN 0x19 b4)
  // // Bit 1: DSG 2 OC EN
  // mp27.writeAdd(0x23, 0x1092);

  // // EN xalert & Vscan
  // // INT1_EN (0x19)
  // // Bit 4: OC
  // mp27.writeAdd(0x19, 0x8810);  

  // // DSCOC_LIM (0x24))
  // // def 0x2910
  // // use 0x2900
  // // Bit 13   : OC DSG Range (0:1x2.5mV ; *1:3x2.5mV)
  // // Bit 12-8 : OC DSG LIM: (5bits:0x29XX = 75mV)
  // mp27.writeAdd(0x24, 0x2900);  
  */


  // --------------------------------------------------------
  // // 22. OK Discharge Short-Circuit (SC_dsg)
  /*
  Serial.println("TEST: Short-Circuit");

  // A. Enable Fault Protection
  // Reg: SCFT_CTRL (0x2A)
  // 0x0033 def
  // 0x0005 use
  // Bit 4 : SC_DSG_FAULT_EN
  // Bit 2*: SC_DSG_INT_EN
  // Bit 0*: SC_DSG_EN_CTRL
  mp27.writeAdd(0x2A, 0x0005);

  // B. Enable Interrupts
  // Reg: INT0_EN (0x19)
  // Bit *5: SC
  // Bit 4: OC
   mp27.writeAdd(0x19, 0x8820); 

  // C. Config
  // DSGSC_CFG (0x2B)
  // def 0x0111
  // def 0x0111
  // Bit 14-8: SC_DCHG_DG : (7bits:0x01XX = 0.3ms)
  // Bit 5   : SC_DCHG_RNG: *0 = x5.5mV ; 1 = x16.5mV
  // Bit 4-0 : SC_DCHG_LIM: (5bits:0x0011 = 99mV)
  mp27.writeAdd(0x2B, 0x0111);  

  // D. Disable OC faults
  // //OCFT_CTRL 0x23
  // // 0x01C7 def
  // // 0x0000 use
  mp27.writeAdd(0x23, 0x0000);
 */

  //

  // // --------------------------------------------------------
  // // 25. OK Charge Over-Current (COC)
  /*
  // Serial.println("TEST: Charge Over-Current (COC)");

  // // A.Enable CHG OC
  // // OCFT_CTRL 0x23
  // // 0x01C7 def
  // // 0x4024 use
  // // Bit 15-14: Int type (*Rising)
  // // Bit 8: CHG OC EN FAULT
  // // Bit 5*: CHG OC EN INT (needs OVER_CURR_INT_EN 0x19 b4)
  // // Bit 2*: CHG OC EN
  // mp27.writeAdd(0x23, 0x4024);

  // // B.Enable CHG OC INT
  // // INT1_EN 0x23
  // // EN xalert & Vscan
  // // Bit 4: OC
  // mp27.writeAdd(0x19, 0x8810);  

  // // C. Disable SC Fault Protection
  // // Reg: SCFT_CTRL (0x2A)
  // // 0x0033 def
  // // 0x0000 use
  // mp27.writeAdd(0x2A, 0x0000);

  // // C. Threshold config
  // // CHGOG_DEG (0x26)
  // // def 0x0410
  // // use 0x0410
  // // Bit   14: 0 (1x5ms)   ; 1 (8*5ms)
  // // Bit 13-8: 0b0001000 (4|20.1ms)
  // // Bit    5: 0 (1x1.6mV) ; 1 (3x1.6mV)
  // // Bit  4-0: (0x0010|16|27.2mV)
  // mp27.writeAdd(0x26, 0x0410); 
  */

  //

  // // --------------------------------------------------------
  // // 26. NOK Short Circuit Charge (SC_C)

  // Serial.println("TEST: Short Circuit Charge (SC_C)");


  // // A. Enable SC OC Protection
  // // Reg: SCFT_CTRL (0x2A)
  // // 0x0033 def
  // // 0x0028 use
  // // Bit 5: SC_CHG_FAULT_EN
  // // Bit *3: SC_CHG_INT_EN
  // // Bit *1: SC_CHG_EN_CTRL
  // mp27.writeAdd(0x2A, 0x000A); 

  // // B.Enable SC OC INT
  // // INT0_EN 0x19
  // // EN xalert & Vscan
  // // Bit 5: SC
  // mp27.writeAdd(0x19, 0x8820);  

  // // C.Disable OC
  // // OCFT_CTRL 0x23
  // // 0x01C7 def
  // // 0x000 use
  // mp27.writeAdd(0x23, 0x0000);

  // // D. Threshold config
  // // CHGSC_CFG (0x2C)
  // // def 0x0811 
  // // use 0x0811
  // // Bit 14-8: Deglitch     0b0001000 (8|1.7mV)
  // // Bit    5: Multiplier   *0 (1x2.5V) ; 1 (3x2.5mV)
  // // Bit  4-0: Threshold    0x0011 (17|45mV)
  // mp27.writeAdd(0x2C, 0x0811); 


  //2)fet control)
    //barebones dsg (ok)
  // mp27.writeAdd(0x13, 0x0102);  //0x10a all SS off -> 0x102 +sc pre turn on
  // mp27.writeAdd(0x1A, 0x2001);   //fet interrupt

    //dsg ss 0.1v/ms (ok)
  // mp27.writeAdd(0x13, 0x0103);  //0x10b SS -> 0x103 +sc pre turn on
  // mp27.writeAdd(0x1A, 0x2001);   //fet interrupt

  //   //dsg ss 1v/ms (ok_selected)
  // mp27.writeAdd(0x13, 0x0103);  //0x10b SS -> 0x103 +s/c pre turn on
  // mp27.writeAdd(0x14, 0x68f5);  //0x68f0 def -> 0x68f5 1v/ms
  // mp27.writeAdd(0x1A, 0x2001);   //fet interrupt

  // //   //dsg ss 1v/ms (nok)
  // mp27.writeAdd(0x13, 0x0103);  //0x10b SS -> 0x103 +s/c pre turn on
  // mp27.writeAdd(0x14, 0x69f5);  //0x68f0 def -> 0x68f5 1v/ms -> 0x69f5 rampup deglitch timer
  // // mp27.writeAdd(0x1A, 0x2001);   //fet interrupt

  //   //dsg ss 1v/ms (nok)
  // mp27.writeAdd(0x13, 0x010b);  //0x10b SS -> 0x103 +s/c pre turn on
  // mp27.writeAdd(0x14, 0x69f5);  //0x68f0 def -> 0x68f5 1v/ms
  // mp27.writeAdd(0x1A, 0x2001);   //fet interrupt

}

// void ISR_nfc();                        //Interrupt 2 ()

// void ISR_nfc(){
//   static unsigned long IRQtimeLast_2 = 0;
//   unsigned long IRQtime_2 = millis();

//   // Interrupt debounce
//   if ((IRQtime_2-IRQtimeLast_2) > IRQ_DEBOUNCE) {
//     IRQFlag_2 = true;
//     // FOR DEBUG
//     Serial.println("Interrupt detected");
//   }
//   // Update the time
//   IRQtimeLast_2 = IRQtime_2;
// }


// typedef struct i2cdevice{
//   uint8_t _devaddr;
//   uint8_t _status;
//   void* _wireObj;
// };
// i2cdevice fGauge    = {0x15, 0x00, &_wireObj};
// i2cdevice bMonitor  = {0x15, 0x00, _wireObj};
// i2cdevice lcdScreen = {0x15, 0x00, _wireObj};

// typedef struct lipobat{
//   uint_fast64_t _chgCount;
//   uint8_t _id;
//   uint8_t _lastV;
// };
// lipobat currentBat;
// lipobat readBat;

// void loadBatInfo(){

//   lipobat readBat;

//   //Startup sequence
//   if(startupFlag){
//     uint64_t data = sdread(Bat);
    
//     //Loads data into variable
//     currentBat ={
//       data._chgCount,
//       data._id,
//       data._lastV,
//     };
//     //Loads data into fuel gauge
//     return I2Cdev::writeWord(
//       fGauge._devaddr,
//       qAddress,
//       currentBat._chgCount,
//       fGauge._wireObj
//     );
//   }

//   //Checks if id present
//   int idNFCList;
//   for( int i=0 ; i<idNFCList.length ; i++)
//   {
//    if (idNFC = idNFCList.value){

//     //Loads data into variable
//     readBat = {
//       data._chgCount,
//       data._id,
//       data._lastV,
//     };

//     //Checks if V within range
//     int dV = data._lastV - adcdata._vTop;   
//     if((-500 < dV) && (dv < 500)){
//       if(I2Cdev::writeWord(
//           fGauge._devaddr,
//           qAddress,
//           currentBat._chgCount,
//           fGauge._wireObj)
//            < 0){
//           return 1;
//         }
//     }

//       //Loads data into fuel gauge
//       return I2Cdev::writeWord(
//         fGauge._devaddr,
//         qAddress,
//         currentBat._chgCount,
//         fGauge._wireObj
//       );
//    }
//   }
//     if(Serial) {
//       Serial.write("New battery Id created:")
//       Serial.println(batId)
//     }
// }


// uint16_t getBatInfo(){
//   uint8_t qAddress = 0x12;
//   uint16_t data = 0;

//   if(I2Cdev::readWord(fGauge._devaddr, qAddress, &data, I2Cdev::readTimeout, fGauge._wireObj) < 0) {
//     if (Serial) {
//         Serial.print("Failed to read register 0x");
//         Serial.print(qAddress, HEX);
//         Serial.println(". Returning 0.");
//     }
//     return 0; // Return 0 on failure
//   }

//   return 1;
// }