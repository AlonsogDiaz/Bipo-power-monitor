#include "MP2790.h"
#include "mp2790_setup.h"

void setupValues(){
  // // Importing data
  Serial.println("Loading configuration values...");
  //Uncomented registers come from the MP2790 programming tool
  mp27.writeAdd(0x00 , 0x00C5 );  // CELLS_CTRL C3: 4cells ; c5: 6cells
  mp27.writeAdd(0x05 , 0x0002 );   // ACT_CFG 0x0000: all off 0x000A: all on
  mp27.writeAdd(0x06 , 0x0020 );   // STB_CFG
  mp27.writeAdd(0x07 , 0x0007 );   // SAFE_CFG
  mp27.writeAdd(0x08 , 0x0005 );   // RGL_CFG
  mp27.writeAdd(0x09 , 0x0500 );   // LOAD_CHARGER_CFG
  mp27.writeAdd(0x0B , 0x0000 );   // GPIO_OUT
  mp27.writeAdd(0x0C , 0x0000 );   // GPIO_CFG
  mp27.writeAdd(0x0D , 0x0061 );   // PINS_CFG
  mp27.writeAdd(0x10 , 0xEF3C );   // WDT_CFG 0x4E9D: WDT on 0x4E9C: WDT off
  mp27.writeAdd(0x12 , 0x0000 );   // FET_CTRL
  mp27.writeAdd(0x13 , 0x051B );   // FET_MODE //turnon timeount ? 0x071b : 0x051b 0x68F0 //0x070A: SS DSG and CHG disabled
  mp27.writeAdd(0x14 , 0x68F5 );   // FET_CFG 0x78F0: 12VCP; 0x68F0: 10Vcp 0x38F0: 7Vcp 0x08F0: 5Vcp-low 
  mp27.writeAdd(0x19 , 0x8E7B );   // INTO_EN
  mp27.writeAdd(0x1A , 0x058E );   // INT1_EN 0x2FFE: normal
  mp27.writeAdd(0x1B , 0x0034 );   // INT_TYPE0
  mp27.writeAdd(0x1C , 0x1500 );   // INT_TYPE1
  mp27.writeAdd(0x1D , 0x04D0 );   // INT_TYPE2
  mp27.writeAdd(0x1E , 0x0000 );   // MASK_INTO
  mp27.writeAdd(0x1F , 0x0000 );   // MASK_INT1
  mp27.writeAdd(0x23 , 0x71BF );   // OCFT_CTRL
  mp27.writeAdd(0x24 , 0x260B );   // DSGOC_LIM
  mp27.writeAdd(0x25 , 0x047F );   // DSGOC_DEG
  mp27.writeAdd(0x26 , 0x0403 );   // CHGOC_DEG
  mp27.writeAdd(0x2A , 0x0015 );   // SCFT_CTRL
  mp27.writeAdd(0x2B , 0x190A );   // DSGSC_CFG
  mp27.writeAdd(0x2C , 0x0811 );   // CHGSC_CFG
  mp27.writeAdd(0x34 , 0x0032 );   // PACKFT_CTRL 
  mp27.writeAdd(0x35 , 0x62B6 );   // CELLFT_CTRL
  mp27.writeAdd(0x36 , 0x0AA0 );   // CELL_HST
  mp27.writeAdd(0x37 , 0x8080 );   // PACK_UV_OV
  mp27.writeAdd(0x38 , 0x00A9 );   // CELL_UV
  mp27.writeAdd(0x39 , 0x00D7 );   // CELL_OV
  mp27.writeAdd(0x3A , 0x03F6 );   // PACK_UV
  mp27.writeAdd(0x3B , 0x050A );   // PACK_OV
  mp27.writeAdd(0x3C , 0x0068 );   // CELL_DEAD_THR
  mp27.writeAdd(0x3D , 0x0002 );   // CELL_MSMT
  mp27.writeAdd(0x44 , 0x0004 );   // NTC_CLR
  mp27.writeAdd(0x46 , 0x000A );   // DIE_CFG
  mp27.writeAdd(0x47 , 0xC4B5 );   // NTC_CFG 0xE4F5:NTC1-4 0xE4B1: NTC1
  mp27.writeAdd(0x48 , 0x0124 );   // NTCC_OTHR_DSG
  mp27.writeAdd(0x49 , 0x034F );   // NTCC_UTHR_DSG
  mp27.writeAdd(0x4A , 0x0124 );   // NTCC_OTHR_CHG
  mp27.writeAdd(0x4B , 0x8B4F );   // NTCC_UTHR_CHG
  mp27.writeAdd(0x4C , 0x809D );   // NTCM_OTHR
  mp27.writeAdd(0x4D , 0xAACC );   // DIE_OT
  mp27.writeAdd(0x4E , 0x0000 );   // SELF_STS
  mp27.writeAdd(0x55 , 0x0501 );   // SFT_GO
  mp27.writeAdd(0x56 , 0x014F );   // SELF_CFG 
  mp27.writeAdd(0x57 , 0x0407 );
  mp27.writeAdd(0x58 , 0x016D );   // REGIN_UV
  mp27.writeAdd(0x59 , 0x00F0 );   // V3P3_UV
  mp27.writeAdd(0x5A , 0x0084 );   // VDD_UV
  mp27.writeAdd(0x5B , 0x6555 );   // SELF_THR
  mp27.writeAdd(0x60 , 0x0000 );   // FT_REC 0x1016 auto recovery
  mp27.writeAdd(0x61 , 0x3DD3 );   // FT0_CFG
  mp27.writeAdd(0x62 , 0x1000 );   // FT1_CFG
  mp27.writeAdd(0x63 , 0x0000 );
  mp27.writeAdd(0x99 , 0x0000 );   // ADC_CTRL
  mp27.writeAdd(0x9A , 0x3F00 );   // CC_CFG
  mp27.writeAdd(0x9B , 0x4000 );   // TRIMG_IPCB
  mp27.writeAdd(0x9C , 0x0377 );   // HR_SCANO
  mp27.writeAdd(0x9D , 0x0033 );   // HR_SCAN1
  mp27.writeAdd(0x9E , 0x00E0 );   // HR_SCAN2
  mp27.writeAdd(0xA0 , 0x0000 );   // SILC_INFO1
  mp27.writeAdd(0xA1 , 0x0043 );
  mp27.writeAdd(0xA2 , 0x0000 );
  mp27.writeAdd(0xA3 , 0x0100 );   // COMM_CFG
  mp27.writeAdd(0xA5 , 0x0000 );   // BAL_LIST
  mp27.writeAdd(0xA6 , 0x0000 );   // BAL_CTRL
  mp27.writeAdd(0xA7 , 0x00F8 );   // BAL_CFG
  mp27.writeAdd(0xA8 , 0x0821 );   // BAL_THR
  mp27.writeAdd(0xAA , 0x0081 );
  mp27.writeAdd(0xAB , 0x9100 );
  mp27.writeAdd(0xAC , 0x2000 );
  mp27.writeAdd(0xAD , 0x0020 );
  mp27.writeAdd(0xAE , 0x01F0 );
  mp27.writeAdd(0xAF , 0x0300 );
  mp27.writeAdd(0xB0 , 0x0000 );
  mp27.writeAdd(0xC2 , 0x0000 );
  mp27.writeAdd(0xC3 , 0x46FD );
  mp27.writeAdd(0xC4 , 0x4D95 );
  mp27.writeAdd(0xC5 , 0x542D );
  mp27.writeAdd(0xC6 , 0x0698 );
  mp27.writeAdd(0xC7 , 0xD698 );
  mp27.writeAdd(0xE4 , 0x0000 );
  mp27.writeAdd(0xE6 , 0x0080 );
  mp27.writeAdd(0xE7 , 0x0001 );
  mp27.writeAdd(0xE8 , 0x8500 );
  mp27.writeAdd(0xE9 , 0x0008 );
  mp27.writeAdd(0xEB , 0x0700 );
  mp27.writeAdd(0xEC , 0xE010 );
  mp27.writeAdd(0xED , 0x10F5 );
  return;

}