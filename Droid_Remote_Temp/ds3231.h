#pragma onec
#include "board_def.h"
#ifdef ENABLE_DS3231
String ds3231_test();
#else
#define ds3231_test() ""
#endif



  // LoRa.write(droidGatewayStatus);
  // LoRa.write(RELAY_STATUS);
  // LoRa.write(bodyControllerStatus);
  // LoRa.write(bodyLEDControllerStatus);
  // LoRa.write(bodyServoStatus);
  // LoRa.write(domePlateControllerStatus);
  // LoRa.write(domeControllerStatus);
  // LoRa.write(BL_LDP_Bright);
  // LoRa.write(BL_MAINT_Bright);
  // LoRa.write(BL_VU_Bright);
  // LoRa.write(BL_CS_Bright);
  // LoRa.write(BL_vuOffsetInt);
  // LoRa.write(BL_vuBaselineInt);
  // LoRa.write(BL_vuOffsetExt);
  // LoRa.write(BL_vuBaselineExt);
  // LoRa.write(BL_BatteryVoltage);
  // LoRa.write(BL_BatteryPercentage);