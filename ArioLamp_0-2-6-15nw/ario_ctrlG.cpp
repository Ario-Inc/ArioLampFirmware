/************************************************************************************************************************************/
/** @file       ario_ctrlG.cpp
 *  @brief      core lamp control logic, lamp programs, scheduler
 *
 *  @author     Shaw-Pin Chen, Product Lead, Ario, Inc.
 *  @created    09-16-16
 *  @last rev   04-18-17
 *
 *
 *  @section    Legal Disclaimer
 *          All contents of this source file and/or any other Ario, Inc. related source files are the explicit property of
 *          Ario, Inc. Do not distribute. Do not copy.
 */
/************************************************************************************************************************************/

#include "globals.h"
#include "ario_ctrlG.h"
#include "application.h"

float LED_COLOR_Cramer[NUM_LED_CH]; // [0]: 6500K, [1]: 4000K, [2]: 1800K top, [3]: 1800K bottom

// converted color mixing density for each PSoC channel accounting for Brightness Level
byte LED_CH_Dens[NUM_LED_CH]; // to be loaded to PSoC, can be simplified here

unsigned int currentSecond, lastSecond, currentDay, lastDay, programEndTime;

/////////////////////////// I2C Slave comm  ////////////////////////////
byte i2cSendBuffer[NUM_BYTES_WRITE];     /* array to hold i2c data bytes (data to send) */
byte i2cRecvBuffer[NUM_BYTES_READ];      /* array to hold i2c data bytes (data read back) */


/////////////////////////////// Time of Day Look Up Tables ////////////////////////////////

uint16_t def_cctArry[]   = { 300, 300, 300, 300, 300,1800,  //  0 -  5:59 AM
                            3000,5000,6500,6500,6500,6500,  //  6 - 11:59 AM
                            6500,6500,6500,6500,5000,4000,  // 12 - 17:59 PM
                            3000,2500,1800,1500, 300, 300 };// 18 - 23:59 PM

uint16_t def_levelArry[] = {  10,  10,  10,  10,  10,  40,  //  0 -  5:59 AM
                              60, 150, 200, 255, 255, 255,  //  6 - 11:59 AM
                             255, 255, 255, 255, 200, 170,  // 12 - 17:59 PM
                             130,  80,  80,  60,  30,  10 };// 18 - 23:59 PM

uint16_t loaded_cctArry[24];
uint16_t loaded_levelArry[24]; // had to be 16-bit int to reuse LUT_ValExtractor function

ArioCtrl::ArioCtrl(){
    lightIsOn           = FALSE;
    nwMode              = NW_MODE_DEFAULT;
    pirEnabled          = FALSE;
    pirDebounceTimer    = millis();
    pirDebounceFlag     = FALSE;
    pirHoldTimeMarker   = millis();
    pirOffTimer         = millis();
    pirReportTimer      = 0;
    alsMeasureFlag      = TRUE;
    alsMeasureTimer     = millis();
    alsMeasureCount     = 0;
    alsRunningSum       = 0;
    alsMeasuredLevel    = -1;
    alsBackgroundLevel  = -1;
    alsAdjustedLevel    = -1;
    alsReportTimer      = millis();
    operatingMode       = MODE_DEFAULT;
    marker              = millis();
    programCounter      = 0;
    maxCCT              = 6500;
    currentVersion      = 0;
    currentCCT          = 0;
    currentLevel        = 0;
    dawnSimDuration     = 0UL;
    bedTimeDuration     = 0UL;
    AMalarmFlag         = TRUE;
    PMalarmFlag         = TRUE;
    amAlarmNow          = FALSE;
    pmAlarmNow          = FALSE;
    rampRegCCTStep      = 0;
    rampRegLevelStep    = 0;
    rampRegCounter      = 0;
    rampRegEndCounter   = 0;
    rampRegNextMode     = MODE_DEFAULT;
    cloudReportFlag     = FALSE;
}
//<<destructor>>
ArioCtrl::~ArioCtrl(){/*nothing to destruct*/}


void ArioCtrl::Ario_Init(void){
    EzI2Cs_Init();
    Set_TimeZone();
    Load_RTC_Schedule();
    Load_Max_CCT();
    Load_Current_Version();
    UART_Init();
    PSoC_Init();
}


void ArioCtrl::Load_RTC_Schedule(void){
    unsigned int select = EEPROM.read(SCHEDULE_SELECT_ADDR);
    if(1 == select){
        for(int i = 0; i < 24; i++){
            uint16_t cctVal;
            EEPROM.get((SCHEDULE_1_CCT_BASE_ADDR + i*2), cctVal);
            loaded_cctArry[i] = cctVal;
            loaded_levelArry[i] = EEPROM.read(SCHEDULE_1_LEVEL_BASE_ADDR + i);
        }
    } else if(2 == select){
        for(int i = 0; i < 24; i++){
            uint16_t cctVal;
            EEPROM.get((SCHEDULE_2_CCT_BASE_ADDR + i*2), cctVal);
            loaded_cctArry[i] = cctVal;
            loaded_levelArry[i] = EEPROM.read(SCHEDULE_2_LEVEL_BASE_ADDR + i);
        }
    } else{
        memcpy(loaded_cctArry, def_cctArry, sizeof loaded_cctArry);
        memcpy(loaded_levelArry, def_levelArry, sizeof loaded_levelArry);
    }
}


void ArioCtrl::Load_Max_CCT(void){
    uint16_t content;
    EEPROM.get(MAX_CCT_ADDR, content);
    maxCCT = constrain(content, MAX_CCT_LOWER_LIMIT, MAX_CCT_UPPER_LIMIT);
}

void ArioCtrl::Load_Current_Version(void){
    currentVersion = EEPROM.read(CURRENT_VERSION_ADDR);
}

void ArioCtrl::PSoC_Init(void){
    Load_RTC_Val();
    EzI2Cs_Read(PSOC_ADDR, 0, i2cRecvBuffer, NUM_BYTES_READ);
    lightIsOn = i2cRecvBuffer[0];
}


void ArioCtrl::Set_TimeZone(void){
    unsigned int offset = 0;
    //if((EEPROM.read(DST_AUTO_CALC_ADDR) == TRUE) && IsDST(Time.day(), Time.month(), Time.weekday())){ // if DST enabled
    if(EEPROM.read(DST_ENABLE_ADDR) == 1){ // if DST enabled
        offset = 1;
    }
    // convert time zone here
    //  to calculate zone, (x+12)*4
    //  to decode zone, (y-48)/4
    float zone = EEPROM.read(USER_TIME_ZONE);
    if((zone == 0xFF) || (zone > 104)){ // If user has not set a time zone
        zone = -8;
    } else{
        if(zone >= 100){ // very special time zones UTC +13 and +14
            zone = zone/4 - 36;
        } else{ // regular time zones
            zone = (zone - 48)/4;
        }
    }
    Time.zone(zone + offset);
}


/*
bool ArioCtrl::IsDST(int dayOfMonth, int month, int dayOfWeek){
  if(month < 3 || month > 11){
    return false;
  }
  if(month > 3 && month < 11){
    return true;
  }
  int previousSunday = dayOfMonth - (dayOfWeek - 1); // Spark Sunday = 1
  //In march, we are DST if our previous sunday was on or after the 8th.
  if(month == 3){
    return previousSunday >= 8;
  }
  //In November we must be before the first sunday to be dst.
  //That means the previous sunday must be before the 1st.
  return previousSunday <= 0;
}
*/


void ArioCtrl::Turn_Lamp_On(byte interactionType){
    String reportStr = "";
    pirHoldTimeMarker = millis(); // resets this timer so the light won't automatically turn off when user turns it on with app
    PSoC_Load_LEDVal(currentCCT, 0);
    PSoC_onOff(0x01);
    lightIsOn = TRUE;
    if(INTERACTION_TYPE_BTN == interactionType){
        reportStr = "true,btn";
    } else if(INTERACTION_TYPE_WEB == interactionType){
        reportStr = "true,web";
    } else if(INTERACTION_TYPE_PIR == interactionType){
        reportStr = "true,pir";
    }
    Report_to_Cloud("power", reportStr);
    if((EEPROM.read(ALS_EN_ADDR) == TRUE) && (alsAdjustedLevel != -1)){
        RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), alsAdjustedLevel, 500UL, MODE_DEFAULT);
    } else {
        RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), ValExtractor_LUT24(loaded_levelArry), 500UL, MODE_DEFAULT);
    }
}

void ArioCtrl::Turn_Lamp_Off(byte interactionType){
    String reportStr = "";
    pirHoldTimeMarker = millis(); // resets this timer so the light won't automatically turn off when user turns it on with app
    //RampTo_Linear_Setup(ValExtractor_LUT24(def_cctArry), 1, 500UL, MODE_DEFAULT); //cool feature but not sure how
    PSoC_onOff(0x00);
    lightIsOn = FALSE;
    if(INTERACTION_TYPE_BTN == interactionType){
        pirOffTimer = millis(); // When user turns off lamp, there is enough time to leave the room before pir turn lights on ~ 1minute
        reportStr = "false,btn";
    } else if(INTERACTION_TYPE_WEB == interactionType){
        pirOffTimer = millis(); // When user turns off lamp, there is enough time to leave the room before pir turn lights on ~ 1minute
        reportStr = "false,web";
    } else if(INTERACTION_TYPE_PIR == interactionType){
        reportStr = "false,pir";
    }
    Report_to_Cloud("power", reportStr);
}




void ArioCtrl::Light_Switch(void){
    if(lightIsOn){
        Turn_Lamp_Off(INTERACTION_TYPE_BTN);
        debugPrint("lights off");
    } else {
        Turn_Lamp_On(INTERACTION_TYPE_BTN);
        debugPrint("lights on");
    }
}

////////////////////// buttons ////////////////////////
void ArioCtrl::TopButton_Action(void){ // increase level
    if(!lightIsOn){
        Light_Switch();
        delay(500); // this is a porential bug that needs to change
    } else{
        Increase_Level();
    }
    pirHoldTimeMarker = millis();
}

void ArioCtrl::MidButton_Action(void){ // decrease level
    if(!lightIsOn){
        Light_Switch();
        delay(500); // potential bug here
    } else {
        Decrease_Level();
    }
    pirHoldTimeMarker = millis();
}


void ArioCtrl::decode_cmd(int cmd){
    debugPrint("command receiveded!");
    pirHoldTimeMarker = millis(); //////////////////////////////////////////////////////////////////////////////////////////////
    switch(cmd){
        case TURN_OFF:
            Light_Switch();
            break;
        case TURN_ON:
            Light_Switch();
            break;
        case DEMO_ON:
            Demo_Init(); // only works if light is turned on
            break;
        case APP_BRIGHTNESS_UP:
            if(lightIsOn) Increase_Brightness_App();
            break;
        case APP_BRIGHTNESS_DOWN:
            if(lightIsOn) Decrease_Brightness_App();
            break;
        case APP_CCT_UP:
            if(lightIsOn) Increase_CCT_App();
            break;
        case APP_CCT_DOWN:
            if(lightIsOn) Decrease_CCT_App();
            break;
        default:
                Cloud_Debug_Print("Unknown command received :(");
            break;
    }
}

/*************************************************************************************************************
/
/               Adjust Mode Code
/
*************************************************************************************************************/
void ArioCtrl::Increase_Level(void){
    if(MODE_ADJUST != operatingMode){
        operatingMode = MODE_ADJUST;
        marker = millis();
    } // stops other non-default mode and grabs the color/brightness setting
    if(nwMode == NW_MODE_SET_CCT){ // change CCT
        if((millis() - marker) >= 15UL){
            if(maxCCT > currentCCT){
                currentCCT += 20;
                PSoC_Load_LEDVal(currentCCT, currentLevel);
                operatingMode = MODE_ADJUST;
                cloudReportFlag = TRUE;
                marker = millis();
            }
        }
    } else{ // change brightness (default mode)
        if((currentCCT < CCT_1800) && (currentLevel > 254)){
            if((millis() - marker) >= 15UL){
                if(CCT_1800 > currentCCT){
                    currentCCT += 20;
                    PSoC_Load_LEDVal(currentCCT, currentLevel);
                    operatingMode = MODE_ADJUST;
                    cloudReportFlag = TRUE;
                    marker = millis();
                }
            }
        } else{
            if((millis() - marker) >= ((100 > currentLevel) ? 10UL : 5UL)){
                if(MAX_BRIGHTNESS > currentLevel){
                    PSoC_Load_LEDVal(currentCCT, ++currentLevel);
                    operatingMode = MODE_ADJUST;
                    cloudReportFlag = TRUE;
                    marker = millis();
                }
            }
        }
    }
}

void ArioCtrl::Decrease_Level(void){
    if(MODE_ADJUST != operatingMode){
        operatingMode = MODE_ADJUST;
        //marker = millis();
    } // stops other non-default mode and grabs the color/brightness setting
    if(nwMode == NW_MODE_SET_CCT){ // change CCT
        if((millis() - marker) >= 15UL){
            if(MIN_CCT < currentCCT){
                currentCCT -= 20;
                PSoC_Load_LEDVal(currentCCT, currentLevel); // there is a bug here in which the decreased CCT could be 1699 instead of 1700
                operatingMode = MODE_ADJUST;
                cloudReportFlag = TRUE;
                marker = millis();
            }
        }
    } else{ // change brightness (default mode)
        if((currentCCT <= (CCT_1800 + 1)) && (MIN_CCT < currentCCT)){
            if((millis() - marker) >= 15UL){
                if(MIN_CCT < currentCCT){
                    currentCCT -= 20;
                    PSoC_Load_LEDVal(currentCCT, currentLevel);
                    operatingMode = MODE_ADJUST;
                    cloudReportFlag = TRUE;
                    marker = millis();
                }
            }
        } else{
            if((millis() - marker) >= ((100 > currentLevel) ? 5UL : 10UL)){
                if((MIN_BRIGHTNESS + 3) < currentLevel){
                    PSoC_Load_LEDVal(currentCCT, --currentLevel);
                    operatingMode = MODE_ADJUST;
                    cloudReportFlag = TRUE;
                    marker = millis();
                }
            }
        }

    }
}

void ArioCtrl::Increase_Brightness_App(void){
    if(currentLevel < MAX_BRIGHTNESS){ // this might be redundant
        float destLevel;
        if(currentLevel > 200){
            destLevel = MAX_BRIGHTNESS;
        } else if(currentLevel > 150){
            destLevel = currentLevel + 70;
        } else if(currentLevel > 80){
            destLevel = currentLevel + 50;
        } else if(currentLevel > 30){
            destLevel = currentLevel + 30;
        } else{
            destLevel = currentLevel + 10;
        }
        marker = millis();
        //cloudReportFlag = TRUE;//////////////////////////////////////////
        RampTo_Linear_Setup(currentCCT, destLevel, 500UL, MODE_ADJUST);
    }
}

void ArioCtrl::Decrease_Brightness_App(void){
    if(currentLevel > (MIN_BRIGHTNESS + 2)){ // this might be redundant
        float destLevel;
        if(currentLevel < 20){
            destLevel = (MIN_BRIGHTNESS + 2);
        } else if(currentLevel < 50){
            destLevel = currentLevel - 15;
        } else if(currentLevel < 80){
            destLevel = currentLevel - 30;
        } else if(currentLevel < 170){
            destLevel = currentLevel - 50;
        } else{
            destLevel = currentLevel - 70;
        }
        marker = millis();
        //cloudReportFlag = TRUE;/////////////////////////////////////////////
        RampTo_Linear_Setup(currentCCT, destLevel, 500UL, MODE_ADJUST);
    }
}

void ArioCtrl::Increase_CCT_App(void){
    if(currentCCT < maxCCT){ // this might be redundant
        float destCCT;
        if(currentCCT > (maxCCT - 1500)){
            destCCT = maxCCT;
        } else{
            destCCT = currentCCT + 1000;
        }
        marker = millis();
        //cloudReportFlag = TRUE;/////////////////////////////////////////////
        RampTo_Linear_Setup(destCCT, currentLevel, 500UL, MODE_ADJUST);
    }
}

void ArioCtrl::Decrease_CCT_App(void){
    if(currentCCT > MIN_CCT){ // this might be redundant
        float destCCT;
        if(currentCCT < 500){
            destCCT = MIN_CCT;
        } else if(currentCCT < CCT_1800){
            destCCT = currentCCT - 750;
        } else if(currentCCT < 2100){
            destCCT = currentCCT - 750;
        } else if(currentCCT < 3000){
            destCCT = currentCCT - 750;
        } else if(currentCCT < 5000){
            destCCT = currentCCT - 700;
        } else{
            destCCT = currentCCT - 700;
        }
        marker = millis();
        //cloudReportFlag = TRUE;/////////////////////////////////////////////
        RampTo_Linear_Setup(destCCT, currentLevel, 500UL, MODE_ADJUST);
    }
}

void ArioCtrl::Set_Brightness(unsigned int brightness){
    if(MAX_BRIGHTNESS < brightness){
        brightness = 255;
    } else if((MIN_BRIGHTNESS + 2) > brightness){
        brightness = 2;
    }
    marker = millis();
    RampTo_Linear_Setup(currentCCT, brightness, 500UL, MODE_ADJUST);
}

void ArioCtrl::Set_CCT(unsigned int cct){
    cct = constrain(cct, MIN_CCT, maxCCT);
    unsigned long delayVal = 500UL;
    int cctDiff = currentCCT - cct;
    if(abs(cctDiff) > 2000){ delayVal = 1000UL; }
    if(abs(cctDiff) > 3000){ delayVal = 1500UL; }
    marker = millis();
    RampTo_Linear_Setup(cct, currentLevel, delayVal, MODE_ADJUST);
}


/*************************************************************************************************************
/
/               Different Lamp Programs Code
/
*************************************************************************************************************/
void ArioCtrl::RampTo_Linear_Setup(float destCCT, float destLevel, unsigned long duration, int nextMode){
    float stepSize = duration/RAMP_DELAY;
    if(!((CCT_6500 < destCCT)||(MIN_CCT > destCCT))){ // if value not out of bound
        rampRegCCTStep = (destCCT-currentCCT)/stepSize;
    } else{
        rampRegCCTStep = 0;
    }
    if(!((MAX_BRIGHTNESS < destLevel)||(MIN_BRIGHTNESS > destLevel))){ // if value not out of bound
        rampRegLevelStep = (destLevel-currentLevel)/stepSize;
    } else{
        rampRegLevelStep = 0;
    }
    rampRegEndCounter = (int)stepSize;
    if(-1 != nextMode){ // used specifically for special program
        rampRegNextMode = nextMode;
        operatingMode = MODE_RAMP;
    }
    rampRegCounter = 0;
}

bool ArioCtrl::RampTo_Linear_Playing(void){
    if(rampRegEndCounter == rampRegCounter){
        return FALSE;
    } else{
        if(millis() - marker >= RAMP_DELAY){
            PSoC_Load_LEDVal(currentCCT+=rampRegCCTStep, currentLevel+=rampRegLevelStep);
            rampRegCounter ++;
            marker = millis();
        }
        return TRUE;
    }
}

void ArioCtrl::Demo_Init(void){
    operatingMode = MODE_DEMO;
    programCounter = 0;
    PSoC_Load_LEDVal(MIN_CCT, 0);
}

bool ArioCtrl::Demo_Playing(void){
    if(0 == programCounter){
        RampTo_Linear_Setup((MIN_CCT+1), 0, 500UL, -1); programCounter++;
    } else if(1 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, 100, 1500UL, -1); programCounter++; }
    } else if(2 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(CCT_1800, 100, 2000UL, -1); programCounter++; }
    } else if(3 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(CCT_6500, MAX_BRIGHTNESS, 7000UL, -1); programCounter++; } // For Demo don't use maxCCT
    } else if(4 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(CCT_1800, MAX_BRIGHTNESS, 9000UL, -1); programCounter++; }
    } else if(5 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, MAX_BRIGHTNESS, 4000UL, -1); programCounter++; }
    } else if(6 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, MIN_BRIGHTNESS, 4000UL, -1); programCounter++; }
    } else if(7 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup((MIN_CCT+1), MIN_BRIGHTNESS, 2000UL, -1); programCounter++; }
    } else if(8 == programCounter){
        if(!RampTo_Linear_Playing()) programCounter++;
    }
    if(8 < programCounter){
        programCounter = 0;
        return FALSE;
    } else{
        return TRUE;
    }
}

void ArioCtrl::DawnSim_Init(void){
    operatingMode = MODE_DAWNSIM;
    dawnSimDuration = EEPROM.read(WAKEUP_ALARM_DURATION_BASE_ADDR + Time.weekday())*ONE_MINUTE;
    PSoC_Load_LEDVal(MIN_CCT, 1);
    programCounter = 0;
    PSoC_onOff(0x01);
    lightIsOn = TRUE;
    Cloud_Debug_Print("Wake Up Alarm begins.");
    Report_to_Cloud("power", "true,alarm");
}

bool ArioCtrl::DawnSim_Playing(void){
    if(0 == programCounter){
        RampTo_Linear_Setup(0, MAX_BRIGHTNESS*0.375, dawnSimDuration*0.375, -1); programCounter++;
    } else if(1 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(CCT_1800, MAX_BRIGHTNESS*0.5, dawnSimDuration*0.125, -1); programCounter++; }
    } else if(2 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup((maxCCT - 1), MAX_BRIGHTNESS, dawnSimDuration*0.5, -1); programCounter++; }
    } else if(3 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(maxCCT, MAX_BRIGHTNESS, HALF_HOUR, -1); programCounter++; } // One hour of max cct persistence
    } else if(4 == programCounter){
        if(!RampTo_Linear_Playing()) programCounter++;
    }
    if(4 < programCounter){
        pirHoldTimeMarker = millis(); // so the lamp does not turn off abruptly right after the program ends
        programCounter = 0;
        return FALSE;
    } else{
        return TRUE;
    }
}

void ArioCtrl::BedTime_Init(void){
    operatingMode = MODE_BEDTIME;
    bedTimeDuration = EEPROM.read(BEDTIME_ALARM_DURATION_BASE_ADDR + Time.weekday())*60;
    programCounter = 0;
    Cloud_Debug_Print("Bedtime Reminder begins.");
}

bool ArioCtrl::BedTime_Playing(void){
    if(0 == programCounter){
        RampTo_Linear_Setup(MIN_CCT, MAX_BRIGHTNESS/2, 2000UL, -1); programCounter++; /*Serial.println("Bedtime alarm begins");*/
    } else if(1 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, 5, 1000UL, -1); programCounter++; }
    } else if(2 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, MAX_BRIGHTNESS/2, 1000UL, -1); programCounter++; }
    } else if(3 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, 5, 1000UL, -1); programCounter++; }
    } else if(4 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, MAX_BRIGHTNESS/2, 1000UL, -1); programCounter++; }
    } else if(5 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, 5, 1000UL, -1); programCounter++; }
    } else if(6 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, MAX_BRIGHTNESS/2, 1000UL, -1); programCounter++; }
    } else if(7 == programCounter){
        if(!RampTo_Linear_Playing()){ RampTo_Linear_Setup(MIN_CCT, MAX_BRIGHTNESS/10, 2000UL, -1); programCounter++; /*Serial.println("Bedtime Phase 2"); Serial.println(Time.timeStr());*/ }
    } else if(8 == programCounter){
        if(!RampTo_Linear_Playing()){ programEndTime = Time.now() + bedTimeDuration; programCounter++; /*Serial.println("Bedtime Phase 3"); Serial.println(Time.timeStr());*/ }
    } else if(9 == programCounter){
        if(Time.now() >= programEndTime){ RampTo_Linear_Setup(MIN_CCT, MIN_BRIGHTNESS, 2000UL, -1); programCounter++; /*Serial.println("Bedtime Phase 4"); Serial.println(Time.timeStr());*/ } //60000UL one minute
    } else if(10 == programCounter){
        if(!RampTo_Linear_Playing()) programCounter++;
    }
    if(10 < programCounter){
        //Turn_Lamp_Off();
        PSoC_onOff(0x00);
        lightIsOn = FALSE;
        pirOffTimer = millis(); // When user turns off lamp, there is enough time to leave the room before pir turn lights on ~ 1minute
        programCounter = 0;
        Report_to_Cloud("power", "false,bed");
        return FALSE;
    } else{
        return TRUE;
    }
}

/*************************************************************************************************************
/
/               Scheduler
/
*************************************************************************************************************/
void ArioCtrl::Scheduler(void){
    if(!lightIsOn){
        operatingMode = MODE_DEFAULT; programCounter = 0; // this might be important for AM Alarm, might not
    }

    Check_Wake_Alarm();

    Check_Bedtime_Reminder();

    Check_PIR_Schedule();

    //Check_WiFi_Schedule();

    if(MODE_RAMP == operatingMode){
        if(!RampTo_Linear_Playing()){
            operatingMode = rampRegNextMode;
        }
    }else if(MODE_ADJUST == operatingMode){
        programCounter = 0;
        // Reports ONLY physical lamp adjustment to cloud (cloud initiated adjustments do not count)
        if((millis()-marker >= CLOUD_REPORT_DELAY) && cloudReportFlag){
            cloudReportFlag = FALSE;
            char publishString1[20];
            char publishString2[20];
            sprintf(publishString1,"%d,btn", (int)currentLevel);
            sprintf(publishString2,"%d,btn", (int)currentCCT);
            Report_to_Cloud("brightness", publishString1);
            Report_to_Cloud("color", publishString2);
        }
        // if user adjusted Max CCT, this needs to change if current CCT > Max CCT
        if(currentCCT > maxCCT){ PSoC_Load_LEDVal(maxCCT, currentLevel); }
        unsigned int holdTime = EEPROM.read(HOLD_TIME_DURTION_ADDR);
        if(holdTime == 0xFF){ holdTime = FACTORY_HOLD_TIME; }
        if(millis()-marker >= (ONE_MINUTE*holdTime)){
            if((EEPROM.read(ALS_EN_ADDR) == TRUE) && (alsAdjustedLevel != -1)){
                RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), alsAdjustedLevel, MODE_CHANGE_FADE_TIME, MODE_DEFAULT);
            } else {
                RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), ValExtractor_LUT24(loaded_levelArry), MODE_CHANGE_FADE_TIME, MODE_DEFAULT);
            }
        }
    } else if(MODE_DEMO == operatingMode){
        if(!Demo_Playing()){
            if((EEPROM.read(ALS_EN_ADDR) == TRUE) && (alsAdjustedLevel != -1)){
                RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), alsAdjustedLevel, 1000UL, MODE_DEFAULT);
            } else {
                RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), ValExtractor_LUT24(loaded_levelArry), 1000UL, MODE_DEFAULT);
            }
        }
    } else if(MODE_DAWNSIM == operatingMode){
        if(!DawnSim_Playing()){
            if((EEPROM.read(ALS_EN_ADDR) == TRUE) && (alsAdjustedLevel != -1)){
                RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), alsAdjustedLevel, MODE_CHANGE_FADE_TIME, MODE_DEFAULT);
            } else {
                RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), ValExtractor_LUT24(loaded_levelArry), MODE_CHANGE_FADE_TIME, MODE_DEFAULT);
            }
        }
    } else if(MODE_BEDTIME == operatingMode){
        if(!BedTime_Playing()){
            operatingMode = MODE_DEFAULT;
        }
    } else{ // default case: operatingMode = MODE_DEFAULT or other unassigned modes
        programCounter = 0;
        Load_RTC_Val();
        Daily_Subroutine();
    }
}


/*************************************************************************************************************
/
/               Scheduler Sub Routines
/
*************************************************************************************************************/
//Time.weekday() retuens an integer:  1 = Sunday, 2 = Monday, 3 = Tuesday, 4 = Wednesday, 5 = Thursday, 6 = Friday, 7 = Saturday
void ArioCtrl::Check_Wake_Alarm(void){
    unsigned int weekday = Time.weekday();
    if ((EEPROM.read(WAKEUP_ALARM_ENABLE_BASE_ADDR + weekday) == TRUE) && !lightIsOn && (MODE_DEMO != operatingMode) && (MODE_BEDTIME != operatingMode)){ // Dawn simulator alarm is set and the light is off
        if((Time.hour() == EEPROM.read(WAKEUP_ALARM_HOUR_BASE_ADDR + weekday)) && (Time.minute() == EEPROM.read(WAKEUP_ALARM_MINUTE_BASE_ADDR + weekday)) && AMalarmFlag){
            AMalarmFlag = FALSE;
            DawnSim_Init();
        }
        if(Time.minute() != EEPROM.read(WAKEUP_ALARM_MINUTE_BASE_ADDR + weekday)) AMalarmFlag = TRUE; // This ensures alarm only triggers once per day
    }
}

void ArioCtrl::Check_Bedtime_Reminder(void){
    unsigned int weekday = Time.weekday();
    if ((EEPROM.read(BEDTIME_ALARM_ENABLE_BASE_ADDR + weekday) == TRUE) && lightIsOn && (MODE_DEMO != operatingMode) && (MODE_DAWNSIM != operatingMode)){ // Dusk simulator alarm is set and light is on
        if((Time.hour() == EEPROM.read(BEDTIME_ALARM_HOUR_BASE_ADDR + weekday)) && (Time.minute() == EEPROM.read(BEDTIME_ALARM_MINUTE_BASE_ADDR + weekday)) && PMalarmFlag){
            PMalarmFlag = FALSE;
            BedTime_Init();
        }
        if(Time.minute() != EEPROM.read(BEDTIME_ALARM_MINUTE_BASE_ADDR + weekday)) PMalarmFlag = TRUE; // This ensures alarm only run once per day
    }
}

void ArioCtrl::Check_PIR_Schedule(void){
    //    0, 0, 0 => disbaled / 0, 0, 1 => disabled (not possible)
    //    1, 0, 0 / 0, 1, 0 / 1, 1, 0 => enabled
    //    1, 1, 1 / 1, 0, 1 / 0, 1, 1 => enabled with time
    if(SENSOR_PIR_AVAILABLE && ((MODE_DEFAULT == operatingMode) || (MODE_ADJUST == operatingMode))){
        if((EEPROM.read(PIR_ON_SET_ADDR) == TRUE) || (EEPROM.read(PIR_OFF_SET_ADDR) == TRUE)){
            if(EEPROM.read(PIR_SCHEDULE_EN_ADDR) == TRUE){ // enable with schedule (daily only)
                int beginTime = EEPROM.read(PIR_BEGIN_HOUR_ADDR)*60 + EEPROM.read(PIR_BEGIN_MINUTE_ADDR);
                int endTime = EEPROM.read(PIR_END_HOUR_ADDR)*60 + EEPROM.read(PIR_END_MINUTE_ADDR);
                int now = Time.hour()*60 + Time.minute();
                if (endTime > beginTime){
                    if((now >= beginTime) && (now < endTime)){
                        pirEnabled = TRUE;
                    } else{
                        pirEnabled = FALSE;
                    }
                } else{
                    if((now >= beginTime) || (now < endTime)){
                        pirEnabled = TRUE;
                    } else{
                        pirEnabled = FALSE;
                    }
                }
            } else{
                pirEnabled = TRUE;
            }
        }
    } else{
        pirEnabled = FALSE;
    }
}


/*************************************************************************************************************
/
/               Alarm & Timer Settings
/
*************************************************************************************************************/
void ArioCtrl::Set_Wake_Alarm(String str){
    unsigned int weekday = str.substring(0,1).toInt();
    unsigned int enable = str.substring(2,3).toInt();
    EEPROM.write((WAKEUP_ALARM_ENABLE_BASE_ADDR + weekday), enable);
    // if(1 == enable){
    if(str.charAt(3) == ','){
        EEPROM.write((WAKEUP_ALARM_HOUR_BASE_ADDR + weekday), str.substring(4,6).toInt());
        EEPROM.write((WAKEUP_ALARM_MINUTE_BASE_ADDR + weekday), str.substring(6,8).toInt());
        EEPROM.write((WAKEUP_ALARM_DURATION_BASE_ADDR + weekday), str.substring(9).toInt());
    }
}

void ArioCtrl::Set_Bedtime_Reminder(String str){
    unsigned int weekday = str.substring(0,1).toInt();
    unsigned int enable = str.substring(2,3).toInt();
    EEPROM.write((BEDTIME_ALARM_ENABLE_BASE_ADDR + weekday), enable);
    // if(1 == enable){
    if(str.charAt(3) == ','){
        EEPROM.write((BEDTIME_ALARM_HOUR_BASE_ADDR + weekday), str.substring(4,6).toInt());
        EEPROM.write((BEDTIME_ALARM_MINUTE_BASE_ADDR + weekday), str.substring(6,8).toInt());
        EEPROM.write((BEDTIME_ALARM_DURATION_BASE_ADDR + weekday), str.substring(9).toInt());
    }
}

void ArioCtrl::Configure_Sensor_PIR(String str){
    // first number is enable turn on, second is turn off, thrid is enable schedule, fourth is scheduled on time, fifth is scheduled off time
    // "1,1,060,1,1950,2010" "1,1,120,0" "0,1,015,1" (just enable schedule)
    pirHoldTimeMarker = millis();
    Cloud_Debug_Print("PIR Timer reset");
    EEPROM.write(PIR_ON_SET_ADDR, str.substring(0,1).toInt());
    EEPROM.write(PIR_OFF_SET_ADDR, str.substring(2,3).toInt());
    EEPROM.write(PIR_ON_DURATION, str.substring(4,7).toInt());
    EEPROM.write(PIR_SCHEDULE_EN_ADDR, str.substring(8,9).toInt());
    if(str.substring(9).length() != 0){
        Cloud_Debug_Print("Setting PIR schedule time!");
        EEPROM.write(PIR_BEGIN_HOUR_ADDR, str.substring(10,12).toInt());
        EEPROM.write(PIR_BEGIN_MINUTE_ADDR, str.substring(12,14).toInt());
        EEPROM.write(PIR_END_HOUR_ADDR, str.substring(15,17).toInt());
        EEPROM.write(PIR_END_MINUTE_ADDR, str.substring(17,19).toInt());
    }
}

void ArioCtrl::Configure_Sensor_ALS(String str){
    bool preEnable = EEPROM.read(ALS_EN_ADDR);
    EEPROM.write(ALS_EN_ADDR, str.substring(0,1).toInt());
    uint8_t sensitivity = constrain(str.substring(2).toInt(), ALS_SENSITIVITY_LOW, ALS_SENSITIVITY_HIGH); // Range Limited. Default is 6500K.
    EEPROM.write(ALS_SENSITIVITY_ADDR, sensitivity);
    if(lightIsOn && (operatingMode == MODE_DEFAULT) && (alsAdjustedLevel != -1)){
        if(!preEnable && (str.substring(0,1).toInt() == 1)){ //switch to enable
            RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), alsAdjustedLevel, 500UL, MODE_DEFAULT);
        } else if(preEnable && (str.substring(0,1).toInt() != 1)){ // switch to disable
            RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), ValExtractor_LUT24(loaded_levelArry), 500UL, MODE_DEFAULT);
        }
    }
}


/*************************************************************************************************************
/
/               Sensor Logic
/
*************************************************************************************************************/
///////////////////////// PIR /////////////////////////
void ArioCtrl::PIR_Routine(void){
    // reset the flag after time out to avoid false triggering
    if((millis() - pirDebounceTimer > 3000UL) && pirDebounceFlag){
        pirDebounceFlag = FALSE;
    }
    if(digitalRead(PIN_SENSOR_PIR)){
        if(!pirDebounceFlag){
            pirDebounceTimer = millis();
            pirDebounceFlag = TRUE;
        }
        if((millis() - pirDebounceTimer > 1600UL) && pirDebounceFlag){ // to avoid false triggering
            pirDebounceFlag = FALSE;
            pirHoldTimeMarker = millis();
            // Report presence detection
            if(millis() - pirReportTimer > PIR_REPORT_PERIOD){
                pirReportTimer = millis();
                Report_to_Cloud("sensor", "pir,true");
            }
            // Use PIR to turn lamp on if settings enabled
            if(pirEnabled && (EEPROM.read(PIR_ON_SET_ADDR) == TRUE) && !lightIsOn && (millis() - pirOffTimer > PIR_OFF_HOLD_DELAY)){
                Turn_Lamp_On(INTERACTION_TYPE_PIR);
            }
        }
    }
    // Use PIR to turn lamp off if settings enabled
    if(pirEnabled && (EEPROM.read(PIR_OFF_SET_ADDR) == TRUE) && lightIsOn && (millis() - pirHoldTimeMarker > (ONE_MINUTE*EEPROM.read(PIR_ON_DURATION)))){
        Turn_Lamp_Off(INTERACTION_TYPE_PIR);
        pirDebounceFlag = FALSE;
    }
}


///////////////////////// Ambient Light Sensor /////////////////////////
void ArioCtrl::ALS_Routine(void){
    // Key Parameter 1: ALS_EN_ADDR
    // Key Parameter 2: ALS_SENSITIVITY_RANGE_ADDR
    if(!alsMeasureFlag && (millis() - alsMeasureTimer > ALS_MEASURE_PERIOD)){ // Measure every 5 minutes
        alsMeasureFlag = TRUE;
        alsMeasureTimer = millis();
        alsMeasureCount = 0;
        alsRunningSum = 0;
    }
    if(alsMeasureFlag){
        if(alsMeasureCount < ALS_SAMPLES_NUMBER){
            if(millis() - alsSampleTimer > ALS_SAMPLE_INTERVAL){
                alsRunningSum += analogRead(PIN_SENSOR_ALS);
            }
            alsMeasureCount++;
        } else{ // acquisition complete
            // alsMeasuredLevel: measured ambient level with lamp inteference
            // alsBackgroundLevel: ambient level without lamp inteference
            // alsAdjustedLevel: adjusted level to run the lamp
            alsMeasuredLevel = alsRunningSum/ALS_SAMPLES_NUMBER;
            // Calculate background ambient level without self-interference of the lamp
            alsBackgroundLevel = alsMeasuredLevel;
            if(lightIsOn){
                alsBackgroundLevel -= 6.95 + 0.38*currentLevel - 0.00065*currentLevel*currentLevel;
            }
            float referenceLevel = ValExtractor_LUT24(loaded_levelArry);
            unsigned int alsSensitivityScale = EEPROM.read(ALS_SENSITIVITY_ADDR);
            if((alsSensitivityScale != ALS_SENSITIVITY_LOW) && (alsSensitivityScale != ALS_SENSITIVITY_MEDIUM) && (alsSensitivityScale != ALS_SENSITIVITY_HIGH)){
                alsSensitivityScale = ALS_SENSITIVITY_DEFAULT;
            }
            float alsScaledLevel = alsBackgroundLevel;
            if(ALS_SENSITIVITY_MEDIUM == alsSensitivityScale){
                if(alsScaledLevel > 3060){ alsScaledLevel = 3060; }
            } else if(ALS_SENSITIVITY_LOW == alsSensitivityScale){
                if(alsScaledLevel > 2040){ alsScaledLevel = 2040; }
            }

            alsScaledLevel /= alsSensitivityScale;
            alsAdjustedLevel = referenceLevel - alsScaledLevel;
            alsAdjustedLevel = constrain(alsAdjustedLevel, 10, 255);

            alsMeasureFlag = FALSE;
            alsMeasureCount = 0;
            alsRunningSum = 0;

            if((EEPROM.read(ALS_EN_ADDR) == TRUE) && lightIsOn && operatingMode == DEFAULT){ // Maybe lightIsOn doesn't matter
                RampTo_Linear_Setup(ValExtractor_LUT24(loaded_cctArry), alsAdjustedLevel, 10000UL, MODE_DEFAULT);
            }
        }
    }
    if((millis() - alsReportTimer > ALS_REPORT_PERIOD) && (alsBackgroundLevel != -1)){
        alsReportTimer = millis();
        char alsStr[40];
        int cctVal = currentCCT;
        int levelVal = currentLevel;
        sprintf(alsStr,"ambient,%d,%d,%d,%d,%d",alsBackgroundLevel,lightIsOn,cctVal,levelVal,operatingMode);
        Report_to_Cloud("sensor", alsStr);
    }
}


void ArioCtrl::Daily_Subroutine(void){ // Currently not used
    currentDay = Time.day();
    if (currentDay != lastDay){
        // if(EEPROM.read(DST_AUTO_CALC_ADDR) == TRUE){ Set_TimeZone(); } // Check for Day Light Savings
        lastDay = currentDay;
    }
}


/*************************************************************************************************************
/
/               Communications
/
*************************************************************************************************************/
///////////////////////// I2C /////////////////////////
void ArioCtrl::EzI2Cs_Init(void){
    Wire.begin();
}

/*  Function Name:  EzI2Cs_Write
    Description:    Write data to a PSoC EzI2Cs I2C slave device
    byte slaveAddr      : Address of target I2C slave device
    byte subAddrValue   : Relative sub-address of the exposed I2C memory locations in PSoC 1
    byte* dataArray     : pointer to array of data bytes
    byte length         : number of bytes to write
*/
void ArioCtrl::EzI2Cs_Write(byte slaveAddr, byte subAddrValue, byte* dataArray, byte length){
    Wire.beginTransmission(slaveAddr); /* transmit to device at address */
    Wire.write(subAddrValue);          /* sends sub address */
    Wire.write(dataArray, length);     /* sends data bytes */
    Wire.endTransmission();            /* stop transmitting */
}

void ArioCtrl::EzI2Cs_Read(byte slaveAddr, byte subAddrValue, byte* dataArray, byte length){
    int index = 0;
    Wire.beginTransmission(slaveAddr);      /* transmit to device at address */
    Wire.write(subAddrValue);               /* send sub address byte */
    Wire.endTransmission();                 /* stop transmitting */
    Wire.requestFrom(slaveAddr, length);    /* request 6 bytes from slave device #2 */
    while(Wire.available() && (index < length)){ // slave may send less than requested
      dataArray[index] = Wire.read(); // receive a byte as character
      index++;
    }
    delay(30); // A delay here is ESSENTIAL!!
}

/*
PSOC I2C Registers
    unsigned char onFlag;       // on of off
    unsigned char brightness;   // brightness
    unsigned char dimValue[4];  // direct control of LED dimming values
*/

// Writes a single byte to the PSoC EzI2C Register
void ArioCtrl::PSoC_WriteSingle(byte subAddr, byte data){
    Wire.beginTransmission(PSOC_ADDR);
    Wire.write(subAddr);
    Wire.write(data);
    Wire.endTransmission();
}

void ArioCtrl::PSoC_LEDVal(byte val0, byte val1, byte val2, byte val3){
    i2cSendBuffer[2] = val0;
    i2cSendBuffer[3] = val1;
    i2cSendBuffer[4] = val2;
    i2cSendBuffer[5] = val3;
    Wire.beginTransmission(PSOC_ADDR);
    Wire.write(2);
    Wire.write(val0);
    Wire.write(val1);
    Wire.write(val2);
    Wire.write(val3);
    Wire.endTransmission();
}

void ArioCtrl::PSoC_onOff(byte onOff){
    i2cSendBuffer[0] = onOff;
    Wire.beginTransmission(PSOC_ADDR);
    Wire.write(0);
    Wire.write(onOff);
    Wire.endTransmission();
}

void ArioCtrl::PSoC_changeLevel(byte level){
    i2cSendBuffer[1] = level;
    Wire.beginTransmission(PSOC_ADDR);
    Wire.write(1);
    Wire.write(level);
    Wire.endTransmission();
}

///////////////////////// UART /////////////////////////
void ArioCtrl::UART_Init(void){
    if(FALSE) Serial.begin(UART_BAUD);
}
void ArioCtrl::debugPrint(String debugMsg){
    if(FALSE){
        Serial.println(debugMsg);
    }
}

/*************************************************************************************************************
/
/               Color Mixing & Functions
/
*************************************************************************************************************/
void ArioCtrl::Load_RTC_Val(void){
    currentSecond = Time.second();
    if (currentSecond != lastSecond){
        if((EEPROM.read(ALS_EN_ADDR) == TRUE) && (alsAdjustedLevel != -1)){
            PSoC_Load_LEDVal(ValExtractor_LUT24(loaded_cctArry), alsAdjustedLevel);
        } else {
            PSoC_Load_LEDVal(ValExtractor_LUT24(loaded_cctArry), ValExtractor_LUT24(loaded_levelArry));
        }
        lastSecond = currentSecond;
    }
}


// returns corresponding CCT or brightness from the 24-value LUTs based on time of day
float ArioCtrl::ValExtractor_LUT24(uint16_t* dataArray){
    int diff = 0;
    int hour = Time.hour();
    float perc = (float)(currentSecond+Time.minute()*60)/3600;
    if(23 == hour){
        diff = dataArray[0] - dataArray[hour];
    } else {
        diff = dataArray[hour+1] - dataArray[hour];
    }
    //return dataArray[hour] + perc*diff;
    return constrain((dataArray[hour] + perc*diff), MIN_BRIGHTNESS, maxCCT); // I hate this constraint but the max brightness is lower than max cct always.
}


//Solve color mixing density using Cramer's Rule:
//        x=> cctHighDens ; y=> cctLowDens
//        ax+by=e where a=> cctHigh ; b=> cctLow ; e=> cctTarget
//        cx+dy=f where c=d=f=1 densities add up to unity
//    solve:
//        determinant = a*d - b*c = cctHigh - cctLow
//        x = (e*d - b*f)/determinant
//        y = (a*f - e*c)/determinant
void ArioCtrl::ColorDens_Calc(float cctTarget){
    if((CCT_6500 >= cctTarget) && (CCT_4000 < cctTarget)){ // Requested CCT is in 4000K - 6500K range
        // determinant = 6500 - 4000
        LED_COLOR_Cramer[0] = (cctTarget - CCT_4000)/(CCT_6500 - CCT_4000);
        LED_COLOR_Cramer[1] = (CCT_6500 - cctTarget)/(CCT_6500 - CCT_4000);
        LED_COLOR_Cramer[2] = 0;
        LED_COLOR_Cramer[3] = 0;
    } else if((CCT_1800 <= cctTarget) && (CCT_4000 >= cctTarget)){ // Requested CCT is in 1800K - 4000K range
        // determinant = 4000 - 1800 = 2200;
        LED_COLOR_Cramer[0] = 0;
        LED_COLOR_Cramer[1] = (cctTarget - CCT_1800)/(CCT_4000 - CCT_1800);
        LED_COLOR_Cramer[2] = (CCT_4000 - cctTarget)/(CCT_4000 - CCT_1800);
        LED_COLOR_Cramer[3] = LED_COLOR_Cramer[2]; // top and bottom have the same mixing density
    } else if((CCT_1800 > cctTarget) && (MIN_CCT <= cctTarget)){ // 1800K top modulated, scaled to 100
        LED_COLOR_Cramer[0] = 0;
        LED_COLOR_Cramer[1] = 0;
        LED_COLOR_Cramer[2] = (cctTarget - MIN_CCT)/(CCT_1800 - MIN_CCT); // has to be divided by (1800-MIN_CCT)
        LED_COLOR_Cramer[3] = 1;
    } else if(MIN_CCT > cctTarget){ // bottom 1800K only
        LED_COLOR_Cramer[0] = 0;
        LED_COLOR_Cramer[1] = 0;
        LED_COLOR_Cramer[2] = 0;
        LED_COLOR_Cramer[3] = 1;
    } else{ // Requested CCT is greater than 6500K => defaults to 6500K
        LED_COLOR_Cramer[0] = 1;
        LED_COLOR_Cramer[1] = 0;
        LED_COLOR_Cramer[2] = 0;
        LED_COLOR_Cramer[3] = 0;
    }
}

void ArioCtrl::PSoC_Load_LEDVal(float cctVal, float brightnessVal){
    currentCCT = cctVal;
    currentLevel = brightnessVal;
    ColorDens_Calc(currentCCT);
    for(int ch = 0; ch < NUM_LED_CH; ch++){
        LED_CH_Dens[ch] = LED_COLOR_Cramer[ch]*currentLevel; // convert to byte
        i2cSendBuffer[ch+2] = LED_CH_Dens[ch]; // can be further optimized here
    }
    Wire.beginTransmission(PSOC_ADDR);
    Wire.write(2);
    for(int i = 2; i < (NUM_LED_CH+2); i++){
        Wire.write(i2cSendBuffer[i]);
    }
    Wire.endTransmission();
}


/*************************************************************************************************************
/
/               Debug & Report Functions
/
*************************************************************************************************************/
void ArioCtrl::Cloud_Print_Schedule(void){
    String publishString1 = "";
    for(int i = 0; i < 24; i++){
        String smallString = String(loaded_cctArry[i]);
        publishString1.concat(" ");
        publishString1.concat(smallString);
    }
    Cloud_Debug_Print("CCT Schedule: ",publishString1);
    String publishString2 = "";
    for(int i = 0; i < 24; i++){
        String smallString = String(loaded_levelArry[i]);
        publishString2.concat(" ");
        publishString2.concat(smallString);
    }
    Cloud_Debug_Print("Brightness Schedule: ",publishString2);
}

void ArioCtrl::Cloud_Debug_Print(String str){
    if((EEPROM.read(CLOUD_DEBUG_ADDR) == TRUE) && Particle.connected()){
        Particle.publish(str);
    }
}

void ArioCtrl::Cloud_Debug_Print(String msgType, String payload){
    if((EEPROM.read(CLOUD_DEBUG_ADDR) == TRUE) && Particle.connected()){
        Particle.publish(msgType, payload);
    }
}

void ArioCtrl::Report_to_Cloud(String msgType, String payload){
    // if(Particle.connected()){
    //     Particle.publish(msgType, payload);
    // }else{
    //     //record everything if not connected to cloud, wait for reconnection or reboot then push data
    // }
}
