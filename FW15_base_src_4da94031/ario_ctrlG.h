/************************************************************************************************************************************/
/** @file       ario_ctrlG.h
 *  @brief      see ario_ctrlG.cpp for description
 *
 *  @author     Shaw-Pin Chen, Product Lead, Ario, Inc.
 *  @created    09-16-16
 *  @last rev   09-16-16
 *
 *  @section    Legal Disclaimer
 *          All contents of this source file and/or any other Ario, Inc. related source files are the explicit property of
 *          Ario, Inc. Do not distribute. Do not copy.
 */
/************************************************************************************************************************************/

#ifndef ario_ctrlG_h
#define ario_ctrlG_h

#include "application.h"
#include "globals.h"

class ArioCtrl
{
    public:
        ArioCtrl();
        ~ArioCtrl();

        bool lightIsOn, cctMode;
        int maxCCT, alsBackgroundLevel;
        unsigned int operatingMode, rampRegNextMode, currentVersion;
        float currentCCT, currentLevel;
        //int currentVersion;

        void Ario_Init(void);
        void Set_TimeZone(void);
        void Load_RTC_Schedule(void);
        void Load_Max_CCT(void);
        void Turn_Lamp_On(byte interactionType);
        void Turn_Lamp_Off(byte interactionType);
        void Light_Switch(void);
        ///////// button functions //////////
        void TopButton_Action(void);
        void MidButton_Action(void);

        void decode_cmd(int cmd);
        void Increase_Brightness_App(void);
        void Decrease_Brightness_App(void);
        void Set_Brightness(unsigned int brightness);
        void Increase_CCT_App(void);
        void Decrease_CCT_App(void);
        void Set_CCT(unsigned int cct);

        void Demo_Init(void);
        void Scheduler(void);

        ///////// Alarm Functions //////////
        void Set_Wake_Alarm(String str);
        void Set_Bedtime_Reminder(String str);
        void Configure_Sensor_PIR(String str);
        void Configure_Sensor_ALS(String str);

        ///////// Sensors Functions //////////
        void PIR_Routine(void);
        void ALS_Routine(void);

        ///////// cloud comm ///////////////
        void Cloud_Print_Schedule(void);
        void Cloud_Debug_Print(String str);
        void Cloud_Debug_Print(String msgType, String payload);
        void Report_to_Cloud(String msgType, String payload);

        ////////// LED Diagnostic //////////////
        void PSoC_onOff(byte onOff);
        void PSoC_LEDVal(byte val0, byte val1, byte val2, byte val3);

    private:
        bool pirEnabled, cloudReportFlag, pirDebounceFlag, alsMeasureFlag, AMalarmFlag, PMalarmFlag, amAlarmNow, pmAlarmNow;
        int alsMeasuredLevel;
        unsigned int programCounter, rampRegCounter, rampRegEndCounter, alsRunningSum, alsMeasureCount;
        unsigned long marker, pirDebounceTimer, pirOffTimer, pirHoldTimeMarker, pirReportTimer, alsMeasureTimer, alsSampleTimer, alsReportTimer, dawnSimDuration, bedTimeDuration;

        // Linear Ramp Mode Register
        float rampRegCCTStep, rampRegLevelStep, alsAdjustedLevel;
        //unsigned int rampRegCounter, rampRegEndCounter;

        void Load_Current_Version(void);

        void PSoC_Init(void);
        //bool IsDST(int dayOfMonth, int month, int dayOfWeek);
        void Increase_Level(void);
        void Decrease_Level(void);

        void RampTo_Linear_Setup(float destCCT, float destLevel, unsigned long duration, int toMode);
        bool RampTo_Linear_Playing(void);

        bool Demo_Playing(void);
        void DawnSim_Init(void);
        bool DawnSim_Playing(void);
        void BedTime_Init(void);
        bool BedTime_Playing(void);

        ////////// time functions //////////
        float ValExtractor_LUT24(uint16_t* dataArray);
        void ColorDens_Calc(float cctTarget);
        void Load_RTC_Val(void); // loads and sends calculated RTC LED values to PSoC
        void PSoC_Load_LEDVal(float cctVal, float brightnessVal);

        ///////// Scheduler Sub Routines //////
        void Check_Wake_Alarm(void);
        void Check_Bedtime_Reminder(void);
        void Check_PIR_Schedule(void);
        void Daily_Subroutine(void);

        ///////// comm functions ///////////
        void EzI2Cs_Init(void);
        void EzI2Cs_Write(byte slaveAddr, byte subAddrValue, byte* dataArray, byte length);
        void EzI2Cs_Read(byte slaveAddr, byte subAddrValue, byte* dataArray, byte length);
        void PSoC_WriteSingle(byte subAddr, byte data);
        void PSoC_changeLevel(byte level);

        ///////// serial debugging ///////////
        void UART_Init(void);
        void debugPrint(String debugMsg);

};

#endif
