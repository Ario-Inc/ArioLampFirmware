/************************************************************************************************************************************/
/** @file       Ario_protoG_v1(main.cpp)
 *  @brief      This is the entry file for the Ario Lamp firmware.
 *  @details    contains main loop
 *
 *  @author     Shaw-Pin Chen, Product Lead, Ario, Inc.
 *  @created    09-16-16
 *  @last rev   04-18-17
 *
 *  @notes      First time configure or factory reset (EEPROM) make sure to exit factory test mode
 *              For commercial deployment make sure to change AP Prefix from "PHOTON" to "ARIO"
 *
 *  @revisions
 *  04-02-17[0.2.2] expanded brightness level lower limit from 5/255 to 2/255
 *  04-06-17[0.2.3] changed statusLightIndicator() to reduce for WiFi pairing/searching brightness and reduce timeout
 *  04-14-17[0.2.4] main - added RAM based time roll-back checker in main loop
 *  04-18-17[0.2.5] main - adjusted roll-back checker in main loop to include non-volatile EEPROM time mark
 *                  cpp -changed boolean comparison DST to numerical
 *                  globals.h - changed DST EEPROM address from 0x002 to 0x00A, added DST_CHECKER_TIME_MARK (0x0A3-0x0A6)
 *  04-23-17[0.2.6] fixed the status indicator LED displaying cctMode color
 *  04-26-17[0.2.6.1] fixed softap prefix (WDD)
 *  04-26-17[0.2.6.2] merged in all teh changes from 0.2.5 (WDD)
 *
 *  @section    Legal Disclaimer
 *          All contents of this source file and/or any other Ario, Inc. related source files are the explicit property of
 *          Ario, Inc. Do not distribute. Do not copy.
 */
/************************************************************************************************************************************/

#include "flashee-eeprom/flashee-eeprom.h"
#include "clickButton/clickButton.h"
#include "photon-wdgs/photon-wdgs.h"

#include "application.h"
#include "globals.h"
//#include "connectivity.h"
#include "ario_ctrlG.h"

//PRODUCT_ID(1811);
//PRODUCT_VERSION(9);

//STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

using namespace Flashee;
FlashDevice* flash;
ClickButton modeButton(PIN_BUTTON_BOTTOM, HIGH, CLICKBTN_PULLUP); // debounced mode button

// Factory Test Variables
bool factoryMode = FALSE;
bool btn1TestFlag = FALSE;
bool btn2TestFlag = FALSE;
bool btn3TestFlag = FALSE;

//LED Diagnostic flag
bool ledCheck = FALSE;
int ledGroup = 0;
unsigned long ledCheckMarker = 0;

// WiFi Reconnect Test Code Variables (excerpt from Particle's wifiMonitor code)
const unsigned long CLOUD_CHECK_PERIOD = 60000;
unsigned long lastCloudCheck = 0;
//My addition for reset
bool wifiResetFlag = FALSE;
unsigned long wifiResetTimeMark = 0;
const unsigned long WIFI_RESET_PERIOD = 600000; // 10 minutes

unsigned long lastTimeCheck = 0;/////////////////////////////////////////////////////////////////////////////////
const unsigned long TIME_CHECK_PERIOD = 300000;////////////////////////////////////////////////////////////////////
unsigned int currentTime, lastTime;

bool offlineMode = FALSE;

// Button results
int function = 0;
unsigned long cctModeTimeOutLimit = millis();
unsigned long statusLEDTimeoutLimit = millis();

bool enterPairingFlag = FALSE;
unsigned long enterPairingTimer = 0;

char arioStateStr[40];
String timeNowStr = "";

int lastScheduleUploadedFlag = 0;

ArioCtrl aCtrl;

// forward function declarations
void factoryTest();
void stateVarConstructor();
void disconnectCheck();
void buttonScanner();
void statusLightManager();
int ctrlArio(String ctrlCmd);
int setArio(String setCmd);
int checkArio(String checkCmd);
int uploadCCT(String cctString);
int uploadLevel(String levelString);
int clearArio(String clearCmd);
void reportMacAddress();
void raiseHand();
void ledDiagnostics();

int measureAmbient(String ambCmd); ///////////////////////////////////////////////////////////////////////////


void setup() {
    Serial.begin(9600);

    aCtrl.Ario_Init();

    PhotonWdgs::begin(true,true,30000,TIMER7);
    System.on(firmware_update_pending, handle_update);

    pinMode(PIN_BUTTON_TOP, INPUT_PULLDOWN); // not really needed, was declared in cpp
    pinMode(PIN_BUTTON_MIDDLE, INPUT_PULLDOWN);
    pinMode(PIN_BUTTON_BOTTOM, INPUT_PULLDOWN);

    if(SENSOR_ALS_AVAILABLE){ pinMode(PIN_SENSOR_ALS, INPUT); } // Do not setup an analog read pin!
    if(SENSOR_PIR_AVAILABLE){ pinMode(PIN_SENSOR_PIR, INPUT_PULLDOWN); }

    Particle.function("arioDo", ctrlArio);
    Particle.function("arioSet", setArio);
    Particle.function("arioCheck", checkArio);
    Particle.function("arioClear", clearArio);
    Particle.function("cctUpload", uploadCCT);
    Particle.function("levelUpload", uploadLevel);
    Particle.function("getAmbient", measureAmbient);

    Particle.variable("fwVersion", "0.2.6.15");
    Particle.variable("state", arioStateStr);
    Particle.variable("maxCCT", aCtrl.maxCCT);
    Particle.variable("time", timeNowStr);
    Particle.variable("schFlag", lastScheduleUploadedFlag);
    Particle.variable("ambient", aCtrl.alsBackgroundLevel);

    flash = Devices::createAddressErase();
    System.set(SYSTEM_CONFIG_SOFTAP_PREFIX, "ARIO"); // Replace with PHOTON to use the Particle app pairing

    // Check if the controller is in factory mode. If EEPROM_DEFAULT_VAL then it is in factory test mode
    if(EEPROM.read(FACTORY_TEST_MODE_ADDR) == 255){ factoryMode = TRUE; /*System.set(SYSTEM_CONFIG_SOFTAP_PREFIX, "ARIO");*/ }

    WiFi.on();
    Particle.connect();

}


void loop() {
    PhotonWdgs::tickle();
    if(!factoryMode && !ledCheck){
        stateVarConstructor();

        aCtrl.Scheduler();

        buttonScanner();

        //disconnectCheck();

        //if(TRUE){ statusLightManager(); }
        if(!aCtrl.cctMode){ statusLightManager(); }///////////////////////////////////////////////////////////////

        if(aCtrl.cctMode && (millis() - cctModeTimeOutLimit >= CCT_MODE_TIMEOUT)){ aCtrl.cctMode = FALSE; RGB.control(false); } // CCT Mode Time Out

        if(SENSOR_PIR_AVAILABLE && (millis() > PIR_STABLE_TIME)){ aCtrl.PIR_Routine(); } // PIR logic

        if(SENSOR_ALS_AVAILABLE && RGB.controlled() && !aCtrl.cctMode && (MODE_DEMO != aCtrl.operatingMode)){ aCtrl.ALS_Routine(); } // ALS logic

        timeCheck();

    }
    else if (ledCheck) { //LED Diagnostic Mode
        ledDiagnostics();
    }
    else { //Factory Testing Mode
        factoryTest();
    }

}

void handle_update(system_event_t event, int param) {
   if(PhotonWdgs::_wwdgRunning) {
       WWDG_DeInit();
   }
   System.enableUpdates();
   PhotonWdgs::_wdgTimer.end();
}

void ledDiagnostics() {
  if(digitalRead(PIN_BUTTON_BOTTOM)) { // Return to normal mode
    ledCheck = FALSE;
    RGB.control(FALSE);
  }
  else {
    RGB.control(TRUE);
    RGB.color(255,0,255);    // Purple
    if (millis() > ledCheckMarker)  {
      if(digitalRead(PIN_BUTTON_TOP)) { // Increment to next LED group
        ledCheckMarker = millis() + 300UL;
        ledGroup++;
        if (ledGroup > 3) {
          ledGroup = 0;
        }
      }
      else if(digitalRead(PIN_BUTTON_MIDDLE)) { // Decrement to previous LED group
        ledCheckMarker = millis() + 300UL;
        ledGroup++;
        if (ledGroup > 3) {
          ledGroup = 0;
        }
      }
    }

    aCtrl.PSoC_onOff(1);
    switch (ledGroup) {
      case 0:
        aCtrl.PSoC_LEDVal(255,0,0,0); // 6500
        break;
      case 1:
        aCtrl.PSoC_LEDVal(0,255,0,0); // 4000
        break;
      case 2:
        aCtrl.PSoC_LEDVal(0,0,0,255); // BOTTOM
        break;
      case 3:
        aCtrl.PSoC_LEDVal(0,0,255,0); // TOP
        break;
    }


  }
}

void factoryTest(){
    if(Particle.connected() && (millis() > PIR_STABLE_TIME)){
        if(SENSOR_PIR_AVAILABLE && digitalRead(PIN_SENSOR_PIR)){
            RGB.color(13,252,29);
            delay(500);
            RGB.color(0,0,0);
        }
        if(SENSOR_ALS_AVAILABLE && (analogRead(PIN_SENSOR_ALS) < 100)){
            RGB.color(252,188,13);
            delay(1000);
            RGB.color(0,0,0);
        }
        if(digitalRead(PIN_BUTTON_TOP)){btn1TestFlag = TRUE; RGB.color(255,255,255); delay(500); RGB.color(0,0,0);}
        if(digitalRead(PIN_BUTTON_MIDDLE)){ btn2TestFlag = TRUE; RGB.color(255,255,255); delay(500); RGB.color(0,0,0);}
        if(digitalRead(PIN_BUTTON_BOTTOM)){ btn3TestFlag = TRUE; RGB.color(255,255,255); delay(500); RGB.color(0,0,0);}

        // must press each of the three buttons at least once
        if(btn1TestFlag && btn2TestFlag && btn3TestFlag){
            EEPROM.write(FACTORY_TEST_MODE_ADDR, 1);
            //RGB.control(FALSE);
            factoryMode = FALSE;
            aCtrl.Report_to_Cloud("test","success,btnsensor");
            RGB.color(13,252,29); delay(100); RGB.color(0,0,0); delay(100);
            RGB.color(13,252,29); delay(100); RGB.color(0,0,0); delay(100);
            RGB.color(13,252,29); delay(100); RGB.color(0,0,0); delay(100);
            RGB.control(FALSE);
            statusLEDTimeoutLimit = millis(); // so the indicator light won't go off right away
        } else {
            RGB.control(TRUE);
        }
    }
}

void statusLightManager(){
    if(digitalRead(PIN_BUTTON_TOP) || digitalRead(PIN_BUTTON_MIDDLE) || digitalRead(PIN_BUTTON_BOTTOM)){
      statusLEDTimeoutLimit = millis();// reset the timeout timer
    }
    if ((offlineMode || Particle.connected()) && (millis() - statusLEDTimeoutLimit >= STATUS_LED_TIMEOUT)) {
      RGB.control(TRUE);
      RGB.color(0, 0, 0);
    }
    else {
      RGB.control(FALSE);
      RGB.brightness(50);
    }
}

void stateVarConstructor(){
    int arioColor = aCtrl.currentCCT;
    int arioBrightness = aCtrl.currentLevel;
    int arioMode = aCtrl.operatingMode;
    if(arioMode == MODE_RAMP){
        arioMode = aCtrl.rampRegNextMode;
    } else if(arioMode == MODE_DEMO){
        arioMode = MODE_DEFAULT;
    }
    sprintf(arioStateStr,"%d,%d,%d,%d,%d", aCtrl.lightIsOn, arioColor, arioBrightness, arioMode, aCtrl.currentVersion);
    timeNowStr = Time.timeStr();
}


void disconnectCheck(){
    // WiFi Reconnect Test Code (excerpt from Particle's wifiMonitor code)
    if (WiFi.ready() && !Particle.connected()) {
        // If we have wifi but not cloud, log some pings and DNS
        if (millis() - lastCloudCheck >= CLOUD_CHECK_PERIOD) {
            lastCloudCheck = millis();
            IPAddress addr = IPAddress(8,8,8,8);
            WiFi.ping(addr, 1);
            const char *host = "io.arioliving.com";
            addr = WiFi.resolve(host);
        }
        if(!wifiResetFlag){
            wifiResetFlag = TRUE;
            wifiResetTimeMark = millis();
        }
        if(wifiResetFlag && (millis() - wifiResetTimeMark >= WIFI_RESET_PERIOD)){
            wifiResetFlag = FALSE;
            System.reset();


            //WiFi.off();
            //WiFi.on();
            //WiFi.connect();
        }
    }
    else {
      wifiResetFlag = FALSE;
    }
}


void buttonScanner(){
    modeButton.Update();
    if(0 != modeButton.clicks) function = modeButton.clicks;
    switch(function){
        case 1:
            if(aCtrl.cctMode){
                aCtrl.cctMode = FALSE;
                RGB.control(FALSE);
            } else{
                aCtrl.Light_Switch();
            }
            break;
        case 2:
            if (digitalRead(PIN_BUTTON_TOP)) {
              ledGroup = 0;
              ledCheckMarker = millis() + 600UL;
              ledCheck = TRUE;
            }
            else if (digitalRead(PIN_BUTTON_MIDDLE)) {
              raiseHand();
            }
            else if (aCtrl.lightIsOn) {
              aCtrl.Demo_Init();
            }
            break;
        case 3:
            //
            break;
        case -1:
            if(digitalRead(PIN_BUTTON_MIDDLE)){
                reportMacAddress();
            } else if(!aCtrl.lightIsOn){
                aCtrl.Light_Switch();
            } else{
                aCtrl.cctMode = TRUE;
                RGB.control(TRUE);
                RGB.color(255, 50, 0);
                cctModeTimeOutLimit = millis(); // reset the time marker
            }
            break;
        case -2:
            if(WiFi.listening() || WiFi.connecting() || WiFi.ready()){
                offlineMode = TRUE;
                WiFi.off();
            } else { // exits offline or listening mode
                offlineMode = FALSE;
                WiFi.on();
                Particle.connect();
            }
            //Test
            //if(TRUE){
            //    String myID = System.deviceID();
            //    aCtrl.Cloud_Debug_Print(myID);
            //}
            break;
        case -3:
            WiFi.disconnect();
            aCtrl.cctMode = FALSE;
            WiFi.listen();
            break;

        case -4:
            reportMacAddress();
            break;
        default:
            break;
    }
    modeButton.clicks = 0; // bug fix need testing: this avoids executing twice
    function = 0;
    if(digitalRead(PIN_BUTTON_TOP)){ aCtrl.TopButton_Action(); cctModeTimeOutLimit = millis(); }
    if(digitalRead(PIN_BUTTON_MIDDLE)){ aCtrl.MidButton_Action(); cctModeTimeOutLimit = millis(); }

    // Alternative way of getting into Listening Mode - Hold down Mode button for ~15 seocnds
    if(digitalRead(PIN_BUTTON_BOTTOM)){
        if(!enterPairingFlag){
            enterPairingFlag = TRUE;
            enterPairingTimer = millis();
        }
        if(enterPairingFlag && (millis() - enterPairingTimer > ENTER_WIFI_PAIRING_TIME)){
            WiFi.disconnect();
            aCtrl.cctMode = FALSE;///////////////////////////////////////////////////////////////////////////////////
            WiFi.listen();
        }
    } else{
        enterPairingFlag = FALSE;
    }
}


// Particle Cloud Functions
int ctrlArio(String ctrlCmd) {
    aCtrl.Cloud_Debug_Print("Ario called to action!");
    if(ctrlCmd.substring(0,3) == "PWR"){
        if((ctrlCmd.charAt(4) == '1') && !aCtrl.lightIsOn){
            aCtrl.Turn_Lamp_On(INTERACTION_TYPE_WEB);
        } else if((ctrlCmd.charAt(4) == '0') && aCtrl.lightIsOn){
            aCtrl.Turn_Lamp_Off(INTERACTION_TYPE_WEB);
        }
    } else if(ctrlCmd.substring(0,3) == "BRI") { // cannot be evaluated the other way around i.e. ("BRI" == command.subString(0,3))
        if(ctrlCmd.substring(4,6) == "UP"){
            aCtrl.Increase_Brightness_App();
        } else if(ctrlCmd.substring(4,8) == "DOWN"){
            aCtrl.Decrease_Brightness_App();
        } else{
            aCtrl.Set_Brightness(ctrlCmd.substring(4).toInt()); // "BRI,20"
        }
    } else if(ctrlCmd.substring(0,3) == "CCT"){
        if(ctrlCmd.substring(4,6) == "UP"){
            aCtrl.Increase_CCT_App();
        } else if(ctrlCmd.substring(4,8) == "DOWN"){
            aCtrl.Decrease_CCT_App();
        } else{
            aCtrl.Set_CCT(ctrlCmd.substring(4).toInt()); // "CCT,1800"
        }
    } else if(ctrlCmd.substring(0,4) == "DEMO"){
        if(aCtrl.lightIsOn) aCtrl.Demo_Init();
    } else{
        aCtrl.decode_cmd(ctrlCmd.toInt());
    }
    return 200;
}

int setArio(String setCmd) {
    if(setCmd.substring(0,3) == "VER"){
        uint8_t newVersion = setCmd.substring(4).toInt();
        EEPROM.write(CURRENT_VERSION_ADDR, newVersion);
        aCtrl.currentVersion = newVersion;
    } else if(setCmd.substring(0,4) == "WAKE"){
        // "WAKE,2,1,0630,120" => enable at Monday (day 2) 6:30 AM for 120 minutes
        // "WAKE,4,0" => disable alarm for Wednesday (retains other settings in memory)
        // Duration cannot exceed 0xFF
        aCtrl.Set_Wake_Alarm(setCmd.substring(5));
        aCtrl.Cloud_Debug_Print("Wake Time Set!");
    } else if(setCmd.substring(0,3) == "BED"){
        aCtrl.Set_Bedtime_Reminder(setCmd.substring(4));
        aCtrl.Cloud_Debug_Print("Bed Time Set!");
    } else if(setCmd.substring(0,3) == "PIR"){
        // first number is enable turn on, second is turn off, thrid is on duration in minutes,
        // fourth is enable schedule, fifth is scheduled on time, sixth is scheduled off time
        aCtrl.Configure_Sensor_PIR(setCmd.substring(4));
        // "PIR,1,1,060,1,1950,2010", everything disabled by default, duration cannot exceed 0xFF
        // user require to set duration if enable auto off first time or the app input default 30 minute duration
        aCtrl.Cloud_Debug_Print("PIR Configured!");
    } else if(setCmd.substring(0,3) == "ALS"){
        aCtrl.Configure_Sensor_ALS(setCmd.substring(4));
        aCtrl.Cloud_Debug_Print("ALS Configured!");
    } else if(setCmd.substring(0,4) == "HOLD"){
        EEPROM.write(HOLD_TIME_DURTION_ADDR, setCmd.substring(5).toInt()); // no need to zero-pad, 60 minutes by default
        aCtrl.Cloud_Debug_Print("Hold Time Adjusted!");
    } else if(setCmd.substring(0,3) == "DST"){
        EEPROM.write(DST_ENABLE_ADDR, setCmd.substring(4,5).toInt()); // "DST,1" to enable, "DST,0" to disable (default), (anything other than 1 would default to disable)
        aCtrl.Set_TimeZone();
    } else if(setCmd.substring(0,4) == "ZONE"){
        // Formula to calculate zone in app, (x+12)*4
        EEPROM.write(USER_TIME_ZONE, setCmd.substring(5).toInt()); // no need to zero-pad , range from 0-104, anything greater would default to central time
        aCtrl.Set_TimeZone();
    } else if(setCmd.substring(0,6) == "MAXCCT"){
        uint16_t setCCT = constrain(setCmd.substring(7).toInt(), MAX_CCT_LOWER_LIMIT, MAX_CCT_UPPER_LIMIT); // Range Limited. Default is 6500K.
        EEPROM.put(MAX_CCT_ADDR, setCCT);
        aCtrl.Load_Max_CCT();
    } else if(setCmd.substring(0,5) == "DEBUG"){
        EEPROM.write(CLOUD_DEBUG_ADDR, setCmd.substring(6,7).toInt()); // "DEBUG,1" to enable, "DDEBUG,0" to disable cloud debug messages
    }
    return 200;
}


int checkArio(String checkCmd) {
    char publishString[40];
    if(checkCmd.substring(0,4) == "WAKE"){
        int day = checkCmd.substring(5).toInt();
        int enable = EEPROM.read(WAKEUP_ALARM_ENABLE_BASE_ADDR + day);
        int hour = EEPROM.read(WAKEUP_ALARM_HOUR_BASE_ADDR + day);
        int min = EEPROM.read(WAKEUP_ALARM_MINUTE_BASE_ADDR + day);
        int duration = EEPROM.read(WAKEUP_ALARM_DURATION_BASE_ADDR + day);
        sprintf(publishString,"%d,%d,%d,%d", enable, hour, min, duration);
        aCtrl.Cloud_Debug_Print("Wake Alarm was set at: ",publishString);
    } else if(checkCmd.substring(0,3) == "BED"){
        int day = checkCmd.substring(4).toInt();
        int enable = EEPROM.read(BEDTIME_ALARM_ENABLE_BASE_ADDR + day);
        int hour = EEPROM.read(BEDTIME_ALARM_HOUR_BASE_ADDR + day);
        int min = EEPROM.read(BEDTIME_ALARM_MINUTE_BASE_ADDR + day);
        int duration = EEPROM.read(BEDTIME_ALARM_DURATION_BASE_ADDR + day);
        sprintf(publishString,"%d,%d,%d,%d", enable, hour, min, duration);
        aCtrl.Cloud_Debug_Print("Bedtime Reminder was set at: ",publishString);
    } else if(checkCmd.substring(0,4) == "TIME"){
        aCtrl.Report_to_Cloud("time", Time.timeStr());
    } else if(checkCmd.substring(0,7) == "FREEMEM"){
        int freemem = System.freeMemory();
        sprintf(publishString,"%u", freemem);
        aCtrl.Cloud_Debug_Print("Free memory: ", publishString);
    } else if(checkCmd.substring(0,8) == "SCHEDULE"){
        aCtrl.Cloud_Print_Schedule();
    } else if(checkCmd.substring(0,3) == "MAC"){
        reportMacAddress();
    } else{
        int addr = checkCmd.toInt();
        //if((addr == MAX_CCT_ADDR) || ((addr >= USER_SCHEDULE_CCT_BASE_ADDR) && (addr < USER_SCHEDULE_LEVEL_BASE_ADDR))){
        //    uint16_t content;
        //    EEPROM.get(addr, content);
        //    sprintf(publishString,"%d", content);
        //} else{
            int content = EEPROM.read(addr);
            sprintf(publishString,"%d", content);
        //}
        aCtrl.Cloud_Debug_Print("Content at the location is: ",publishString);
    }
    return 200;
}





// "1,aaaabbbbccccddddeeeeffffgggghhhhiiiijjjjkkkkllll"
int uploadCCT(String cctString){
    int part = cctString.substring(0,1).toInt(); // part 1 or part 2 of LUT
    String content = cctString.substring(2); // extract the rest of the string
    if(((1 != part)&&(2 != part))||(content.length()!= 48)){ // check formatting
        aCtrl.Cloud_Debug_Print("CCT Schedule Format Not Correct!");
        return 404;
    }
    int addr;
    if(EEPROM.read(SCHEDULE_SELECT_ADDR) != 1){ // default 0xFF or 0x02
        addr = SCHEDULE_1_CCT_BASE_ADDR;
    } else{
        addr = SCHEDULE_2_CCT_BASE_ADDR;
    }
    if((1 == part) && (0 == lastScheduleUploadedFlag)){
        for(int i = 0; i < 12; i++){
            uint16_t val = content.substring(i*4, (i*4 + 4)).toInt();
            EEPROM.put((addr + i*2), val);
        }
        lastScheduleUploadedFlag = 1; // CCT part 1 upload complete
        aCtrl.Cloud_Debug_Print("CCT part 1 upload complete!");
    } else if((2 == part) && (1 == lastScheduleUploadedFlag)){
        addr += 24;
        for(int i = 0; i < 12; i++){
            uint16_t val = content.substring(i*4, (i*4 + 4)).toInt();
            EEPROM.put((addr + i*2), val);
        }
        lastScheduleUploadedFlag = 2; // CCT part 2 upload complete
        aCtrl.Cloud_Debug_Print("CCT part 2 upload complete!");
    } else{
        lastScheduleUploadedFlag = 0; // reset the flag so any upload has to start fresh
        aCtrl.Cloud_Debug_Print("Incorrect Upload Order!");
        return 404;
    }
    return 200;
}


int uploadLevel(String levelString){
    int part = levelString.substring(0,1).toInt(); // part 1 or part 2 of LUT
    String content = levelString.substring(2); // extract the rest of the string
    if(((1 != part)&&(2 != part))||(content.length() != 36)){ // check formatting
        aCtrl.Cloud_Debug_Print("Level Schedule Format Not Correct!");
        return 404;
    }
    int addr;
    if(EEPROM.read(SCHEDULE_SELECT_ADDR) != 1){ // default 0xFF or 0x02
        addr = SCHEDULE_1_LEVEL_BASE_ADDR;
    } else{
        addr = SCHEDULE_2_LEVEL_BASE_ADDR;
    }
    if((1 == part) && (2 == lastScheduleUploadedFlag)){
        for(int i = 0; i < 12; i++){
            byte val = content.substring(i*3, (i*3 + 3)).toInt();
            EEPROM.put((addr + i), val);
        }
        lastScheduleUploadedFlag = 3; // Level part 2 upload complete
        aCtrl.Cloud_Debug_Print("Level part 1 upload complete!");
    } else if((2 == part) && (3 == lastScheduleUploadedFlag)){
        addr += 12;
        for(int i = 0; i < 12; i++){
            byte val = content.substring(i*3, (i*3 + 3)).toInt();
            EEPROM.put((addr + i), val);
        }
        if(EEPROM.read(SCHEDULE_SELECT_ADDR) != 1){ // update the EEPROM schedule selector
            EEPROM.write(SCHEDULE_SELECT_ADDR, 1);
        } else{
            EEPROM.write(SCHEDULE_SELECT_ADDR, 2);
        }
        aCtrl.Load_RTC_Schedule();
        lastScheduleUploadedFlag = 0; // reset the flag for the next upload
        aCtrl.Report_to_Cloud("upload", "success,schedule");
    } else{
        lastScheduleUploadedFlag = 0; // reset the flag so any upload has to start fresh
        aCtrl.Cloud_Debug_Print("Incorrect Upload Order!");
        return 404;
    }
    return 200;
}


int clearArio(String clearCmd) {
    if(clearCmd == "EEPROM"){
        EEPROM.clear();
        EEPROM.write(FACTORY_TEST_MODE_ADDR, 1); // Must write here otherwise the lamp would enter factory mode upon reboot
        factoryMode = FALSE;
        aCtrl.Load_RTC_Schedule();
        aCtrl.Cloud_Debug_Print("EEPROM Cleared!");
        //set EEPROM FACTORY MODE ADDRESS TO TRUE; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //} else if(clearCmd == "WIFICRED"){
    //    Particle.publish("WiFi Credentials Clearing!");
    //    WiFi.clearCredentials();
    } else if(clearCmd == "SCHEDULE"){ // returns to default schedule
        EEPROM.write(SCHEDULE_SELECT_ADDR, 0xFF);
        aCtrl.Load_RTC_Schedule();
    } else if(clearCmd == "FACTORY"){ // complete factory reset
        EEPROM.clear();
        aCtrl.Load_RTC_Schedule();
    } else if(clearCmd == "WDD"){ // testing
      EEPROM.write(FACTORY_TEST_MODE_ADDR, 1); // Must write here otherwise the lamp would enter factory mode upon reboot
      factoryMode = FALSE;
      Serial.write("Here!");
    }
    return 200;
}

void raiseHand() {
    if(Particle.connected()){ // so it doesn't trigger any offline logging when calling Report_to_Cloud()
        aCtrl.Report_to_Cloud("raiseHand", "true");
        RGB.control(TRUE);
        RGB.color(255,220,0);
        delay(2000);
        RGB.control(FALSE);
    }
}

void reportMacAddress(){
    if(Particle.connected()){ // so it doesn't trigger any offline logging when calling Report_to_Cloud()
        char macString[17];
        byte mac[6];
        WiFi.macAddress(mac);
        sprintf(macString,"%02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        aCtrl.Report_to_Cloud("production", macString);
        RGB.control(TRUE);
        RGB.color(255,105,180);
        delay(2000);
        RGB.control(FALSE);
    }
}


int measureAmbient(String ambCmd){ ///////////////////////////////////////////////////////////////////////////
    if(SENSOR_ALS_AVAILABLE && RGB.controlled() && !aCtrl.cctMode){
        char ambientStr[10];
        double val = 0;
        for(int i = 0; i < 100; i++){
            val += analogRead(PIN_SENSOR_ALS);
            delay(5);
        }
        int ambVal = val/100;
        sprintf(ambientStr,"%d",ambVal);
        aCtrl.Report_to_Cloud("ambient", ambientStr);
    }
    return 200;
}


void timeCheck(){
    if(millis() - lastTimeCheck >= TIME_CHECK_PERIOD) {
        lastTimeCheck = millis();

        aCtrl.Set_TimeZone();

        currentTime = Time.local();
        // RAM comparison
        if(currentTime < lastTime){
            aCtrl.Report_to_Cloud("time", "Time Sync RAM");
        }
        lastTime = currentTime;
        // EEPROM comparison
        unsigned long eepromTimeMarker;
        EEPROM.get(DST_CHECKER_TIME_MARK, eepromTimeMarker);
        if(currentTime < eepromTimeMarker){
            aCtrl.Report_to_Cloud("time", "Time Sync EEPROM");
        }
        EEPROM.put(DST_CHECKER_TIME_MARK, currentTime);
    }
}
