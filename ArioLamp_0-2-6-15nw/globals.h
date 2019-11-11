/************************************************************************************************************************************/
/** @file		globals.h
 * 	@brief		contains all the high level protos and defs used by all modules of the firmware
 * 	@details	includes sizes, addresses, timing and IO Definitions
 *
 * 	@author		Shaw-Pin Chen, Product Lead, Ario, Inc.
 * 	@created	09-15-16
 * 	@last rev	04-18-17
 *
 * 	@section	Legal Disclaimer
 * 			All contents of this source file and/or any other Ario, Inc. related source files are the explicit property of
 *          Ario, Inc. Do not distribute. Do not copy.
 */
/************************************************************************************************************************************/

#ifndef globals_h
#define globals_h
#include "application.h"
#include "ario_ctrlG.h"


// SYSTEM VARIABLES----------------------------------------------------------------------------------------------------------------//
#define SENSOR_ALS_AVAILABLE    TRUE // Ambient light sensor (analog readout)
#define SENSOR_PIR_AVAILABLE    TRUE // Proximity detector (digital readout)
#define EXT_MEM_AVAILABLE       TRUE // True ONLY for Particle P1


// PIN MAPPING----------------------------------------------------------------------------------------------------------------------//
#define PIN_BUTTON_TOP          (D7)//(D7)
#define PIN_BUTTON_MIDDLE       (D5)//(D5)
#define PIN_BUTTON_BOTTOM       (D3)//(D3)
#define PIN_SENSOR_ALS          (A3)
#define PIN_SENSOR_PIR          (D6)


// LOGIC----------------------------------------------------------------------------------------------------------------------------//
#define ON  1
#define OFF 0


// TIME & PARAMETERS------------------------------------------------------------------------------------------------------------------//
#define RAMP_DELAY              5UL
#define ONE_HOUR                3600000UL
#define HALF_HOUR               1800000UL
#define FIFTEEN_MINUTES         900000UL
#define ONE_MINUTE              60000UL

#define CLOUD_REPORT_DELAY      5000UL  // delay time it takes for the lamp to report user button light adjustment
#define CCT_MODE_TIMEOUT        10000UL // after user enter CCT mode and no action, mode times out
#define STATUS_LED_TIMEOUT      6000UL  // after no button action status indicator LED times out
#define ENTER_WIFI_PAIRING_TIME 15000UL
#define MODE_CHANGE_FADE_TIME   15000UL

#define PIR_STABLE_TIME             30000UL // this is fixed based on component manufacturer
#define PIR_OFF_HOLD_DELAY          ONE_MINUTE // time before PIR is allowed to turn lamp back on if enabled after user turned lamp off
#define PIR_REPORT_PERIOD           ONE_MINUTE*10 // reports presence detection
#define ALS_MEASURE_PERIOD          ONE_MINUTE
#define ALS_REPORT_PERIOD           HALF_HOUR
#define ALS_SAMPLE_INTERVAL         10UL
#define ALS_SAMPLES_NUMBER          100
#define ALS_SENSITIVITY_DEFAULT     ALS_SENSITIVITY_HIGH
#define ALS_SENSITIVITY_HIGH        16 // 4080/255
#define ALS_SENSITIVITY_MEDIUM      12 // 3060/255
#define ALS_SENSITIVITY_LOW         8  // 2040/255


// USER INTERACTION TYPE------------------------------------------------------------------------------------------------------------//
#define INTERACTION_TYPE_BTN    0x0
#define INTERACTION_TYPE_WEB    0x1
#define INTERACTION_TYPE_PIR    0x2


// COMMUNICATION & LAMP CONFIGURATIONS----------------------------------------------------------------------------------------------//
#define I2C_SPEED       400000
#define UART_BAUD       9600

#define NUM_LED_CH      4
#define PSOC_ADDR       1      // I2C slave address of PSoC
#define NUM_BYTES_READ  6      // Number of bytes to read from PSoC
#define NUM_BYTES_WRITE 6      // Number of bytes to write to PSoC

#define MAX_CCT_UPPER_LIMIT 6500
#define MAX_CCT_LOWER_LIMIT 4000
#define CCT_6500            6500
#define CCT_4000            4000
#define CCT_1800            1800 // need to change the naming for this
#define MIN_CCT             300
#define MAX_BRIGHTNESS      255
#define MIN_BRIGHTNESS      0
#define FACTORY_HOLD_TIME   60 // in minutes


// LAMP OPERATING MODES-------------------------------------------------------------------------------------------------------------//
#define MODE_DEFAULT    0
#define MODE_DEMO       1
#define MODE_ADJUST     2
#define MODE_DAWNSIM    3
#define MODE_BEDTIME    4
#define MODE_RAMP       5


// CLOUD COMMAND CODE (TO BE RETIRED SOON-------------------------------------------------------------------------------------------//
#define TURN_OFF        (0xA0)
#define TURN_ON         (0xA1)
#define DEMO_ON         (0xA2)
#define APP_BRIGHTNESS_DOWN (0x10)
#define APP_BRIGHTNESS_UP   (0x11)
#define APP_CCT_DOWN (0x12)
#define APP_CCT_UP   (0x13)


// LAMP SETTINGS EEPROM ADDRESS SPACE-----------------------------------------------------------------------------------------------//
// Basic Lamp Settings
#define FACTORY_TEST_MODE_ADDR              (0x000) // 255 means factory test mode. currently only used for PIR and ALS test
#define CURRENT_VERSION_ADDR                (0x001)
#define DST_ENABLE_ADDR                     (0x00A) // Boolean, disbaled by default
#define USER_TIME_ZONE                      (0x003) // -8 Pacific Time by default
#define HOLD_TIME_DURTION_ADDR              (0x004) // 60 minutes by default
#define MAX_CCT_ADDR                        (0x005) // 6500(K) by default, reserve 2 bytes

#define SCHEDULE_SELECT_ADDR                (0x007) // 1: user schedule 1, 0: user schedule 2, anything else: factory default schedule
#define CLOUD_DEBUG_ADDR                    (0x009) // false by default

// Wake Up Alarms & Bedtime Reminders
#define WAKEUP_ALARM_ENABLE_BASE_ADDR       (0x010) // Boolean, disbaled by default
#define WAKEUP_ALARM_HOUR_BASE_ADDR         (0x020) // in hours, zero pad to 2 digits
#define WAKEUP_ALARM_MINUTE_BASE_ADDR       (0x030) // in minutes, zero pad to 2 digits
#define WAKEUP_ALARM_DURATION_BASE_ADDR     (0x040) // in minutes, no need to zero pad
#define BEDTIME_ALARM_ENABLE_BASE_ADDR      (0x050) // Boolean, disbaled by default
#define BEDTIME_ALARM_HOUR_BASE_ADDR        (0x060) // in hours, zero pad to 2 digits
#define BEDTIME_ALARM_MINUTE_BASE_ADDR      (0x070) // in minutes, zero pad to 2 digits
#define BEDTIME_ALARM_DURATION_BASE_ADDR    (0x080) // in minutes, no need to zero pad

// PIR Sensor Address Space
#define PIR_ON_SET_ADDR                     (0x090) // Boolean
#define PIR_OFF_SET_ADDR                    (0x091) // Boolean
#define PIR_ON_DURATION                     (0x092) // in minutes, zero pad to 3 digits
#define PIR_SCHEDULE_EN_ADDR                (0x094) // Boolean
#define PIR_BEGIN_HOUR_ADDR                 (0x095) // in hours, zero pad to 2 digits
#define PIR_BEGIN_MINUTE_ADDR               (0x097) // in minutes, zero pad to 2 digits
#define PIR_END_HOUR_ADDR                   (0x099) // in hours, zero pad to 2 digits
#define PIR_END_MINUTE_ADDR                 (0x09A) // in minutes, zero pad to 2 digits

// Ambient Light Sensor Address Space
#define ALS_EN_ADDR                         (0x0A0) // Boolean
#define ALS_SENSITIVITY_ADDR                (0x0A1)

// DST Time Checker Storage
#define DST_CHECKER_TIME_MARK               (0x0A3) // unsigned long 4 bytes

// Custom Schedule Address Space
#define SCHEDULE_1_CCT_BASE_ADDR            (0x100) // Each value stored as 2 bytes
#define SCHEDULE_2_CCT_BASE_ADDR            (0x180) // Each value stored as 2 bytes
#define SCHEDULE_1_LEVEL_BASE_ADDR          (0x200) // Each value stored as 1 byte
#define SCHEDULE_2_LEVEL_BASE_ADDR          (0x280) // Each value stored as 1 byte

// No Web addition
#define OFFLINE_MODE_ADDR                    (0x008) // 1: offline mode engaged
#define NW_MODE_DEFAULT     0
#define NW_MODE_SET_CCT     1
#define NW_MODE_SET_WAKE    2
#define NW_MODE_SET_BED     3
#define NW_MODE_SET_PIR     4
#define NW_MODE_ALARM_SCH   5
#define NW_MODE_PIR_SCH     6
#define NW_MODE_SYNC_TIME   7
#define NW_MODE_TIMEOUT     10000UL


#endif
