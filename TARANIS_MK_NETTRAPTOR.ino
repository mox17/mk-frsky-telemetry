/*
Author George Chatzisavvidis.
Projects that used for creation of Taranis-Mikrokopter project was
Altastation and MavlinkFrsky.

Modified by Erling Stage.
*/
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "HardwareSerial.h"
#include "FrSkySPort.h"
#include "altapix.h"
#include "altatools.c"

// Pins usage
#define led 13  // when sending FrSKY data
#define mkPin 2 // when processing

#define xDEBUG_TRACES

#if defined(DEBUG_TRACES)
#define debugSerial            Serial3
#define DEBUG_PRINT(arg)       debugSerial.print(arg)
#define DEBUG_PRINT2(arg,a2)   debugSerial.print(arg,a2)
#define DEBUG_PRINTLN(arg)     debugSerial.println(arg)
#define DEBUG_PRINTF1(arg,p1)  debugSerial.printf(arg,p1)
//#define PRINTNZ(v)           do {if (v) { debugSerial.print(#v ": "); debugSerial.println(v); } } while(0);
#define PRINTNZ(v)
#else
#define DEBUG_PRINT(arg)
#define DEBUG_PRINT2(arg,a2)
#define DEBUG_PRINTLN(arg)  
#define DEBUG_PRINTF1(arg,p1)
#define PRINTNZ(v)
#endif

#define MKSerial            Serial2

typedef enum MK_Command {
    MK_Request_OSD_Data, 
    MK_Version_Request, 
    MK_Request_Display
} MK_Command_t;

void makeCmdString(MK_Command_t typeCmd, char *cmdStringRequest);
void redirectUartNc(void);
int readMKdata(char *commandLine);
char readDataType(char *mkLine);

// ******************************************
uint8_t    mk_error_code = 0;
uint32_t   mk_used_capacity = 0;

// Message # 1  SYS_STATUS 
uint16_t   mk_voltage_battery = 0;    // 1000 = 1V
int16_t    mk_current_battery = 0;    //  10 = 1A

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 0;            // 0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    mk_sat_visible = 0;        // numbers of visible satelites
// FrSky Taranis uses the first recieved lat/long as homeposition. 
int32_t    mk_latitude = 0;           // 585522540;
int32_t    mk_longitude = 0;          // 162344467;
int32_t    mk_gps_altitude = 0;       // 1000 = 1m

// Message #74 VFR_HUD 
//int32_t    ap_airspeed = 0;
uint32_t   mk_groundspeed = 0;
uint32_t   mk_compass_heading = 0;
uint16_t   ap_throttle = 0;

// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
int32_t    mk_bar_altitude = 0;    // 100 = 1m
int32_t    mk_climb_rate = 0;      // 100 = 1m/s
int32_t    mk_operating_radius = 0; // m  (fixed at 250 for "normal" license?)

// RAW IMU 
int32_t    ap_accX = 0;
int32_t    ap_accY = 0;
int32_t    ap_accZ = 0;

// These are special for FrSky
//int32_t    adc2 = 0;               // 100 = 1.0V
//int32_t    vfas = 0;               // 100 = 1.0V
int32_t    gps_status = 0;         // (mk_sat_visible * 10) + ap_fixtype
                                   // ex. 83 = 8 satellites visible, 3D lock 
uint8_t    ap_cell_count = 4;      // This is the norm for Mikrokopter


int VersionReceived = 0; // no version seen yet
int mk_success = 0;
int statusReadCommandLine = 0;
char commandLine[maxFrameLen+1];
char cmdStringRequest[30];
unsigned char rawDataDecoded[500];
int nbCrcError = 0;
int sendMKCmd = 1;  // Shall we send a command to MK at next opportunity
MK_Command_t cmdMK = MK_Request_OSD_Data;


void decodeOSD()
{
    s_MK_NaviData *NaviData;
    digitalWrite(mkPin, HIGH);
    decode64(commandLine, rawDataDecoded, strlen(commandLine));
    NaviData = (s_MK_NaviData*)&rawDataDecoded;

    if (NaviData->SatsInUse >= 6) {
        ap_fixtype=3;
    }

    //GPS_Pos_t    CurrentPosition;	// see ubx.h for details
    mk_longitude                 = NaviData->CurrentPosition.Longitude;
    PRINTNZ(mk_longitude);
    mk_latitude                  = NaviData->CurrentPosition.Latitude;
    PRINTNZ(mk_latitude);
    mk_gps_altitude              = NaviData->CurrentPosition.Altitude; // mm
    PRINTNZ(mk_gps_altitude);
    gps_status                   = NaviData->CurrentPosition.Status; //  GPS STATUS 0 or 1 
    PRINTNZ(gps_status);
    //---GPS_Pos_t    TargetPosition;
    //---GPS_PosDev_t TargetPositionDeviation;
    //---GPS_Pos_t    HomePosition;
    //---GPS_PosDev_t HomePositionDeviation;
    //---uint8_t      WaypointIndex;    // index of current waypoints running from 0 to WaypointNumber-1
    //---uint8_t      WaypointNumber;   // number of stored waypoints
    mk_sat_visible               = NaviData->SatsInUse; // number of satellites used for position solution
    PRINTNZ(mk_sat_visible);
    mk_bar_altitude              = NaviData->Altimeter; // height according to air pressure in cm
    PRINTNZ(mk_bar_altitude);
    mk_climb_rate                = NaviData->Variometer; // climb(+) and sink(-) rate
    PRINTNZ(mk_climb_rate);
    //---uint16_t     FlyingTime;		// in seconds
    mk_voltage_battery           = NaviData->UBat; // Battery Voltage in 0.1 Volts
    PRINTNZ(mk_voltage_battery);
    mk_groundspeed               = NaviData->GroundSpeed; // speed over ground in cm/s (2D)
    PRINTNZ(mk_groundspeed );
    //---int16_t      Heading;		// current flight direction in ° as angle to north
    mk_compass_heading           = NaviData->CompassHeading; // current compass value in °
    PRINTNZ(mk_compass_heading);
    //---int8_t       AngleNick;		// current Nick angle in 1°
    //---int8_t       AngleRoll;		// current Rick angle in 1°
    //---uint8_t      RC_Quality;		// RC_Quality
    //---uint8_t      FCFlags;		// Flags from FC
    //---uint8_t      NCFlags;		// Flags from NC
    mk_error_code                = NaviData->Errorcode; // 0 --> okay
    PRINTNZ(mk_error_code);
    mk_operating_radius          = NaviData->OperatingRadius; // current operation radius around the Home Position in m
    PRINTNZ(mk_operating_radius);
    //---int16_t      TopSpeed;		// velocity in vertical direction in cm/s
    //---uint8_t      TargetHoldTime;	// time in s to stay at the given target, counts down to 0 if target has been reached
    //---uint8_t      RC_RSSI;		// Receiver signal strength (since version 2 added)
    //---int16_t      SetpointAltitude;	// setpoint for altitude
    //---uint8_t      Gas;			// for future use
    mk_current_battery           = NaviData->Current; // actual current in 0.1A steps
    PRINTNZ(mk_current_battery);
    mk_used_capacity             = NaviData->UsedCapacity; // used capacity in mAh
    PRINTNZ(mk_used_capacity); 
    digitalWrite(mkPin, LOW);
    mk_success = 1;
}

void decodeVersion()
{
    str_VersionInfo *VersionInfo;

    decode64(commandLine, rawDataDecoded, strlen(commandLine)); //? add decode status..? 
    VersionInfo = (str_VersionInfo *)&rawDataDecoded;
    char line2[20];

    DEBUG_PRINTLN( "Version ");
    sprintf(line2, "Nc: %u.%u", VersionInfo->SWMajor, VersionInfo->SWMinor);
    DEBUG_PRINTLN(line2);
}

void decodeLCD()
{
    DEBUG_PRINTLN("decodeLCD()");
    s_MK_LcdScreen *LcdScreen;        
    char gpsInfo[11];
    int menuIndex; 

    decode64(commandLine, rawDataDecoded, strlen(commandLine)); //? add decode status..? 
    LcdScreen = (s_MK_LcdScreen *)&rawDataDecoded;
    menuIndex = LcdScreen->Menuitem;        
    if (menuIndex == 2) {
        extractGpsInfo(LcdScreen->DisplayText, gpsInfo);
        DEBUG_PRINTLN("gpsInfo=");
        DEBUG_PRINTLN(gpsInfo);
    }
}

void setup()
{
#if defined(DEBUG_TRACES)
    debugSerial.begin(115200);
    debugSerial.print("\e[2JMikrokopter-FrSKY telemetry gateway\r\n");
#endif
    FrSkySPort_Setup();
    FrSkySPort_Init();
    MKSerial.begin(57600);
    pinMode(led, OUTPUT);
    pinMode(mkPin, OUTPUT);
    //analogReference(DEFAULT);
 
    redirectUartNc();
    cmdMK = MK_Request_OSD_Data; // Set 1st type of data to request from MK
}


void loop() 
{
    static unsigned long lastCMdTime=0;
    unsigned long timeNow;

    FrSkySPort_Process();  // Scan for RX polling messages
  
    if (sendMKCmd){ // we dont need to send the command to mk on each loop, the mk is instructed to output data on regular basis    
        // we need to fetch OSD data  
        if (!VersionReceived) {
            cmdMK = MK_Version_Request;
        }
        
        timeNow = millis();
        if (timeNow - lastCMdTime > 3000) {  // MK repeating OSD subscrition lasts 4 seconds
            lastCMdTime = timeNow;
            makeCmdString(cmdMK, cmdStringRequest);
            if (mk_success == 0) {
                redirectUartNc();
            }
            MKSerial.write(cmdStringRequest);
            //DEBUG_PRINTLN(cmdStringRequest);
        }
    }

    statusReadCommandLine = readMKdata(commandLine);  // Get MK data (non blocking)
    if (statusReadCommandLine == 1) {
        // we have a frame lets check CRC
        //DEBUG_PRINTLN("=");
        int crcStatus = checkCRC(commandLine, strlen(commandLine));
        if (!crcStatus) {
            if (nbCrcError > 10) {
                DEBUG_PRINTLN("D: BrokenCRC");
                nbCrcError = 0;
            }
            sendMKCmd = 1; // we set it to 0 to force the commandIssue on the next Loop
        } else {
            nbCrcError = 0;
            // we have a frame with valid CRC lets check the type.
            char dataType = readDataType(commandLine);   

            if (dataType == '\0') {
                sendMKCmd = 1; // force new command in the next Loop
            } else if (dataType == 'O') { // we have OSD data
                decodeOSD();
                cmdMK = MK_Request_OSD_Data;
            } else if (dataType == 'V') { // we have VERSION data we need to display version and set version check and maybe check the version compatibility
                decodeVersion();
                VersionReceived = 1;
                sendMKCmd = 1; // tell MK to send dataRequest
                cmdMK = MK_Request_Display;
            } else if (dataType == 'L') { 
                decodeLCD();
                cmdMK = MK_Request_OSD_Data;
            }
        }
    } else if (statusReadCommandLine == 2) {
        if (DEBUGME) {
            DEBUG_PRINTLN("D: Frame Broken");
            sendMKCmd = 0; // we set it to 0 to force the commandIssue on the next Loop
        }
    } else if (statusReadCommandLine == 3) {
        if (DEBUGME) {
            DEBUG_PRINTLN("D: Frame TimeOut");
            sendMKCmd = 0; // we set it to 0 to force the commandIssue on the next Loop 
        }
    }
    FrSkySPort_Process(); 
}

/**
 * Select data from MK Navi board. See http://wiki.mikrokopter.de/en/SerialCommands
 * Output can switched back to NC debug by sending the magic packet "0x1B,0x1B,0x55,0xAA,0x00"
 */
void redirectUartNc(void)
{
    MKSerial.write(0x1b);
    MKSerial.write(0x1b);
    MKSerial.write(0x55);
    MKSerial.write(0xaa);
    MKSerial.write(0x00);
    delay(50);
}

/**
 * Build a command to MikroKopter. 
 * @param typeCmd the command to be sent
 * MK_Request_OSD_Data : A repeating transmission of NaviDataStruct (s_MK_NaviData).
 * MK_Version_Request  : Version information in a VersionStruct (str_VersionInfo).
 * MK_Request_Display  : Menu display (s_MK_LcdScreen).
 * @param cmdStringRequest is an output buffer.
 */
void makeCmdString(MK_Command_t typeCmd, char *cmdStringRequest)
{
    char cmdData[20];
    char toEncode[1];
    char cmdFrame[30];
    char cmdHeader[5];
    char CRC[5];

    *cmdStringRequest = 0; // start with empty string.
    if (typeCmd == MK_Request_OSD_Data) {
        // We want osdData
        //#cO=DzdMHTAAe[ype>==A=================================s]A=====eM====@?====n==>=I]=====s]AR>M@S=M==xe
        cmdHeader[0] = '#'; // frame start
        cmdHeader[1] = 'a' + NC_ADDRESS; // navi Addr = c
        cmdHeader[2] = 'o'; // osdData = o
        cmdHeader[3] = '\0';
        toEncode[0]  = 10; // Interval of refresh of the OSD data in 10ms unit. approx 2000ms
        encode64(toEncode, cmdData, sizeof(toEncode));
        sprintf(cmdFrame, "%s%s", cmdHeader, cmdData);
        addCRC(cmdFrame, CRC);
        sprintf(cmdStringRequest, "%s%s%s", cmdHeader, cmdData, CRC);
    } else if (typeCmd == MK_Version_Request) {
        // We want a VersionStruct
        // Doesn't matter which board we address this request to.
        cmdHeader[0] = '#'; // frame start
        cmdHeader[1] = 'a'; // navi Addr = C
        cmdHeader[2] = 'v'; // osdData = o
        cmdHeader[3] = '\0';
        sprintf(cmdFrame, "%s", cmdHeader);
        addCRC(cmdFrame, CRC);
        sprintf(cmdStringRequest, "%s%s", cmdHeader, CRC);
    } else if (typeCmd == MK_Request_Display) {
        cmdHeader[0]='#'; // frame start
        cmdHeader[1]='a'; // navi Addr = C
        cmdHeader[2]='l'; // osdData = o
        cmdHeader[3]='\0';
        toEncode[0] = 2;
        encode64(toEncode, cmdData, sizeof(toEncode));
        sprintf(cmdFrame, "%s%s", cmdHeader, cmdData);
        addCRC(cmdFrame, CRC);
        sprintf(cmdStringRequest, "%s%s%s", cmdHeader, cmdData, CRC);
    }
}


typedef enum mk_read_state {expect_start, expect_end} mk_read_state_t;

/**
 * Read data from MikroKopter.
 * Always starting with '#' (which is not present in data) and always ending with '\r'.
 * @return 0 == partial, 1 == full frame, 2 == broken frame, 3 == timeout,
 */
int readMKdata(char *commandLine)
{
    static mk_read_state_t read_state = expect_start;
    static unsigned long timeStart=0;
    static int charIndex = 0;
    unsigned long timeNow = millis();
    int ret = 0;
    char incomingChar = 0;

    do {
        switch (read_state) {
        case expect_start :
            if (MKSerial.available() > 0) {
                incomingChar = MKSerial.read();
                if (incomingChar == '#') {
                    read_state = expect_end;
                    timeStart = timeNow;
                    commandLine[0] = incomingChar;
                    charIndex = 1;
                    ret = 0; // partial data
                    //digitalWrite(led, HIGH);
                }
            } 
            break;
        
        case expect_end :
            if (MKSerial.available() > 0) {
                incomingChar = MKSerial.read();
                if (incomingChar != '\r') {
                    commandLine[charIndex++] = incomingChar;
                    if (charIndex > maxFrameLen) {
                        read_state = expect_start;
                        commandLine[0] = 0;
                        //digitalWrite(led, LOW);
                        return 2; // broken frame
                    }
                } else {
                    read_state = expect_start;
                    //digitalWrite(led, LOW);
                    return 1; // full frame
                }
            } else {
                if (timeNow - timeStart > SERIAL_READ_TIMEOUT) {
                    read_state = expect_start;
                    commandLine[0] = 0;
                    //digitalWrite(led, LOW);
                    return 3; // timeout
                }
            }
            break;
        }
    } while (MKSerial.available() > 0);
    return ret;
}
    

/**
 * Classify type of MK response.
 * 'D' : DebugOutStruct
 * 'L' : Request Display 
 * 'O' : OSD data NaviDataStruct
 * 'l' : ??? not documented
 * 'V' : VersionStruct
 * 'H' : char[80] DisplayText
 */
char readDataType(char *mkLine)
{
    char dataType = mkLine[2];

    if (strchr("DLOlVH", dataType) != NULL) {
        return dataType;
    }
    return '\0';
}


