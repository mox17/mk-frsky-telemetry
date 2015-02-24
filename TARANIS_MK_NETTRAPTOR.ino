/*
Author George Chatzisavvidis.
Projects that used for creation of Taranis-Mikrokopter project was
Altastation and MavlinkFrsky.

Modified by Erling Stage.
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "SoftwareSerialWithHalfDuplex.h"
#include "FrSkySPort.h"
#include "altapix.h"
#include "altatools.c"

// Pins usage
// Pin 9   SPort to X8R (or X4R) receiver
// Pin 10  Debug RX pin
// Pin 11  Debug TX pin
// Pin 13  Activity LED (built in on arduino pro mini board)

//#define debugSerial       Serial2
SoftwareSerialWithHalfDuplex debugSerial(10, 11); // no input expected

#define MKSerial            Serial
#define START               1
#define MSG_RATE            10              // Hertz


typedef enum MK_Command {
    MK_Request_OSD_Data, 
    MK_Version_Request, 
    MK_Request_Display
} MK_Command_t;

void makeCmdString(MK_Command_t typeCmd, char *cmdStringRequest);


// ******************************************
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;

// Message # 1  SYS_STATUS 
uint16_t   ap_voltage_battery = 0;    // 1000 = 1V
int16_t    ap_current_battery = 0;    //  10 = 1A

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 0;            //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;        // numbers of visible satelites
// FrSky Taranis uses the first recieved lat/long as homeposition. 
int32_t    ap_latitude = 0;           // 585522540;
int32_t    ap_longitude = 0;          // 162344467;
int32_t    ap_gps_altitude = 0;       // 1000 = 1m

// Message #74 VFR_HUD 
//int32_t    ap_airspeed = 0;
uint32_t   ap_groundspeed = 0;
uint32_t   ap_heading = 0;
uint16_t   ap_throttle = 0;

// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
int32_t    ap_bar_altitude = 0;    // 100 = 1m
int32_t    ap_climb_rate = 0;      // 100= 1m/s

// Message #27 RAW IMU 
int32_t    ap_accX = 0;
int32_t    ap_accY = 0;
int32_t    ap_accZ = 0;

int32_t    ap_accX_old = 0;
int32_t    ap_accY_old = 0;
int32_t    ap_accZ_old = 0;

// ******************************************
// These are special for FrSky
//int32_t    adc2 = 0;               // 100 = 1.0V
//int32_t    vfas = 0;               // 100 = 1.0V
int32_t    gps_status = 0;         // (ap_sat_visible * 10) + ap_fixtype
                                   // ex. 83 = 8 satellites visible, 3D lock 
uint8_t    ap_cell_count = 4;      // This is the norm for Mikrokopter

#define led 13

///////////////////////////////
char commandLine[maxFrameLen+1];
char cmdStringRequest[30];
unsigned char rawDataDecoded[200];
int VersionReceived = 0; // no version seen yet
int statusReadCommandLine = 0;
 
int nbCrcError = 0;
int sendMKCmd = 1;  // Shall we send a command to MK at next opportunity
MK_Command_t cmdMK = MK_Request_OSD_Data;


void decodeOSD()
{
    s_MK_NaviData *NaviData;

    sendMKCmd = 0; // No need to send more commands for this, now that we have a response.
    decode64(commandLine,rawDataDecoded,strlen(commandLine)); //? add decode status..? 

    NaviData = (s_MK_NaviData*)&rawDataDecoded;
 
    ap_gps_altitude              = NaviData->CurrentPosition.Altitude;
    gps_status                   = NaviData->CurrentPosition.Status; //  GPS STATUS 0 or 1 
    //ap_gps_altitude            = NaviData->Altimeter;
    ap_longitude                 = NaviData->CurrentPosition.Longitude;
    ap_latitude                  = NaviData->CurrentPosition.Latitude;
    //ap_voltage_battery         = NaviData->UBat/10;
    ap_voltage_battery           = NaviData->UBat;
    //ap_bar_altitude            = NaviData->Altimeter / MK_ALTI_FACTOR;
    ap_bar_altitude              = NaviData->Altimeter;
    //ap_groundspeed             = NaviData->GroundSpeed/100;
    ap_groundspeed               = NaviData->GroundSpeed;
    //ap_current_battery         = NaviData->Current/10;
    ap_current_battery           = NaviData->Current;
    ap_current_battery           = NaviData->Current*10;
    ap_sat_visible               = NaviData->SatsInUse;
    ap_heading                   = NaviData->CompassHeading;
    ap_climb_rate                = NaviData->Variometer;
    ap_custom_mode               = NaviData->UsedCapacity;
    //ap_throttle                = NaviData->Gas;
    ap_throttle                  = NaviData->SatsInUse;
    ap_base_mode                 = NaviData->Errorcode;

    if (NaviData->SatsInUse >= 6) {
        ap_fixtype=3;
    }
}

void decodeVersion()
{
    str_VersionInfo *VersionInfo;

    decode64(commandLine, rawDataDecoded, strlen(commandLine)); //? add decode status..? 
    VersionInfo = (str_VersionInfo *)&rawDataDecoded;
    char line1[20];
    char line2[20];

    sprintf(line1, "Version ");
    sprintf(line2, "Nc: %u.%u", VersionInfo->SWMajor, VersionInfo->SWMinor);
    debugSerial.println(line1);
    debugSerial.println(line2);

    sendMKCmd = 1; // tell MK to send dataRequest
    VersionReceived = 1;
    cmdMK = MK_Request_Display;
}

void decodeLCD()
{
    s_MK_LcdScreen *LcdScreen;        
    char gpsInfo[11];
    int menuIndex; 

    decode64(commandLine, rawDataDecoded, strlen(commandLine)); //? add decode status..? 
    //memcpy((unsigned char *)&LcdScreen, (unsigned char *)&rawDataDecoded, sizeof(LcdScreen));
    LcdScreen = (s_MK_LcdScreen *)&rawDataDecoded;
    menuIndex = LcdScreen->Menuitem;        
    if (menuIndex == 2) {
        extractGpsInfo(LcdScreen->DisplayText, gpsInfo);
        debugSerial.print("gpsInfo=");
        debugSerial.println(gpsInfo);
        cmdMK = MK_Request_OSD_Data;
    }
}

void setup()
{  
    debugSerial.begin(57600);
    FrSkySPort_Init();
    MKSerial.begin(57600);
    pinMode(led, OUTPUT);
    analogReference(DEFAULT);
 
    redirectUartNc();
    cmdMK = MK_Request_OSD_Data; // Set 1st type of data to request from MK
    debugSerial.print("Hello world\n");
}


void loop() 
{
    FrSkySPort_Process();
  
    //sprintf(cmdName, "OSD");
    if (sendMKCmd){ // we dont need to send the command to mk on each loop, the mk is instructed to output data on regular basis    
        // we need to fetch OSD data  
        if (!VersionReceived) {
            cmdMK = MK_Version_Request; //MK_Request_OSD_Data;
        }

        makeCmdString(cmdMK, cmdStringRequest);
        MKSerial.println(cmdStringRequest);
        debugSerial.println(cmdStringRequest);
    }

    // Get MK data (non blocking)
    statusReadCommandLine = readMKdata(commandLine);
    if (statusReadCommandLine == 1) {
        // we have a frame lets check CRC
        int crcStatus = checkCRC(commandLine, strlen(commandLine));
        if (!crcStatus) {
            if (nbCrcError > 10) {
                debugSerial.println("D: BrokenCRC");
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
                //sprintf(cmdName,"D2"); // ask for "display" content
            } else if (dataType == 'V') { // we have VERSION data we need to display version and set version check and maybe check the version compatibility
                decodeVersion();
            } else if (dataType == 'L') { 
                decodeLCD();
            }
        }
    } else if (statusReadCommandLine == 2) {
        if (DEBUGME) {
            debugSerial.println("D: Frame Broken");
            sendMKCmd = 0; // we set it to 0 to force the commandIssue on the next Loop
        }
    } else if (statusReadCommandLine == 3) {
        if (DEBUGME) {
            debugSerial.println("D: Frame TimeOut");
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
    MKSerial.print(0x1b);
    MKSerial.print(0x1b);
    MKSerial.print(0x55);
    MKSerial.print(0xaa);
    MKSerial.print(0x00);
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
        toEncode[0]  = 200; // Interval of refresh of the OSD data in 10ms unit. approx 2000ms
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
                    charIndex = 0;
                    ret = 0; // partial data
                    digitalWrite(led, HIGH);
                }
            } 
            break;
        
        case expect_end :
            if (MKSerial.available() > 0) {
                incomingChar = MKSerial.read();
                if (incomingChar != '\r') {
                    commandLine[charIndex++] = incomingChar;
                    if (charIndex > maxFrameLen) {
                        ret = 2; // broken frame
                        read_state = expect_start;
                        commandLine[0] = 0;
                        digitalWrite(led, LOW);
                        return ret;
                    }
                } else {
                    ret = 1; // full frame
                    read_state = expect_start;
                    digitalWrite(led, LOW);
                    return ret;
                }
            } else {
                if (timeNow - timeStart > SERIAL_READ_TIMEOUT) {
                    ret = 3; // timeout
                    read_state = expect_start;
                    commandLine[0] = 0;
                    digitalWrite(led, LOW);
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


