/*
Author George Chatzisavvidis.
Projects that used for creation of Taranis-Mikrokopter project was
Altastation and MavlinkFrsky.

Modified by Erling Stage.
*/

#include <stdint.h>elsqe
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


// ******************************************
// Message # 0  HEARTBEAT 
//uint8_t    ap_type = 0;
//uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;
//uint8_t    ap_system_status = 0;
//uint8_t    ap_mavlink_version = 0;

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
//int32_t    vfas = 0;               // 100 = 1,0V
int32_t    gps_status = 0;         // (ap_sat_visible * 10) + ap_fixtype
                                   // ex. 83 = 8 satellites visible, 3D lock 
uint8_t    ap_cell_count = 4;      // This is the norm for Mikrokopter

// ******************************************
//uint8_t     MavLink_Connected;
//uint8_t     buf[MAVLINK_MAX_PACKET_LEN];

//uint16_t  hb_count;

//unsigned long MavLink_Connected_timer;
//unsigned long hb_timer;
//unsigned long acc_timer;

int led = 13;

///////////////////////////////
char commandLine[maxFrameLen+1];
//char debugString[200];
//int status = 0;
char cmdStringRequest[30];
unsigned char rawDataDecoded[200];
int checkVersionStatus = 0;
int statusReadCommandLine = 0;
 
int nbCrcError = 0;
int cmdSend = 0;
char cmdName[10];
char gpsInfo[11];
int menuIndex; 
//char menuName[25];


void setup()
{  
    debugSerial.begin(57600);
    FrSkySPort_Init();
    MKSerial.begin(57600);
    pinMode(led, OUTPUT);
    analogReference(DEFAULT);
 
    redirectUartNc(); 
    sprintf(cmdName, "OSD");  // Set 1st type of data to request from MK
}


void decodeOSD()
{
    s_MK_NaviData NaviData;
    GPS_Pos_t currpos;

    cmdSend = 1; // No need to send more commands for this, now that we have a response.
    decode64(commandLine,rawDataDecoded,strlen(commandLine)); //? add decode status..? 

    memcpy((unsigned char *)&NaviData, (unsigned char *)&rawDataDecoded, sizeof(NaviData));
 
    // cool we are done , we have osdData Struct an we can construct ANY gui we want .
    //-----------------PUSHING DATA FROM MK TO TARANIS------------------....
   
    currpos.Latitude             = NaviData.CurrentPosition.Latitude;
    currpos.Longitude            = NaviData.CurrentPosition.Longitude;
    ap_gps_altitude              = NaviData.CurrentPosition.Altitude;
    gps_status                   = NaviData.CurrentPosition.Status; //  GPS STATUS 0 or 1 
    //ap_gps_altitude            = NaviData.Altimeter;
    ap_longitude                 = currpos.Longitude;
    ap_latitude                  = currpos.Latitude;
    //ap_voltage_battery         = NaviData.UBat/10;
    ap_voltage_battery           = NaviData.UBat;
    //ap_bar_altitude            = NaviData.Altimeter / MK_ALTI_FACTOR;
    ap_bar_altitude              = NaviData.Altimeter;
    //ap_groundspeed             = NaviData.GroundSpeed/100;
    ap_groundspeed               = NaviData.GroundSpeed;
    //ap_current_battery         = NaviData.Current/10;
    ap_current_battery           = NaviData.Current;
    ap_current_battery           = NaviData.Current*10;
    ap_sat_visible               = NaviData.SatsInUse;
    ap_heading                   = NaviData.CompassHeading;
    ap_climb_rate                = NaviData.Variometer;
    ap_custom_mode               = NaviData.UsedCapacity;
    //ap_throttle                = NaviData.Gas;
    ap_throttle                  = NaviData.SatsInUse;
    ap_base_mode                 = NaviData.Errorcode;

    if (NaviData.SatsInUse >= 6) {
        ap_fixtype=3;
    }
}

void decodeVersion()
{
    decode64(commandLine, rawDataDecoded, strlen(commandLine)); //? add decode status..? 
    str_VersionInfo VersionInfo;
    memcpy((unsigned char *)&VersionInfo, (unsigned char *)&rawDataDecoded, sizeof(VersionInfo));
    char line1[20];
    char line2[20];

    sprintf(line1, "Version ");
    sprintf(line2, "Nc: %u.%u", VersionInfo.SWMajor, VersionInfo.SWMinor);
    //lcdClearLine(1);
    debugSerial.println(line1);
    debugSerial.println(line2);

    // delay(2000);
    cmdSend = 0; // tell him to resend dataRequest
    checkVersionStatus = 1; 
    sprintf(cmdName,"D2");
}

void decodeLCD()
{
    s_MK_LcdScreen LcdScreen;        

    decode64(commandLine, rawDataDecoded, strlen(commandLine)); //? add decode status..? 
    memcpy((unsigned char *)&LcdScreen, (unsigned char *)&rawDataDecoded, sizeof(LcdScreen));
    menuIndex = LcdScreen.Menuitem;        
    if (menuIndex == 2) {
        extractGpsInfo(LcdScreen.DisplayText, gpsInfo);
        debugSerial.print("gpsInfo=");
        debugSerial.println(gpsInfo);
        sprintf(cmdName, "OSD");
    }
}

void loop() 
{
    digitalWrite(led, LOW);
    //delay(10);  // ES why??
     
    FrSkySPort_Process(); // listen
    digitalWrite(led, HIGH);
  
    //sprintf(cmdName, "OSD");
    if (!cmdSend){ // we dont need to send the command to mk on each loop , the mk is instructed to output data on regular basis    
        // we need to fetch OSD data  
        if (!checkVersionStatus) {
            sprintf(cmdName,"OSD");
        }

        makeCmdString(cmdName, cmdStringRequest);
        MKSerial.println(cmdStringRequest);
        //debugSerial.println(cmdStringRequest);
        cmdSend = 1;
    }

    // MK is sending the infos for OSD we need to read them and build the GUI
    statusReadCommandLine = readCommandLine(commandLine);
    if (statusReadCommandLine == 1) {
        // we have a frame lets check CRC
        int crcStatus = checkCRC(commandLine,strlen(commandLine));
        if (!crcStatus) {
            if (nbCrcError > 10) {
                debugSerial.println("D: BrokenCRC");
                nbCrcError = 0;
            }
            cmdSend = 0; // we set it to 0 to force the commandIssue on the next Loop
        } else {
            nbCrcError = 0;
            // we have a frame with valid CRC lets check the type.
            char dataType = readDataType(commandLine);   

            if (dataType == '\0') {
                cmdSend = 0; // we set it to 0 to force the commandIssue on the next Loop
            } else if (dataType == 'O') { // we have OSD data its cool its what we want.
                decodeOSD();
                //delay(200);
                //sprintf(cmdName,"D2"); // ask for "display" content
            }
            else if (dataType == 'V') { // we have VERSION data we need to display version and set version check and maybe check the version compatibility
                decodeVersion();
            } 
            else if (dataType == 'L') { 
                decodeLCD();
            }
        }
    } else if (statusReadCommandLine == 2) {
        if (DEBUGME) {
            debugSerial.println("D: Frame Broken");
            cmdSend = 0; // we set it to 0 to force the commandIssue on the next Loop
        }
    } else if (statusReadCommandLine == 3) {
        if (DEBUGME) {
            debugSerial.println("D: Frame TimeOut");
            cmdSend = 0; // we set it to 0 to force the commandIssue on the next Loop 
        }
    }
    digitalWrite(led,HIGH);
    //PUSH TO TARANIS
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
 * 'OSD' :  a repeating transmission of NaviDataStruct (s_MK_NaviData).
 * 'VER' : Version information in a VersionStruct (str_VersionInfo).
 * 'D2'  : Menu display (s_MK_LcdScreen).
 * @param cmdStringRequest is an output buffer.
 */
void makeCmdString(char *typeCmd, char *cmdStringRequest)
{
    char cmdData[20];
    char toEncode[1];
    char cmdFrame[30];
    char cmdHeader[5];

    *cmdStringRequest = 0; // start with empty string.
    if (strcmp(typeCmd, "OSD") == 0) {
        // we want osdData
        //#cO=DzdMHTAAe[ype>==A=================================s]A=====eM====@?====n==>=I]=====s]AR>M@S=M==xe
        cmdHeader[0] = '#'; // frame start
        cmdHeader[1] = 'a' + NC_ADDRESS; // navi Addr = c
        cmdHeader[2] = 'o'; // osdData = o
        cmdHeader[3] = '\0';
        toEncode[0]  = 200; // Interval of refresh of the OSD data in 10ms unit. approx 2000ms
        encode64(toEncode, cmdData, sizeof(toEncode));
        sprintf(cmdFrame, "%s%s", cmdHeader, cmdData);
        char CRC[5];
        addCRC(cmdFrame, CRC);
        sprintf(cmdStringRequest, "%s%s%s", cmdHeader, cmdData, CRC);
    } else if (strcmp(typeCmd, "VER")  == 0) {
        // we want a VersionStruct
        // Doesn't matter which board we address this request to.
        cmdHeader[0] = '#'; // frame start
        cmdHeader[1] = 'a'; // navi Addr = C
        cmdHeader[2] = 'v'; // osdData = o
        cmdHeader[3] = '\0';
        sprintf(cmdFrame, "%s", cmdHeader);
        char CRC[5];
        addCRC(cmdFrame, CRC);
        sprintf(cmdStringRequest, "%s%s", cmdHeader, CRC);
    } else if (strcmp(typeCmd, "D2")  == 0) {
        // we want osdData
        cmdHeader[0]='#'; // frame start
        cmdHeader[1]='a'; // navi Addr = C
        cmdHeader[2]='l'; // osdData = o
        cmdHeader[3]='\0';
        toEncode[0] = 2;
        encode64(toEncode, cmdData, sizeof(toEncode));
        sprintf(cmdFrame, "%s%s", cmdHeader, cmdData);
        char CRC[5];
        addCRC(cmdFrame, CRC);
        sprintf(cmdStringRequest, "%s%s%s", cmdHeader, cmdData, CRC);
    }
}

/**
 * Read a line of data from MikroKopter.
 * The data is formatted in a flavor of base64.
 * Always starting with '#' (which is not present in data) and always ending with '\r'.
 */
int readCommandLine(char *commandLine)
{
    int hasLine = 0;
    int charIndex = 0;
    char incomingChar;
    const char startChar = '#';
    const char endChar = '\r';
    int inFrame = 0;
    int brokenFrame = 0;
    int timeOut = 0;
    unsigned long timeStart = millis();
    unsigned long timeNow = 0;
    while (hasLine == 0) {
        // implementer le timeout
        timeNow = millis();
        if (timeNow - timeStart > SERIAL_READ_TIMEOUT) {
            timeOut = 1;
            hasLine = 1;
        }
        // read the incoming byte:
        if (MKSerial.available() > 0) {
            incomingChar = MKSerial.read();
            if (incomingChar == startChar) {
                inFrame = 1;
            }
            if (inFrame && (incomingChar == endChar)) {
                inFrame = 0;
                hasLine = 1;
            }
            if (inFrame) {
                commandLine[charIndex] = incomingChar;
                charIndex++;
                if (charIndex > maxFrameLen) {
                    brokenFrame = 1;
                    hasLine=1;
                }
            }
        }
    }
    if (brokenFrame) {
        commandLine[0] = '\0';
        return 2;
    } else if (timeOut) {
        commandLine[0] = '\0';
        return 3;
    } else {
        commandLine[charIndex] = '\0';
    }
    return 1;
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


        //------------------------------------------------------------------------------------
        //--------------------------------MK DATA TO SEND-------------------------------------
        //------------------------------------------------------------------------------------
        //          GPS_Pos_t    CurrentPosition;		// see ubx.h for details
        //          GPS_Pos_t    TargetPosition;
        //          GPS_PosDev_t TargetPositionDeviation;
        //          GPS_Pos_t    HomePosition;
        //          GPS_PosDev_t HomePositionDeviation;
        //          WaypointIndex;		// index of current waypoints running from 0 to WaypointNumber-1
        //          WaypointNumber;		// number of stored waypoints
        //          SatsInUse;		// number of satellites used for position solution
        //          Altimeter; 		// hight according to air pressure
        //          Variometer;		// climb(+) and sink(-) rate
        //          FlyingTime;		// in seconds
        //          UBat;			// Battery Voltage in 0.1 Volts
        //          GroundSpeed;		// speed over ground in cm/s (2D)
        //          Heading;		// current flight direction in ° as angle to north
        //          CompassHeading;		// current compass value in °
        //          AngleNick;		// current Nick angle in 1°
        //          AngleRoll;		// current Rick angle in 1°
        //          RC_Quality;		// RC_Quality
        //          FCFlags;		// Flags from FC
        //          NCFlags;		// Flags from NC
        //          Errorcode;		// 0 --> okay
        //          OperatingRadius;               // current operation radius around the Home Position in m
        //          TopSpeed;		// velocity in vertical direction in cm/s
        //          TargetHoldTime;		// time in s to stay at the given target, counts down to 0 if target has been reached
        //          RC_RSSI;		// Receiver signal strength (since version 2 added)
        //          SetpointAltitude;			// setpoint for altitude
        //          Gas;						// for future use
        //          Current;					// actual current in 0.1A steps
        //          UsedCapacity;				// used capacity in mAh
        //
        
        
        //This is the data we send to FrSky, you can change this to have your own
        //set of data
        //******************************************************
        //Data transmitted to FrSky Taranis:
        //Cell          ( Voltage of Cell=Cells/4. [V] This is my LiPo pack 4S ) 
        //Cells         ( Voltage from LiPo [V] )
        //A2            ( Analog voltage from input A0 on Teensy 3.1 )
        //Alt           ( Altitude from baro.  [m] )
        //GAlt          ( Altitude from GPS   [m])
        //HDG           ( Compass heading  [deg])
        //Rpm           ( Throttle when ARMED [%] )
        //AccX          ( AccX m/s ? )
        //AccY          ( AccY m/s ? )
        //AccZ          ( AccZ m/s ? )
        //VSpd          ( Vertical speed [m/s] )
        //Speed         ( Ground speed from GPS,  [km/h] )
        //T1            ( GPS status = ap_sat_visible*10) + ap_fixtype )
        //T2            ( ARMED=1, DISARMED=0 )
        //Vfas          ( same as Cells )
        //Longitud    
        //Latitud
        //Dist          ( Will be calculated by FrSky Taranis as the distance from first received lat/long = Home Position
        //
        //******************************************************


