/*
Author George Chatzisavvidis.
Projects that used for creation of Taranis-Mikrokopter project was
Altastation and MavlinkFrsky.

Modified by Erling Stage.
*/
#include <Arduino.h>
#include <stdint.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <EEPROM.h>
#include "HardwareSerial.h"
#include "FrSkySPort.h"
#include "altapix.h"
#include "altatools.c"


// Pins usage
#define LED 13  // when sending FrSKY data (or che setting calibration voltages)
#define CALIB 6 // Calibration pin
#define mkPin 2 // when processing

#define xDEBUG_TRACES

#if defined(DEBUG_TRACES)
#define debugSerial            Serial  //3 // pin 8 output
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

#define MKSerial                Serial2

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
uint16_t   mk_error_code_1 = 0;       // Combine (Errorcode | NCFlags << 8)
uint16_t   mk_error_code_2 = 0;       // Combine (FCFlags   | FCFlags2 << 8)
uint16_t   mk_used_capacity = 0;      // mAh
uint16_t   mk_voltage_battery = 0;    // 1000 = 1V
int16_t    mk_current_battery = 0;    // 10 = 1A
uint8_t    mk_sat_visible = 0;        // numbers of visible satelites
int32_t    mk_latitude = 0;           // 585522540;
int32_t    mk_longitude = 0;          // 162344467;
int32_t    mk_gps_altitude = 0;       // 1000 = 1m
uint32_t   mk_groundspeed = 0;
uint32_t   mk_compass_heading = 0;
uint32_t   mk_heading = 0;            // GPS heading
uint16_t   mk_flying_time = 0;
int16_t    mk_vertical_speed;         // cm/s

// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset' as zero altitude
int32_t    mk_bar_altitude = 0;    // 100 = 1m
int32_t    mk_climb_rate = 0;      // 100 = 1m/s
int32_t    mk_operating_radius = 0; // m  (fixed at 250 for "normal" license?)

int32_t    mk_gps_status = 0;      // (mk_sat_visible * 10) + mk_fixtype
                                   // ex. 83 = 8 satellites visible, 3D lock 
#define MAX_CELLS 4                // This is the norm for Mikrokopter
const uint16_t a_port[MAX_CELLS] = {9,6,3,0};   // Which analog ports on Teensy are used [cell-0,cell-1,cell-2,cell-3]
uint8_t    ap_cell_count = MAX_CELLS;      
uint16_t   cells_raw[MAX_CELLS] = {};          // ADC readings 
uint16_t   cells_calibrated[MAX_CELLS] = {};   // calibrated ADC readings
uint16_t   calibration[MAX_CELLS] = {};
bool calibrated = false;
bool force_calibration = false;
int VersionReceived = 0; // no version seen yet
int mk_success = 0;
int statusReadCommandLine = 0;
char commandLine[maxFrameLen+1];
char cmdStringRequest[30];
unsigned char rawDataDecoded[500];
int nbCrcError = 0;
int sendMKCmd = 1;  // Shall we send a command to MK at next opportunity
MK_Command_t cmdMK = MK_Request_OSD_Data;


/* 
 * The objective is to read 4 voltages from balancing connector.
 * The calibration is achieved by pulling digital pin 6 low when a reference voltage is applied to all pins.
 * The FrSKY cell voltage is coded to max allow 8.191V, expressed as 4095 (2^12-1).
 * The max voltage will be 4 * 4.2 = 16.8. 
 * Define 18V as max input. The voltage divider should give 1.2 volt.
 * full range on input
 */
void readCells()
{
    int i;
    uint16_t c;
    static int32_t cells_avg[MAX_CELLS]={};

    for (i=0; i<MAX_CELLS; i++) {
        // Use an exponential moving average of 16 samples
        cells_avg[i] -= cells_avg[i] >> 4;     // cells_avg -= cells_avg/N   N=16   (cells_avg = input * 16)
        cells_avg[i] += analogRead(a_port[i]); // cells_avg += input/N
        cells_raw[i] = cells_avg[i] >> 4;
        c = calibration[i];  // This the reading for 16.8V
        cells_calibrated[i] = map(cells_avg[i], 0, (c?c:3822)*16, 0, 3822);
        DEBUG_PRINT(cells_calibrated[i]);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN("");
}

/**
 * \return 1 for success, 0 for not calibrated
 */
int readCalibration()
{
    // First check signature
    byte check = 0;
    byte cal_data[sizeof(calibration)*sizeof(calibration[0])];
    int res = false;
    
    check = EEPROM.read(0);
    if (check == 42) {
        DEBUG_PRINTLN("Found calibration data");
        for (unsigned int i=0; i<sizeof(cal_data); i++) {
            cal_data[i] = EEPROM.read(i+1);
        }
        memcpy(calibration, cal_data, sizeof(cal_data));
        for (int i=0; i< MAX_CELLS; i++) {
            DEBUG_PRINTLN(calibration[i]);
        }
        res = true;
    } else {
        DEBUG_PRINTLN("No calibration data found");
    }
    return res;
}

void writeCalibration()
{
    byte cal_data[sizeof(calibration)*sizeof(calibration[0])];
    
    DEBUG_PRINTLN("Writing calibration data");
    // First write signature
    EEPROM.write(0, 42);
    memcpy(cal_data, calibration, sizeof(cal_data));
    for (unsigned int i=0; i<sizeof(cal_data); i++) {
         EEPROM.write(i+1, cal_data[i]);
    }
    for (int k=0; k < MAX_CELLS; k++) {
        DEBUG_PRINTLN(calibration[k]);
    }
}

int calibrate()
{
    // When this function is called, a voltage of 16.8 volts is applied (through divider network) to all 4 inputs.
    // This means that a reading of 4095*16.8/18.0 = 3822 is expected.
    // Save the reading as reference, and adjust calibrated reading to match this range.
    int ok=true;

    memcpy(calibration, cells_raw, sizeof(cells_raw)*sizeof(cells_raw[0]));
    for (int i=0; i<MAX_CELLS; i++) {
        DEBUG_PRINTLN(calibration[i]);
        if (calibration[i] < 1000) {
            ok = false;
        }
    }
    if (ok) {
        writeCalibration();
    } else {
        DEBUG_PRINTLN("Bad calibration data");
        for (int i=0; i<MAX_CELLS; i++) {
            calibration[i] = 0; // Indicate no calibration
        }
    }
    return ok;
}

/**
 * Return cell N voltage in format suitable for 12 bit 0-8.191V range maps to 0..4095.
 * cell_calibrated 4095 == 18.0V; 8.192/18*4096 = 1864
 * return range 4095 == 8.192V
 * N = 0..3 range
 */
uint16_t cell_voltage(uint16_t cellNo)
{
    uint16_t voltage = 0;
    uint16_t result = 0;

    if (cellNo < MAX_CELLS) {
        if (cellNo > 0) {
            voltage = cells_calibrated[cellNo] - cells_calibrated[cellNo-1];
        } else {
            voltage = cells_calibrated[cellNo];
        }
        result = map(voltage, 0, 1864, 0, 4095); // Scale up reading of 1864 to 4095, 8.192V
        if (result > 0xfff) {
            result = 0xfff;
        }
    }
    //DEBUG_PRINT(cellNo);
    //DEBUG_PRINT(" ");
    //DEBUG_PRINTLN(result);
    return result;  
}

void decodeOSD()
{
    s_MK_NaviData *NaviData;

    digitalWrite(mkPin, HIGH);
    decode64(commandLine, rawDataDecoded, strlen(commandLine));
    NaviData = (s_MK_NaviData*)&rawDataDecoded;

    mk_longitude                 = NaviData->CurrentPosition.Longitude; // in 1E-7 deg see ubx.h for details
    PRINTNZ(mk_longitude);
    mk_latitude                  = NaviData->CurrentPosition.Latitude; // in 1E-7 deg
    PRINTNZ(mk_latitude);
    mk_gps_altitude              = NaviData->CurrentPosition.Altitude; /// in mm
    PRINTNZ(mk_gps_altitude);
    mk_gps_status                = NaviData->CurrentPosition.Status; //  GPS STATUS INVALID=0,NEWDATA=1,PROCESSED=2 
    //---GPS_Pos_t    TargetPosition;
    //---GPS_PosDev_t TargetPositionDeviation;
    //---GPS_Pos_t    HomePosition;
    //---GPS_PosDev_t HomePositionDeviation;
    //---uint8_t      WaypointIndex;    // index of current waypoints running from 0 to WaypointNumber-1
    //---uint8_t      WaypointNumber;   // number of stored waypoints
    mk_sat_visible               = NaviData->SatsInUse; // number of satellites used for position solution
    PRINTNZ(mk_sat_visible);
    mk_bar_altitude              = NaviData->Altimeter; // height according to air pressure in cm
    mk_climb_rate                = NaviData->Variometer; // climb(+) and sink(-) rate
    mk_flying_time               = NaviData->FlyingTime; // in seconds
    mk_voltage_battery           = NaviData->UBat; // Battery Voltage in 0.1 Volts
    mk_groundspeed               = NaviData->GroundSpeed; // speed over ground in cm/s (2D)
    mk_heading                   = NaviData->Heading; // current flight direction in ° as angle to north
    mk_compass_heading           = NaviData->CompassHeading; // current compass value in °
    PRINTNZ(mk_compass_heading);
    //---int8_t       AngleNick;	// current Nick angle in 1°
    //---int8_t       AngleRoll;	// current Rick angle in 1°
    //---uint8_t      RC_Quality;	// RC_Quality
    //---uint8_t      FCFlags;		// Flags from FC
    //---uint8_t      NCFlags;		// Flags from NC
    mk_error_code_1              = NaviData->Errorcode | (NaviData->NCFlags  << 8);
    mk_error_code_2              = NaviData->FCFlags   | (NaviData->FCStatusFlags2 << 8);
    mk_operating_radius          = NaviData->OperatingRadius; // current operation radius around the Home Position in m
    mk_vertical_speed            = NaviData->TopSpeed; // velocity in vertical direction in cm/s
    //---uint8_t      TargetHoldTime;	// time in s to stay at the given target, counts down to 0 if target has been reached
    //---int16_t      SetpointAltitude;	// setpoint for altitude
    //---uint8_t      Gas;		// for future use
    mk_current_battery           = NaviData->Current; // actual current in 0.1A steps
    mk_used_capacity             = NaviData->UsedCapacity; // used capacity in mAh
    digitalWrite(mkPin, LOW);
    mk_success = 1;
}

void decodeVersion()
{
    str_VersionInfo *VersionInfo;

    decode64(commandLine, rawDataDecoded, strlen(commandLine));
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
    delay(10000); // delay to set up USB terminal with teensy
    debugSerial.begin(115200);
    debugSerial.print("Mikrokopter-FrSKY telemetry gateway\r\n");
#endif

    pinMode(LED, OUTPUT);
    pinMode(mkPin, OUTPUT);
    pinMode(CALIB, INPUT_PULLUP);    // Pin is used to detect calibration request

    analogReference(INTERNAL);   // 1.2V reference
    analogReadResolution(12);    // 0..4095 ADC range

    force_calibration = !digitalRead(CALIB); // 
    calibrated = readCalibration();
    if (force_calibration) {
        DEBUG_PRINTLN("Forced calibration");
        digitalWrite(LED, HIGH);   // Solid LED light when expecting calibration voltage (16.8V)
    }

    FrSkySPort_Setup();
    MKSerial.begin(57600);
 
    redirectUartNc();
    cmdMK = MK_Request_OSD_Data; // Set 1st type of data to request from MK
}


void loop() 
{
    static unsigned long lastCMdTime=0;
    static unsigned long lastCellTime=0;
    unsigned long timeNow;
    uint16_t calibrationDone;

    if (force_calibration) {
        calibrationDone = digitalRead(CALIB);
        if (calibrationDone > 0) {
            DEBUG_PRINTLN("Pin released - calibrating");
            // When pin 6 is no longer tied to ground, calibration takes place, and values are store in EEPROM.
            calibrate();
            calibrated = true;
            force_calibration = false;
            digitalWrite(LED, LOW);
        }
        return;
    }

    FrSkySPort_Process();  // Scan for RX polling messages
  
    if (sendMKCmd){ // we dont need to send the command to mk on each loop, the mk is instructed to output data on regular basis    
#ifdef MK_VERSION_NEEDED
        // we need to fetch OSD data  
        if (!VersionReceived) {
            cmdMK = MK_Version_Request;
        }
#endif
        timeNow = millis();

        if ((timeNow - lastCellTime) > 100) {
            readCells();
            lastCellTime = timeNow;
        }

        if (timeNow - lastCMdTime > (mk_success ? 3000 : 300)) {  // MK repeating OSD subscrition lasts 4 seconds
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
                        return 2; // broken frame
                    }
                } else {
                    read_state = expect_start;
                    return 1; // full frame
                }
            } else {
                if (timeNow - timeStart > SERIAL_READ_TIMEOUT) {
                    read_state = expect_start;
                    commandLine[0] = 0;
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


