#ifndef __ALTAPIX_H__
#define __ALTAPIX_H__

/**
 * Mikrokopter data structures and helper routines for formatting and decoding of messages.
 */

#define maxFrameLen 254
#define DEBUGME 1
#define SERIAL_READ_TIMEOUT 10000 // 10 sec to read a valid frame
//#define MK_ALTI_FACTOR 22.5 // some value that correct the altitude reference to alti 0 i guess

#define FC_ADDRESS      1  //(b)
#define NC_ADDRESS      2  //(c)
#define MK3MAG_ADDRESS  3  //(d)
#define BL_CTRL_ADDRESS 5  //(f)

int checkCRC(char *, int );
void addCRC(char *,char *);
void encode64(char *,char *, int );
void decode64(char *Rx,unsigned char *,int);
char *ftoa(char *, double , int);
void extractGpsInfo(char*,char*);

typedef struct
{
    int32_t Longitude;      // in 1E-7 deg
    int32_t Latitude;       // in 1E-7 deg
    int32_t Altitude;       // in mm
    uint8_t Status;         // validity of data
} __attribute__((packed)) GPS_Pos_t;


typedef struct
{
    uint16_t Distance;      // distance to target in dm
    int16_t Bearing;        // course to target in deg
}  __attribute__((packed)) GPS_PosDev_t;


typedef struct 
{
    unsigned char Digital[2];
    int16_t Analog[32];    // Debugwerte
} s_MK_Debug;


struct str_VersionInfo
{
    unsigned char SWMajor;
    unsigned char SWMinor;
    unsigned char ProtoMajor;
    unsigned char ProtoMinor;
    unsigned char SWPatch;
    unsigned char HardwareError[5];
};


typedef struct
{
    uint8_t  Menuitem;
    uint8_t  MaxMenuItem;
    char DisplayText[81];
} __attribute__((packed)) s_MK_LcdScreen;


typedef struct
{
    char DisplayText[81];
} __attribute__((packed)) s_MK_LcdScreen_Item;


typedef struct
{
    uint8_t  Menuitem;
} __attribute__((packed)) s_MK_CMD_RequestDisplay;


typedef struct
{
    uint8_t      Version;		// version of the data structure
    GPS_Pos_t    CurrentPosition;	// see ubx.h for details
    GPS_Pos_t    TargetPosition;
    GPS_PosDev_t TargetPositionDeviation;
    GPS_Pos_t    HomePosition;
    GPS_PosDev_t HomePositionDeviation;
    uint8_t      WaypointIndex;		// index of current waypoints running from 0 to WaypointNumber-1
    uint8_t      WaypointNumber;	// number of stored waypoints
    uint8_t      SatsInUse;		// number of satellites used for position solution
    int16_t      Altimeter; 		// hight according to air pressure
    int16_t      Variometer;		// climb(+) and sink(-) rate
    uint16_t     FlyingTime;		// in seconds
    uint8_t      UBat;			// Battery Voltage in 0.1 Volts
    uint16_t     GroundSpeed;		// speed over ground in cm/s (2D)
    int16_t      Heading;		// current flight direction in ° as angle to north
    int16_t      CompassHeading;	// current compass value in °
    int8_t       AngleNick;		// current Nick angle in 1°
    int8_t       AngleRoll;		// current Rick angle in 1°
    uint8_t      RC_Quality;		// RC_Quality
    uint8_t      FCFlags;		// Flags from FC
    uint8_t      NCFlags;		// Flags from NC
    uint8_t      Errorcode;		// 0 --> okay
    uint8_t      OperatingRadius;       // current operation radius around the Home Position in m
    int16_t      TopSpeed;		// velocity in vertical direction in cm/s
    uint8_t      TargetHoldTime;	// time in s to stay at the given target, counts down to 0 if target has been reached
    uint8_t      FCStatusFlags2;	// StatusFlags2 (since version 5 added)
    int16_t      SetpointAltitude;	// setpoint for altitude
    uint8_t      Gas;			// for future use
    uint16_t     Current;		// actual current in 0.1A steps
    uint16_t     UsedCapacity;		// used capacity in mAh
} __attribute__((packed)) s_MK_NaviData;

// ------- NCFlags -------------------------------------
#define NC_FLAG_FREE                            0x01
#define NC_FLAG_PH                              0x02
#define NC_FLAG_CH                              0x04
#define NC_FLAG_RANGE_LIMIT                     0x08
#define NC_FLAG_NOSERIALLINK                    0x10
#define NC_FLAG_TARGET_REACHED                  0x20
#define NC_FLAG_MANUAL                          0x40
#define NC_FLAG_GPS_OK                          0x80

// ------- FCStatusFlags -------------------------------
#define FC_STATUS_MOTOR_RUN                     0x01
#define FC_STATUS_FLY                           0x02
#define FC_STATUS_CALIBRATE                     0x04
#define FC_STATUS_START                         0x08
#define FC_STATUS_EMERGENCY_LANDING             0x10
#define FC_STATUS_LOWBAT                        0x20
#define FC_STATUS_VARIO_TRIM_UP                 0x40
#define FC_STATUS_VARIO_TRIM_DOWN               0x80

// ------- FCStatusFlags2 ------------------------------
#define FC_STATUS2_CAREFREE_ACTIVE              0x01
#define FC_STATUS2_ALTITUDE_CONTROL_ACTIVE      0x02
#define FC_STATUS2_FAILSAFE_ACTIVE              0x04
#define FC_STATUS2_OUT1                         0x08
#define FC_STATUS2_OUT2                         0x10
#define FC_STATUS2_RES1                         0x20
#define FC_STATUS2_RES2                         0x40
#define FC_STATUS2_RES3                         0x80


#endif

