// Frsky SPort speed
#define FrSkySPort_BAUD              57600

// Frsky Sensor-ID to use. 
#define SENSOR_ID1                   0x1B // ID of sensor. Must be something that is polled by FrSky RX
#define SENSOR_ID2                   0x0D
#define SENSOR_ID3                   0x34
#define SENSOR_ID4                   0x67

// Frsky-specific
#define START_STOP                   0x7e
#define DATA_FRAME                   0x10


//Frsky DATA ID's 
#define FR_ID_SPEED                  0x0830 // (int) float * 1000 [knots]
#define FR_ID_VFAS                   0x0210 // (int) float * 100 [V]
#define FR_ID_CURRENT                0x0200 // (int) float * 10 [A]
#define FR_ID_RPM                    0x050F // int [rpm]
#define FR_ID_ALTITUDE               0x0100 // (int) float * 100 [m]
#define FR_ID_FUEL                   0x0600 // int 0~100 [%]
#define FR_ID_ADC1                   0xF102 // 0-255 (0-3.3v input)  
#define FR_ID_ADC2                   0xF103 // 0-255 (0-3.3v input)  
#define FR_ID_LATLONG                0x0800
#define FR_ID_CAP_USED               0x0600
#define FR_ID_VARIO                  0x0110
#define FR_ID_CELLS                  0x0300     
#define FR_ID_CELLS_LAST             0x030F      
#define FR_ID_HEADING                0x0840 // (int) float * 100 [°] (0~359.99°)
#define FR_ID_ACCX                   0x0700 // (int) float * 100 [g]
#define FR_ID_ACCY                   0x0710 // (int) float * 100 [g]
#define FR_ID_ACCZ                   0x0720 // (int) float * 100 [g]
#define FR_ID_T1                     0x0400 // int [°C]
#define FR_ID_T2                     0x0410 // int [°C]
#define FR_ID_GPS_ALT                0x0820 // (int) float * 100 [m]


