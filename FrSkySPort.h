// Frsky SPort parameters
#define FrSkySPort_Serial            Serial1
#define FrSkySPort_C1                UART0_C1
#define FrSkySPort_C3                UART0_C3
#define FrSkySPort_S2                UART0_S2
#define FrSkySPort_BAUD              57600

// Frsky Sensor-ID to use. 
#define SENSOR_ID1                   0x1B // 27 : ID of sensor. Must be something that is polled by FrSky RX
#define SENSOR_ID2                   0x0D // 13
#define SENSOR_ID3                   0x34 // 20
#define SENSOR_ID4                   0x67 // 07
#define SENSOR_ID5                   0x53 // 19 xtra
#define SENSOR_ID6                   0x95 // 21 xtra

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

void FrSkySPort_Init(void);
void FrSkySPort_Process(void);
void FrSkySPort_SendPackage(uint16_t id, uint32_t value);
void FrSkySPort_TransmitByte(uint8_t b);
