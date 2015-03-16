// Frsky SPort communication parameters
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

#define SENSOR_ID_VARIO              0x00 // 0
#define SENSOR_ID_FLVSS              0xA1 // 1
#define SENSOR_ID_FAS                0x22 // 2
#define SENSOR_ID_GPS                0x83 // 3
#define SENSOR_ID_RPM                0xE4 // 4
#define SENSOR_ID_SP2UH              0x45 // 5
#define SENSOR_ID_SP2UR              0xC6 // 6

// Frsky protocol specific
#define START_STOP                   0x7e
#define DATA_FRAME                   0x10

//Frsky DATA ID's 
#define FR_ID_ALTITUDE               0x0100 // (int) float * 100 [m]   int32_t
#define FR_ID_VARIO                  0x0110 // cm/s                    int16_t
#define FR_ID_CURRENT                0x0200 // (int) float * 10 [A]  non-linear handling of fasOffset else 0   uint16_t
#define FR_ID_VFAS                   0x0210 // (int) float * 100 [V]  divided by 10 on reception               uint16_t
#define FR_ID_CELLS                  0x0300 // |vvvvvvvvvvvv|vvvvvvvvvvvv|cccc|bbbb|  v = 1/500V  0-8.191V   
#define FR_ID_CELLS_LAST             0x030F      
#define FR_ID_T1                     0x0400 // int [°C]              int16_t
#define FR_ID_T2                     0x0410 // int [°C]              int16_t
#define FR_ID_RPM                    0x050F // int [rpm]             uint16_t
#define FR_ID_CAP_USED               0x0600 // AKA. FR_ID_FUEL       uint16_t
#define FR_ID_FUEL                   0x0600 // int 0~100 [%]         uint16_t
#define FR_ID_ACCX                   0x0700 // (int) float * 100 [g] int16_t
#define FR_ID_ACCY                   0x0710 // (int) float * 100 [g] int16_t
#define FR_ID_ACCZ                   0x0720 // (int) float * 100 [g] int16_t
#define FR_ID_LATLONG                0x0800 // 2* 32bit
#define FR_ID_GPS_ALT                0x0820 // (int) float * 100 [m] 16bit.16bit
#define FR_ID_SPEED                  0x0830 // (int) float * 1000 [knots]  16bit.16bit
#define FR_ID_HEADING                0x0840 // (int) float * 100 [°] (0~359.99°)  16bit.16bit
#define FR_ID_GPS_TIME_DATE          0x0850 // GPS_TIME_DATE_FIRST_ID - two messages, bytes:YYMMDDff or hhmmss00 
#define FR_ID_A3_FIRST               0x0900 // A3_FIRST_ID  frskyData.analog[TELEM_ANA_A3].set((SPORT_DATA_U32(packet)*255+165)/330, UNIT_VOLTS);
#define FR_ID_A4_FIRST               0x0910 // A4_FIRST_ID  frskyData.analog[TELEM_ANA_A4].set((SPORT_DATA_U32(packet)*255+165)/330, UNIT_VOLTS);
#define FR_ID_AIR_SPEED              0x0A00 // 0.1 knt/h  uint16_t
#define FR_ID_RSSI                   0xF101 // used by the radio system
#define FR_ID_ADC1                   0xF102 // 0-255 (0-3.3v input)  
#define FR_ID_ADC2                   0xF103 // 0-255 (0-3.3v input)  
#define FR_ID_BATT                   0xF104 // used by the radio system
#define FR_ID_SWR                    0xF105 // used by the radio system
#define FR_ID_XJT_VERSION            0xf106

void FrSkySPort_Init(void);
void FrSkySPort_Process(void);
void FrSkySPort_SendPackage(uint16_t id, uint32_t value);
void FrSkySPort_TransmitByte(uint8_t b);
