#include "HardwareSerial.h"
#include "FrSkySPort.h"

uint16_t crc;                       // used for crc calc of frsky-packet
uint8_t  lastRx;                    // Last byte received from FrSKY. Look for 7E <dev ID>
uint8_t cell_count = 0;
uint32_t latlong = 0;
uint32_t new_data_counter = 0;
uint32_t skip_data_counter = 0;
uint32_t dev_poll_counter = 0;

class TelemetryItem
{
    public:
        // These are the data types we can send. They are related to the FrSKY defined ID's FR_ID_...
        enum dataType {
            TI_FR_ID_SPEED,
            TI_FR_ID_RPM,
            TI_FR_ID_CURRENT,
            TI_FR_ID_ALTITUDE,
            TI_FR_ID_AIR_SPEED,
            TI_FR_ID_LONG,  // FrSKY uses a combined FR_ID_LATLONG and use a data bit to encode LAT/LONG
            TI_FR_ID_LAT,   // FrSKY uses a combined FR_ID_LATLONG and use a data bit to encode LAT/LONG
            TI_FR_ID_HEADING,
            TI_FR_ID_ADC2,
            TI_FR_ID_CELLS12, // FrSKY uses FR_ID_CELLS and encode which cells in data
            TI_FR_ID_CELLS23, // FrSKY uses FR_ID_CELLS and encode which cells in data
            TI_FR_ID_CELLS45, // FrSKY uses FR_ID_CELLS and encode which cells in data
            TI_FR_ID_ACCX,
            TI_FR_ID_ACCY,
            TI_FR_ID_ACCZ,
            TI_FR_ID_VFAS,
            TI_FR_ID_T1,
            TI_FR_ID_T2,
            TI_FR_ID_VARIO,
            TI_FR_ID_GPS_ALT,
            TI_FR_ID_FUEL,
            TI_NBR_ITEMS
        };
        TelemetryItem(dataType id, uint16_t frid);
        uint32_t FetchData(dataType, bool &valid);
        bool transmitData();

    private:
        dataType m_id;
        uint16_t m_frid;
        uint16_t m_skip;
        uint16_t m_skipmax;
        bool m_valid;
        uint32_t m_data;
};

TelemetryItem::TelemetryItem(dataType id, uint16_t frid)
{
    m_id    = id;
    m_frid  = frid;
    m_skip  = 0;
    m_skipmax = 30; // unchanging data gets skipped this many times
    m_data  = 0;
    m_valid = false; // no data yet
}

/**
 * \brief convert MK data to FrSKY format
 * \return 32 bit data item
 */
uint32_t TelemetryItem::FetchData(dataType type, bool &valid)
{
    uint32_t value=0;
    valid = true;
    
    switch (type) {
    case TI_FR_ID_SPEED:
        value = mk_groundspeed;          // knt to km/h conversion missing?
        break;

    case TI_FR_ID_RPM:
        value = mk_sat_visible * 2;    //  * 2 if number of blades on Taranis is set to 2
        break;

    case TI_FR_ID_CURRENT:
        value = mk_current_battery; 
        break; 

    case TI_FR_ID_ALTITUDE:
        value = mk_bar_altitude;         // from barometer, 100 = 1m
        break;       

    case TI_FR_ID_LONG:
        if (mk_sat_visible>2) {
            if (mk_longitude < 0) {
                latlong = ((abs(mk_longitude)/100)*6)  | 0xC0000000;
            } else {
                latlong = ((abs(mk_longitude)/100)*6)  | 0x80000000;
            }
            value = latlong;
        } else {
            valid = false;
        }
        break;

    case TI_FR_ID_LAT:
        if (mk_sat_visible>2) {
            if (mk_latitude < 0 ) {
                latlong = ((abs(mk_latitude)/100)*6) | 0x40000000;
            } else {
                latlong = ((abs(mk_latitude)/100)*6);
            }
            value = latlong;
        } else {
            valid = false;
        }
        break;  

    case TI_FR_ID_HEADING:
        value = mk_heading * 100;   // 10000 = 100 deg
        break;

    case TI_FR_ID_ADC2:
        value = mk_voltage_battery; // 
        break;

    case TI_FR_ID_CELLS12:
        value = (cell_voltage(0) << 20) | (cell_voltage(1) << 8) | (2 << 4); // Battery cells 0 and 1, 2 cells
        //DEBUG_PRINTLN(value);
        break;

    case TI_FR_ID_CELLS23:
        if (ap_cell_count > 2) {
            uint32_t offset;
            offset = ap_cell_count > 3 ? 0x02: 0x01;
            value = (cell_voltage(2) << 20) | (cell_voltage(3) << 8) | (2 << 4) | offset;  // Battery cell 2,3
            //DEBUG_PRINTLN(value);
        } else {
            valid = false;
        }
        break;

    case TI_FR_ID_CELLS45:
        if (ap_cell_count > 4) {
            //uint32_t offset;
            //offset = ap_cell_count > 5 ? 0x04: 0x03;
            //value = (temp << 20) | (temp << 8) | offset;  // Battery cell 2,3
            valid = false;
        } else {
            valid = false;
        }
        break;     

    case TI_FR_ID_ACCX:
        value = mk_compass_heading;    
        break;

    case TI_FR_ID_ACCY:
        value = mk_flying_time; 
        break; 

    case TI_FR_ID_ACCZ:
        value = mk_vertical_speed; 
        break; 

    case TI_FR_ID_VFAS:
        value = mk_voltage_battery * 10;
        break;   

    case TI_FR_ID_T1:
        value = mk_error_code_1; 
        break; 

    case TI_FR_ID_T2:
        value = mk_error_code_2; 
        break;

    case TI_FR_ID_VARIO:
        value = mk_climb_rate;       // cm/s        
        break;

    case TI_FR_ID_GPS_ALT:
        if (mk_gps_status % 10 == 3) {
            value = mk_gps_altitude / 10;   // from GPS, units: MK mm, FrSKY cm
        } else {
            valid = false;
        }
        break;

    case TI_FR_ID_FUEL:
        value = mk_used_capacity; 
        break;      

    case TI_FR_ID_AIR_SPEED:
        value = mk_gps_status;
        break;

    default:
        valid = false;
        break;
    }
    return value;
}

/**
 * \brief Fetch & send data if available and (new or stale)
 * \return 1 if data was sent, 
 */
bool TelemetryItem::transmitData(void)
{
    bool valid = false;
    uint32_t data = FetchData(m_id, valid);

    if (valid) {
        if ((data != m_data) || (m_skip > m_skipmax)) {
            FrSkySPort_SendPackage(m_frid, data);
            m_data = data;  // Save copy of what was sent
            m_skip = 0;
            return true;
        } else {
            m_skip++;
        }
    }
    return false;
}

/**
 * \brief A collection of TelemetryItem s.
 * Handles mapping to FrSKY ID's and encoding logic.
 */
class TelemetrySet 
{
public: 
    TelemetrySet(const TelemetryItem::dataType activeIds[], uint16_t len);
    void SendItem();

private:
    TelemetryItem* m_items[TelemetryItem::TI_NBR_ITEMS] = {}; //! Map data type to object
    const uint16_t m_frId[TelemetryItem::TI_NBR_ITEMS] = { // Mapping from internal ID's to FrSKY ID's
                                 FR_ID_SPEED,       //TI_FR_ID_SPEED
                                 FR_ID_RPM,         //TI_FR_ID_RPM
                                 FR_ID_CURRENT,     //TI_FR_ID_CURRENT
                                 FR_ID_ALTITUDE,    //TI_FR_ID_ALTITUDE
                                 FR_ID_AIR_SPEED,   //TI_FR_ID_AIR_SPEED
                                 FR_ID_LATLONG,     //TI_FR_ID_LONG
                                 FR_ID_LATLONG,     //TI_FR_ID_LAT
                                 FR_ID_HEADING,     //TI_FR_ID_HEADING
                                 FR_ID_ADC2,        //TI_FR_ID_ADC2
                                 FR_ID_CELLS,       //TI_FR_ID_CELLS12
                                 FR_ID_CELLS,       //TI_FR_ID_CELLS23
                                 FR_ID_CELLS,       //TI_FR_ID_CELLS45
                                 FR_ID_ACCX,        //TI_FR_ID_ACCX
                                 FR_ID_ACCY,        //TI_FR_ID_ACCY
                                 FR_ID_ACCZ,        //TI_FR_ID_ACCZ
                                 FR_ID_VFAS,        //TI_FR_ID_VFAS
                                 FR_ID_T1,          //TI_FR_ID_T1
                                 FR_ID_T2,          //TI_FR_ID_T2
                                 FR_ID_VARIO,       //TI_FR_ID_VARIO
                                 FR_ID_GPS_ALT,     //TI_FR_ID_GPS_ALT
                                 FR_ID_FUEL,        //TI_FR_ID_FUEL
                               };       //! Map data type to FrSKY ID
    uint16_t m_count; //! Number of active telemetry items
    uint16_t m_next;  //! Next telemetry item to send
    uint16_t m_idx[TelemetryItem::TI_NBR_ITEMS] = {};
};

TelemetrySet::TelemetrySet(const TelemetryItem::dataType activeIds[], uint16_t len)
{
    uint16_t i;
    TelemetryItem::dataType type;
    uint16_t frtype;

    m_count = len;
    m_next = 0;
    for (i=0; i<len; i++) {
        type = activeIds[i];
        frtype = m_frId[type];
        m_items[type] = new TelemetryItem(type, frtype);  // Map each object according to data type
        m_idx[i] = type;  // map consequtive index to used data type
    }
}

/**
 * \brief Send the next ready TelemetryItem
 */
void TelemetrySet::SendItem()
{
    bool done=false;
    uint16_t tries=0;  // 
    do {
        done = m_items[m_idx[m_next]]->transmitData();
        if (done) {
            //DEBUG_PRINT("*");
            new_data_counter++;
        }
        m_next++;
        tries++;
        if (m_next >= m_count) {
            m_next = 0;
        }
        if (!done && (tries >= m_count)) {
            // Only try each item once per round.
            //DEBUG_PRINT("-");
            skip_data_counter++;
            done = true;
        }
    } while (!done);
}

// These are the telemetry items we will actually transmit.
// Comment out items not relevant to save bandwidth.
const TelemetryItem::dataType activeItems[] = {
    TelemetryItem::TI_FR_ID_SPEED,
    TelemetryItem::TI_FR_ID_RPM,
    TelemetryItem::TI_FR_ID_CURRENT,
    TelemetryItem::TI_FR_ID_ALTITUDE,
    TelemetryItem::TI_FR_ID_AIR_SPEED,
    TelemetryItem::TI_FR_ID_LONG,
    TelemetryItem::TI_FR_ID_LAT,
    TelemetryItem::TI_FR_ID_HEADING,
    //TelemetryItem::TI_FR_ID_ADC2,
    TelemetryItem::TI_FR_ID_CELLS12,
    TelemetryItem::TI_FR_ID_CELLS23,
    //TelemetryItem::TI_FR_ID_CELLS45,
    TelemetryItem::TI_FR_ID_ACCX,
    TelemetryItem::TI_FR_ID_ACCY,
    TelemetryItem::TI_FR_ID_ACCZ,
    TelemetryItem::TI_FR_ID_VFAS,
    TelemetryItem::TI_FR_ID_T1,
    TelemetryItem::TI_FR_ID_T2,
    TelemetryItem::TI_FR_ID_VARIO,
    TelemetryItem::TI_FR_ID_GPS_ALT,
    TelemetryItem::TI_FR_ID_FUEL,
};

void FrSkySPort_Initialize(void)  
{
    FrSkySPort_Serial.begin(FrSkySPort_BAUD);
    FrSkySPort_C3 = 0x10;           // Tx invert
    FrSkySPort_C1 = 0xA0;           // Single wire mode
    FrSkySPort_S2 = 0x10;           // Rx Invert
}

TelemetrySet *channel=NULL; //! This points to the object with the TelemetrySet

void FrSkySPort_Setup(void)
{
    FrSkySPort_Initialize();
    channel = new TelemetrySet(activeItems, sizeof(activeItems));
}

void FrSkySPort_Process(void) 
{
    uint8_t data = 0;
    while ( FrSkySPort_Serial.available()) {
        data = FrSkySPort_Serial.read();
        if (lastRx == START_STOP) {
            // FrSKY scans all known nodes PLUS ONE of the unknown (in round robin) for each cycle.
            // Recognizing more ID's means that releative bandwith increases. In this case to 6/7th
            if ((data == SENSOR_ID1) /*|| (data == SENSOR_ID2) || 
                (data == SENSOR_ID3) || (data == SENSOR_ID4) ||
                (data == SENSOR_ID5) || (data == SENSOR_ID6)*/) {
                channel->SendItem();
            } else {
                // Some other sensor was probed
                // digitalWrite(mkPin, HIGH);
                // delay(2);
                // digitalWrite(mkPin, LOW);
                //DEBUG_PRINT2(data,HEX);
                dev_poll_counter++;
                if (dev_poll_counter % 300 == 0) {
                    uint32_t tot;
                    tot = new_data_counter + skip_data_counter + dev_poll_counter;
                    DEBUG_PRINT("New:"); DEBUG_PRINT(100*new_data_counter/tot);
                    DEBUG_PRINT(" Skip:"); DEBUG_PRINT(100*skip_data_counter/tot);
                    DEBUG_PRINT(" Poll:"); DEBUG_PRINTLN(100*dev_poll_counter/tot);
                }
            }
        }
        lastRx = data;
    }
}

void FrSkySPort_SendByte(uint8_t b) 
{
    FrSkySPort_TransmitByte( b );
    // CRC update
    crc += b; //0-1FF
    crc += ( crc >> 8 ); //0-100
    crc &= 0x00ff;
}

void FrSkySPort_TransmitByte(uint8_t b) 
{
    if ( b == 0x7e ) {
        FrSkySPort_Serial.write( 0x7d );
        FrSkySPort_Serial.write( 0x5e );
    } else if ( b == 0x7d ) {
        FrSkySPort_Serial.write( 0x7d );
        FrSkySPort_Serial.write( 0x5d );
    } else {
        FrSkySPort_Serial.write( b );
    } 
}

void FrSkySPort_SendCrc() 
{
    FrSkySPort_TransmitByte( 0xFF-crc );
    crc = 0; // CRC reset
}

void FrSkySPort_SendPackage(uint16_t id, uint32_t value) 
{
    digitalWrite(LED, HIGH);  // Flash LED when sending telemetry.
    FrSkySPort_C3 |= 32;      //  Transmit direction, to S.Port
    FrSkySPort_SendByte(DATA_FRAME);
    uint8_t *bytes = (uint8_t*)&id;
    FrSkySPort_SendByte(bytes[0]);
    FrSkySPort_SendByte(bytes[1]);
    bytes = (uint8_t*)&value;
    FrSkySPort_SendByte(bytes[0]);
    FrSkySPort_SendByte(bytes[1]);
    FrSkySPort_SendByte(bytes[2]);
    FrSkySPort_SendByte(bytes[3]);
    FrSkySPort_SendCrc();
    FrSkySPort_Serial.flush();
    FrSkySPort_C3 ^= 32;      // Transmit direction, from S.Port
    digitalWrite(LED, LOW);
}

