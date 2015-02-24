
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


