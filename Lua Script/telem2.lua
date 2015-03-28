--do the local variables here ie 
local besttime = 0
local bat = 0
local amps = 0
local time = 0
local sats = 0
local speed = 0
local mkerror
local timer

local text = "Mikrokopter"
local temp = 0
local errorcode

local error_txt = {
    0 = "Mikrokopter",
		1 = "FC not compatible",
		2 = "MK3Mag not compatible ",
		3 = "no FC communication  ",
		4 = "no MK3Mag communication  ",
		5 = "no GPS communication ",
		6 = "bad compass value ",
		7 = "RC Signal lost ",
		8 = "FC spi rx error ",
		9 = "ERR: no NC communication ",
		10 = "ERR: FC Nick Gyro ",
		11 = "ERR: FC Roll Gyro ",
		12 = "ERR: FC Yaw Gyro ",
		13 = "ERR: FC Nick ACC ",
		14 = "ERR: FC Roll ACC ",
		15 = "ERR: FC Z-ACC ",
		16 = "ERR: Pressure sensor ",
		17 = "ERR: FC I2C ",
		18 = "ERR: Bl Missing ",
		19 = "Mixer Error ",
		20 = "FC: Carefree Error ",
		21 = "ERR: GPS lost ",
		22 = "ERR: Magnet Error ",
		23 = "Motor restart ",
		24 = "BL Limitation ",
		25 = "Waypoint range ",
		26 = "ERR:No SD-Card ",
		27 = "ERR:SD Logging aborted ",
		28 = "ERR:Flying range! ",
		29 = "ERR:Max Altitude ",
		30 = "No GPS Fix ",
		31 = "compass not calibrated ",
		32 = "BL Selftest error ",
		33 = "no ext. compass",
		34 = "compass sensor",
		}

--etc

local function init()
    -- Initial set up here. Not always needed. This will be run once at model load.
end

local function background()
		--tim1 = model.getTimer(1)
		--stuff here is done all time 
			timer = model.getTimer(0)
end

local function run(event)

		background()
		
		print(event)  

		--lines
		lcd.drawLine(  0, 31, 211, 31, SOLID, 0)	
		lcd.drawLine(  0, 53, 211, 53, SOLID, 0)	
		lcd.drawLine(106,  0, 106, 53, SOLID, 0)
		
		-- bat
		lcd.drawText(15,   3, getValue(216) .. " V", MIDSIZE)   --"vfas"
		lcd.drawTimer(65, 20, timer.value, 0)                   --timer
		lcd.drawText(15,  20, getValue(208) .. " mAh",0)        --"fuel" current mAh
		
		lcd.drawText(15,  33, getValue(207) .. " sats", 0)      --"rpm" sats in use
		lcd.drawText(15,  41, getValue(211) .. " km/h", 0)      --"gps-speed"
		
		lcd.drawText(108,  3, "Alt:" .. getValue("altitude") .. " m gAlt:" .. getValue(213) .. "m",0)  --altitude and gps altitude
		lcd.drawText(108, 12, "Dir: " .. getValue(223),0)       --heading
		lcd.drawText(108, 20, "I: " .. getValue(217),0)         --current
		
		lcd.drawText(108, 33, "Home: " .. getValue(212) .. " m" ,0) --gps distance
		lcd.drawText(108, 41, "Dir: home", 0)
		
		--This happens when telemetry screen is on
		
		lcd.drawText (60, 56, "Mikrokopter", 0)

		background()

		print(event)  
		
		--lines
		lcd.drawLine(  0, 31, 211, 31, SOLID, 0)	
		lcd.drawLine(  0, 53, 211, 53, SOLID, 0)	
		lcd.drawLine(106,  0, 106, 53, SOLID, 0)
	
		-- bat
		lcd.drawText(15, 3, getValue(216) .. " V", MIDSIZE)   --Voltage
		lcd.drawTimer(65, 20, timer.value, 0)                 --timer
		lcd.drawText(15,20, getValue(208) .. " mAh",0)        --"fuel" current mAh
	
		lcd.drawText(15,33, getValue(207) .. " sats", 0)      --sats in use
		lcd.drawText(15,41, getValue(211) .. " km/h", 0)      --"gps speed"
	
		lcd.drawText( 108,3, "Alt:" .. getValue("altitude") .. " m gAlt:" .. getValue(213) .. "m",0)  --altitude and gps altitude
		lcd.drawText( 108,12, "Dir: " .. getValue(223),0)     --"heading"
		lcd.drawText( 108,20, "I: " .. getValue(217),0)       --"current"
	
		lcd.drawText( 108,33, "Home: " .. getValue(212) .. " m" , 0) --"distance"
		lcd.drawText( 108,41, "Dir: home", 0)

	  local t2 = getValue(210) -- temp2 Temperature 2 [degrees celsius]
	  
    text = error_txt[t2]
		
		if t2 == 0 then 
        lcd.drawText (105 - 50 / 2, 56, text, 0)
		else 
        lcd.drawText (20, 56, text, 0)
		end 
end

return { init=init, background=background, run=run }
