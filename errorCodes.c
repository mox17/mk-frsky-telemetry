#include "errorCodes.h"
#include <string.h>

// good to keep them < 20 because the screen is 20chr big
const char  ERROR_CODE_0[]="No Error";
const char  ERROR_CODE_1[]="FC not compat"; 
const char  ERROR_CODE_2[]="MK3Mag not compat"; 
const char  ERROR_CODE_3[]="no FC comm" ;
const char  ERROR_CODE_4[]="no MK3Mag comm"; 
const char  ERROR_CODE_5[]="no GPS comm"; 
const char  ERROR_CODE_6[]="bad compass value";
const char  ERROR_CODE_7[]="RC Signal lost"; 
const char  ERROR_CODE_8[]="FC spi rx error"; 
const char  ERROR_CODE_9[]="not used";
const char  ERROR_CODE_10[]="ERR: FC Nick Gyro";
const char  ERROR_CODE_11[]="ERR: FC Roll Gyro";
const char  ERROR_CODE_12[]="ERR: FC Yaw Gyro";
const char  ERROR_CODE_13[]="ERR: FC Nick ACC";
const char  ERROR_CODE_14[]="ERR: FC Roll ACC";
const char  ERROR_CODE_15[]="ERR: FC Z-ACC";
const char  ERROR_CODE_16[]="ERR: Pressure sensor";
const char  ERROR_CODE_17[]="ERR: FC I2C";
const char  ERROR_CODE_18[]="ERR: Bl Missing";
const char  ERROR_CODE_19[]="Mixer Error";
const char  ERROR_CODE_20[]="FC: Carefree Error";

const char * ErrorStrPtrs[] = {ERROR_CODE_0,
                               ERROR_CODE_1,
                               ERROR_CODE_2,
                               ERROR_CODE_3,
                               ERROR_CODE_4,
                               ERROR_CODE_5,
                               ERROR_CODE_6, 
                               ERROR_CODE_7,
                               ERROR_CODE_8,
                               ERROR_CODE_9,
                               ERROR_CODE_10,
                               ERROR_CODE_11,
                               ERROR_CODE_12,
                               ERROR_CODE_13,
                               ERROR_CODE_14,
                               ERROR_CODE_15,
                               ERROR_CODE_16,
                               ERROR_CODE_17,
                               ERROR_CODE_18,
                               ERROR_CODE_19,
                               ERROR_CODE_20};


void resolveError(unsigned int errorNum, char *errorBuf)
{
    if (errorNum<=20) {
        strcpy(errorBuf,ErrorStrPtrs[errorNum]);
    } else {
        errorBuf[0]='\0';
    }  
/*  
    switch(errorNum) {
    case 0:
        strcpy(errorBuf,ERROR_CODE_0);
        break;
    case 1:
        strcpy(errorBuf,ERROR_CODE_1);
        break;
    case 2:
        strcpy(errorBuf,ERROR_CODE_2);
        break;
    case 3:
        strcpy(errorBuf,ERROR_CODE_3);
        break;
    case 4:
        strcpy(errorBuf,ERROR_CODE_4);
        break;
    case 5:
        strcpy(errorBuf,ERROR_CODE_5);
        break;
    case 6:
        strcpy(errorBuf,ERROR_CODE_6);
        break;
    case 7:
        strcpy(errorBuf,ERROR_CODE_7);
        break;
    case 8:
        strcpy(errorBuf,ERROR_CODE_8);
        break;
    case 9:
        strcpy(errorBuf,ERROR_CODE_9);
        break;
    case 10:
        strcpy(errorBuf,ERROR_CODE_10);
        break;
    case 11:
        strcpy(errorBuf,ERROR_CODE_11);
        break;
    case 12:
        strcpy(errorBuf,ERROR_CODE_12);
        break;
    case 13:
        strcpy(errorBuf,ERROR_CODE_13);
        break;
    case 14:
        strcpy(errorBuf,ERROR_CODE_14);
        break;
    case 15:
        strcpy(errorBuf,ERROR_CODE_15);
        break;
    case 16:
        strcpy(errorBuf,ERROR_CODE_16);
        break;
    case 17:
        strcpy(errorBuf,ERROR_CODE_17);
        break;
    case 18:
        strcpy(errorBuf,ERROR_CODE_18);
        break;
    case 19:
        strcpy(errorBuf,ERROR_CODE_19);
        break;
    case 20:
        strcpy(errorBuf,ERROR_CODE_20);
        break;
    default:
        errorBuf[0]='\0';
    }
*/
}

