#ifndef __ALTATOOLS_H__
#define __ALTATOOLS_H__


void addCRC(char *TXBuff,char *CRC);
int  checkCRC(char *t_InData, int Length);
void decode64(char *RxdBuffer, unsigned char *ptrOut, int len);
void encode64(char *Data, char *TX_Buff, int Length);
void extractGpsInfo(char* menuStr,char* gpsInfo);

#endif

