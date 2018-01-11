#ifndef _MY_SERIAL_H_
#define _MY_SERIAL_H_

#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/type.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <semaphore>
#include <string.h>
#include "AppSysTypeDefine.h"

#define SERIAL_COM_BAUDRATE (115200)
#define DATA_BITS (8)
#define PARITY  ('N')
#define STOP_BIT  (1)

#define MAX_BUF (64)

class MYSERIAL
{
public:
  MYSERIAL();
  ~MYSERIAL();
/*******************************************************************
 Function
*******************************************************************/
  S32 ApplicationSerialStart(const string ttyDevPath);
  static void *ApplicationSerialRun(void *arg);
  S32 ApplicationMainLoop();

private:
/*******************************************************************
 Function
*******************************************************************/
  S32 ConfigSerialCom(S32 fd, U32 baud_rate, U8 data_bits, U8 parity, U8 stop_bits);
  S32 OnSerialComRead();
  S32 OnSerialComWrite(U8 *data, U16 len);
  S32 OnReceive();

  pthread_t threadHandler;

  S32 ttyFd;
  U8 idNum;
  U8 RxBuf[MAX_BUF];
  U8 mRxData[MAX_BUF];
  U8 mHeadFlag;
}


#endif
