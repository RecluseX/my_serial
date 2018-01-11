#include "my_serial.h"

MYSERIAL::MYSERIAL()
{

}

MYSERIAL::~MYSERIAL()
{

}

S32 MYSERIAL::ConfigSerialCom(S32 fd, U32 baud_rate, U8 data_bits, U8 parity, U8 stop_bits)
{
  struct termios new_cfg, old_cfg;
  int speed;

  if (tcgetattr(fd, &old_cfg) != 0) {
    perror("tcgetattr");
    return -1;
  }

  new_cfg = old_cfg;
  cfmakeraw(&new_cfg);
  new_cfg.c_cflag &= ~CSIZE;

  switch (baud_rate) {
    case 2400:
        speed = B2400;
        break;

    case 4800:
        speed = B4800;
        break;

    case 9600:
        speed = B9600;
        break;

    case 19200:
        speed = B19200;
        break;

    case 38400:
        speed = B38400;
        break;

    default:
    case 115200:
        speed = B115200;
        break;
  }
  cfsetispeed(&new_cfg, speed);
  cfsetospeed(&new_cfg, speed);

  switch (data_bits) {
    case 7:
        new_cfg.c_cflag |= CS7;
        break;

    default:
    case 8:
        new_cfg.c_cflag |= CS8;
        break;
  }

  switch (parity) {
    default:
    case 'n':
    case 'N':
        new_cfg.c_cflag &= ~PARENB;
        new_cfg.c_iflag &= ~INPCK;
        break;

    case 'o':
    case 'O':
        new_cfg.c_cflag |= (PARODD | PARENB);
        new_cfg.c_iflag |= INPCK;
        break;

    case 'e':
    case 'E':
        new_cfg.c_cflag |= PARENB;
        new_cfg.c_cflag &= ~PARODD;
        new_cfg.c_iflag |= INPCK;
        break;

    case 's': /* as no parity */
    case 'S':
        new_cfg.c_cflag &= ~PARENB;
        new_cfg.c_cflag &= ~CSTOPB;
        break;
  }

  switch (stop_bits) {
    default:
    case 1:
        new_cfg.c_cflag &= ~CSTOPB;
        break;

    case 2:
        new_cfg.c_cflag |= CSTOPB;
        break;
  }

  new_cfg.c_cc[VTIME] = 0;
  new_cfg.c_cc[VMIN] = 1;
  tcflush(fd, TCIFLUSH);

  if ((tcsetattr(fd, TCSANOW, &new_cfg)) != 0) {
    perror("tcsetattr");
    return -1;
  }

  return 0;
}

S32 MYSERIAL::ApplicationSerialStart(const string ttyDevPath)
{
  S32 s32Ret;

  ttyFd = open((S8*)ttyDevPath.c_str(), O_RDWR | O_NOCTTY);
  if (ttyFd == -1)
  {
    return 1;
  }

  s32Ret = ConfigSerialCom(ttyFd, SERIAL_COM_BAUDRATE, DATA_BITS, PARITY, STOP_BIT);
  if (s32Ret != 0)
  {
    return 2;
  }

  s32Ret = pthread_create((pthread_t*)&threadHandler, NULL, ApplicationSerialRun, this);
  if (s32Ret != 0)
  {
    return 3;
  }

  return 0;
}

void *MYSERIAL::ApplicationSerialRun(void *arg)
{
  MYSERIAL *pclMCUHost;

  pthread_detach(pthread_self());

  pclMCUHost = reinterpret_cast<MYSERIAL*>(arg);
  pclMCUHost -> ApplicationMainLoop();

  return NULL;
}

S32 MYSERIAL::OnSerialComRead()
{
  U16 nread;
  U8 *rx = RxBuf;
}
