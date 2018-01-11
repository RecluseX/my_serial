#ifdef _APPSYS_TYPE_DEFINE_H_
#define _APPSYS_TYPE_DEFINE_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#define CONST   const
#define STATIC  static
#define INLINE  STATIC __inline
#define EXTERN  extern

typedef char               S8;              /*8-bit*/
typedef short              S16;            /*16-bit*/
typedef int                S32;            /*32-bit*/

typedef unsigned char         U8;              /*8-bit*/
typedef unsigned short        U16;            /*16-bit*/
typedef unsigned int          U32;            /*32-bit*/

typedef float                 FLOAT;

using std::string;
using std::vector;

typedef struct ODOMDATA{
    int LDist;
    int RDist;
    int Gyro;
	float vL;
	float vR;
	float wz;
    int timeStamp;
} Odom_Type;

#define INT8_MAX        (+127)
#define INT16_MAX       (+32767)
#define INT32_MAX       (+2147483647L)

#define INT8_MIN        (-INT8_MAX - 1)
#define INT16_MIN       (-INT16_MAX - 1)
#define INT32_MIN       (-INT32_MAX - 1)

#define UINT8_MAX       (0xffU)
#define UINT16_MAX      (0xffffU)
#define UINT32_MAX      (0xffffffffUL)

#define MAX(a,b)        (((a)>(b))?(a):(b))
#define MIN(a,b)        (((a)<(b))?(a):(b))

#endif
