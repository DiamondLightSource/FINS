/*
	Factory Intelligent Network Service
	
	This is an asyn driver, supporting various asyn interfaces, which sends Hostlink command using FINS
	requests and receive replies from the CPU unit of the PLC.

	This is a test version which uses the same asyn interface but generates ASCII hostlink commands.
	Eventually the core code will be extracted and the user will select an asyn interpose layer to use
	FINS or hostlink.
	
	Testing with our CJ1_CPU12s shows a maximum receive data length of 268 words 
		1099 bytes total (header 23, terminator 4), data length 1072 bytes (maximum 1076)

	Interfaces:
	
		asynOctet
		r	FINS_MODEL
		w	FINS_CYCLE_TIME_RESET
		
		Int32
		r	FINS_DM_READ
		r	FINS_AR_READ
		r	FINS_IO_READ
		r	FINS_DM_READ_32
		r	FINS_AR_READ_32
		r	FINS_IO_READ_32
		r	FINS_CYCLE_TIME_MEAN
		r	FINS_CYCLE_TIME_MAX
		r	FINS_CYCLE_TIME_MIN
		r	FINS_CPU_STATUS
		r	FINS_CPU_MODE
		w	FINS_DM_WRITE
		w	FINS_DM_WRITE_NOREAD
		w	FINS_AR_WRITE
		w	FINS_AR_WRITE_NOREAD
		w	FINS_IO_WRITE
		w	FINS_IO_WRITE_NOREAD
		w	FINS_CYCLE_TIME_RESET
		w	FINS_DM_WRITE_32
		w	FINS_DM_WRITE_32_NOREAD
		w	FINS_AR_WRITE_32
		w	FINS_AR_WRITE_32_NOREAD
		w	FINS_IO_WRITE_32
		w	FINS_IO_WRITE_32_NOREAD
		
		Int16Array
		r	FINS_DM_READ
		r	FINS_AR_READ
		r	FINS_IO_READ
		r	FINS_CLOCK_READ
		w	FINS_DM_WRITE
		w	FINS_AR_WRITE
		w	FINS_IO_WRITE
		
		Int32Array
		r	FINS_DM_READ_32
		r	FINS_AR_READ_32
		r	FINS_IO_READ_32
		r	FINS_CYCLE_TIME
		w	FINS_DM_WRITE_32
		w	FINS_AR_WRITE_32
		w	FINS_IO_WRITE_32
		
		Float32Array
		r	FINS_DM_READ_32
		r	FINS_AR_READ_32
		r	FINS_IO_READ_32
		w	FINS_DM_WRITE_32
		w	FINS_AR_WRITE_32
		w	FINS_IO_WRITE_32
		
	ASYN_CANBLOCK is set because the driver must wait for the reply
	ASYN_MULTIDEVICE is set so that the address field can be used to set the PLC's memory address
	
	See W342, Section 3-5 Sub-section 353

	0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8
		
	@ x x F A x 0 0 0 0 0 0 x x X X X X - - - - x x * r
	
	x x     Unit number
	F A     Header code
	x       Response wait time: 10ms units
	0 0     ICF
	0 0     DA2
	0 0     SA2
	x x     SID Source ID counter
	X X X X FINS command code
	- - - - data
	x x     FCS
	* r     Terminator

	0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 2 2 2 2
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3
	
	@ x x F A 0 0 4 0 0 0 0 0 x x X X X X x x x x - - - - x x * r
	
	x x     Unit number
	F A     Header code
	x x     SID Source ID counter
	X X X X FINS command code
	x x x x FINS response code
	- - - - data
	x x     FCS
	* r     Terminator
*/

#include <stdio.h>
#include <string.h>
#include <errno.h>

#ifdef vxWorks
#include <ioLib.h>
#else
#include <fcntl.h>
#endif

#include <cantProceed.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsAssert.h>
#include <epicsTime.h>
#include <asynDriver.h>
#include <asynDrvUser.h>
#include <asynOctet.h>
#include <asynInt32.h>
#include <asynFloat64.h>
#include <asynInt16Array.h>
#include <asynInt32Array.h>
#include <asynFloat32Array.h>
#include <iocsh.h>
#include <registryFunction.h>
#include <epicsExport.h>
#include <epicsEndian.h>

#include <osiSock.h>

#define BESWAP32(a)	(((a) & 0x0000ffff) << 16) | (((a) & 0xffff0000) >> 16)
#define LESWAP32(a)	(((a) & 0x00ff00ff) <<  8) | (((a) & 0xff00ff00) >>  8)

#undef WSWAP32

#if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
	#define WSWAP32 LESWAP32
#else
	#define WSWAP32 BESWAP32
#endif

/* PLC memory  types */

#define DM	"82"
#define IO	"B0"
#define AR	"B3"
#define CT	"89"

/* offsets in the receive buffer */

#define MRES	19
#define SRES	21
#define RESP	23

/* rx header @00FA0040000000 + command(4) + mres(2) + sres(2) + data(4*n) + checksum(2) + *\r */

#define HEADER_LENGTH	(15 + 4 + 2 + 2)
#define MIN_RESP_LEN	27

/* constants */

#define FINS_MAX_WORDS		268
#define FINS_MAX_MSG		((FINS_MAX_WORDS) * 2 + 100)
#define FINS_TIMEOUT		1
#define FINS_MODEL_LENGTH	20

typedef struct drvPvt
{
	epicsMutexId mutexId;

	int connected;
	int fd;
	
	const char *portName;
	asynInterface common;
	asynInterface drvUser;
	asynInterface octet;
	asynInterface int32;
	asynInterface float64;
	asynInterface int16Array;
	asynInterface int32Array;
	asynInterface float32Array;
	void *pasynPvt;				/* For registerInterruptSource */
	
	uint8_t node;

	epicsUInt8 sid;				/* seesion id - increment for each message */
	
	epicsFloat32 tMax, tMin, tLast;	/* Max and Min and last response time of PLC */
	
	char reply[FINS_MAX_MSG];
	char message[FINS_MAX_MSG];
	char buffer[FINS_MAX_MSG];
	
} drvPvt;

static void flushUDP(const char *func, drvPvt *pdrvPvt, asynUser *pasynUser);
static void FINSerror(drvPvt *pdrvPvt, asynUser *pasynUser, const unsigned char mres, const unsigned char sres);

/*** asynCommon methods ***************************************************************************/

static void report(void *drvPvt, FILE *fp, int details);
static asynStatus aconnect(void *drvPvt, asynUser *pasynUser);
static asynStatus adisconnect(void *drvPvt, asynUser *pasynUser);
static asynCommon asyn = { report, aconnect, adisconnect };

/*** asynOctet methods ****************************************************************************/

static asynStatus udpRead (void *drvPvt, asynUser *pasynUser, char *data, size_t maxchars, size_t *nbytesTransfered, int *eomReason);
static asynStatus udpWrite(void *drvPvt, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransfered);
static asynStatus flushIt (void *drvPvt, asynUser *pasynUser);

/*** asynInt32 methods ****************************************************************************/

static asynStatus WriteInt32(void *drvPvt, asynUser *pasynUser, epicsInt32 value);
static asynStatus ReadInt32(void *drvPvt, asynUser *pasynUser, epicsInt32 *value);

static asynInt32 ifaceInt32 = { WriteInt32, ReadInt32, NULL, NULL, NULL};

/*** asynFloat64 methods **************************************************************************/

static asynStatus WriteFloat64(void *drvPvt, asynUser *pasynUser, epicsFloat64 value);
static asynStatus ReadFloat64(void *drvPvt, asynUser *pasynUser, epicsFloat64 *value);

static asynFloat64 ifaceFloat64 = { WriteFloat64, ReadFloat64, NULL, NULL};

/*** asynInt16Array methods ***********************************************************************/

static asynStatus WriteInt16Array(void *drvPvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements);
static asynStatus ReadInt16Array(void *drvPvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements, size_t *nIn);

static asynInt16Array ifaceInt16Array = { WriteInt16Array, ReadInt16Array, NULL, NULL};

/*** asynInt32Array methods ***********************************************************************/

static asynStatus WriteInt32Array(void *drvPvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements);
static asynStatus ReadInt32Array(void *drvPvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements, size_t *nIn);

static asynInt32Array ifaceInt32Array = { WriteInt32Array, ReadInt32Array, NULL, NULL};

/*** asynFloat32Array *****************************************************************************/

static asynStatus WriteFloat32Array(void *drvPvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements);
static asynStatus ReadFloat32Array(void *drvPvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements, size_t *nIn);

static asynFloat32Array ifaceFloat32Array = { WriteFloat32Array, ReadFloat32Array, NULL, NULL};

/*** asynDrvUser **********************************************************************************/

static asynStatus drvUserCreate (void *drvPvt, asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser, const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroy(void *drvPvt,asynUser *pasynUser);

static asynDrvUser ifaceDrvUser = { drvUserCreate, drvUserGetType, drvUserDestroy};

/**************************************************************************************************/

enum FINS_COMMANDS
{
	FINS_NULL,
	FINS_DM_READ, FINS_DM_WRITE, FINS_DM_WRITE_NOREAD,
	FINS_IO_READ, FINS_IO_WRITE, FINS_IO_WRITE_NOREAD,
	FINS_AR_READ, FINS_AR_WRITE, FINS_AR_WRITE_NOREAD,
	FINS_CT_READ, FINS_CT_WRITE,
	FINS_DM_READ_32, FINS_DM_WRITE_32, FINS_DM_WRITE_32_NOREAD,
	FINS_IO_READ_32, FINS_IO_WRITE_32, FINS_IO_WRITE_32_NOREAD,
	FINS_AR_READ_32, FINS_AR_WRITE_32, FINS_AR_WRITE_32_NOREAD,
	FINS_CT_READ_32, FINS_CT_WRITE_32, FINS_CT_WRITE_32_NOREAD,
	FINS_READ_MULTI,
	FINS_WRITE_MULTI,
	FINS_SET_MULTI_TYPE,
	FINS_SET_MULTI_ADDR,
	FINS_CLR_MULTI,
	FINS_MODEL,
	FINS_CPU_STATUS,
	FINS_CPU_MODE,
	FINS_CYCLE_TIME_RESET,
	FINS_CYCLE_TIME,
	FINS_CYCLE_TIME_MEAN,
	FINS_CYCLE_TIME_MAX,
	FINS_CYCLE_TIME_MIN,
	FINS_MONITOR,
	FINS_CLOCK_READ,
	FINS_EXPLICIT
};

extern int errno;

int finsHostlinkInit(const char *portName, const char *dev)
{
	static const char *FUNCNAME = "finsHostlinkInit";
	drvPvt *pdrvPvt;
	asynStatus status;
	asynOctet *pasynOctet;
	
	pdrvPvt = callocMustSucceed(1, sizeof(drvPvt), FUNCNAME);
	pdrvPvt->portName = epicsStrDup(portName);
	
	pasynOctet = callocMustSucceed(1, sizeof(asynOctet), FUNCNAME);
	
	status = pasynManager->registerPort(portName, ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0);

	if (status != asynSuccess)
	{
		printf("%s: driver registerPort failed\n", FUNCNAME);
		return (-1);
	}
	
/* asynCommon */

	pdrvPvt->common.interfaceType = asynCommonType;
	pdrvPvt->common.pinterface = (void *) &asyn;
	pdrvPvt->common.drvPvt = pdrvPvt;

	status = pasynManager->registerInterface(portName, &pdrvPvt->common);
	
	if (status != asynSuccess)
	{
		printf("%s: registerInterface common failed\n", FUNCNAME);
		return (-1);
	}

/* drvUser */

	pdrvPvt->drvUser.interfaceType = asynDrvUserType;
	pdrvPvt->drvUser.pinterface = &ifaceDrvUser;
	pdrvPvt->drvUser.drvPvt = pdrvPvt;

	status = pasynManager->registerInterface(portName, &pdrvPvt->drvUser);

	if (status != asynSuccess)
	{
		printf("%s: registerInterface drvUser failed\n", FUNCNAME);
		return 0;
	}
	
/* asynOctet methods */

	pasynOctet->write = udpWrite;
	pasynOctet->read = udpRead;
	pasynOctet->flush = flushIt;

	pdrvPvt->octet.interfaceType = asynOctetType;
	pdrvPvt->octet.pinterface = pasynOctet;
	pdrvPvt->octet.drvPvt = pdrvPvt;

	status = pasynOctetBase->initialize(portName, &pdrvPvt->octet, 0, 0, 0);
	
	if (status == asynSuccess)
	{
		status = pasynManager->registerInterruptSource(portName, &pdrvPvt->octet, &pdrvPvt->pasynPvt);
	}
		
	if (status != asynSuccess)
	{
		printf("%s: registerInterface asynOctet failed\n", FUNCNAME);
		return (-1);
	}

/* asynInt32 */

	pdrvPvt->int32.interfaceType = asynInt32Type;
	pdrvPvt->int32.pinterface = &ifaceInt32;
	pdrvPvt->int32.drvPvt = pdrvPvt;
	
	status = pasynInt32Base->initialize(portName, &pdrvPvt->int32);
	
	if (status == asynSuccess)
	{
		status = pasynManager->registerInterruptSource(portName, &pdrvPvt->int32, &pdrvPvt->pasynPvt);
	}
		
	if (status != asynSuccess)
	{
		printf("%s: registerInterface asynInt32 failed\n", FUNCNAME);
		return (-1);
	}
	
/* asynFloat64 */

	pdrvPvt->float64.interfaceType = asynFloat64Type;
	pdrvPvt->float64.pinterface = &ifaceFloat64;
	pdrvPvt->float64.drvPvt = pdrvPvt;
	
	status = pasynFloat64Base->initialize(portName, &pdrvPvt->float64);
		
	if (status != asynSuccess)
	{
		printf("%s: registerInterface asynFloat64 failed\n", FUNCNAME);
		return (-1);
	}
	
/* asynInt16Array */

	pdrvPvt->int16Array.interfaceType = asynInt16ArrayType;
	pdrvPvt->int16Array.pinterface = &ifaceInt16Array;
	pdrvPvt->int16Array.drvPvt = pdrvPvt;
	
	status = pasynInt16ArrayBase->initialize(portName, &pdrvPvt->int16Array);
	
	if (status == asynSuccess)
	{
		status = pasynManager->registerInterruptSource(portName, &pdrvPvt->int16Array, &pdrvPvt->pasynPvt);
	}
		
	if (status != asynSuccess)
	{
		printf("%s: registerInterface asynInt16Array failed\n", FUNCNAME);
		return (-1);
	}

/* asynInt32Array */

	pdrvPvt->int32Array.interfaceType = asynInt32ArrayType;
	pdrvPvt->int32Array.pinterface = &ifaceInt32Array;
	pdrvPvt->int32Array.drvPvt = pdrvPvt;
	
	status = pasynInt32ArrayBase->initialize(portName, &pdrvPvt->int32Array);
	
	if (status == asynSuccess)
	{
		status = pasynManager->registerInterruptSource(portName, &pdrvPvt->int32Array, &pdrvPvt->pasynPvt);
	}
		
	if (status != asynSuccess)
	{
		printf("%s: registerInterface asynInt32Array failed\n", FUNCNAME);
		return (-1);
	}

/* asynFloat32Array */

	pdrvPvt->float32Array.interfaceType = asynFloat32ArrayType;
	pdrvPvt->float32Array.pinterface = &ifaceFloat32Array;
	pdrvPvt->float32Array.drvPvt = pdrvPvt;
	
	status = pasynFloat32ArrayBase->initialize(portName, &pdrvPvt->float32Array);
	
	if (status == asynSuccess)
	{
		status = pasynManager->registerInterruptSource(portName, &pdrvPvt->float32Array, &pdrvPvt->pasynPvt);
	}
		
	if (status != asynSuccess)
	{
		printf("%s: registerInterface asynFloat32Array failed\n", FUNCNAME);
		return (-1);
	}
	
/* open the serial device */

	errno = 0;
	
	if ((pdrvPvt->fd = open(dev, O_RDWR, 0)) < 0)
	{
		printf("%s: Can't open serial device: %s", FUNCNAME, strerror(errno));
		return (-1);
	}
	
	pdrvPvt->tMin = 100.0;
	
 	return (0);
}

static void report(void *pvt, FILE *fp, int details)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	
	fprintf(fp, "%s: connected %s \n", pdrvPvt->portName, (pdrvPvt->connected ? "Yes" : "No"));
/*	fprintf(fp, "    Max: %.4fs  Min: %.4fs  Last: %.4fs\n", pdrvPvt->tMax, pdrvPvt->tMin, pdrvPvt->tLast); */
}

static asynStatus aconnect(void *pvt, asynUser *pasynUser)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	asynStatus status;
	int addr;
	
	status = pasynManager->getAddr(pasynUser, &addr);
    
	if (status != asynSuccess) return status;
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s finsHostlink:connect addr %d\n", pdrvPvt->portName, addr);
	
	if (addr >= 0)
	{
		pasynManager->exceptionConnect(pasynUser);
		return (asynSuccess);
	}
	
	if (pdrvPvt->connected)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s finsHostlink:connect port already connected\n", pdrvPvt->portName);
		return (asynError);
	}

	pdrvPvt->connected = 1;
	pasynManager->exceptionConnect(pasynUser);
	return (asynSuccess);
}

static asynStatus adisconnect(void *pvt, asynUser *pasynUser)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	asynStatus status;
	int addr;
	
	status = pasynManager->getAddr(pasynUser, &addr);
    
	if (status != asynSuccess) return status;
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s finsHostlink:disconnect addr %d\n", pdrvPvt->portName, addr);

	if (addr >= 0)
	{
		pasynManager->exceptionDisconnect(pasynUser);
		return (asynSuccess);
	}
	
	if (!pdrvPvt->connected)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s finsHostlink:disconnect port not connected\n", pdrvPvt->portName);
		return (asynError);
	}
	
	pdrvPvt->connected = 0;
	pasynManager->exceptionDisconnect(pasynUser);
	
	return (asynSuccess);
}

static asynStatus flushIt(void *pvt, asynUser *pasynUser)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s flush\n", pdrvPvt->portName);

	if (pdrvPvt->fd >= 0)
	{
		flushUDP("flushIt", pdrvPvt, pasynUser);
	}
	
 	return (asynSuccess);
}

/******************************************************************************/

static void flushUDP(const char *func, drvPvt *pdrvPvt, asynUser *pasynUser)
{
	int bytes;
	
	do {
	
		fd_set rfds;
		struct timeval tv;
		
		FD_ZERO(&rfds);
		FD_SET(pdrvPvt->fd, &rfds);
		
		tv.tv_sec = 0;
		tv.tv_usec = 100000;
		
		switch (select(pdrvPvt->fd + 1, &rfds, NULL, NULL, &tv))
		{
			case -1:
			{
				return;
				break;
			}
			
			case 0:
			{
				return;
				break;
			}
			
			default:
			{
				break;
			}
		}

		bytes = read(pdrvPvt->fd, pdrvPvt->reply, FINS_MAX_MSG);
			
		if (bytes > 0)
		{
			asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, flushed %d bytes.\n", func, pdrvPvt->portName, bytes);
		}
	}
	while (bytes > 0);
}
		
/******************************************************************************/

/* a simple xor checksum */

static unsigned char checksum(const char *m)
{
	const int len = strlen(m);
	int i;
	unsigned char k = m[0];
	
	for (i = 1; i < len; i++)
	{
		k ^= m[i];
	}
	
	return (k);
}

/******************************************************************************/

/* vxWorks doesn't like %2hhx */

static int extractAndCompareChecksum(drvPvt * const pdrvPvt, const size_t pos)
{
#ifdef vxWorks
	unsigned short kcalc, krecv;
#else
	unsigned char kcalc, krecv;
#endif

/* copy data up to but not including the checksum and calculate the checksum */
			
	strncpy((char *) pdrvPvt->buffer, pdrvPvt->reply, pos);
	pdrvPvt->buffer[pos] = 0;
	
	kcalc = checksum(pdrvPvt->buffer);
	
/* extract the received checksum */

#ifdef vxWorks	
	sscanf(&pdrvPvt->reply[pos], "%2hx", &krecv);
#else
	sscanf(&pdrvPvt->reply[pos], "%2hhx", &krecv);
#endif
	
	if (kcalc != krecv)
	{
		return (-1);
	}
	
	return (0);
}

/******************************************************************************/

static int ReadUntilTerminator(drvPvt *pdrvPvt, asynUser *pasynUser)
{
	static const char * const FUNCNAME = "ReadUntilTerminator";
	int recvlen, totalLength = 0;
	fd_set rfds;
	struct timeval tv;
	
/* timeout */

	if (pasynUser->timeout > 0.0)
	{
		tv.tv_sec = (long) pasynUser->timeout;
		tv.tv_usec = 0;
	}
	else
	{
		tv.tv_sec = FINS_TIMEOUT;
		tv.tv_usec = 0;
	}
	
	memset(pdrvPvt->reply, 0, sizeof(pdrvPvt->reply));
	
	do
	{	
		FD_ZERO(&rfds);
		FD_SET(pdrvPvt->fd, &rfds);

		errno = 0;
		
		switch (select(pdrvPvt->fd + 1, &rfds, NULL, NULL, &tv))
		{
			case -1:
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, select() failed with %s.\n", FUNCNAME, pdrvPvt->portName, strerror(errno));
	
				return (-1);
				break;
			}
			
			case 0:
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, select() timeout.\n", FUNCNAME, pdrvPvt->portName);

				return (-1);
				break;
			}
			
			default:
			{
				break;
			}
		}

		errno = 0;
	
		memset(pdrvPvt->buffer, 0, sizeof(pdrvPvt->buffer)); 
	
		if ((recvlen = read(pdrvPvt->fd, pdrvPvt->buffer, FINS_MAX_MSG)) < 0)
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, read() with %s.\n", FUNCNAME, pdrvPvt->portName, strerror(errno));
			return (-1);
		}

		asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->buffer, recvlen, "%s: port %s, received %d bytes.\n", FUNCNAME, pdrvPvt->portName, recvlen);
		
		epicsSnprintf(pdrvPvt->reply, sizeof(pdrvPvt->reply), "%s%s", pdrvPvt->reply, pdrvPvt->buffer);
		totalLength += recvlen;

		if (pdrvPvt->buffer[recvlen - 1] == '\r')
		{
			return (totalLength);
		}
	}
	while (1);
	
	return (-1);
}

/*
	Form a FINS read message, send request, wait for the reply and check for errors
	
	This function knows about various message types an forms the correct message
	and processes the reply based on pasynUser->reason.

	data		epicsInt16, epicsInt32 or epicsFloat32 data is written here
	nelements	number of 16 or 32 bit words to read
	address	PLC memory address
	asynSize	sizeof(epicsInt16) for asynInt16Array or sizeof(epicsInt32) for asynInt16Array and asynInt32Array.
*/

static int finsHostlinkread(drvPvt *pdrvPvt, asynUser *pasynUser, void *data, const size_t nelements, const epicsUInt16 address, size_t *transfered, size_t asynSize)
{
	static const char * const FUNCNAME = "finsHostlinkread";
	int recvlen, sendlen = 0;
	epicsTimeStamp ets, ete;

/* initialise header */

	static const char * const header = "@00FA000000000";
	
	switch (pasynUser->reason)
	{
	
	/* Memory read */
	
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		{

		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_READ:
				case FINS_DM_WRITE:
				{
					sprintf(pdrvPvt->message, "%s" "0101"  "%s" "%04u" "00" "%04x", header, DM, address, (unsigned int) nelements);
					break;
				}

				case FINS_AR_READ:
				case FINS_AR_WRITE:
				{
					sprintf(pdrvPvt->message, "%s" "0101"  "%s" "%04u" "00" "%04x", header, AR, address, (unsigned int) nelements);
					break;
				}

				case FINS_IO_READ:
				case FINS_IO_WRITE:
				{
					sprintf(pdrvPvt->message, "%s" "0101"  "%s" "%04u" "00" "%04x", header, IO, address, (unsigned int) nelements);
					break;
				}

				default:
				{
					asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, bad switch.\n", FUNCNAME, pdrvPvt->portName);
					return (-1);
				}
			}

			break;
		}

		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{

		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_READ_32:
				case FINS_DM_WRITE_32:
				{
					sprintf(pdrvPvt->message, "%s" "0101"  "%s" "%04u" "00" "%04x", header, DM, address, (unsigned int) nelements * 2);
					break;
				}
				
				case FINS_AR_READ_32:
				case FINS_AR_WRITE_32:
				{
					sprintf(pdrvPvt->message, "%s" "0101"  "%s" "%04u" "00" "%04x", header, AR, address, (unsigned int) nelements * 2);
					break;
				}
				
				case FINS_IO_READ_32:
				case FINS_IO_WRITE_32:
				{
					sprintf(pdrvPvt->message, "%s" "0101"  "%s" "%04u" "00" "%04x", header, IO, address, (unsigned int) nelements * 2);
					break;
				}
				
				default:
				{
					return (-1);
				}
			}
			
			break;
		}
		
	/* Multiple memory read */
	
	/*
		Allow the user to configure a number of non-consecutive 16 bit memory locations and types.
		The address parameter is used as an index into the array.
	*/
		case FINS_READ_MULTI:
		{
#if 0
			unsigned char *mm = &pdrvPvt->message[COM];
			
			pdrvPvt->message[MRC] = 0x01;
			pdrvPvt->message[SRC] = 0x04;

			sendlen = COM;
			
			for (n = 0; n < pdrvPvt->mmList[address].length; n++)
			{
				*mm++ = pdrvPvt->mmList[address].type[n];
				*mm++ = pdrvPvt->mmList[address].addr[n] >> 8;
				*mm++ = pdrvPvt->mmList[address].addr[n] & 0xff;
				*mm++ = 0x00;
				
				sendlen += 4;
			}
#endif
			break;
		}
		
		case FINS_MODEL:
		{
			sprintf(pdrvPvt->message, "%s%s%02d01", header, "0502", address);
			break;
		}
		
		case FINS_CPU_STATUS:
		case FINS_CPU_MODE:
		{
			sprintf(pdrvPvt->message, "%s%s", header, "0601");
			break;
		}
	
		case FINS_CYCLE_TIME:
		case FINS_CYCLE_TIME_MEAN:
		case FINS_CYCLE_TIME_MAX:
		case FINS_CYCLE_TIME_MIN:
		{
			sprintf(pdrvPvt->message, "%s%s", header, "062001");	
			break;
		}

		case FINS_CLOCK_READ:
		{
			sprintf(pdrvPvt->message, "%s%s", header, "0701");
			break;
		}
			
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (-1);
		}
	}
	
/* flush any old data */

	flushUDP("finsHostlinkread", pdrvPvt, pasynUser);

/* add the checksum */

	sprintf(pdrvPvt->buffer, "%02X*\r", checksum(pdrvPvt->message));
	strcat(pdrvPvt->message, pdrvPvt->buffer);

	sendlen = strlen(pdrvPvt->message);
	
	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->message, sendlen, "%s: port %s, sending %d bytes.\n", FUNCNAME, pdrvPvt->portName, sendlen);

	epicsTimeGetCurrent(&ets);
	
/* send request */

	errno = 0;
	
	if (write(pdrvPvt->fd, pdrvPvt->message, sendlen) != sendlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, write() failed with %s.\n", FUNCNAME, pdrvPvt->portName, strerror(errno));
		return (-1);
	}

/* receive reply with timeout */

	if ((recvlen = ReadUntilTerminator(pdrvPvt, pasynUser)) < 0)
	{
		return (-1);
	}

	epicsTimeGetCurrent(&ete);
	
	{
		const double diff = epicsTimeDiffInSeconds(&ete, &ets);
	
		if (diff > pdrvPvt->tMax) pdrvPvt->tMax = diff;
		if (diff < pdrvPvt->tMin) pdrvPvt->tMin = diff;
		
		pdrvPvt->tLast = diff;
	}
	
/*	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->reply, recvlen, "%s: port %s, received %d bytes.\n", FUNCNAME, pdrvPvt->portName, recvlen); */

/* check response code */

	if (strncmp(&pdrvPvt->reply[MRES], "0000", 4) != 0)
	{
		short mres, sres;
		
		sscanf(&pdrvPvt->reply[MRES], "%2hx", &mres);
		sscanf(&pdrvPvt->reply[SRES], "%2hx", &sres);
		
		FINSerror(pdrvPvt, pasynUser, mres, sres);
		return (-1);
	}

	if (extractAndCompareChecksum(pdrvPvt, recvlen - 4) < 0)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, checksum error.\n", FUNCNAME, pdrvPvt->portName);
		return (-1);
	}

/* extract data */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		{

		/* asynInt16Array */
		
			if (asynSize == sizeof(epicsUInt16))
			{
				int i;
				epicsUInt32 *ptrs = (epicsUInt32 *) &pdrvPvt->reply[RESP];
				epicsUInt16 *ptrd = (epicsUInt16 *) data;

				for (i = 0; i < nelements; i++)
				{
					sscanf((const char *) &ptrs[i], "%4hx", (unsigned short *) &ptrd[i]);
				}
				
				asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %d 16-bit words.\n", FUNCNAME, pdrvPvt->portName, (int) nelements);
			}
			else
			
		/* asynInt32 * 1 */
		
			{
				sscanf(&pdrvPvt->reply[RESP], "%4x", (unsigned int *) data);

				asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %d 16-bit word.\n", FUNCNAME, pdrvPvt->portName, (int) nelements);
			}

			if (transfered)
			{
				*transfered = nelements;
			}
			
			break;
		}

		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{		
			int i;
			epicsUInt32 *ptrs = (epicsUInt32 *) &pdrvPvt->reply[RESP];
			epicsUInt32 *ptrd = (epicsUInt32 *) data;

			for (i = 0; i < nelements; i++)
			{
				sscanf((const char *) &ptrs[i], "%8x", (unsigned int *) &ptrd[i]);
			}
				
			asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %d 32-bit words.\n", FUNCNAME, pdrvPvt->portName, (int) nelements);
		
			if (transfered)
			{
				*transfered = nelements;
			}
			
			break;
		}
		
/* return a string of 20 chars - each character byte encoded as two hex characters so space = ASCII(2) + ASCII(0) */

		case FINS_MODEL:
		{
			int i;
			char *ptrs = (char *) &pdrvPvt->reply[RESP + 4];
			char *ptrd = (char *) data;
			
			for (i = 0; i < FINS_MODEL_LENGTH; i++)
			{
				short c;
				
				sscanf(&ptrs[2*i], "%2hx", &c);
				ptrd[i] = c;
			}
			
			if (transfered)
			{
				*transfered = FINS_MODEL_LENGTH;
			}
			
			break;
		}

/* return status - epicsInt32 */

		case FINS_CPU_STATUS:
		{
			sscanf(&pdrvPvt->reply[RESP + 0], "%2x", (unsigned int *) data);
			
			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}
		
/* return mode - epicsInt32 */

		case FINS_CPU_MODE:
		{
			sscanf(&pdrvPvt->reply[RESP + 2], "%2x", (unsigned int *) data);

			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}

/* return 3 parameters - epicsInt32 */

		case FINS_CYCLE_TIME:
		{
			unsigned int *dat = (unsigned int *) data;

			sscanf(&pdrvPvt->reply[RESP + 0], "%8x%8x%8x", &dat[0], &dat[1], &dat[2]);

			if (transfered)
			{
				*transfered = 3;
			}
			
			break;
		}
		
/* return mean - epicsInt32 */

		case FINS_CYCLE_TIME_MEAN:
		{
			sscanf(&pdrvPvt->reply[RESP + 0], "%8x", (unsigned int *) data);
			
			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}
		
/* return max - epicsInt32 */

		case FINS_CYCLE_TIME_MAX:
		{
			sscanf(&pdrvPvt->reply[RESP + 4], "%8x", (unsigned int *) data);

			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}
		
/* return min - epicsInt32 */

		case FINS_CYCLE_TIME_MIN:
		{
			sscanf(&pdrvPvt->reply[RESP + 8], "%8x", (unsigned int *) data);

			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}

		case FINS_CLOCK_READ:
		{
			unsigned short *dat = (unsigned short *) data;

			sscanf(&pdrvPvt->reply[RESP + 0], "%2hd%2hd%2hd%2hd%2hd%2hd%2hd", &dat[0], &dat[1], &dat[2], &dat[3], &dat[4], &dat[5], &dat[6]);
			
			if (transfered)
			{
				*transfered = 7;
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (-1);
		}
	}

	return (0);	
}

/*
	asynSize is either sizeof(epicsInt16) for asynInt16Array or sizeof(epicsInt32) for asynInt16Array and asynInt32Array.
*/

static int finsHostlinkwrite(drvPvt *pdrvPvt, asynUser *pasynUser, const void *data, size_t nwords, const epicsUInt16 address, size_t asynSize)
{
	static const char * const FUNCNAME = "finsHostlinkwrite";
	int recvlen, sendlen;
	epicsTimeStamp ets, ete;
	
/* initialise header */

	static const char * const header = "@00FA000000000";

	switch (pasynUser->reason)
	{
	
	/* Memory write */
	
		case FINS_DM_WRITE:
		case FINS_DM_WRITE_NOREAD:
		case FINS_AR_WRITE:
		case FINS_AR_WRITE_NOREAD:
		case FINS_IO_WRITE:
		case FINS_IO_WRITE_NOREAD:
		{
				
		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_WRITE:
				case FINS_DM_WRITE_NOREAD:
				{
					sprintf(pdrvPvt->message, "%s" "0102"  "%s" "%04u" "00" "%04x", header, DM, address, (unsigned int) nwords);
					break;
				}
				
				case FINS_AR_WRITE:
				case FINS_AR_WRITE_NOREAD:
				{
					sprintf(pdrvPvt->message, "%s" "0102"  "%s" "%04u" "00" "%04x", header, AR, address, (unsigned int) nwords);
					break;
				}
				
				case FINS_IO_WRITE:
				case FINS_IO_WRITE_NOREAD:
				{
					sprintf(pdrvPvt->message, "%s" "0102"  "%s" "%04u" "00" "%04x", header, IO, address, (unsigned int) nwords);
					break;
				}
				
				default:
				{
					asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, bad switch.\n", FUNCNAME, pdrvPvt->portName);
					return (-1);
				}
			}

		/* asynInt16Array */
		
			if (asynSize == sizeof(epicsUInt16))
			{
				int i;
				epicsUInt16 *ptrs = (epicsUInt16 *) data;

				for (i = 0; i < nwords; i++)
				{
					sprintf(pdrvPvt->buffer, "%04X", ptrs[i]);
				
					strcat(pdrvPvt->message, pdrvPvt->buffer);
				}
								
				asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %d 16-bit words.\n", FUNCNAME, pdrvPvt->portName, (int) nwords);
			}
			else
			
		/* asynInt32 * 1 */
		
			{
				sprintf(pdrvPvt->buffer, "%04X", *((unsigned int *) data));
				strcat(pdrvPvt->message, pdrvPvt->buffer);
				
				asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %d 16-bit word.\n", FUNCNAME, pdrvPvt->portName, (int) nwords);				
			}
						
			break;
		}

		case FINS_DM_WRITE_32:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32:
		case FINS_AR_WRITE_32_NOREAD:
		case FINS_IO_WRITE_32:
		case FINS_IO_WRITE_32_NOREAD:
		{
				
		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_WRITE_32:
				case FINS_DM_WRITE_32_NOREAD:
				{
					sprintf(pdrvPvt->message, "%s" "0102"  "%s" "%04u" "00" "%04x", header, DM, address,(unsigned int)  nwords);
					break;
				}
				
				case FINS_AR_WRITE_32:
				case FINS_AR_WRITE_32_NOREAD:
				{
					sprintf(pdrvPvt->message, "%s" "0102"  "%s" "%04u" "00" "%04x", header, AR, address, (unsigned int) nwords);
					break;
				}
				
				case FINS_IO_WRITE_32:
				case FINS_IO_WRITE_32_NOREAD:
				{
					sprintf(pdrvPvt->message, "%s" "0102"  "%s" "%04u" "00" "%04x", header, IO, address, (unsigned int) nwords);
					break;
				}
				
				default:
				{
					return (-1);
				}
			}
			
		/* convert data  */

			{
				int i;
				epicsUInt32 *ptrs = (epicsUInt32 *) data;

				for (i = 0; i < nwords / 2; i++)
				{
					sprintf(pdrvPvt->buffer, "%08X", ptrs[i]);
				
					strcat(pdrvPvt->message, pdrvPvt->buffer);
				}
				
				asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %d 32-bit words.\n", FUNCNAME, pdrvPvt->portName, (int) nwords >> 1);
			}
			
			break;
		}

	/* cycle time reset */
	
		case FINS_CYCLE_TIME_RESET:
		{
			sprintf(pdrvPvt->message, "%s%s", header, "062000");
						
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (-1);
		}
	}

/* add the checksum and terminator */

	sprintf(pdrvPvt->buffer, "%02X*\r", checksum(pdrvPvt->message));
	strcat(pdrvPvt->message, pdrvPvt->buffer);

	sendlen = strlen(pdrvPvt->message);
	
/* flush any old data */

	flushUDP("finsHostlinkwrite", pdrvPvt, pasynUser);
	
	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->message, sendlen, "%s: port %s, sending %d bytes.\n", FUNCNAME, pdrvPvt->portName, sendlen);
	
	epicsTimeGetCurrent(&ets);
	
/* send request */

	errno = 0;
	
	if (write(pdrvPvt->fd, pdrvPvt->message, sendlen) != sendlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, write() failed with %s.\n", FUNCNAME, pdrvPvt->portName, strerror(errno));
		return (-1);
	}

/* receive reply with timeout */

	{
		fd_set rfds;
		struct timeval tv;
		
		FD_ZERO(&rfds);
		FD_SET(pdrvPvt->fd, &rfds);
		
	/* timeout */

		if (pasynUser->timeout > 0.0)
		{
			tv.tv_sec = (long) pasynUser->timeout;
			tv.tv_usec = 0;
		}
		else
		{
			tv.tv_sec = FINS_TIMEOUT;
			tv.tv_usec = 0;
		}
		
		errno = 0;
		
		switch (select(pdrvPvt->fd + 1, &rfds, NULL, NULL, &tv))
		{
			case -1:
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, select() failed with %s.\n", FUNCNAME, pdrvPvt->portName, strerror(errno));

				return (-1);
				break;
			}
			
			case 0:
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, select() timeout.\n", FUNCNAME, pdrvPvt->portName);
				
				return (-1);
				break;
			}
			
			default:
			{
				break;
			}
		}
	}

	if ((recvlen = read(pdrvPvt->fd, pdrvPvt->reply, FINS_MAX_MSG)) < 0)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, read() error.\n", FUNCNAME, pdrvPvt->portName);
		return (-1);
	}

	epicsTimeGetCurrent(&ete);

	{
		const double diff = epicsTimeDiffInSeconds(&ete, &ets);
	
		if (diff > pdrvPvt->tMax) pdrvPvt->tMax = diff;
		if (diff < pdrvPvt->tMin) pdrvPvt->tMin = diff;
		
		pdrvPvt->tLast = diff;
	}
	
	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->reply, recvlen, "%s: port %s, received %d bytes.\n", FUNCNAME, pdrvPvt->portName, recvlen);

/* check response code */

	if (strncmp(&pdrvPvt->reply[MRES], "0000", 4) != 0)
	{
		FINSerror(pdrvPvt, pasynUser, pdrvPvt->reply[MRES], pdrvPvt->reply[SRES]);
		return (-1);
	}

/* checksum */

	if (extractAndCompareChecksum(pdrvPvt, recvlen - 4) < 0)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, checksum error.\n", FUNCNAME, pdrvPvt->portName);
		return (-1);
	}
	
	return (0);
}

/*** asynOctet ************************************************************************************/

/*

	We use asynOctet to read character strings.
	We could also use it for EXPLICIT MESSAGE SEND (0x28 0x01) commands
*/

static asynStatus udpRead(void *pvt, asynUser *pasynUser, char *data, size_t maxchars, size_t *nbytesTransfered, int *eomReason)
{
	static const char * const FUNCNAME = "udpRead";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	*eomReason = 0;
	*nbytesTransfered = 0;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}
	
/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_MODEL:
		{
			type = "FINS_MODEL";
			
			if (maxchars < FINS_MODEL_LENGTH)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, length is not >= %d for FINS_MODEL\n", FUNCNAME, pdrvPvt->portName, addr, FINS_MODEL_LENGTH);
				return (asynError);
			}
			
			break;
		}
		
	/* no more reasons for asynOctetRead */
	
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

/* send FINS request */

	if (finsHostlinkread(pdrvPvt, pasynUser, (void *) data, maxchars, addr, nbytesTransfered, 0) < 0)
	{
		return (asynError);
	}
	
	if (eomReason)
	{
		*eomReason |= ASYN_EOM_END;
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %d bytes.\n", FUNCNAME, pdrvPvt->portName, addr, (int) *nbytesTransfered);

   	return (asynSuccess);
}

/*
	Form a FINS write message, send request, wait for the reply and check for errors

	Parameters required:

		command type	read, write, cpu status, cycle time etc. Set by pasynUser->reason
		memory type		DM, IO, AR, CT. Set by pasynUser->reason
		start address	Set by asyn address
		data length		Determined by record type
		
		asyn("FINS0", 0xffff, 1) FINS_MODEL, FINS_CYCLE_TIME_RESET, ...
		
	nwords is only used for memory access operations like DM, AR, IO arrays of DM, AR, IO 16/32 access 
*/

static asynStatus udpWrite(void *pvt, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransfered)
{
	static const char * const FUNCNAME = "udpWrite";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	*nbytesTransfered = 0;

	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}
	
/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_CYCLE_TIME_RESET:
		{
			type = "FINS_CYCLE_TIME_RESET";
			
			break;			/* numchars is not used because the message has a fixed size */
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);
	
/* form FINS message and send data */
	
	if (finsHostlinkwrite(pdrvPvt, pasynUser, (void *) data, numchars, addr, 0) < 0)
	{
		return (asynError);
	}

/* assume for now that we can always write the full request */

	*nbytesTransfered = numchars;

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %d bytes.\n", FUNCNAME, pdrvPvt->portName, addr, (int) numchars);

   	return (asynSuccess);
}

/*** asynInt32 ************************************************************************************/

static asynStatus ReadInt32(void *pvt, asynUser *pasynUser, epicsInt32 *value)
{
	static const char * const FUNCNAME = "ReadInt32";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		{
			type = "FINS_DM_READ";
			break;
		}
		
		case FINS_AR_READ:
		{
			type = "FINS_AR_READ";
			break;
		}
		
		case FINS_IO_READ:
		{
			type = "FINS_IO_READ";
			break;
		}
		
		case FINS_DM_READ_32:
		{
			type = "FINS_DM_READ_32";
			break;
		}
		
		case FINS_AR_READ_32:
		{
			type = "FINS_AR_READ_32";
			break;
		}
		
		case FINS_IO_READ_32:
		{
			type = "FINS_IO_READ_32";
			break;
		}
		
		case FINS_CYCLE_TIME_MEAN:
		{
			type = "FINS_CYCLE_TIME_MEAN";
			break;
		}
		
		case FINS_CYCLE_TIME_MAX:
		{
			type = "FINS_CYCLE_TIME_MAX";
			break;
		}
		
		case FINS_CYCLE_TIME_MIN:
		{
			type = "FINS_CYCLE_TIME_MIN";
			break;
		}
		
		case FINS_CPU_STATUS:
		{
			type = "FINS_CPU_STATUS";
			break;
		}
		
		case FINS_CPU_MODE:
		{
			type = "FINS_CPU_MODE";
			break;
		}

	/* this gets called at initialisation by write methods */
	
		case FINS_DM_WRITE:
		case FINS_IO_WRITE:
		case FINS_AR_WRITE:
		case FINS_CT_WRITE:
		case FINS_DM_WRITE_32:
		case FINS_IO_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_CT_WRITE_32:
		{
			type = "WRITE";
			break;
		}

		case FINS_DM_WRITE_NOREAD:
		case FINS_IO_WRITE_NOREAD:
		case FINS_AR_WRITE_NOREAD:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_IO_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32_NOREAD:
		{
			asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, WRITE_NOREAD\n", FUNCNAME, pdrvPvt->portName, addr);
			return (asynError);
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, no such command %d.\n", FUNCNAME, pdrvPvt->portName, addr, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

/* send FINS request */

	if (finsHostlinkread(pdrvPvt, pasynUser, (void *) value, 1, addr, NULL, sizeof(epicsUInt32)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read 1 word.\n", FUNCNAME, pdrvPvt->portName, addr);

	return (asynSuccess);
}

static asynStatus WriteInt32(void *pvt, asynUser *pasynUser, epicsInt32 value)
{
	static const char * const FUNCNAME = "WriteInt32";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		{
			type = "FINS_DM_WRITE";
			break;
		}

		case FINS_DM_WRITE_NOREAD:
		{
			type = "FINS_DM_WRITE_NOREAD";
			break;
		}
		
		case FINS_AR_WRITE:
		{
			type = "FINS_AR_WRITE";
			break;
		}

		case FINS_AR_WRITE_NOREAD:
		{
			type = "FINS_AR_WRITE_NOREAD";
			break;
		}
		
		case FINS_IO_WRITE:
		{
			type = "FINS_IO_WRITE";
			break;
		}
		
		case FINS_IO_WRITE_NOREAD:
		{
			type = "FINS_IO_WRITE_NOREAD";
			break;
		}

		case FINS_CYCLE_TIME_RESET:
		{
			type = "FINS_CYCLE_TIME_RESET";
			break;
		}

		case FINS_DM_WRITE_32:
		{
			type = "FINS_DM_WRITE_32";
			break;
		}

		case FINS_DM_WRITE_32_NOREAD:
		{
			type = "FINS_DM_WRITE_32_NOREAD";
			break;
		}
		
		case FINS_AR_WRITE_32:
		{
			type = "FINS_AR_WRITE_32";
			break;
		}
		
		case FINS_AR_WRITE_32_NOREAD:
		{
			type = "FINS_AR_WRITE_32_NOREAD";
			break;
		}
		
		case FINS_IO_WRITE_32:
		{
			type = "FINS_IO_WRITE_32";
			break;
		}
		
		case FINS_IO_WRITE_32_NOREAD:
		{
			type = "FINS_IO_WRITE_32_NOREAD";
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);
	
	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		case FINS_DM_WRITE_NOREAD:
		case FINS_AR_WRITE:
		case FINS_AR_WRITE_NOREAD:
		case FINS_IO_WRITE:
		case FINS_IO_WRITE_NOREAD:
		case FINS_CYCLE_TIME_RESET:
		{
			
		/* form FINS message and send data */

			if (finsHostlinkwrite(pdrvPvt, pasynUser, (void *) &value, sizeof(epicsInt16) / sizeof(epicsInt16), addr, sizeof(epicsUInt32)) < 0)
			{
				return (asynError);
			}
			
			break;
		}

		case FINS_DM_WRITE_32:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32:
		case FINS_AR_WRITE_32_NOREAD:
		case FINS_IO_WRITE_32:
		case FINS_IO_WRITE_32_NOREAD:
		{
			
		/* form FINS message and send data */

			if (finsHostlinkwrite(pdrvPvt, pasynUser, (void *) &value, sizeof(epicsInt32) / sizeof(epicsInt16), addr, sizeof(epicsUInt32)) < 0)
			{
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote 1 word.\n", FUNCNAME, pdrvPvt->portName, addr);

	return (asynSuccess);
}

/*** asynFloat64 **********************************************************************************/

static asynStatus ReadFloat64(void *pvt, asynUser *pasynUser, epicsFloat64 *value)
{
	static const char * const FUNCNAME = "ReadFloat64";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		{
			type = "FINS_DM_READ";
			break;
		}
		
		case FINS_AR_READ:
		{
			type = "FINS_AR_READ";
			break;
		}
		
		case FINS_IO_READ:
		{
			type = "FINS_IO_READ";
			break;
		}
		
		case FINS_DM_READ_32:
		{
			type = "FINS_DM_READ_32";
			break;
		}
		
		case FINS_AR_READ_32:
		{
			type = "FINS_AR_READ_32";
			break;
		}
		
		case FINS_IO_READ_32:
		{
			type = "FINS_IO_READ_32";
			break;
		}
		
		case FINS_CYCLE_TIME_MEAN:
		{
			type = "FINS_CYCLE_TIME_MEAN";
			break;
		}
		
		case FINS_CYCLE_TIME_MAX:
		{
			type = "FINS_CYCLE_TIME_MAX";
			break;
		}
		
		case FINS_CYCLE_TIME_MIN:
		{
			type = "FINS_CYCLE_TIME_MIN";
			break;
		}
		
		case FINS_CPU_STATUS:
		{
			type = "FINS_CPU_STATUS";
			break;
		}
		
		case FINS_CPU_MODE:
		{
			type = "FINS_CPU_MODE";
			break;
		}

	/* this gets called at initialisation by write methods */
	
		case FINS_DM_WRITE:
		case FINS_IO_WRITE:
		case FINS_AR_WRITE:
		case FINS_CT_WRITE:
		case FINS_DM_WRITE_32:
		case FINS_IO_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_CT_WRITE_32:
		{
			type = "WRITE";
			break;
		}

		case FINS_DM_WRITE_NOREAD:
		case FINS_IO_WRITE_NOREAD:
		case FINS_AR_WRITE_NOREAD:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_IO_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32_NOREAD:
		{
			asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, WRITE_NOREAD\n", FUNCNAME, pdrvPvt->portName, addr);
			return (asynError);
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, no such command %d.\n", FUNCNAME, pdrvPvt->portName, addr, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

/* send FINS request */

	{
		volatile epicsFloat32 val;
		volatile epicsUInt32 * const v = (epicsUInt32 *) &val;
		
		if (finsHostlinkread(pdrvPvt, pasynUser, (void *) &val, sizeof(epicsInt32) / sizeof(epicsInt16), addr, NULL, sizeof(epicsUInt32)) < 0)
		{
			return (asynError);
		}

		*v = WSWAP32(*v);
			
		*value = (epicsFloat64) val;
	}
	
	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read 1 word.\n", FUNCNAME, pdrvPvt->portName, addr);

	return (asynSuccess);
}

static asynStatus WriteFloat64(void *pvt, asynUser *pasynUser, epicsFloat64 value)
{
	static const char * const FUNCNAME = "WriteFloat64";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		{
			type = "FINS_DM_WRITE_32";
			break;
		}

		case FINS_DM_WRITE_32_NOREAD:
		{
			type = "FINS_DM_WRITE_32_NOREAD";
			break;
		}
		
		case FINS_AR_WRITE_32:
		{
			type = "FINS_AR_WRITE_32";
			break;
		}
		
		case FINS_AR_WRITE_32_NOREAD:
		{
			type = "FINS_AR_WRITE_32_NOREAD";
			break;
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);
	
	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32:
		case FINS_AR_WRITE_32_NOREAD:
		{
			volatile epicsFloat32 val = (epicsFloat32) value;
			volatile epicsUInt32 * const v = (epicsUInt32 *) &val;

			*v = WSWAP32(*v);
			
		/* form FINS message and send data */

			if (finsHostlinkwrite(pdrvPvt, pasynUser, (void *) &val, sizeof(epicsInt32) / sizeof(epicsInt16), addr, sizeof(epicsUInt32)) < 0)
			{
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote 1 word.\n", FUNCNAME, pdrvPvt->portName, addr);

	return (asynSuccess);
}

/*** asynInt16Array *******************************************************************************/

static asynStatus ReadInt16Array(void *pvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements, size_t *nIn)
{
	static const char * const FUNCNAME = "ReadInt16Array";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		{
			type = "FINS_DM_READ";
			break;
		}
		
		case FINS_AR_READ:
		{
			type = "FINS_AR_READ";
			break;
		}
		case FINS_IO_READ:
		{
			type = "FINS_IO_READ";
			break;
		}
		
		case FINS_CLOCK_READ:
		{
			type = "FINS_CLOCK_READ";
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		{
			if (nelements > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big.\n", FUNCNAME, pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		case FINS_CLOCK_READ:
		{
			if (nelements != 7)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_CLOCK_READ size != 7.\n", FUNCNAME, pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}
		
/* send FINS request */

	if (finsHostlinkread(pdrvPvt, pasynUser, (char *) value, nelements, addr, nIn, sizeof(epicsUInt16)) < 0)
	{
		*nIn = 0;
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %d 16-bit words.\n", FUNCNAME, pdrvPvt->portName, addr, (int) *nIn);

	return (asynSuccess);
}

static asynStatus WriteInt16Array(void *pvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements)
{
	static const char * const FUNCNAME = "WriteInt16Array";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		{
			type = "FINS_DM_WRITE";
			break;
		}
		
		case FINS_AR_WRITE:
		{
			type = "FINS_AR_WRITE";
			break;
		}
		case FINS_IO_WRITE:
		{
			type = "FINS_IO_WRITE";
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		{
			if (nelements > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big.\n", FUNCNAME, pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
/* form FINS message and send data */

	if (finsHostlinkwrite(pdrvPvt, pasynUser, (void *) value, nelements * sizeof(epicsInt16) / sizeof(epicsInt16), addr, sizeof(epicsUInt16)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %d 16-bit words.\n", FUNCNAME, pdrvPvt->portName, addr, (int) nelements);

	return (asynSuccess);
}

/*** asynInt32Array *******************************************************************************/

static asynStatus ReadInt32Array(void *pvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements, size_t *nIn)
{
	static const char * const FUNCNAME = "ReadInt32Array";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		{
			type = "FINS_DM_READ_32";
			break;
		}
		
		case FINS_AR_READ_32:
		{
			type = "FINS_AR_READ_32";
			break;
		}
		
		case FINS_IO_READ_32:
		{
			type = "FINS_IO_READ_32";
			break;
		}
		
		case FINS_CYCLE_TIME:
		{
			type = "FINS_CYCLE_TIME";
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		{
			if ((nelements * 2) > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big\n", FUNCNAME, pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}

		case FINS_CYCLE_TIME:
		{
			if (nelements != 3)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request %d too small.\n", FUNCNAME, pdrvPvt->portName, addr, (int) nelements);
				return (asynError);
			}
			
			break;
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

/* send FINS request */

	if (finsHostlinkread(pdrvPvt, pasynUser, (void *) value, nelements, addr, nIn, sizeof(epicsUInt32)) < 0)
	{
		*nIn = 0;
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %d 32-bit words.\n", FUNCNAME, pdrvPvt->portName, addr, (int) *nIn);
	
	return (asynSuccess);
}

static asynStatus WriteInt32Array(void *pvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements)
{
	static const char * const FUNCNAME = "WriteInt32Array";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		{
			type = "FINS_DM_WRITE_32";
			break;
		}
		
		case FINS_AR_WRITE_32:
		{
			type = "FINS_AR_WRITE_32";
			break;
		}
		
		case FINS_IO_WRITE_32:
		{
			type = "FINS_IO_WRITE_32";
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{
			if ((nelements * 2) > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big.\n", FUNCNAME, pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
/* form FINS message and send data */

	if (finsHostlinkwrite(pdrvPvt, pasynUser, (void *) value, nelements * sizeof(epicsInt32) / sizeof(epicsInt16), addr, sizeof(epicsUInt32)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %d 32-bit words.\n", FUNCNAME, pdrvPvt->portName, addr, (int) nelements);

	return (asynSuccess);
}

/*** asynFloat32Array *****************************************************************************/

/*
	Read 32 bit values from the PLC which are encoded as IEEE floats
*/

static asynStatus ReadFloat32Array(void *pvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements, size_t *nIn)
{
	static const char * const FUNCNAME = "ReadFloat32Array";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		{
			type = "FINS_DM_READ_32";
			break;
		}
		
		case FINS_AR_READ_32:
		{
			type = "FINS_AR_READ_32";
			break;
		}
		
		case FINS_IO_READ_32:
		{
			type = "FINS_IO_READ_32";
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		{
			if ((nelements * 2) > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big.\n", FUNCNAME, pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
/* send FINS request */

	if (finsHostlinkread(pdrvPvt, pasynUser, (void *) value, nelements, addr, nIn, sizeof(epicsInt32)) < 0)
	{
		*nIn = 0;
		return (asynError);
	}

/* Need to perform word swapping */

	{
		int i;
		
		for (i = 0; i < nelements; i++)
		{
			unsigned int *p = (unsigned int *) &value[i];
			
			*p = WSWAP32(*p);
		}
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %d floats.\n", FUNCNAME, pdrvPvt->portName, addr, (int) *nIn);
	
	return (asynSuccess);
}

static asynStatus WriteFloat32Array(void *pvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements)
{
	static const char * const FUNCNAME = "WriteFloat32Array";
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	char *type = NULL;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		{
			type = "FINS_DM_WRITE_32";
			break;
		}
		
		case FINS_AR_WRITE_32:
		{
			type = "FINS_AR_WRITE_32";
			break;
		}
		
		case FINS_IO_WRITE_32:
		{
			type = "FINS_IO_WRITE_32";
			break;
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", FUNCNAME, pdrvPvt->portName, addr, type);

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{
			if ((nelements * 2) > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big.\n", FUNCNAME, pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", FUNCNAME, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

/* Need to perform word swapping */

	{
		int i;
		
		for (i = 0; i < nelements; i++)
		{
			unsigned int *p = (unsigned int *) &value[i];
			
			*p = WSWAP32(*p);
		}
	}
	
/* form FINS message and send data */

	if (finsHostlinkwrite(pdrvPvt, pasynUser, (void *) value, nelements * sizeof(epicsFloat32) / sizeof(epicsInt16), addr, sizeof(epicsInt32)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %d floats.\n", FUNCNAME, pdrvPvt->portName, addr, (int) nelements);

	return (asynSuccess);
}

/*** asynDrvUser **********************************************************************************/

static asynStatus drvUserDestroy(void *drvPvt,asynUser *pasynUser)
{
	return asynSuccess;
}

static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser, const char **pptypeName, size_t *psize)
{
	*psize = 0;
	return (asynSuccess);
}

static asynStatus drvUserCreate(void *pvt, asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;

	if (drvInfo)
	{
		if (strcmp("FINS_DM_READ", drvInfo) == 0)
		{
			pasynUser->reason = FINS_DM_READ;
		}
		else
		if (strcmp("FINS_DM_READ_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_DM_READ_32;
		}
		else
		if (strcmp("FINS_DM_WRITE", drvInfo) == 0)
		{
			pasynUser->reason = FINS_DM_WRITE;
		}
		else
		if (strcmp("FINS_DM_WRITE_NOREAD", drvInfo) == 0)
		{
			pasynUser->reason = FINS_DM_WRITE_NOREAD;
		}
		else
		if (strcmp("FINS_DM_WRITE_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_DM_WRITE_32;
		}
		else
		if (strcmp("FINS_DM_WRITE_32_NOREAD", drvInfo) == 0)
		{
			pasynUser->reason = FINS_DM_WRITE_32_NOREAD;
		}
		else
		if (strcmp("FINS_IO_READ", drvInfo) == 0)
		{
			pasynUser->reason = FINS_IO_READ;
		}
		else
		if (strcmp("FINS_IO_READ_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_IO_READ_32;
		}
		else
		if (strcmp("FINS_IO_WRITE", drvInfo) == 0)
		{
			pasynUser->reason = FINS_IO_WRITE;
		}
		else
		if (strcmp("FINS_IO_WRITE_NOREAD", drvInfo) == 0)
		{
			pasynUser->reason = FINS_IO_WRITE_NOREAD;
		}
		else
		if (strcmp("FINS_IO_WRITE_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_IO_WRITE_32;
		}
		else
		if (strcmp("FINS_IO_WRITE_32_NOREAD", drvInfo) == 0)
		{
			pasynUser->reason = FINS_IO_WRITE_32_NOREAD;
		}
		else
		if (strcmp("FINS_AR_READ", drvInfo) == 0)
		{
			pasynUser->reason = FINS_AR_READ;
		}
		else
		if (strcmp("FINS_AR_READ_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_AR_READ_32;
		}
		else
		if (strcmp("FINS_AR_WRITE", drvInfo) == 0)
		{
			pasynUser->reason = FINS_AR_WRITE;
		}
		else
		if (strcmp("FINS_AR_WRITE_NOREAD", drvInfo) == 0)
		{
			pasynUser->reason = FINS_AR_WRITE_NOREAD;
		}
		else
		if (strcmp("FINS_AR_WRITE_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_AR_WRITE_32;
		}
		else
		if (strcmp("FINS_AR_WRITE_32_NOREAD", drvInfo) == 0)
		{
			pasynUser->reason = FINS_AR_WRITE_32_NOREAD;
		}
		else
		if (strcmp("FINS_CT_READ", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CT_READ;
		}
		else
		if (strcmp("FINS_CT_WRITE", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CT_WRITE;
		}
		else
		if (strcmp("FINS_CPU_STATUS", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CPU_STATUS;
		}
		else
		if (strcmp("FINS_CPU_MODE", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CPU_MODE;
		}
		else
		if (strcmp("FINS_MODEL", drvInfo) == 0)
		{
			pasynUser->reason = FINS_MODEL;
		}
		else
		if (strcmp("FINS_CYCLE_TIME_RESET", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CYCLE_TIME_RESET;
		}
		else
		if (strcmp("FINS_CYCLE_TIME", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CYCLE_TIME;
		}
		else
		if (strcmp("FINS_CYCLE_TIME_MEAN", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CYCLE_TIME_MEAN;
		}
		else
		if (strcmp("FINS_CYCLE_TIME_MAX", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CYCLE_TIME_MAX;
		}
		else
		if (strcmp("FINS_CYCLE_TIME_MIN", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CYCLE_TIME_MIN;
		}
		else
		if (strcmp("FINS_MONITOR", drvInfo) == 0)
		{
			pasynUser->reason = FINS_MONITOR;
		}
		else
		if (strcmp("FINS_CLOCK_READ", drvInfo) == 0)
		{
			pasynUser->reason = FINS_CLOCK_READ;
		}
		else
		if (strcmp("FINS_EXPLICIT", drvInfo) == 0)
		{
			pasynUser->reason = FINS_EXPLICIT;
		}
		else
		{
			pasynUser->reason = FINS_NULL;
		}

		asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "drvUserCreate: port %s, %s = %d\n", pdrvPvt->portName, drvInfo, pasynUser->reason);

		return (asynSuccess);
	}

	return (asynError);
}

static const char * const error01 = "Local node error";
static const char * const error02 = "Destination node error";
static const char * const error03 = "Communications controller error";
static const char * const error04 = "Not executable";
static const char * const error05 = "Routing error";
static const char * const error10 = "Command format error";
static const char * const error11 = "Parameter error";
static const char * const error20 = "Read not possible";
static const char * const error21 = "Write not possible";
static const char * const error22 = "Not executable in curent mode";
static const char * const error23 = "No unit";
static const char * const error24 = "Start/Stop not possible";
static const char * const error25 = "Unit error";
static const char * const error26 = "Command error";
static const char * const error30 = "Access rights error";
static const char * const error40 = "Abort error";

static void FINSerror(drvPvt *pdrvPvt, asynUser *pasynUser, const unsigned char mres, const unsigned char sres)
{
	if (mres & 0x80)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, Relay Error Flag\n", pdrvPvt->portName);
		
		FINSerror(pdrvPvt, pasynUser, mres ^ 0x80, sres);
	}
	
	switch (mres)
	{
		case 0x01:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error01, sres);
			break;
		}
		
		case 0x02:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error02, sres);
			break;
		}
		
		case 0x03:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error03, sres);
			break;
		}
		
		case 0x04:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error04, sres);
			break;
		}
		
		case 0x05:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error05, sres);
			break;
		}
		
		case 0x10:
		{
			switch (sres)
			{
				case 1:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - command is too long.\n", pdrvPvt->portName, error10);
						break;
				default:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error10, sres);
						break;
			}
			
			break;
		}
		
		case 0x11:
		{
			switch (sres)
			{
				case 0x0B:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - response block is too long.\n", pdrvPvt->portName, error11);
						break;				
				default:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error11, sres);
						break;
			}
			
			break;
		}
		
		case 0x20:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error20, sres);
			break;
		}
		
		case 0x21:
		{
			switch (sres)
			{
				case 1:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - area is read-only\n", pdrvPvt->portName, error21);
						break;
				case 2:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - area is protected\n", pdrvPvt->portName, error21);
						break;
				case 8:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - data cannot be changed\n", pdrvPvt->portName, error21);
						break;
				default:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error21, sres);
						break;
			}
			
			break;
		}
		
		case 0x22:
		{
			switch (sres)
			{
				case 1:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - mode is wrong.\n", pdrvPvt->portName, error22);
						break;
				case 3:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - program mode.\n", pdrvPvt->portName, error22);
						break;
				case 4:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - debug mode.\n", pdrvPvt->portName, error22);
						break;
				case 5:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - monitor mode.\n", pdrvPvt->portName, error22);
						break;
				case 6:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s - run mode.\n", pdrvPvt->portName, error22);
						break;
				default:	asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error22, sres);
						break;
			}
			
			break;
		}
		
		case 0x23:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error23, sres);
			break;
		}
		
		case 0x24:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %se 0x%02x\n", pdrvPvt->portName, error24, sres);
			break;
		}
		
		case 0x25:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error25, sres);
			break;
		}
		
		case 0x26:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error26, sres);
			break;
		}
		
		case 0x30:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error30, sres);
			break;
		}
		
		case 0x40:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, %s 0x%02x\n", pdrvPvt->portName, error40, sres);
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s, Error 0x%02x/0x%02x\n", pdrvPvt->portName, mres, sres);
			break;
		}
	}
}

static const iocshArg finsHostlinkInitArg0 = { "portName", iocshArgString };
static const iocshArg finsHostlinkInitArg1 = { "IP address", iocshArgString };

static const iocshArg *finsHostlinkInitArgs[] = { &finsHostlinkInitArg0, &finsHostlinkInitArg1};
static const iocshFuncDef finsHostlinkInitFuncDef = { "finsHostlinkInit", 2, finsHostlinkInitArgs};

static void finsHostlinkInitCallFunc(const iocshArgBuf *args)
{
	finsHostlinkInit(args[0].sval, args[1].sval);
}

static void finsHostlinkRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsHostlinkInitFuncDef, finsHostlinkInitCallFunc);
	}
}

epicsExportRegistrar(finsHostlinkRegister);

/**************************************************************************************************/

/*
	This is a test function to send a FINS data memory read request for two words from
	address 100 to the specified IP address. It will print the data received as hex, or
	a helpful error message if something fails.
*/

int finsTestHostlink(char *dev)
{
	int fd;
	struct sockaddr_in addr;
	const int addrlen = sizeof(struct sockaddr_in);
	char *message;
	int recvlen, sendlen = 0;
	
	message = (char *) callocMustSucceed(1, FINS_MAX_MSG, "finsTest");
	
/* open a serial device */

	fd = open(dev, 0, O_RDWR);
	
	if (fd < 0)
	{
		perror("finsTest: open");
		return (-1);
	}
	
	bzero((char *) &(addr), addrlen);

/* send a simple FINS command */

	strcpy(message, "@00FA000000000" DM "0101" "100" "0001");

/* send request */

	if (write(fd, message, sendlen) != sendlen)
	{
		perror("finsTest: write");
		close(fd);
		return (-1);
	}

/* receive reply with timeout */

	{
		fd_set rfds;
		struct timeval tv;
		
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		
	/* timeout */

		tv.tv_sec = FINS_TIMEOUT;
		tv.tv_usec = 0;

		switch (select(fd + 1, &rfds, NULL, NULL, &tv))
		{
			case -1:
			{
				perror("finsTest: select");
	
				return (-1);
				break;
			}
			
			case 0:
			{
				perror("finsTest: select");

				return (-1);
				break;
			}

			default:
			{
				break;
			}
		}
	}

	if ((recvlen = read(fd, message, FINS_MAX_MSG)) < 0)
	{
		perror("finsTest: read");
		close(fd);
		return (-1);
	}

	{
		int i;
		
		for (i = 0; i < recvlen; i++)
		{
			printf("0x%02x ", message[i]);
		}
	
		puts("");
	}

/* Illegal response length check */
	
	if (recvlen < MIN_RESP_LEN)
	{
		puts("finsTest: receive length too small.");
	}

/* check response code */

	if ((message[MRES] != 0x00) || (message[SRES] != 0x00))
	{
		if (message[MRES] & 0x80)
		{
			puts("finsTest: Relay Error Flag set");
			
			message[MRES] ^= 0x80;
		}
		
		switch (message[MRES])
		{
			case 0x01:
			{
				printf("%s 0x%02x\n", error01, message[SRES]);
				break;
			}
		
			case 0x02:
			{
				printf("%s 0x%02x\n", error02, message[SRES]);
				break;
			}
		
			case 0x03:
			{
				printf("%s 0x%02x\n", error03, message[SRES]);
				break;
			}
		
			case 0x04:
			{
				printf("%s 0x%02x\n", error04, message[SRES]);
				break;
			}
		
			case 0x05:
			{
				printf("%s 0x%02x\n", error05, message[SRES]);
				break;
			}
		
			case 0x10:
			{
				printf("%s 0x%02x\n", error10, message[SRES]);
				break;
			}
		
			case 0x11:
			{
				printf("%s 0x%02x\n", error11, message[SRES]);
				break;
			}
		
			case 0x20:
			{
				printf("%s 0x%02x\n", error20, message[SRES]);
				break;
			}
		
			case 0x21:
			{
				printf("%s 0x%02x\n", error21, message[SRES]);
				break;
			}
		
			case 0x22:
			{
				printf("%s 0x%02x\n", error22, message[SRES]);
				break;
			}
		
			case 0x23:
			{
				printf("%s 0x%02x\n", error23, message[SRES]);
				break;
			}
		
			case 0x24:
			{
				printf("%s 0x%02x\n", error24, message[SRES]);
				break;
			}
		
			case 0x25:
			{
				printf("%s 0x%02x\n", error25, message[SRES]);
				break;
			}
		
			case 0x26:
			{
				printf("%s 0x%02x\n", error26, message[SRES]);
				break;
			}
		
			case 0x30:
			{
				printf("%s 0x%02x\n", error30, message[SRES]);
				break;
			}
		
			case 0x40:
			{
				printf("%s 0x%02x\n", error40, message[SRES]);
				break;
			}
		
			default:
			{
				printf("Error 0x%02x/0x%02x\n", message[MRES], message[SRES]);
				break;
			}
		}
	}
		
	close(fd);
	
	return (0);
}

static const iocshArg finsTestHostlinkArg0 = { "Serial device", iocshArgString };

static const iocshArg *finsTestHostlinkArgs[] = { &finsTestHostlinkArg0};
static const iocshFuncDef finsTestHostlinkFuncDef = { "finsTestHostlink", 1, finsTestHostlinkArgs};

static void finsTestHostlinkCallFunc(const iocshArgBuf *args)
{
	finsTestHostlink(args[0].sval);
}

static void finsTestHostlinkRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsTestHostlinkFuncDef, finsTestHostlinkCallFunc);
	}
}

epicsExportRegistrar(finsTestHostlinkRegister);

/**************************************************************************************************/
