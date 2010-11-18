/*
	Factory Intelligent Network Service
	
	This is an asyn driver, supporting various asyn interfaces, which acts as both a
	UDP server and client to send requests and receive replies from the Ethernet unit of the PLC.

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
		w	FINS_AR_WRITE
		w	FINS_IO_WRITE
		w	FINS_CYCLE_TIME_RESET
		w	FINS_DM_WRITE_32
		w	FINS_AR_WRITE_32
		w	FINS_IO_WRITE_32
		
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
	
	The commands supported by this driver are for CPU units. They will probably not work if
	commands are sent directly to a CJ1W-PNT21 PROFINET IO Controller.
	
	We assume that the PLC Ethernet unit receives commands on UDP port 9600. It sends replies to the
	port number we use to send the request.
	
	
*/

#include <stdio.h>
#include <string.h>

#ifdef linux
#include <byteswap.h>
#endif

#ifdef vxWorks
#include <sockLib.h>
#include <inetLib.h>
#endif

#include <cantProceed.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsAssert.h>
#include <asynDriver.h>
#include <asynDrvUser.h>
#include <asynOctet.h>
#include <asynInt32.h>
#include <asynInt16Array.h>
#include <asynInt32Array.h>
#include <asynFloat32Array.h>
#include <iocsh.h>
#include <registryFunction.h>
#include <epicsExport.h>
#include <epicsEndian.h>

#include <osiUnistd.h>
#include <osiSock.h>

/* PLC memory  types */

#define DM	0x82
#define IO	0xB0
#define AR	0xB3
#define CT	0x89

/* offsets into the FINS UDP packet */

#define ICF	0
#define RSV	1
#define GCT	2
#define DNA	3
#define DA1	4
#define DA2	5
#define SNA	6
#define SA1	7
#define SA2	8
#define SID	9

#define MRC	10
#define SRC	11
#define COM	12

#define MRES	12
#define SRES	13

#define RESP	14

#define MIN_RESP_LEN	14

/* constants */

#define FINS_UDP_PORT		9600
#define FINS_MAX_WORDS		500
#define FINS_MAX_MSG		((FINS_MAX_WORDS) * 2 + 100)
#define FINS_MAX_HEADER		32
#define FINS_TIMEOUT		1

#define FINS_MODEL_LENGTH	20
#define DEBUG_LEN		256

#ifdef linux
#define BSWAP16(a)	bswap_16((a))
#define BSWAP32(a)	bswap_32((a))
#else
#define BSWAP16(a)	(((a) & 0x00ff) << 8) | (((a) & 0xff00) >> 8)
#define BSWAP32(a)	(((a) & 0x000000ff) << 24) | (((a) & 0x0000ff00) << 8) | (((a) & 0x00ff0000) >> 8) | (((a) & 0xff000000) >> 24)
#endif


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
	asynInterface int16Array;
	asynInterface int32Array;
	asynInterface float32Array;
	void *pasynPvt;			/* For registerInterruptSource */
	
	uint8_t node;
	
	char *debug;
	char *data;
	size_t nbytes;
	size_t msize;

	epicsUInt8 sid;			/* seesion id - increment for each message */
	
	struct sockaddr_in addr;	/* PLC destination address */
	
	uint8_t reply[FINS_MAX_MSG];	/* incoming message buffer */
	uint8_t message[FINS_MAX_MSG];	/* message buffer */
	
} drvPvt;

static void FINSerror(drvPvt *pdrvPvt, asynUser *pasynUser, const char *name, const unsigned char mres, const unsigned char sres);

/*** asynCommon methods ***************************************************************************/

static void report(void *drvPvt,FILE *fp,int details);
static asynStatus aconnect(void *drvPvt,asynUser *pasynUser);
static asynStatus adisconnect(void *drvPvt,asynUser *pasynUser);
static asynCommon asyn = { report, aconnect, adisconnect };

/*** asynOctet methods ****************************************************************************/

static asynStatus udpRead (void *drvPvt, asynUser *pasynUser, char *data, size_t maxchars, size_t *nbytesTransfered,int *eomReason);
static asynStatus udpWrite(void *drvPvt, asynUser *pasynUser, const char *data, size_t numchars,size_t *nbytesTransfered);
static asynStatus flushIt (void *drvPvt, asynUser *pasynUser);

/*** asynInt32 methods ****************************************************************************/

static asynStatus WriteInt32(void *drvPvt, asynUser *pasynUser, epicsInt32 value);
static asynStatus ReadInt32(void *drvPvt, asynUser *pasynUser, epicsInt32 *value);

static asynInt32 ifaceInt32 = { WriteInt32, ReadInt32, NULL, NULL, NULL};

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

asynStatus drvUserCreate (void *drvPvt, asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser, const char **pptypeName, size_t *psize);
asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser);

static asynDrvUser ifaceDrvUser = { drvUserCreate, NULL, NULL };

/**************************************************************************************************/

enum FINS_COMMANDS
{
	FINS_NULL,
	FINS_DM_READ, FINS_DM_WRITE,
	FINS_IO_READ, FINS_IO_WRITE,
	FINS_AR_READ, FINS_AR_WRITE,
	FINS_CT_READ, FINS_CT_WRITE,
	FINS_DM_READ_32, FINS_DM_WRITE_32,
	FINS_IO_READ_32, FINS_IO_WRITE_32,
	FINS_AR_READ_32, FINS_AR_WRITE_32,
	FINS_CT_READ_32, FINS_CT_WRITE_32,
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

#define FUNCNAME "finsUDPInit"

static int finsUDPInit(const char *portName, const char *address)
{
	drvPvt *pdrvPvt;
	asynStatus status;
	asynOctet *pasynOctet;

	printf("%s: PLC IP address %s\n", FUNCNAME, address);
	
	pdrvPvt = callocMustSucceed(1, sizeof(drvPvt), FUNCNAME);
	pdrvPvt->portName = epicsStrDup(portName);
	
	pasynOctet = callocMustSucceed(1, sizeof(asynOctet), FUNCNAME);

/* a couple of buffers for tx and rx data */
	
	pdrvPvt->data = callocMustSucceed(1, 1500, FUNCNAME);
	
	pdrvPvt->debug = callocMustSucceed(1, DEBUG_LEN, FUNCNAME);
	
/* asynCommon */

	pdrvPvt->common.interfaceType = asynCommonType;
	pdrvPvt->common.pinterface = (void *) &asyn;
	pdrvPvt->common.drvPvt = pdrvPvt;

/* ASYN_CANBLOCK but not ASYN_MULTIDEVICE */

	status = pasynManager->registerPort(portName, ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0);

	if (status != asynSuccess)
	{
		printf("%s: driver registerPort failed\n", FUNCNAME);
		return (-1);
	}
	
/* common */

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
	pasynOctet->setInputEos = NULL;
	pasynOctet->getInputEos = NULL;
	
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
	
/* create a UDP socket */

	if ((pdrvPvt->fd = epicsSocketCreate(PF_INET, SOCK_DGRAM, 0)) < 0)
	{
		printf("%s: Can't create socket: %s", FUNCNAME, strerror(SOCKERRNO));
		return -1;
	}

	{
	
	/* create incoming FINS server port - dynamically allocated */
	
		struct sockaddr_in addr;
		const int addrlen = sizeof(struct sockaddr_in);
		
		bzero((char *) &(addr), addrlen);

		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_family = AF_INET;
		addr.sin_port = htons(0);

		if (bind(pdrvPvt->fd, (struct sockaddr *) &addr, addrlen) < 0)
		{
			epicsSocketDestroy(pdrvPvt->fd);
			
			perror("bind failed");
			return (-1);
		}
		
	/* find our port number */
	
		{
			struct sockaddr_in name;
#ifdef vxWorks
			int namelen;
#else
			socklen_t namelen;
#endif			
			getsockname(pdrvPvt->fd, (struct sockaddr *) &name, &namelen);

			printf("%s: port %d bound\n", FUNCNAME, name.sin_port);
		}
		
	/* destination port address used later in sendto() */

		bzero((char *) &pdrvPvt->addr, addrlen);
	
		pdrvPvt->addr.sin_family = AF_INET;
		pdrvPvt->addr.sin_port = htons(FINS_UDP_PORT);

	/*
		We will send on the same socket as we receive. This means that our transmit
		port number is the same as our receive port number. The PLC will sends its
		reply to the same port number as we use for transmitting.
	*/
		if (inet_aton((char *) address, &pdrvPvt->addr.sin_addr) == 0)
		{
			printf("Bad IP address %s\n", address);
			return (-1);
		}

	/* node address is last byte of IP address */
		
		pdrvPvt->node = ntohl(pdrvPvt->addr.sin_addr.s_addr) & 0xff;
		
		printf("%s: PLC node %d\n", FUNCNAME, pdrvPvt->node);
	}
	
 	return (0);
}

static void report(void *pvt, FILE *fp, int details)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	
	pdrvPvt = pdrvPvt;
}

static asynStatus aconnect(void *pvt, asynUser *pasynUser)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	asynStatus status;
	int addr;
	
	status = pasynManager->getAddr(pasynUser, &addr);
    
	if (status != asynSuccess) return status;
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s finsUDP:connect addr %d\n", pdrvPvt->portName, addr);
	
	if (addr >= 0)
	{
		pasynManager->exceptionConnect(pasynUser);
		return (asynSuccess);
	}
	
	if (pdrvPvt->connected)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s finsUDP:connect port already connected\n", pdrvPvt->portName);
		return (asynError);
	}

	pdrvPvt->connected = 1;
	pasynManager->exceptionConnect(pasynUser);
	return (asynSuccess);
}

static asynStatus adisconnect(void *pvt,asynUser *pasynUser)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	asynStatus status;
	int addr;
	
	status = pasynManager->getAddr(pasynUser, &addr);
    
	if (status != asynSuccess) return status;
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s finsUDP:disconnect addr %d\n", pdrvPvt->portName, addr);

	if (addr >= 0)
	{
		pasynManager->exceptionDisconnect(pasynUser);
		return (asynSuccess);
	}
	
	if (!pdrvPvt->connected)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s finsUDP:disconnect port not connected\n", pdrvPvt->portName);
		return (asynError);
	}
	
	pdrvPvt->connected = 0;
	pasynManager->exceptionDisconnect(pasynUser);
	
	return (asynSuccess);
}

static asynStatus flushIt(void *pvt,asynUser *pasynUser)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	char cbuf[2048];

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s flush\n", pdrvPvt->portName);

	if (pdrvPvt->fd >= 0)
	{
		while (recv(pdrvPvt->fd, cbuf, sizeof(cbuf), MSG_DONTWAIT) > 0) continue;
	}

 	return (asynSuccess);
}

/******************************************************************************/

/*
	Form a FINS read message, send request, wait for the reply and check for errors
	
	This function knows about various message types an forms the correct message
	and processes the reply based on pasynUser->reason.
	
	Document W421 says that the maximum FINS message size is 2012 bytes, which is larger than the MTU.
	We'll limit the maximum number of words to 500 which will be sufficient for all of our current applications.

*/

static int finsUDPread(drvPvt *pdrvPvt, asynUser *pasynUser, void *data, const size_t nelements, const epicsUInt16 address, size_t *transfered)
{
	unsigned char reply[FINS_MAX_MSG];
	unsigned char message[FINS_MAX_MSG];
	int recvlen, sendlen = 0;
	const int addrlen = sizeof(struct sockaddr_in);
	
/* initialise header */

	message[ICF] = 0x80;
	message[RSV] = 0x00;
	message[GCT] = 0x02;

	message[DNA] = 0x00;
	message[DA1] = pdrvPvt->node;
	message[DA2] = 0x00;

	message[SNA] = 0x00;
	message[SA1] = 0x01;
	message[SA2] = 0x00;

	switch (pasynUser->reason)
	{
	
	/* Memory read */
	
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		{
			message[MRC] = 0x01;
			message[SRC] = 0x01;

		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_READ:
				{
					message[COM] = DM;
					break;
				}
				
				case FINS_AR_READ:
				{
					message[COM] = AR;
					break;
				}
				
				case FINS_IO_READ:
				{
					message[COM] = IO;
					break;
				}
				
				default:
				{
					return (-1);
				}
			}

		/* start address */

			message[COM+1] = address >> 8;
			message[COM+2] = address & 0xff;
			message[COM+3] = 0x00;

		/* length */

			message[COM+4] = nelements >> 8;
			message[COM+5] = nelements & 0xff;

			sendlen = COM + 6;
			
			break;
		}

		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		{
			message[MRC] = 0x01;
			message[SRC] = 0x01;

		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_READ_32:
				{
					message[COM] = DM;
					break;
				}
				
				case FINS_AR_READ_32:
				{
					message[COM] = AR;
					break;
				}
				
				case FINS_IO_READ_32:
				{
					message[COM] = IO;
					break;
				}
				
				default:
				{
					return (-1);
				}
			}

		/* start address */

			message[COM+1] = address >> 8;
			message[COM+2] = address & 0xff;
			message[COM+3] = 0x00;

		/* length */

			message[COM+4] = (nelements << 1) >> 8;
			message[COM+5] = (nelements << 1) & 0xff;

			sendlen = COM + 6;
			
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
			unsigned char *mm = &message[COM];
			
			message[MRC] = 0x01;
			message[SRC] = 0x04;

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
			message[MRC] = 0x05;
			message[SRC] = 0x02;

		/* address is unit number */
		
			message[COM + 0] = address & 0xff;
			message[COM + 1] = 1;
			
			sendlen = COM + 2;
			
			break;
		}
		
		case FINS_CPU_STATUS:
		case FINS_CPU_MODE:
		{
			message[MRC] = 0x06;
			message[SRC] = 0x01;
			
			sendlen = COM;

			break;
		}
	
		case FINS_CYCLE_TIME:
		case FINS_CYCLE_TIME_MEAN:
		case FINS_CYCLE_TIME_MAX:
		case FINS_CYCLE_TIME_MIN:
		{
			message[MRC] = 0x06;
			message[SRC] = 0x20;

			message[COM] = 0x01;
			
			sendlen = COM + 1;
			
			break;
		}

		case FINS_CLOCK_READ:
		{
			message[MRC] = 0x07;
			message[SRC] = 0x01;
			
			sendlen = COM;

			break;
		}
			
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, no such command %d\n", pdrvPvt->portName, pasynUser->reason);
			return (-1);
		}
	}
	
	message[SID] = pdrvPvt->sid++;
#if 0
	{
		int i;
			
		for (i = 0; i < sendlen; i++)
		{
			printf("0x%02x ", message[i]);
		}
	
		puts("");
	}
#endif
	
/* send request */

	if (sendto(pdrvPvt->fd, message, sendlen, 0, (struct sockaddr *) &pdrvPvt->addr, addrlen) != sendlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, failed to send complete message\n", pdrvPvt->portName);
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

		switch (select(pdrvPvt->fd + 1, &rfds, NULL, NULL, &tv))
		{
			case -1:
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, select() error\n", pdrvPvt->portName);
	
				return (-1);
				break;
			}
			
			case 0:
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, select() timeout\n", pdrvPvt->portName);

				return (-1);
				break;
			}
			
			default:
			{
				break;
			}
		}
	}

	if ((recvlen = recvfrom(pdrvPvt->fd, reply, FINS_MAX_MSG, 0, NULL, NULL)) < 0)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, recvfrom() error\n", pdrvPvt->portName);
		return (-1);
	}
#if 0
	{
		int i;
		
		for (i = 0; i < recvlen; i++)
		{
			printf("0x%02x ", reply[i]);
		}
	
		puts("");
	}
#endif

/* Illegal response length check */
	
	if (recvlen < MIN_RESP_LEN)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, receive length too small\n", pdrvPvt->portName);
		return (-1);
	}
	
	if ((message[DNA] != reply[SNA]) || (message[DA1] != reply[SA1]) || (message[DA2] != reply[SA2]))
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, illegal source address received\n", pdrvPvt->portName);
		return (-1);
	}

/* SID check */
	
	if (message[SID] != reply[SID])
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, wrong SID received\n", pdrvPvt->portName);
		return (-1);
	}

/* command check */

	if ((reply[MRC] != message[MRC]) || (reply[SRC] != message[SRC]))
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPread: port %s, wrong MRC/SRC received\n", pdrvPvt->portName);
		return (-1);
	}

/* check response code */

	if ((reply[MRES] != 0x00) || (reply[SRES] != 0x00))
	{
		FINSerror(pdrvPvt, pasynUser, "finsUDPread", reply[MRES], reply[SRES]);
		return (-1);
	}

/* extract data */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		{
			if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
			{
				int i;
				epicsUInt16 *ptrs = (epicsUInt16 *) &reply[RESP];
				epicsUInt16 *ptrd = (epicsUInt16 *) data;

				asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->debug, DEBUG_LEN, "swapping %d 16 bit words", nelements);

				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = BSWAP16(ptrs[i]);
				}
			}
			else
			{
				const size_t nbytes = nelements * sizeof(epicsUInt16);							

				memcpy(data, &reply[RESP], nbytes);
				
				asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->debug, DEBUG_LEN, "copying %d 16 bit words", nelements);
			}
			
		/* check the number of elements received */
		
			if (transfered)
			{
				*transfered = (recvlen - RESP) / sizeof(epicsUInt16);
			}
			
			break;
		}

		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		{		
			if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
			{
				int i;
				epicsUInt32 *ptrs = (epicsUInt32 *) &reply[RESP];
				epicsUInt32 *ptrd = (epicsUInt32 *) data;
				
				asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->debug, DEBUG_LEN, "swapping %d 32 bit words", nelements);

				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = BSWAP32(ptrs[i]);
				}
			}
			else
			{
				const size_t nbytes = nelements * sizeof(epicsUInt32);

				memcpy(data, &reply[RESP], nbytes);
				
				asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->debug, DEBUG_LEN, "copying %d 32 bit words", nelements);
			}
			
		/* check the number of elements received */
		
			if (transfered)
			{
				*transfered = (recvlen - RESP) / sizeof(epicsUInt32);
			}
			
			break;
		}
		
/* return a string of 20 chars */

		case FINS_MODEL:
		{
			memcpy(data, &reply[RESP + 2], 20);
			
			if (transfered)
			{
				*transfered = FINS_MODEL_LENGTH;
			}
			
			break;
		}

/* return status - epicsInt32 */

		case FINS_CPU_STATUS:
		{
			*(epicsInt32 *)(data) = reply[RESP + 0];
			
			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}
		
/* return mode - epicsInt32 */

		case FINS_CPU_MODE:
		{
			*(epicsInt32 *)(data) = reply[RESP + 1];			

			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}

/* return 3 parameters - epicsInt32 */

		case FINS_CYCLE_TIME:
		{
			if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
			{
				int i;
				epicsInt32 *rep = (epicsInt32 *) &reply[RESP + 0];
				epicsInt32 *dat = (epicsInt32 *) data;

				for (i = 0; i < 3; i++)
				{
					dat[i] = BSWAP32(rep[i]);
				}
				
				if (transfered)
				{
					*transfered = 3;
				}
			}
			else
			{
				memcpy(data, &reply[RESP], 3 * sizeof(epicsInt32));

				if (transfered)
				{
					*transfered = 3;
				}
			}
			
			break;
		}
		
/* return mean - epicsInt32 */

		case FINS_CYCLE_TIME_MEAN:
		{
			if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
			{
				const epicsInt32 *rep = (epicsInt32 *) &reply[RESP + 0];

				*(epicsInt32 *)(data) = BSWAP32(*rep);
			}
			else
			{
				*(epicsInt32 *)(data) = reply[RESP + 0];
			}

			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}
		
/* return max - epicsInt32 */

		case FINS_CYCLE_TIME_MAX:
		{
			if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
			{
				const epicsInt32 *rep = (epicsInt32 *) &reply[RESP + 4];

				*(epicsInt32 *)(data) = BSWAP32(*rep);
			}
			else
			{
				*(epicsInt32 *)(data) = reply[RESP + 4];
			}

			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}
		
/* return min - epicsInt32 */

		case FINS_CYCLE_TIME_MIN:
		{
			if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
			{
				const epicsInt32 *rep = (epicsInt32 *) &reply[RESP + 8];

				*(epicsInt32 *)(data) = BSWAP32(*rep);
			}
			else
			{
				*(epicsInt32 *)(data) = reply[RESP + 8];
			}

			if (transfered)
			{
				*transfered = 1;
			}
			
			break;
		}

		case FINS_CLOCK_READ:
		{
			epicsInt8  *rep = (epicsInt8 *)  &reply[RESP + 0];
			epicsInt16 *dat = (epicsInt16 *) data;
			int i;
				
			for (i = 0; i < 7; i++)
			{
				*dat++ = *rep++;
			}
				
			if (transfered)
			{
				*transfered = 7;
			}
			
			break;
		}
		
		default:
		{
			break;
		}
	}

	return (0);	
}


static int finsUDPwrite(drvPvt *pdrvPvt, asynUser *pasynUser, const void *data, size_t nwords, const epicsUInt16 address)
{
	unsigned char reply[FINS_MAX_MSG];
	unsigned char message[FINS_MAX_MSG];
	int recvlen, sendlen;
	const int addrlen = sizeof(struct sockaddr_in);

/* initialise header */

	message[ICF] = 0x80;
	message[RSV] = 0x00;
	message[GCT] = 0x02;

	message[DNA] = 0x00;
	message[DA1] = pdrvPvt->node;
	message[DA2] = 0x00;

	message[SNA] = 0x00;
	message[SA1] = 0x01;
	message[SA2] = 0x00;
	
	switch (pasynUser->reason)
	{
	
	/* Memory write */
	
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		{
			message[MRC] = 0x01;
			message[SRC] = 0x02;
				
		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_WRITE:
				{
					message[COM] = DM;
					break;
				}
				
				case FINS_AR_WRITE:
				{
					message[COM] = AR;
					break;
				}
				
				case FINS_IO_WRITE:
				{
					message[COM] = IO;
					break;
				}
				
				default:
				{
					return (-1);
				}
			}
			
		/* start address */

			message[COM+1] = address >> 8;
			message[COM+2] = address & 0xff;
			message[COM+3] = 0x00;

		/* length */

			message[COM+4] = nwords >> 8;
			message[COM+5] = nwords & 0xff;

		/* convert data - PLC is Big Endian */

			if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
			{
				int i;
				epicsUInt16 *ptrd = (epicsUInt16 *) &message[COM + 6];
				epicsUInt16 *ptrs = (epicsUInt16 *) data;
				
				asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->debug, DEBUG_LEN, "swapping %d 16 bit words", nwords);

				for (i = 0; i < nwords; i++)
				{
					ptrd[i] = BSWAP16(ptrs[i]);
				}
			}
			else
			{
				memcpy(&message[COM+6], data, nwords * sizeof(short));
				
				asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->debug, DEBUG_LEN, "copying %d 16 bit words", nwords);
			}
			
			sendlen = COM + 6 + nwords * sizeof(short);
			
			break;
		}

		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{
			message[MRC] = 0x01;
			message[SRC] = 0x02;
				
		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_WRITE_32:
				{
					message[COM] = DM;
					break;
				}
				
				case FINS_AR_WRITE_32:
				{
					message[COM] = AR;
					break;
				}
				
				case FINS_IO_WRITE_32:
				{
					message[COM] = IO;
					break;
				}
				
				default:
				{
					return (-1);
				}
			}
			
		/* start address */

			message[COM+1] = address >> 8;
			message[COM+2] = address & 0xff;
			message[COM+3] = 0x00;

		/* length */

			message[COM+4] = nwords >> 8;
			message[COM+5] = nwords & 0xff;

		/* convert data - PLC is Big Endian */

			if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)
			{
				int i;
				epicsUInt32 *ptrd = (epicsUInt32 *) &message[COM + 6];
				epicsUInt32 *ptrs = (epicsUInt32 *) data;
				
				asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->debug, DEBUG_LEN, "swapping %d 32 bit words", nwords >> 1);

				for (i = 0; i < nwords / 2; i++)
				{
					ptrd[i] = BSWAP32(ptrs[i]);
				}
			}
			else
			{
				memcpy(&message[COM+6], data, nwords * sizeof(short));
				
				asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pdrvPvt->debug, DEBUG_LEN, "copying %d 32 bit words", nwords >> 1);
			}
			
			sendlen = COM + 6 + nwords * sizeof(short);
			
			break;
		}

	/* cycle time reset */
	
		case FINS_CYCLE_TIME_RESET:
		{
			message[MRC] = 0x06;
			message[SRC] = 0x20;
			message[COM] = 0x00;
			
			sendlen = COM + 1;

			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, no such command %d\n", pdrvPvt->portName, pasynUser->reason);
			return (-1);
		}
	}
		
	message[SID] = pdrvPvt->sid++;
#if 0
	{
		int i;
		
		for (i = 0; i < sendlen; i++)
		{
			printf("0x%02x ", message[i]);
		}
		
		puts("");
	}
#endif
/* send request */

	errno = 0;
	
	if (sendto(pdrvPvt->fd, message, sendlen, 0, (struct sockaddr *) &pdrvPvt->addr, addrlen) != sendlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, failed to send complete message\n", pdrvPvt->portName);
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
		
		switch (select(pdrvPvt->fd + 1, &rfds, NULL, NULL, &tv))
		{
			case -1:
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, select() failed\n", pdrvPvt->portName);

				return (-1);
				break;
			}
			
			case 0:
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, select() timeout\n", pdrvPvt->portName);
				
				return (-1);
				break;
			}
			
			default:
			{
				break;
			}
		}
	}

	if ((recvlen = recvfrom(pdrvPvt->fd, reply, FINS_MAX_MSG, 0, NULL, NULL)) < 0)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, recvfrom() error\n", pdrvPvt->portName);
		return (-1);
	}
#if 0
	{
		int i;
		
		for (i = 0; i < recvlen; i++)
		{
			printf("0x%02x ", reply[i]);
		}
	
		puts("");
	}
#endif
/* Illegal response length check */
	
	if (recvlen < MIN_RESP_LEN)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, receive length too small\n", pdrvPvt->portName);
		return (-1);
	}
	
	if ((message[DNA] != reply[SNA]) || (message[DA1] != reply[SA1]) || (message[DA2] != reply[SA2]))
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, illegal source address received\n", pdrvPvt->portName);
		return (-1);
	}

/* SID check */
	
	if (message[SID] != reply[SID])
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, wrong SID received\n", pdrvPvt->portName);
		return (-1);
	}

/* command check */

	if ((reply[MRC] != message[MRC]) || (reply[SRC] != message[SRC]))
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "finsUDPwrite: port %s, wrong MRC/SRC received\n", pdrvPvt->portName);
		return (-1);
	}

/* check response code */

	if ((reply[MRES] != 0x00) || (reply[SRES] != 0x00))
	{
		FINSerror(pdrvPvt, pasynUser, "finsUDPwrite", reply[MRES], reply[SRES]);
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
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	if (eomReason) *eomReason = 0;
	if (nbytesTransfered) *nbytesTransfered = 0;
	
	if (!nbytesTransfered)
	{
		puts("nbytesTransfered is null");
		return (asynError);
	}
	
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
			if (maxchars < FINS_MODEL_LENGTH)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s length is not >= %d for FINS_MODEL\n", pdrvPvt->portName, FINS_MODEL_LENGTH);
				return (asynError);
			}
			
			break;
		}
		
	/* no more reasons for asynOctetRead */
	
		default:
		{
			*nbytesTransfered = 0;
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s asynUDP:read addr %d, max %d\n", pdrvPvt->portName, addr, maxchars);

/* send FINS request */

	if (finsUDPread(pdrvPvt, pasynUser, (void *) data, maxchars, addr, nbytesTransfered) < 0)
	{
		*nbytesTransfered = 0;
		return (asynError);
	}
	
	if (eomReason)
	{
		*eomReason |= ASYN_EOM_END;
	}
	
/*	asynPrintIO(pasynUser,ASYN_TRACEIO_DRIVER,data,nout, "asynUDP nbytesTransfered %d\n",*nbytesTransfered); */

   	return (asynSuccess);
}

/*
	Form a FINS write message, send request, wait for the reply and check for errors

	Parameters required:
	
		IP address	Set during initialisation
		command type	read, write, cpu status, cycle time etc. Set by pasynUser->reason
		memory type	DM, IO, AR, CT. Set by pasynUser->reason
		start address	Set by asyn address
		data length	Determined by record type
		
		asyn("FINS0", 0xffff, 1) FINS_MODEL, FINS_CYCLE_TIME_RESET, ...
		
	nwords is only used for memory access operations like DM, AR, IO arrays of DM, AR, IO 16/32 access 
*/

static asynStatus udpWrite(void *pvt, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransfered)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}
	
	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s asynUDP:write addr 0x%x, chars %d\n", pdrvPvt->portName, addr, numchars);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_CYCLE_TIME_RESET:
		{
			break;			/* numchars is not used because the message has a fixed size */
		}
		
		default:
		{
			return (asynError);
		}
	}
		
/* form FINS message and send data */
	
	if (finsUDPwrite(pdrvPvt, pasynUser, (void *) data, numchars, addr) < 0)
	{
		*nbytesTransfered = 0;
		return (asynError);
	}
	
	*nbytesTransfered = numchars;

/*	asynPrintIO(pasynUser,ASYN_TRACEIO_DRIVER,data,nout, "asynUDP nbytesTransfered %d\n",*nbytesTransfered); */

   	return (asynSuccess);
}

/*** asynInt32 ************************************************************************************/

static asynStatus ReadInt32(void *pvt, asynUser *pasynUser, epicsInt32 *value)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s ReadInt32 addr 0x%x\n", pdrvPvt->portName, addr);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		case FINS_CYCLE_TIME_MEAN:
		case FINS_CYCLE_TIME_MAX:
		case FINS_CYCLE_TIME_MIN:
		case FINS_CPU_STATUS:
		case FINS_CPU_MODE:
		{			
			break;
		}

		default:
		{
			return (asynError);
		}
	}

/* send FINS request */

	if (finsUDPread(pdrvPvt, pasynUser, (void *) value, 1, addr, NULL) < 0)
	{
		return (asynError);
	}

	return (asynSuccess);
}

static asynStatus WriteInt32(void *pvt, asynUser *pasynUser, epicsInt32 value)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s WriteInt32 addr 0x%x\n", pdrvPvt->portName, addr);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		case FINS_CYCLE_TIME_RESET:
		{
			epicsUInt16 val = (epicsUInt16) (value & 0xffff);
			
		/* form FINS message and send data */

			if (finsUDPwrite(pdrvPvt, pasynUser, (void *) &val, sizeof(epicsInt16) / sizeof(epicsInt16), addr) < 0)
			{
				return (asynError);
			}
			
			break;
		}

		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{

		/* form FINS message and send data */

			if (finsUDPwrite(pdrvPvt, pasynUser, (void *) &value, sizeof(epicsInt32) / sizeof(epicsInt16), addr) < 0)
			{
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			return (asynError);
		}
	}

	return (asynSuccess);
}

/*** asynInt16Array *******************************************************************************/

static asynStatus ReadInt16Array(void *pvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements, size_t *nIn)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s ReadInt16Array addr 0x%x, words %d\n", pdrvPvt->portName, addr, nelements);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		{
			if (nelements > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s ReadInt16Array addr 0x%x, request too big\n", pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		case FINS_CLOCK_READ:
		{
			if (nelements != 7)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s FINS_CLOCK_READ size != 7\n", pdrvPvt->portName);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			return (asynError);
		}
	}
		
/* send FINS request */

	if (finsUDPread(pdrvPvt, pasynUser, (char *) value, nelements, addr, nIn) < 0)
	{
		*nIn = 0;
		return (asynError);
	}
	
	return (asynSuccess);
}

static asynStatus WriteInt16Array(void *pvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s WriteInt16Array addr 0x%x, words %d\n", pdrvPvt->portName, addr, nelements);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		{
			if (nelements > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s WriteInt16Array addr 0x%x, request too big\n", pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			return (asynError);
		}
	}
	
/* form FINS message and send data */

	if (finsUDPwrite(pdrvPvt, pasynUser, (void *) value, nelements, addr) < 0)
	{
		return (asynError);
	}
		
	return (asynSuccess);
}

/*** asynInt32Array *******************************************************************************/

static asynStatus ReadInt32Array(void *pvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements, size_t *nIn)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s ReadInt32Array addr 0x%x, long words %d\n", pdrvPvt->portName, addr, nelements);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		{
			if ((nelements * 2) > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s ReadInt32Array addr 0x%x, request too big\n", pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}

		case FINS_CYCLE_TIME:
		{
			if (nelements != 3)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s ReadInt16Array, request %d too small\n", pdrvPvt->portName, nelements);
				return (asynError);
			}
			
			break;
		}

		default:
		{
			return (asynError);
		}
	}
		
/* send FINS request */

	if (finsUDPread(pdrvPvt, pasynUser, (void *) value, nelements, addr, nIn) < 0)
	{
		*nIn = 0;
		return (asynError);
	}
	
	return (asynSuccess);
}

static asynStatus WriteInt32Array(void *pvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s WriteInt32Array addr 0x%x, long words %d\n", pdrvPvt->portName, addr, nelements);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{
			if ((nelements * 2) > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s WriteInt32Array addr 0x%x, request too big\n", pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			return (asynError);
		}
	}
		
/* form FINS message and send data */

	if (finsUDPwrite(pdrvPvt, pasynUser, (void *) value, nelements * sizeof(epicsInt32) / sizeof(epicsInt16), addr) < 0)
	{
		return (asynError);
	}
	
	return (asynSuccess);
}

/*** asynFloat32Array *****************************************************************************/

/*
	Read 32 bit values from the PLC which is encoded as IEEE floats
*/

static asynStatus ReadFloat32Array(void *pvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements, size_t *nIn)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s ReadFloa32Array addr 0x%x, floats %d\n", pdrvPvt->portName, addr, nelements);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		{
			if ((nelements * 2) > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s ReadFloat32Array addr 0x%x, request too big\n", pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			return (asynError);
		}
	}
	
/* send FINS request */

	if (finsUDPread(pdrvPvt, pasynUser, (void *) value, nelements, addr, nIn) < 0)
	{
		*nIn = 0;
		return (asynError);
	}
	
	return (asynSuccess);
}

static asynStatus WriteFloat32Array(void *pvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements)
{
	drvPvt *pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	status = pasynManager->getAddr(pasynUser, &addr);
	
	if (status != asynSuccess)
	{
		return (status);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s WriteFloat32Array addr 0x%x, floats %d\n", pdrvPvt->portName, addr, nelements);

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{
			if ((nelements * 2) > FINS_MAX_WORDS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s WriteFloat32Array addr 0x%x, request too big\n", pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			return (asynError);
		}
	}
	
/* form FINS message and send data */

	if (finsUDPwrite(pdrvPvt, pasynUser, (void *) value, nelements * sizeof(epicsFloat32) / sizeof(epicsInt16), addr) < 0)
	{
		return (asynError);
	}

	return (asynSuccess);
}

/*** asynDrvUser **********************************************************************************/

asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize)
{
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
		if (strcmp("FINS_DM_WRITE_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_DM_WRITE_32;
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
		if (strcmp("FINS_IO_WRITE_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_IO_WRITE_32;
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
		if (strcmp("FINS_AR_WRITE_32", drvInfo) == 0)
		{
			pasynUser->reason = FINS_AR_WRITE_32;
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
		
		printf("reason %s %d\n", drvInfo, pasynUser->reason);

		return (asynSuccess);
	}

	return (asynError);
}

static void FINSerror(drvPvt *pdrvPvt, asynUser *pasynUser, const char *name, const unsigned char mres, const unsigned char sres)
{
	switch (mres)
	{
		case 0x01:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Local node error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x02:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Destination node error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x03:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Communications controller error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x04:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Not executable 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x05:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Routing error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x10:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Command format error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x11:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Parameter error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x20:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Read not possible 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x21:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Write not possible 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x22:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Not executable in curent mode 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x23:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, No unit 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x24:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Start/Stop not possible 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x25:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Unit error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x26:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Command error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x30:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Access rights error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		case 0x40:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Abort error 0x%02x\n", name, pdrvPvt->portName, sres);
			break;
		}
		
		default:
		{
			break;
		}
	}
}

static const iocshArg finsUDPInitArg0 = { "portName", iocshArgString };
static const iocshArg finsUDPInitArg1 = { "IP address", iocshArgString };

static const iocshArg *finsUDPInitArgs[] = { &finsUDPInitArg0, &finsUDPInitArg1};
static const iocshFuncDef finsUDPInitFuncDef = { "finsUDPInit", 2, finsUDPInitArgs};

static void finsUDPInitCallFunc(const iocshArgBuf *args)
{
	finsUDPInit(args[0].sval, args[1].sval);
}

static void finsUDPRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsUDPInitFuncDef, finsUDPInitCallFunc);
	}
}

epicsExportRegistrar(finsUDPRegister);

/**************************************************************************************************/
