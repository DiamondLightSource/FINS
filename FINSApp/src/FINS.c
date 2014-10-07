/*
	Factory Intelligent Network Service
	
	This is an asyn driver, supporting various asyn interfaces, to send requests and receive replies from the CPU unit of the PLC.

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
		r	FINS_WR_READ
		r	FINS_HR_READ
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
		w	FINS_SET_RESET_CANCEL
		
		Int16Array
		r	FINS_DM_READ
		r	FINS_AR_READ
		r	FINS_IO_READ
		r	FINS_CLOCK_READ
		r	FINS_MM_READ
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
		w	FINS_DM_WRITE_32
		w	FINS_AR_WRITE_32
		
		Float64
		r	FINS_DM_READ_32
		r	FINS_AR_READ_32
		w	FINS_DM_WRITE_32
		w	FINS_AR_WRITE_32
		
	The commands supported by this driver are for CPU units.
*/

#include <stdio.h>
#include <string.h>
#include <errno.h>

#ifdef vxWorks
#include <sockLib.h>
#include <inetLib.h>
#endif

#include <cantProceed.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsAssert.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <epicsEndian.h>
#include <epicsThread.h>
#include <errlog.h>
#include <ellLib.h>

#include <asynDriver.h>
#include <asynDrvUser.h>
#include <asynOctet.h>
#include <asynOctetSyncIO.h>
#include <asynInt32.h>
#include <asynFloat64.h>
#include <asynInt16Array.h>
#include <asynInt32Array.h>
#include <asynFloat32Array.h>
#include <asynCommonSyncIO.h>
#include <asynStandardInterfaces.h>

#include <drvAsynIPPort.h>

#include <iocsh.h>
#include <registryFunction.h>
#include <osiUnistd.h>
#include <osiSock.h>

#include "FINS.h"

static void FINSerror(const drvPvt * const pdrvPvt, asynUser *pasynUser, const char *name, const unsigned char mres, const unsigned char sres);

/*** asynCommon methods ***************************************************************************/

static void report(void *drvPvt, FILE *fp, int details);
static asynStatus aconnect(void *drvPvt, asynUser *pasynUser);
static asynStatus adisconnect(void *drvPvt, asynUser *pasynUser);
static asynCommon ifacecommon = { report, aconnect, adisconnect };

/*** asynOctet methods ****************************************************************************/

static asynStatus octetRead (void *drvPvt, asynUser *pasynUser, char *data, size_t maxchars, size_t *nbytesTransfered, int *eomReason);
static asynStatus octetWrite(void *drvPvt, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransfered);
static asynStatus octetFlush(void *drvPvt, asynUser *pasynUser);

static asynOctet ifaceOctet = { octetWrite, octetRead, octetFlush, NULL, NULL, NULL, NULL, NULL, NULL};

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
static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser);

static asynDrvUser ifaceDrvUser = { drvUserCreate, drvUserGetType, drvUserDestroy};

/**************************************************************************************************/

extern int errno;
static int finsInit(const char *portName, const char *dev, const int snode);

/* double linked list for Multiple Memory reads */

static ELLLIST mmList;

/**************************************************************************************************/

int finsNETInit(const char *portName, const char *dev, const int snode)
{
	return finsInit(portName, dev, (snode < 0) ? 0 : snode);
}

/**************************************************************************************************/

int finsDEVInit(const char *portName, const char *dev)
{
	return finsInit(portName, dev, -1);
}

/**************************************************************************************************/
/*
	A modified version of the old initialisation function which calls drvAsynIPPortConfigure
	to set up the UDP connection.
*/

int finsUDPInit(const char *portName, const char *address)
{
	char *adds;
	
	adds = (char *) callocMustSucceed(1, strlen(address) + 10, __func__);

	if(strchr(address, ':') == NULL)
    {
        // no port provided - default to 9600
        epicsSnprintf(adds, strlen(address) + 10, "%s:9600 udp", address);
    }
    else
    {
        epicsSnprintf(adds, strlen(address) + 10, "%s udp", address);
    }

	if (drvAsynIPPortConfigure(address, adds, 0, 0, 0) == 0)
	{
		return finsInit(portName, address, FINS_SOURCE_ADDR);
	}

	return (-1);
}

int finsTCPInit(const char *portName, const char *address)
{
	char *adds;

    adds = (char *) callocMustSucceed(1, strlen(address) + 10, __func__);

	if(strchr(address, ':') == NULL)
	{
	    // no port provided - default to 9600
	    epicsSnprintf(adds, strlen(address) + 10, "%s:9600 tcp", address);
	}
	else
	{
	    epicsSnprintf(adds, strlen(address) + 10, "%s tcp", address);
	}
	
	if (drvAsynIPPortConfigure(address, adds, 0, 0, 0) == 0)
	{
		return finsInit(portName, address, 0);
	}

	return (-1);
}

/**************************************************************************************************/
/*
	For TCP connections we need this extra FINS Frame data for requesting a node number and
	in every FINS message and reply.
*/

static void AddCommand(drvPvt * const pdrvPvt, const size_t sendlen, const unsigned int command)
{
	unsigned int *FINSframe = (unsigned int *) pdrvPvt->message;
	
	FINSframe[FINS_MODE_HEADER]  = BSWAP32(FINS_TCP_HEADER);
	FINSframe[FINS_MODE_COMMAND] = BSWAP32(command);
	FINSframe[FINS_MODE_ERROR]   = BSWAP32(0x00000000);
	
	if (command == FINS_NODE_CLIENT_COMMAND)
	{
		FINSframe[FINS_MODE_LENGTH] = BSWAP32(0x0C);
	}
	else
	{
		FINSframe[FINS_MODE_LENGTH] = BSWAP32(sendlen + 8);
	}
}

/**************************************************************************************************/

static int FINSnodeRequest(drvPvt * const pdrvPvt)
{
	unsigned int *FINSframe = (unsigned int *) pdrvPvt->message;
	size_t sentlen = 0, recdlen = 0;
	int eomReason = 0;
	asynStatus status;
	
/* initialise the buffer */

	AddCommand(pdrvPvt, 0, FINS_NODE_CLIENT_COMMAND);
	
	status = pasynOctetSyncIO->writeRead(pdrvPvt->pasynUser, (void *) pdrvPvt->message, FINS_MODE_SEND_SIZE, (void *) pdrvPvt->message, FINS_MODE_RECV_SIZE, 1.0, &sentlen, &recdlen, &eomReason);

	FINSframe[FINS_MODE_COMMAND] = BSWAP32(FINSframe[FINS_MODE_COMMAND]);
	FINSframe[FINS_MODE_ERROR]   = BSWAP32(FINSframe[FINS_MODE_ERROR]);
	FINSframe[FINS_MODE_CLIENT]  = BSWAP32(FINSframe[FINS_MODE_CLIENT]);
	
/* check command type and error code */

	if ((status == asynSuccess) && (FINSframe[FINS_MODE_COMMAND] == FINS_NODE_SERVER_COMMAND) && (FINSframe[FINS_MODE_ERROR] == 0))
	{
		pdrvPvt->nodevalid = 1;

		pdrvPvt->snode = FINSframe[FINS_MODE_CLIENT];
			
		return (0);
	}
	
/* Disconnect the TCP link. We have auto-connect selected */

	pasynCommonSyncIO->disconnectDevice(pdrvPvt->pasynUserCommon);
	return (-1);
}

/**************************************************************************************************/
/*
	Connection management for the TCP asyn port
	
	If we lose the link we have to resend the FINS Node Address Send command to obtain a new node address
*/

void exceptCallback(asynUser *pasynUser, asynException exception)
{
	drvPvt * const pdrvPvt = (drvPvt *) pasynUser->drvUser;
	int connected;
	
	if (exception != asynExceptionConnect)
	{
		return;
	}

	pasynManager->isConnected(pasynUser, &connected);

/*	printf("exceptionCallback  %s\n", (connected) ? "connected" : "disconnected"); */
	
/* request a node number each time we connect to the PLC */

	if (connected == 0)
	{
		pdrvPvt->nodevalid = 0;
	}
}

/**************************************************************************************************/

static int finsInit(const char *portName, const char *dev, const int snode)
{
	asynStatus status;
	asynStandardInterfaces *pInterfaces;
	asynInterface *poctetasynInterface;
	
	drvPvt *pdrvPvt = callocMustSucceed(1, sizeof(drvPvt), __func__);
	pdrvPvt->portName = epicsStrDup(portName);
	pdrvPvt->tLast = -1.0;

	pdrvPvt->pasynUser = pasynManager->createAsynUser(0, 0);
	pdrvPvt->pasynUserCommon = pasynManager->createAsynUser(0, 0);

	status = pasynManager->registerPort(portName, ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0);

	if (status != asynSuccess)
	{
		printf("%s: registerPort failed\n", __func__);
		return (-1);
	}

/* Create asyn interfaces and register with asynManager */

	pInterfaces = &pdrvPvt->asynStdInterfaces;
	
	pInterfaces->common.pinterface = (void *) &ifacecommon;
	pInterfaces->drvUser.pinterface = (void *) &ifaceDrvUser;
	pInterfaces->octet.pinterface = (void *) &ifaceOctet;
	pInterfaces->int32.pinterface = (void *)&ifaceInt32;
	pInterfaces->float64.pinterface = (void *) &ifaceFloat64;
	pInterfaces->int16Array.pinterface = (void *) &ifaceInt16Array;
	pInterfaces->int32Array.pinterface = (void *) &ifaceInt32Array;
	pInterfaces->float32Array.pinterface = (void *) &ifaceFloat32Array;

	status = pasynStandardInterfacesBase->initialize(pdrvPvt->portName, pInterfaces, pdrvPvt->pasynUser, pdrvPvt);
	
	if (status != asynSuccess)
	{
		errlogPrintf("%s: port %s can't register standard interfaces: %s\n", __func__, pdrvPvt->portName, pdrvPvt->pasynUser->errorMessage);
		return (-1);
	}

/* connect to the parent port and save the asynUser */
	
  	if (pasynOctetSyncIO->connect(dev, 0, &pdrvPvt->pasynUser, NULL))
	{
		printf("%s: pasynOctetSyncIO->connect: %s\n", __func__, pdrvPvt->pasynUser->errorMessage);
		return (-1);
	}
	
/* no need to continue if it isn't a network device */

	if (snode < 0)
	{
		pdrvPvt->snode = 0;
		pdrvPvt->type = HOSTLINK_type;
		
		return (0);
	}
	
/* we need this to force the TCP connection to disconnect */

  	if (pasynCommonSyncIO->connect(dev, 0, &pdrvPvt->pasynUserCommon, NULL))
	{
		printf("%s: pasynCommonSyncIO->connect: %s\n", __func__, pdrvPvt->pasynUserCommon->errorMessage);
		return (-1);
	}
	
/* find the octet interface of our parent port ignoring the interpose layer */

	poctetasynInterface = pasynManager->findInterface(pdrvPvt->pasynUser, asynOctetType, 0);
		
	if (!poctetasynInterface)
	{
		printf("%s findInterface error for asynOctetType %s\n", portName, pdrvPvt->pasynUser->errorMessage);
		pasynManager->freeAsynUser(pdrvPvt->pasynUser);
		free(pdrvPvt);
			
		return (-1);
	}

/* save a pointer to our private data for the call to exceptCallback */

	pdrvPvt->pasynUser->drvUser = (void *) pdrvPvt;
	
/* find the IP address and extract the node number */

	pdrvPvt->ipaddr = epicsStrDup(((ttyController_t *) poctetasynInterface->drvPvt)->IPHostName);
	
	aToIPAddr(pdrvPvt->ipaddr, FINS_NET_PORT, &pdrvPvt->addr);
	pdrvPvt->dnode = ntohl(pdrvPvt->addr.sin_addr.s_addr) & 0xff;
	
/* detect the type of PLC connection required */

	switch (((ttyController_t *) poctetasynInterface->drvPvt)->socketType)
	{
		case SOCK_DGRAM:
		{
			pdrvPvt->type = FINS_UDP_type;
/*			puts("FINS_UDP_type"); */
			
			break;
		}
		
		case SOCK_STREAM:
		default:
		{
			pdrvPvt->type = FINS_TCP_type;
/*			puts("FINS_TCP_type"); */
			
			break;
		}
	}
	
/* set or exchange node numbers. From W421, section 7-4 */

	if (pdrvPvt->type == FINS_TCP_type)
	{
		FINSnodeRequest(pdrvPvt);

	/* for monitoring connections/disconnections */

		status = pasynManager->exceptionCallbackAdd(pdrvPvt->pasynUser, exceptCallback);
	}
	else if (pdrvPvt->type == FINS_UDP_type)
	{
		pdrvPvt->snode = (snode != 0) ? snode : FINS_SOURCE_ADDR;
	}
	
 	return (0);
}

/**************************************************************************************************/

static void report(void *pvt, FILE *fp, int details)
{
	const drvPvt * const pdrvPvt = (drvPvt *) pvt;
	
	fprintf(fp, "%s: connected %s \n", pdrvPvt->portName, (pdrvPvt->connected ? "Yes" : "No"));
	
	if ((pdrvPvt->type == FINS_TCP_type) || (pdrvPvt->type == FINS_UDP_type))
	{
		fprintf(fp, "    PLC IP: %s\n", pdrvPvt->ipaddr);
		fprintf(fp, "    Node: %d -> Node: %d\n", pdrvPvt->snode, pdrvPvt->dnode);
	}
	
	fprintf(fp, "    Min: %.4fs  Max: %.4fs  Last: %.4fs\n", pdrvPvt->tMin, pdrvPvt->tMax, pdrvPvt->tLast);
}

/**************************************************************************************************/

static asynStatus aconnect(void *pvt, asynUser *pasynUser)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	asynStatus status;
	int addr;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s connect addr %d\n", pdrvPvt->portName, addr);
	
	if (addr >= 0)
	{
		pasynManager->exceptionConnect(pasynUser);
		return (asynSuccess);
	}
	
	if (pdrvPvt->connected)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s already connected\n", pdrvPvt->portName);
		return (asynError);
	}

	pdrvPvt->connected = 1;
	pasynManager->exceptionConnect(pasynUser);
	return (asynSuccess);
}

/**************************************************************************************************/

static asynStatus adisconnect(void *pvt, asynUser *pasynUser)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	asynStatus status;
	int addr;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s disconnect addr %d\n", pdrvPvt->portName, addr);

	if (addr >= 0)
	{
		pasynManager->exceptionDisconnect(pasynUser);
		return (asynSuccess);
	}
	
	if (!pdrvPvt->connected)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s already disconnected\n", pdrvPvt->portName);
		return (asynError);
	}
	
	pdrvPvt->connected = 0;
	pasynManager->exceptionDisconnect(pasynUser);
	
	return (asynSuccess);
}

/**************************************************************************************************/

static asynStatus octetFlush(void *pvt, asynUser *pasynUser)
{
	const drvPvt * const pdrvPvt = (drvPvt *) pvt;
	
	pasynOctetSyncIO->flush(pdrvPvt->pasynUser);

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s\n", __func__, pdrvPvt->portName);

 	return (asynSuccess);
}
		
/******************************************************************************/
/*

Standard FINS header. DA1 must be the target node address: the last byte of the IP address

*/
/******************************************************************************/

static void InitHeader(drvPvt * const pdrvPvt)
{
	pdrvPvt->message[ICF] = 0x80;
	pdrvPvt->message[RSV] = 0x00;
	pdrvPvt->message[GCT] = FINS_GATEWAY;

	pdrvPvt->message[DNA] = 0x00;
	pdrvPvt->message[DA1] = pdrvPvt->dnode;
	pdrvPvt->message[DA2] = 0x00;

	pdrvPvt->message[SNA] = 0x00;
	pdrvPvt->message[SA1] = pdrvPvt->snode;
	pdrvPvt->message[SA2] = 0x00;
}

/**************************************************************************************************/
/*
	We only support word addresses, no bit addressing so COM+3 is zero
	
	address:	16-bit address
	nelements:	number of 16-bit words to transfer
*/
/**************************************************************************************************/

static void InitAddrSize(drvPvt * const pdrvPvt, const epicsUInt16 address, const epicsUInt16  nelements, const size_t asynSize)
{
	pdrvPvt->message[COM+1] = address >> 8;
	pdrvPvt->message[COM+2] = address & 0xff;
	pdrvPvt->message[COM+3] = 0x00;

	pdrvPvt->message[COM+4] = (nelements * asynSize / sizeof(epicsUInt16)) >> 8;
	pdrvPvt->message[COM+5] = (nelements * asynSize / sizeof(epicsUInt16)) & 0xff;
}

/**************************************************************************************************/
/*

	Populate the FINS message based on pasynUser->reason
	Called by finsRead

	address	PLC source/destination address for memory/counter/timer commands
	nelements	number of 16/32 words to read for memory/counter/timer commands
	sendlen	calculated size of message
	recvlen	expected size of reply
*/
/**************************************************************************************************/

static int BuildReadMessage(drvPvt * const pdrvPvt, asynUser *pasynUser, const size_t address, const size_t nelements, size_t *sendlen, size_t *recvlen)
{
	InitHeader(pdrvPvt);

	switch (pasynUser->reason)
	{
	
	/* Memory read */
	
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		case FINS_WR_READ:
		case FINS_HR_READ:
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		{
			pdrvPvt->mrc = 0x01;
			pdrvPvt->src = 0x01;

		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_READ:
				case FINS_DM_WRITE:
				{
					pdrvPvt->message[COM] = DM;
					break;
				}
				
				case FINS_AR_READ:
				case FINS_AR_WRITE:
				{
					pdrvPvt->message[COM] = AR;
					break;
				}
				
				case FINS_IO_READ:
				case FINS_IO_WRITE:
				{
					pdrvPvt->message[COM] = IO;
					break;
				}
				
				case FINS_WR_READ:
				{
					pdrvPvt->message[COM] = WR;
					break;
				}
				
				case FINS_HR_READ:
				{
					pdrvPvt->message[COM] = HR;
					break;
				}
				
				default:
				{
					asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, bad switch line %d.\n", __func__, pdrvPvt->portName, __LINE__);
					return (-1);
				}
			}
			
			InitAddrSize(pdrvPvt, address, nelements, sizeof(epicsUInt16));

		/* send header + memory type + address + size, receiver header + data */
				
			*sendlen = COM + COMMAND_DATA_OFFSET;
			*recvlen = RESP + sizeof(epicsUInt16) * nelements;
			
			break;
		}

		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{
			pdrvPvt->mrc = 0x01;
			pdrvPvt->src = 0x01;

		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_READ_32:
				case FINS_DM_WRITE_32:
				{
					pdrvPvt->message[COM] = DM;
					break;
				}
				
				case FINS_AR_READ_32:
				case FINS_AR_WRITE_32:
				{
					pdrvPvt->message[COM] = AR;
					break;
				}
				
				case FINS_IO_READ_32:
				case FINS_IO_WRITE_32:
				{
					pdrvPvt->message[COM] = IO;
					break;
				}
				
				default:
				{
					asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, bad switch line %d.\n", __func__, pdrvPvt->portName, __LINE__);
					return (-1);
				}
			}

			InitAddrSize(pdrvPvt, address, nelements, sizeof(epicsUInt32));
			
			*sendlen = COM + COMMAND_DATA_OFFSET;
			*recvlen = RESP + sizeof(epicsUInt32) * nelements;
			
			break;
		}

		case FINS_MODEL:
		{
			pdrvPvt->mrc = 0x05;
			pdrvPvt->src = 0x02;

		/* address is unit number */
		
			pdrvPvt->message[COM + 0] = address & 0xff;
			pdrvPvt->message[COM + 1] = 1;
			
			*sendlen = COM + 2;
			*recvlen = RESP + 2 + FINS_MODEL_LEN;
			
			break;
		}
		
		case FINS_CPU_STATUS:
		case FINS_CPU_MODE:
		{
			pdrvPvt->mrc = 0x06;
			pdrvPvt->src = 0x01;
			
			*sendlen = COM;
			*recvlen = RESP + FINS_CPU_STATE_LEN;

			break;
		}
	
		case FINS_CYCLE_TIME:
		case FINS_CYCLE_TIME_MEAN:
		case FINS_CYCLE_TIME_MAX:
		case FINS_CYCLE_TIME_MIN:
		{
			pdrvPvt->mrc = 0x06;
			pdrvPvt->src = 0x20;

			pdrvPvt->message[COM] = 0x01;
			
			*sendlen = COM + 1;
			*recvlen = RESP + FINS_CYCLE_TIME_LEN * sizeof(epicsUInt32);
			
			break;
		}

		case FINS_CLOCK_READ:
		{
			pdrvPvt->mrc = 0x07;
			pdrvPvt->src = 0x01;
			
			*sendlen = COM;
			*recvlen = RESP + FINS_CLOCK_READ_LEN * sizeof(epicsUInt8);
		
			break;
		}
		
		case FINS_MM_READ:
		{
			int i;
			MultiMemArea *MM;
			
			pdrvPvt->mrc = 0x01;
			pdrvPvt->src = 0x04;
						
			MM = (MultiMemArea *) ellNth(&mmList, address + 1);
						
			for (i = 0; (i < nelements) && (MM->area[i]); i++)
			{
				pdrvPvt->message[COM + 4 * i + 0] = MM->area[i];
				pdrvPvt->message[COM + 4 * i + 1] = MM->address[i] >> 8;
				pdrvPvt->message[COM + 4 * i + 2] = MM->address[i] & 0xff;
				pdrvPvt->message[COM + 4 * i + 3] = 0x00;
			}
			
			*sendlen = COM + 4 * i;
			*recvlen = RESP + 3 * i;

			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (-1);
		}
	}

	pdrvPvt->message[MRC] = pdrvPvt->mrc;
	pdrvPvt->message[SRC] = pdrvPvt->src;
	pdrvPvt->message[SID] = ++pdrvPvt->sid;

/* add the FINS TCP command */

	if (pdrvPvt->type == FINS_TCP_type)
	{
	
	/* shift the data to make space for the FINS Frame Send Command */
	
		memmove(pdrvPvt->message + FINS_SEND_FRAME_SIZE, pdrvPvt->message, sizeof(pdrvPvt->message) - FINS_SEND_FRAME_SIZE);
		
		AddCommand(pdrvPvt, *sendlen, FINS_FRAME_SEND_COMMAND);
			
		*sendlen += FINS_SEND_FRAME_SIZE;
		*recvlen += FINS_SEND_FRAME_SIZE;
	}

	return (0);
}

/**************************************************************************************************/

/*
	check the response codes MRES & SRES, the SID, message codes and addresses
*/

static int CheckData(drvPvt * const pdrvPvt, asynUser *pasynUser)
{
	
/* check response code */

	if ((pdrvPvt->message[MRES] != 0x00) || (pdrvPvt->message[SRES] != 0x00))
	{
		FINSerror(pdrvPvt, pasynUser, __func__, pdrvPvt->message[MRES], pdrvPvt->message[SRES]);
		return (-1);
	}
	
/* SID check - probably received a UDP packet out of order */
	
	if (pdrvPvt->sid != pdrvPvt->message[SID])
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, SID %u sent, wrong SID %u received.\n", __func__, pdrvPvt->portName, (epicsUInt8) pdrvPvt->sid, (epicsUInt8) pdrvPvt->message[SID]);
		return (-1);
	}
	
/* command check */

	if ((pdrvPvt->message[MRC] != pdrvPvt->mrc) || (pdrvPvt->message[SRC] != pdrvPvt->src))
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, wrong MRC/SRC received.\n", __func__, pdrvPvt->portName);
		return (-1);
	}
	
/* source address check */
	
	if ((pdrvPvt->message[DA1] != pdrvPvt->snode) || (pdrvPvt->message[SA1] != pdrvPvt->dnode))
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, illegal source address received. %d = %d, %d = %d\n", __func__, pdrvPvt->portName, pdrvPvt->message[DA1], pdrvPvt->snode, pdrvPvt->message[SA1], pdrvPvt->dnode);
		return (-1);
	}

	return (0);
}

/**************************************************************************************************/

static void UpdateTimes(drvPvt * const pdrvPvt, epicsTimeStamp *ets, epicsTimeStamp *ete)
{
	epicsTimeGetCurrent(ete);

	{
		const double diff = epicsTimeDiffInSeconds(ete, ets);
	
		if (pdrvPvt->tLast == -1)
		{
			pdrvPvt->tMax = diff;
			pdrvPvt->tMin = diff;
		}
		else
		{
			if (diff > pdrvPvt->tMax) pdrvPvt->tMax = diff;
			if (diff < pdrvPvt->tMin) pdrvPvt->tMin = diff;
		}
		
		pdrvPvt->tLast = diff;
	}
}

/**************************************************************************************************/
/*
	Form a FINS read message, send request, wait for the reply and check for errors
	
	This function knows about various message types an forms the correct message
	and processes the reply based on pasynUser->reason.
	
	data		epicsInt16, epicsInt32 or epicsFloat32
	nelements	number of items to read
	address	PLC memory address
	transferred	normally the same as nelements
	asynSize	sizeof(epicsInt16) for asynInt16Array or sizeof(epicsInt32) for asynInt32, asynInt32Array, asynFloat32Array, asynFloat64
			defines the type of data to be returned to asyn
*/
/**************************************************************************************************/

static int finsRead(drvPvt * const pdrvPvt, asynUser *pasynUser, void *data, const size_t nelements, const epicsUInt16 address, size_t *transferred, size_t asynSize)
{
	size_t sendlen = 0, sentlen = 0, recvlen = 0, recdlen = 0;
	int eomReason = 0;
	asynStatus status;
	epicsTimeStamp ets, ete;

	if (nelements < 1)
	{
		return (0);
	}

	if ((pdrvPvt->type == FINS_TCP_type) && (pdrvPvt->nodevalid != 1))
	{
		if (FINSnodeRequest(pdrvPvt) < 0)
		{
			return (-1);
		}
	}
	
/* return the size of the message to write and the expected size of the message to read */

	if (BuildReadMessage(pdrvPvt, pasynUser, address, nelements, &sendlen, &recvlen) < 0)
	{
		return (-1);
	}

/* using %lu and casting to unsigned long instead of using %zu because of our old PPC/vxWorks gcc compiler */

	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, (char *) pdrvPvt->message, sendlen, "%s: port %s, sending %lu bytes, expecting %lu bytes.\n", __func__, pdrvPvt->portName, (unsigned long) sendlen, (unsigned long) recvlen);
	
	if (pasynUser->timeout <= 0.0)
	{
		 pasynUser->timeout = FINS_TIMEOUT;
	}
	
	epicsTimeGetCurrent(&ets);
	
	status = pasynOctetSyncIO->writeRead(pdrvPvt->pasynUser, (char *) pdrvPvt->message, sendlen, (char *) pdrvPvt->message, recvlen, pasynUser->timeout, &sentlen, &recdlen, &eomReason);

	UpdateTimes(pdrvPvt, &ets, &ete);
	
	switch (status)
	{
		case asynSuccess:
		{
			break;
		}
		
		case asynTimeout:
		case asynOverflow:
		case asynError:
		case asynDisconnected:
		case asynDisabled:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, writeRead() failed with %s.\n", __func__, pdrvPvt->portName, asynStatusMessages[status]);
			return (-1);
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, unknown asyn return code %d.\n", __func__, pdrvPvt->portName, status);
			return (-1);
		}
	}

	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, (char *) pdrvPvt->message, recdlen, "%s: port %s, received %lu bytes.\n", __func__, pdrvPvt->portName, (unsigned long) recdlen);

	if (sentlen != sendlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, writeRead() write failed. %lu != %lu\n", __func__, pdrvPvt->portName, (unsigned long) sentlen, (unsigned long) sendlen);
		return (-1);
	}
	
	if (recdlen != recvlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, writeRead() read failed.\n", __func__, pdrvPvt->portName);
		return (-1);
	}

/* check and strip the TCP FINS header */

	if (pdrvPvt->type == FINS_TCP_type)
	{
		const unsigned int ferror = BSWAP32(((unsigned int *) pdrvPvt->message)[FINS_MODE_ERROR]);
		 
		if (ferror != FINS_ERROR_NORMAL) 
		{
			pasynCommonSyncIO->disconnectDevice(pdrvPvt->pasynUserCommon);
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, FINS Frame Send error 0x%x.\n", __func__, pdrvPvt->portName, ferror);
			
			return (-1);
		}
		
		memmove(pdrvPvt->message, pdrvPvt->message + FINS_SEND_FRAME_SIZE, sizeof(pdrvPvt->message) - FINS_SEND_FRAME_SIZE);
	}
	
	if (CheckData(pdrvPvt, pasynUser) < 0)
	{
		return (-1);
	}

/* extract data */

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		case FINS_WR_READ:
		case FINS_HR_READ:
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		{
			int i;
			epicsUInt16 *ptrs = (epicsUInt16 *) &pdrvPvt->message[RESP];
			
		/* asynInt16Array */
		
			if (asynSize == sizeof(epicsUInt16))
			{
				epicsUInt16 *ptrd = (epicsUInt16 *) data;

				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = (epicsUInt16) BSWAP16(ptrs[i]);
				}
			}
			else
			
		/* asynInt32 * 1 */
		
			{			
				epicsUInt32 *ptrd = (epicsUInt32 *) data;

				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = (epicsUInt32) BSWAP16(ptrs[i]);
				}
			}

			asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %s %lu 16-bit word(s).\n", __func__, pdrvPvt->portName, SWAPT, (unsigned long) nelements);
			
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
			epicsUInt32 *ptrs = (epicsUInt32 *) &pdrvPvt->message[RESP];
			epicsUInt32 *ptrd = (epicsUInt32 *) data;

			for (i = 0; i < nelements; i++)
			{
				ptrd[i] = WSWAP32(ptrs[i]);
			}
				
			asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, swapping %lu 32-bit word(s).\n", __func__, pdrvPvt->portName, (unsigned long) nelements);
			
			break;
		}
		
/* return a string of 20 chars, packed as two characters per word */

		case FINS_MODEL:
		{
			memcpy(data, &pdrvPvt->message[RESP + 2], nelements);
			
			break;
		}

/* return status - epicsInt32 */

		case FINS_CPU_STATUS:
		{
			*(epicsInt32 *)(data) = pdrvPvt->message[RESP + 0];
			
			break;
		}
		
/* return mode - epicsInt32 */

		case FINS_CPU_MODE:
		{
			*(epicsInt32 *)(data) = pdrvPvt->message[RESP + 1];			
			
			break;
		}

/* return 3 parameters - epicsInt32 */

		case FINS_CYCLE_TIME:
		{
			int i;
			epicsInt32 *rep = (epicsInt32 *) &pdrvPvt->message[RESP + 0];
			epicsInt32 *dat = (epicsInt32 *) data;

			for (i = 0; i < nelements; i++)
			{
				dat[i] = BSWAP32(rep[i]);
			}
				
			break;
		}
		
/* return mean - epicsInt32 */

		case FINS_CYCLE_TIME_MEAN:
		{
			const epicsInt32 *rep = (epicsInt32 *) &pdrvPvt->message[RESP + 0];

			*(epicsInt32 *)(data) = BSWAP32(*rep);

			break;
		}
		
/* return max - epicsInt32 */

		case FINS_CYCLE_TIME_MAX:
		{
			const epicsInt32 *rep = (epicsInt32 *) &pdrvPvt->message[RESP + 4];

			*(epicsInt32 *)(data) = BSWAP32(*rep);
			
			break;
		}
		
/* return min - epicsInt32 */

		case FINS_CYCLE_TIME_MIN:
		{
			const epicsInt32 *rep = (epicsInt32 *) &pdrvPvt->message[RESP + 8];

			*(epicsInt32 *)(data) = BSWAP32(*rep);

			break;
		}

/* asynInt16array */

		case FINS_CLOCK_READ:	/* convert from BCD to dec */
		{
			epicsInt8  *rep = (epicsInt8 *)  &pdrvPvt->message[RESP + 0];
			epicsInt16 *dat = (epicsInt16 *) data;
			int i;
			
			for (i = 0; i < nelements; i++)
			{
				*dat++ = *rep++;
			}

			break;
		}
		
		case FINS_MM_READ:
		{
			int i;
			MultiMemAreaPair *ptrs = (MultiMemAreaPair *) &pdrvPvt->message[RESP];
			epicsUInt16 *ptrd = (epicsUInt16 *) data;
			
			for (i = 0; i < nelements; i++)
			{
				ptrd[i] = (epicsUInt16) BSWAP16(ptrs[i].address);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (-1);
		}
	}
	
	if (transferred) *transferred = nelements;

	return (0);	
}

/**************************************************************************************************/

static int BuildWriteMessage(drvPvt * const pdrvPvt, asynUser *pasynUser, const epicsUInt16 address, const size_t nelements, size_t *sendlen, size_t *recvlen, const size_t asynSize, const void *data)
{
	InitHeader(pdrvPvt);
	
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
			pdrvPvt->mrc = 0x01;
			pdrvPvt->src = 0x02;
				
			switch (pasynUser->reason)
			{	
				case FINS_DM_WRITE:
				case FINS_DM_WRITE_NOREAD:
				{
					pdrvPvt->message[COM] = DM;
					break;
				}
				
				case FINS_AR_WRITE:
				case FINS_AR_WRITE_NOREAD:
				{
					pdrvPvt->message[COM] = AR;
					break;
				}
				
				case FINS_IO_WRITE:
				case FINS_IO_WRITE_NOREAD:
				{
					pdrvPvt->message[COM] = IO;
					break;
				}
				
				default:
				{
					asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, bad switch.\n", __func__, pdrvPvt->portName);
					return (-1);
				}
			}
			
			InitAddrSize(pdrvPvt, address, nelements, sizeof(epicsUInt16));

		/* asynInt16Array */
		
			if (asynSize == sizeof(epicsUInt16))
			{
				int i;
				epicsUInt16 *ptrd = (epicsUInt16 *) &pdrvPvt->message[COM + COMMAND_DATA_OFFSET];
				epicsUInt16 *ptrs = (epicsUInt16 *) data;

				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = BSWAP16(ptrs[i]);
				}

				asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %s %lu 16-bit word(s).\n", __func__, pdrvPvt->portName, SWAPT, (unsigned long) nelements);
			}
			else
			
		/* asynInt32 * 1 */
		
			{
				int i;
				epicsUInt16 *ptrd = (epicsUInt16 *) &pdrvPvt->message[COM + COMMAND_DATA_OFFSET];
				epicsUInt32 *ptrs = (epicsUInt32 *) data;

				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = BSWAP16((epicsUInt16) ptrs[i]);
				}

				asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, %s %lu 16-bit word(s).\n", __func__, pdrvPvt->portName, SWAPT, (unsigned long) nelements);
			}
			
			*sendlen = COM + COMMAND_DATA_OFFSET + nelements * sizeof(epicsUInt16);
			*recvlen = RESP + 0;
						
			break;
		}

		case FINS_DM_WRITE_32:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32:
		case FINS_AR_WRITE_32_NOREAD:
		case FINS_IO_WRITE_32:
		case FINS_IO_WRITE_32_NOREAD:
		{
			pdrvPvt->mrc = 0x01;
			pdrvPvt->src = 0x02;
				
		/* memory type */

			switch (pasynUser->reason)
			{	
				case FINS_DM_WRITE_32:
				case FINS_DM_WRITE_32_NOREAD:
				{
					pdrvPvt->message[COM] = DM;
					break;
				}
				
				case FINS_AR_WRITE_32:
				case FINS_AR_WRITE_32_NOREAD:
				{
					pdrvPvt->message[COM] = AR;
					break;
				}
				
				case FINS_IO_WRITE_32:
				case FINS_IO_WRITE_32_NOREAD:
				{
					pdrvPvt->message[COM] = IO;
					break;
				}
				
				default:
				{
					return (-1);
				}
			}
			
			InitAddrSize(pdrvPvt, address, nelements, sizeof(epicsUInt32));
			
		/* convert data  */

			{
				int i;
				epicsUInt32 *ptrd = (epicsUInt32 *) &pdrvPvt->message[COM + COMMAND_DATA_OFFSET];
				epicsUInt32 *ptrs = (epicsUInt32 *) data;
				
				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = WSWAP32(ptrs[i]);
				}
				
				asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, swapping %lu 32-bit word(s).\n", __func__, pdrvPvt->portName, (unsigned long) nelements);
			}

			*sendlen = COM + COMMAND_DATA_OFFSET + nelements * sizeof(epicsUInt32);
			*recvlen = RESP + 0;
			
			break;
		}

	/* cycle time reset */
	
		case FINS_CYCLE_TIME_RESET:
		{
			pdrvPvt->mrc = 0x06;
			pdrvPvt->src = 0x20;
			pdrvPvt->message[COM] = 0x00;
			
			*sendlen = COM + 1;
			*recvlen = RESP + 0;
			
			break;
		}
	
	/* clear all bits that have been forced on or off */
	
		case FINS_SET_RESET_CANCEL:
		{
			pdrvPvt->mrc = 0x23;
			pdrvPvt->src = 0x02;
			
			*sendlen = COM + 0;
			*recvlen = RESP + 0;
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, command %s not supported.\n", __func__, pdrvPvt->portName, FINS_names[pasynUser->reason]);
			return (-1);
		}
	}
	
	pdrvPvt->message[MRC] = pdrvPvt->mrc;
	pdrvPvt->message[SRC] = pdrvPvt->src;
	pdrvPvt->message[SID] = ++pdrvPvt->sid;

/* add the FINS TCP command */

	if (pdrvPvt->type == FINS_TCP_type)
	{
	
	/* shift the data to make space for the FINS Frame Send Command */
	
		memmove(pdrvPvt->message + FINS_SEND_FRAME_SIZE, pdrvPvt->message, sizeof(pdrvPvt->message) - FINS_SEND_FRAME_SIZE);
		
		AddCommand(pdrvPvt, *sendlen, FINS_FRAME_SEND_COMMAND);
			
		*sendlen += FINS_SEND_FRAME_SIZE;
		*recvlen += FINS_SEND_FRAME_SIZE;
	}
		
	return (0);
}

/**************************************************************************************************/
/*
	asynSize is either sizeof(epicsInt16) for asynInt16Array or sizeof(epicsInt32) for asynInt16Array and asynInt32Array.
*/
/**************************************************************************************************/
	
static int finsWrite(drvPvt * const pdrvPvt, asynUser *pasynUser, const void *data, const size_t nelements, const epicsUInt16 address, const size_t asynSize)
{
	size_t sendlen = 0, sentlen = 0, recvlen = 0, recdlen = 0;
	int eomReason = 0;
	asynStatus status;
	epicsTimeStamp ets, ete;

	if ((pdrvPvt->type == FINS_TCP_type) && (pdrvPvt->nodevalid != 1))
	{
		if (FINSnodeRequest(pdrvPvt) < 0)
		{
			return (-1);
		}
	}
	
	BuildWriteMessage(pdrvPvt, pasynUser, address, nelements, &sendlen, &recvlen, asynSize, data);
	
	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, (char *) pdrvPvt->message, sendlen, "%s: port %s, sending %lu bytes.\n", __func__, pdrvPvt->portName, (unsigned long) sendlen);
	
	epicsTimeGetCurrent(&ets);
	
/* set the time out of writes to the asynOctet port to be the time out specified in the record */

	if (pasynUser->timeout <= 0.0)
	{
		pasynUser->timeout = 1.0;
	}
	
	status = pasynOctetSyncIO->writeRead(pdrvPvt->pasynUser, (char *) pdrvPvt->message, sendlen, (char *) pdrvPvt->message, recvlen, pasynUser->timeout, &sentlen, &recdlen, &eomReason);

	UpdateTimes(pdrvPvt, &ets, &ete);
	
	switch (status)
	{
		case asynSuccess:
		{
			break;
		}
		
		case asynTimeout:
		case asynOverflow:
		case asynError:
		case asynDisconnected:
		case asynDisabled:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, writeRead() failed with %s.\n", __func__, pdrvPvt->portName, asynStatusMessages[status]);
			return (-1);
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, unknown asyn return code %d.\n", __func__, pdrvPvt->portName, status);
			return (-1);
		}
	}

	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, (char *) pdrvPvt->message, recvlen, "%s: port %s, received %lu bytes.\n", __func__, pdrvPvt->portName, (unsigned long) recvlen);

	if (sentlen != sendlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, writeRead() write failed. %lu != %lu\n", __func__, pdrvPvt->portName, (unsigned long) sentlen, (unsigned long) sendlen);
		return (-1);
	}
	
	if (recdlen != recvlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, writeRead() read failed.\n", __func__, pdrvPvt->portName);
		return (-1);
	}
	
/* check and strip the TCP FINS header */

	if (pdrvPvt->type == FINS_TCP_type)
	{
		const unsigned int ferror = BSWAP32(((unsigned int *) pdrvPvt->message)[FINS_MODE_ERROR]);
		 
		if (ferror != FINS_ERROR_NORMAL) 
		{
			pasynCommonSyncIO->disconnectDevice(pdrvPvt->pasynUserCommon);
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, FINS Frame Send error 0x%x.\n", __func__, pdrvPvt->portName, ferror);
			
			return (-1);
		}
		
		memmove(pdrvPvt->message, pdrvPvt->message + FINS_SEND_FRAME_SIZE, sizeof(pdrvPvt->message) - FINS_SEND_FRAME_SIZE);
	}	

	if (CheckData(pdrvPvt, pasynUser) < 0)
	{
		return (-1);
	}

	return (0);
}

/*** asynOctet ************************************************************************************/

static asynStatus octetRead(void *pvt, asynUser *pasynUser, char *data, size_t maxchars, size_t *nbytesTransferred, int *eomReason)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	*eomReason = 0;
	*nbytesTransferred = 0;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}
	
	switch (pasynUser->reason)
	{
		case FINS_MODEL:
		{
			if (maxchars < FINS_MODEL_LEN)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, length is not >= %d for FINS_MODEL\n", __func__, pdrvPvt->portName, addr, FINS_MODEL_LEN);
				return (asynError);
			}
			
			break;
		}
		
	/* no more reasons for asynOctetRead */
	
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);

/* send FINS request */

	if (finsRead(pdrvPvt, pasynUser, (void *) data, maxchars, addr, nbytesTransferred, 0) < 0)
	{
		return (asynError);
	}
	
	if (eomReason)
	{
		*eomReason |= ASYN_EOM_END;
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %lu bytes.\n", __func__, pdrvPvt->portName, addr, (unsigned long) *nbytesTransferred);

   	return (asynSuccess);
}

/* Form a FINS write message, send request, wait for the reply and check for errors */

static asynStatus octetWrite(void *pvt, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransferred)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	*nbytesTransferred = 0;

	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}
	
	switch (pasynUser->reason)
	{
		case FINS_CYCLE_TIME_RESET:
		{
			break;
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
	
/* form FINS message and send data */
	
	if (finsWrite(pdrvPvt, pasynUser, (void *) data, numchars, addr, 0) < 0)
	{
		return (asynError);
	}

/* assume for now that we can always write the full request */

	*nbytesTransferred = numchars;

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %lu bytes.\n", __func__, pdrvPvt->portName, addr, (unsigned long) numchars);

   	return (asynSuccess);
}

/*** asynInt32 ************************************************************************************/

static asynStatus ReadInt32(void *pvt, asynUser *pasynUser, epicsInt32 *value)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		case FINS_WR_READ:
		case FINS_HR_READ:
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

	/* these get called at initialisation by write methods */
	
		case FINS_DM_WRITE:
		case FINS_IO_WRITE:
		case FINS_AR_WRITE:
		case FINS_CT_WRITE:
		case FINS_DM_WRITE_32:
		case FINS_IO_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_CT_WRITE_32:
		{
			break;
		}

	/* user selects these when they don't want to initialise the record by performing a read first */
	
		case FINS_DM_WRITE_NOREAD:
		case FINS_IO_WRITE_NOREAD:
		case FINS_AR_WRITE_NOREAD:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_IO_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32_NOREAD:
		{
			asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, WRITE_NOREAD\n", __func__, pdrvPvt->portName, addr);
			return (asynError);
		}

	/* don't try and perform a read to initialise the PV */
	
		case FINS_SET_RESET_CANCEL:
		{
			return (asynError);
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, no such command %d.\n", __func__, pdrvPvt->portName, addr, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
	
/* send FINS request */

	if (finsRead(pdrvPvt, pasynUser, (void *) value, ONE_ELEMENT, addr, NULL, sizeof(epicsUInt32)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read 1 value.\n", __func__, pdrvPvt->portName, addr);

	return (asynSuccess);
}

static asynStatus WriteInt32(void *pvt, asynUser *pasynUser, epicsInt32 value)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}
	
	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		case FINS_DM_WRITE_NOREAD:
		case FINS_AR_WRITE:
		case FINS_AR_WRITE_NOREAD:
		case FINS_IO_WRITE:
		case FINS_IO_WRITE_NOREAD:
		case FINS_CYCLE_TIME_RESET:
		case FINS_DM_WRITE_32:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32:
		case FINS_AR_WRITE_32_NOREAD:
		case FINS_IO_WRITE_32:
		case FINS_IO_WRITE_32_NOREAD:
		case FINS_SET_RESET_CANCEL:
		{
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, command %s not supported.\n", __func__, pdrvPvt->portName, FINS_names[pasynUser->reason]);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);

/* form FINS message and send data */

	if (finsWrite(pdrvPvt, pasynUser, (void *) &value, ONE_ELEMENT, addr, sizeof(epicsUInt32)) < 0)
	{
		return (asynError);
	}
	
	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote 1 value.\n", __func__, pdrvPvt->portName, addr);

	return (asynSuccess);
}

/*** asynFloat64 **********************************************************************************/

static asynStatus ReadFloat64(void *pvt, asynUser *pasynUser, epicsFloat64 *value)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	epicsFloat32 val;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		{
			break;
		}
		
	/* this gets called at initialisation by write methods */
	
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		{
			break;
		}

		case FINS_DM_WRITE_NOREAD:
		case FINS_AR_WRITE_NOREAD:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32_NOREAD:
		{
			asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, WRITE_NOREAD\n", __func__, pdrvPvt->portName, addr);
			return (asynError);
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, no such command %d.\n", __func__, pdrvPvt->portName, addr, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);

/* send FINS request */

	if (finsRead(pdrvPvt, pasynUser, (void *) &val, ONE_ELEMENT, addr, NULL, sizeof(epicsUInt32)) < 0)
	{
		return (asynError);
	}

	*value = (epicsFloat64) val;
	
	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read 1 word.\n", __func__, pdrvPvt->portName, addr);

	return (asynSuccess);
}

static asynStatus WriteFloat64(void *pvt, asynUser *pasynUser, epicsFloat64 value)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	epicsFloat32 val = (epicsFloat32) value;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32:
		case FINS_AR_WRITE_32_NOREAD:
		{
			break;
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
	
/* form FINS message and send data */

	if (finsWrite(pdrvPvt, pasynUser, (void *) &val, ONE_ELEMENT, addr, sizeof(epicsUInt32)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote 1 word.\n", __func__, pdrvPvt->portName, addr);

	return (asynSuccess);
}

/*** asynInt16Array *******************************************************************************/

static asynStatus ReadInt16Array(void *pvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements, size_t *nIn)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
		case FINS_WR_READ:
		case FINS_HR_READ:
		{
			if (((pdrvPvt->type == FINS_UDP_type) && (nelements > FINS_MAX_UDP_WORDS)) || ((pdrvPvt->type == FINS_TCP_type) && (nelements > FINS_MAX_TCP_WORDS)) || ((pdrvPvt->type == HOSTLINK_type) && (nelements > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		case FINS_CLOCK_READ:
		{
			if (nelements != FINS_CLOCK_READ_LEN)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_CLOCK_READ size != %d.\n", __func__, pdrvPvt->portName, addr, FINS_CLOCK_READ_LEN);
				return (asynError);
			}
			
			break;
		}
		
		case FINS_MM_READ:
		{
			if (nelements > FINS_MM_MAX_ADDRS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_MM_READ size > %d.\n", __func__, pdrvPvt->portName, addr, FINS_MM_MAX_ADDRS);
				return (asynError);
			}
			
			if (addr >= ellCount(&mmList))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_MM_READ invalid entry.\n", __func__, pdrvPvt->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
	
/* send FINS request */

	if (finsRead(pdrvPvt, pasynUser, (void *) value, nelements, addr, nIn, sizeof(epicsUInt16)) < 0)
	{
		*nIn = 0;
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %lu 16-bit word(s).\n", __func__, pdrvPvt->portName, addr, (unsigned long) *nIn);

	return (asynSuccess);
}

static asynStatus WriteInt16Array(void *pvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

/* check reason */

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		case FINS_AR_WRITE:
		case FINS_IO_WRITE:
		{
			if (((pdrvPvt->type == FINS_UDP_type) && (nelements > FINS_MAX_UDP_WORDS)) || ((pdrvPvt->type == FINS_TCP_type) && (nelements > FINS_MAX_TCP_WORDS)) || ((pdrvPvt->type == HOSTLINK_type) && (nelements > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);

/* form FINS message and send data */

	if (finsWrite(pdrvPvt, pasynUser, (void *) value, nelements, addr, sizeof(epicsUInt16)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %lu 16-bit word(s).\n", __func__, pdrvPvt->portName, addr, (unsigned long) nelements);

	return (asynSuccess);
}

/*** asynInt32Array *******************************************************************************/

static asynStatus ReadInt32Array(void *pvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements, size_t *nIn)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		case FINS_IO_READ_32:
		{
			if (((pdrvPvt->type == FINS_UDP_type) && ((nelements * 2) > FINS_MAX_UDP_WORDS)) || ((pdrvPvt->type == FINS_TCP_type) && ((nelements * 2) > FINS_MAX_TCP_WORDS)) || ((pdrvPvt->type == HOSTLINK_type) && ((nelements * 2) > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}

		case FINS_CYCLE_TIME:
		{
			if (nelements != FINS_CYCLE_TIME_LEN)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_CYCLE_TIME size != %d.\n", __func__, pdrvPvt->portName, addr, FINS_CYCLE_TIME_LEN);
				return (asynError);
			}
			
			break;
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);

/* send FINS request */

	if (finsRead(pdrvPvt, pasynUser, (void *) value, nelements, addr, nIn, sizeof(epicsUInt32)) < 0)
	{
		*nIn = 0;
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %lu 32-bit word(s).\n", __func__, pdrvPvt->portName, addr, (unsigned long) *nIn);
	
	return (asynSuccess);
}

static asynStatus WriteInt32Array(void *pvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		case FINS_IO_WRITE_32:
		{
			if (((pdrvPvt->type == FINS_UDP_type) && ((nelements * 2) > FINS_MAX_UDP_WORDS)) || ((pdrvPvt->type == FINS_TCP_type) && ((nelements * 2) > FINS_MAX_TCP_WORDS)) || ((pdrvPvt->type == HOSTLINK_type) && ((nelements * 2) > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
	
/* form FINS message and send data */

	if (finsWrite(pdrvPvt, pasynUser, (void *) value, nelements, addr, sizeof(epicsUInt32)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %lu 32-bit word(s).\n", __func__, pdrvPvt->portName, addr, (unsigned long) nelements);

	return (asynSuccess);
}

/*** asynFloat32Array *****************************************************************************/

/*
	Read 32 bit values from the PLC which are encoded as IEEE floats
*/

static asynStatus ReadFloat32Array(void *pvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements, size_t *nIn)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_READ_32:
		case FINS_AR_READ_32:
		{
			if (((pdrvPvt->type == FINS_UDP_type) && ((nelements * 2) > FINS_MAX_UDP_WORDS)) || ((pdrvPvt->type == FINS_TCP_type) && ((nelements * 2) > FINS_MAX_TCP_WORDS)) || ((pdrvPvt->type == HOSTLINK_type) && ((nelements * 2) > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
	
/* send FINS request */

	if (finsRead(pdrvPvt, pasynUser, (void *) value, nelements, addr, nIn, sizeof(epicsInt32)) < 0)
	{
		*nIn = 0;
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %lu float(s).\n", __func__, pdrvPvt->portName, addr, (unsigned long) *nIn);
	
	return (asynSuccess);
}

static asynStatus WriteFloat32Array(void *pvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;
	int addr;
	asynStatus status;

	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE_32:
		case FINS_AR_WRITE_32:
		{
			if (((pdrvPvt->type == FINS_UDP_type) && ((nelements * 2) > FINS_MAX_UDP_WORDS)) || ((pdrvPvt->type == FINS_TCP_type) && ((nelements * 2) > FINS_MAX_TCP_WORDS)) || ((pdrvPvt->type == HOSTLINK_type) && ((nelements * 2) > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvt->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvt->portName, addr, FINS_names[pasynUser->reason]);
	
/* form FINS message and send data */

	if (finsWrite(pdrvPvt, pasynUser, (void *) value, nelements, addr, sizeof(epicsInt32)) < 0)
	{
		return (asynError);
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %lu float(s).\n", __func__, pdrvPvt->portName, addr, (unsigned long) nelements);

	return (asynSuccess);
}

/*** asynDrvUser **********************************************************************************/

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
	return (asynSuccess);
}

static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser, const char **pptypeName, size_t *psize)
{
	*psize = 0;
	return (asynSuccess);
}

asynStatus drvUserCreate(void *pvt, asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize)
{
	drvPvt * const pdrvPvt = (drvPvt *) pvt;

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
		if (strcmp("FINS_WR_READ", drvInfo) == 0)
		{
			pasynUser->reason = FINS_WR_READ;
		}
		else
		if (strcmp("FINS_HR_READ", drvInfo) == 0)
		{
			pasynUser->reason = FINS_HR_READ;
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
		if (strcmp("FINS_SET_RESET_CANCEL", drvInfo) == 0)
		{
			pasynUser->reason = FINS_SET_RESET_CANCEL;
		}
		else
		if (strcmp("FINS_MM_READ", drvInfo) == 0)
		{
			pasynUser->reason = FINS_MM_READ;
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
static const char * const error22 = "Not executable in current mode";
static const char * const error23 = "No unit";
static const char * const error24 = "Start/Stop not possible";
static const char * const error25 = "Unit error";
static const char * const error26 = "Command error";
static const char * const error30 = "Access rights error";
static const char * const error40 = "Abort error";

static void FINSerror(const drvPvt * const pdrvPvt, asynUser *pasynUser, const char *name, const unsigned char mres, const unsigned char sres)
{
	if (mres & 0x80)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Relay Error Flag\n", name, pdrvPvt->portName);
		
		FINSerror(pdrvPvt, pasynUser, name, mres ^ 0x80, sres);
	}
	
	switch (mres)
	{
		case 0x01:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error01, sres);
			break;
		}
		
		case 0x02:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error02, sres);
			break;
		}
		
		case 0x03:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error03, sres);
			break;
		}
		
		case 0x04:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error04, sres);
			break;
		}
		
		case 0x05:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error05, sres);
			break;
		}
		
		case 0x10:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error10, sres);
			break;
		}
		
		case 0x11:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error11, sres);
			break;
		}
		
		case 0x20:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error20, sres);
			break;
		}
		
		case 0x21:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error21, sres);
			break;
		}
		
		case 0x22:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error22, sres);
			break;
		}
		
		case 0x23:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error23, sres);
			break;
		}
		
		case 0x24:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %se 0x%02x\n", name, pdrvPvt->portName, error24, sres);
			break;
		}
		
		case 0x25:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error25, sres);
			break;
		}
		
		case 0x26:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error26, sres);
			break;
		}
		
		case 0x30:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error30, sres);
			break;
		}
		
		case 0x40:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, %s 0x%02x\n", name, pdrvPvt->portName, error40, sres);
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, Error 0x%02x/0x%02x\n", name, pdrvPvt->portName, mres, sres);
			break;
		}
	}
}

/*** ioc shell ************************************************************************************/

static const iocshArg finsNETInitArg0 = { "port name", iocshArgString };
static const iocshArg finsNETInitArg1 = { "network device", iocshArgString };
static const iocshArg finsNETInitArg2 = { "local node address", iocshArgInt };

static const iocshArg *finsNETInitArgs[] = { &finsNETInitArg0, &finsNETInitArg1, &finsNETInitArg2};
static const iocshFuncDef finsNETInitFuncDef = { "finsNETInit", 3, finsNETInitArgs};

static void finsNETInitCallFunc(const iocshArgBuf *args)
{
	finsNETInit(args[0].sval, args[1].sval, args[2].ival);
}

static void finsNETRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsNETInitFuncDef, finsNETInitCallFunc);
	}
}

epicsExportRegistrar(finsNETRegister);

/*------------------------------------------------------------------------------------------------*/

static const iocshArg finsDEVInitArg0 = { "port name", iocshArgString };
static const iocshArg finsDEVInitArg1 = { "serial device", iocshArgString };

static const iocshArg *finsDEVInitArgs[] = { &finsDEVInitArg0, &finsDEVInitArg1};
static const iocshFuncDef finsDEVInitFuncDef = { "finsDEVInit", 2, finsDEVInitArgs};

static void finsDEVInitCallFunc(const iocshArgBuf *args)
{
	finsDEVInit(args[0].sval, args[1].sval);
}

static void finsDEVRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsDEVInitFuncDef, finsDEVInitCallFunc);
	}
}

epicsExportRegistrar(finsDEVRegister);

/*------------------------------------------------------------------------------------------------*/

static const iocshArg finsUDPInitArg0 = { "port name", iocshArgString };
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

/*------------------------------------------------------------------------------------------------*/

static const iocshArg finsTCPInitArg0 = { "port name", iocshArgString };
static const iocshArg finsTCPInitArg1 = { "IP address", iocshArgString };

static const iocshArg *finsTCPInitArgs[] = { &finsTCPInitArg0, &finsTCPInitArg1};
static const iocshFuncDef finsTCPInitFuncDef = { "finsTCPInit", 2, finsTCPInitArgs};

static void finsTCPInitCallFunc(const iocshArgBuf *args)
{
	finsTCPInit(args[0].sval, args[1].sval);
}

static void finsTCPRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsTCPInitFuncDef, finsTCPInitCallFunc);
	}
}

epicsExportRegistrar(finsTCPRegister);

/**************************************************************************************************/

/*
	This is a test function to send a FINS data memory read request for two words from
	address 100 to the specified IP address. It will print the data received as hex, or
	a helpful error message if something fails.
*/

int finsTest(char *address)
{
	int fd;
	struct sockaddr_in addr;
	const int addrlen = sizeof(struct sockaddr_in);
	uint8_t node;
	unsigned char *message;
	int recvlen, sendlen = 0;
	
	message = (unsigned char *) callocMustSucceed(1, FINS_MAX_MSG, "finsTest");
	
/* open a datagram socket */

	fd = socket(PF_INET, SOCK_DGRAM, 0);
	
	if (fd < 0)
	{
		perror("finsTest: socket");
		return (-1);
	}
	
	bzero((char *) &(addr), addrlen);

/* ask for a free port for incoming UDP packets */

	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(0);

/* bind socket to address */

	if (bind(fd, (struct sockaddr *) &addr, addrlen) < 0)
	{
		perror("finsTest: bind failed");
		
		close(fd);
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
		getsockname(fd, (struct sockaddr *) &name, &namelen);

		printf("finsTest: port %d bound\n", name.sin_port);
	}
	
/* destination port address used later in sendto() */

	bzero((char *) &addr, addrlen);

/*
	addr.sin_family = AF_INET;
	addr.sin_port = htons(FINS_UDP_PORT);
*/

/* convert IP address */

	if (aToIPAddr(address, FINS_NET_PORT, &addr) < 0)
	{
		close(fd);
		printf("finsTest: Bad IP address %s\n", address);
		return (-1);
	}

/* node address is last byte of IP address */
		
	node = ntohl(addr.sin_addr.s_addr) & 0xff;
		
	printf("PLC node %d\n", node);

/* send a simple FINS command */

	message[ICF] = 0x80;
	message[RSV] = 0x00;
	message[GCT] = 0x02;

	message[DNA] = 0x00;
	message[DA1] = node;		/* destination node */
	message[DA2] = 0x00;

	message[SNA] = 0x00;
	message[SA1] = 0x01;		/* source node */
	message[SA2] = 0x00;

	message[MRC] = 0x01;
	message[SRC] = 0x01;
	message[COM] = DM;		/* data memory read */

	message[COM+1] = 100 >> 8;
	message[COM+2] = 100 & 0xff;
	message[COM+3] = 0x00;		/* start address */

	message[COM+4] = 2 >> 8;
	message[COM+5] = 2 & 0xff;	/* length */

	sendlen = COM + 6;

/* send request */

	if (sendto(fd, message, sendlen, 0, (struct sockaddr *) &addr, addrlen) != sendlen)
	{
		perror("finsTest: sendto");
		close(fd);
		return (-1);
	}

/* receive reply with time out */

	{
		fd_set rfds;
		struct timeval tv;
		
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		
	/* time out */

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

	{
		struct sockaddr from_addr;
#ifdef vxWorks
		int iFromLen = 0;
#else
		socklen_t iFromLen = 0;
#endif
		if ((recvlen = recvfrom(fd, message, FINS_MAX_MSG, 0, &from_addr, &iFromLen)) < 0)
		{
			perror("finsTest: recvfrom");
			close(fd);
			return (-1);
		}
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

static const iocshArg finsTestArg0 = { "IP address", iocshArgString };

static const iocshArg *finsTestArgs[] = { &finsTestArg0};
static const iocshFuncDef finsTestFuncDef = { "finsTest", 1, finsTestArgs};

static void finsTestCallFunc(const iocshArgBuf *args)
{
	finsTest(args[0].sval);
}

static void finsTestRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsTestFuncDef, finsTestCallFunc);
	}
}

epicsExportRegistrar(finsTestRegister);

/**************************************************************************************************/

/* Create a list of 'memory area' & 'address' pairs for the Read Multiple Memory Area command */

static int initlist = 1;

int finsMultiMemoryAreaInit(char *s)
{
	MultiMemArea *MM;

/* initialise the linked list */

	if (initlist)
	{
		initlist = 0;
		
		ellInit(&mmList);
	}

/* allocate space to 10 memory type / memory address pairs */
	
	MM = (MultiMemArea *) callocMustSucceed(1, sizeof(MultiMemArea), "finsMultiMemoryAreaInit");
		
/* and add to our list */

	ellAdd(&mmList, &(MM->node));
	
/* scan for memory type / memory address pairs */

	{
		int num = 0, i, p = 0;

		do
		{
			i = sscanf(s += num, "%hi%hi,%n", &(MM->area[p]), &(MM->address[p]), &num);
			p++;
		}
		while ((i == 2) && (p < FINS_MM_MAX_ADDRS));
	}
	
	return (0);
}

int finsMultiMemoryAreaDump(void)
{
	int i, j;
	MultiMemArea *MM;
	
	for (i = 0, MM = (MultiMemArea *) ellFirst(&mmList); MM; i++, MM = (MultiMemArea *) ellNext(&(MM->node)))
	{
		printf("%2d: ", i);
		
		for (j = 0; j < FINS_MM_MAX_ADDRS; j++)
		{
			if (MM->area[j] == 0x00) break;
			
			printf("%s0x%02x 0x%04x", (j > 0) ? ", " : "", MM->area[j], MM->address[j]);
		}
		
		puts("");
	}
	
	return (0);
}

static const iocshArg finsMultiMemoryAreaInitArg0 = { "area/address", iocshArgString };

static const iocshArg *finsMultiMemoryAreaInitArgs[] = { &finsMultiMemoryAreaInitArg0};
static const iocshFuncDef finsMultiMemoryAreaInitFuncDef = { "finsMultiMemoryAreaInit", 1, finsMultiMemoryAreaInitArgs};

static void finsMultiMemoryAreaInitCallFunc(const iocshArgBuf *args)
{
	finsMultiMemoryAreaInit(args[0].sval);
}

static void finsMultiMemoryAreaInitRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsMultiMemoryAreaInitFuncDef, finsMultiMemoryAreaInitCallFunc);
	}
}

epicsExportRegistrar(finsMultiMemoryAreaInitRegister);

/**************************************************************************************************/
