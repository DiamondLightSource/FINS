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

#define SIM_DATA_SIZE 65536

typedef struct drvPvtSim
{
	int connected;
	int type;
	int nodevalid;

	const char *portName;
	const char *ipaddr;

	asynStandardInterfaces asynStdInterfaces;

	asynUser *pasynUser;
	asynUser *pasynUserCommon;

	epicsUInt8 dnode, snode;		/* source and destination node addresses */
	epicsUInt8 sid;				/* session id - incremented for each message */
	epicsUInt8 mrc, src;
	epicsFloat32 tMax, tMin, tLast;	/* Max and Min and last response time of PLC */
	epicsUInt8 message[FINS_MAX_MSG];

	struct sockaddr_in addr;

	epicsUInt16 simData[SIM_DATA_SIZE];
} drvPvtSim;

/*** asynCommon methods ***************************************************************************/

static void reportSim(void *drvPvtSim, FILE *fp, int details);
static asynStatus aconnectSim(void *drvPvtSim, asynUser *pasynUser);
static asynStatus adisconnectSim(void *drvPvtSim, asynUser *pasynUser);
static asynCommon ifacecommonSim = { reportSim, aconnectSim, adisconnectSim };

/*** asynOctet methods ****************************************************************************/

static asynStatus octetReadSim (void *drvPvtSim, asynUser *pasynUser, char *data, size_t maxchars, size_t *nbytesTransfered, int *eomReason);
static asynStatus octetWriteSim(void *drvPvtSim, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransfered);
static asynStatus octetFlushSim(void *drvPvtSim, asynUser *pasynUser);

static asynOctet ifaceOctetSim = { octetWriteSim, octetReadSim, octetFlushSim, NULL, NULL, NULL, NULL, NULL, NULL};

/*** asynInt32 methods ****************************************************************************/

static asynStatus WriteInt32Sim(void *drvPvtSim, asynUser *pasynUser, epicsInt32 value);
static asynStatus ReadInt32Sim(void *drvPvtSim, asynUser *pasynUser, epicsInt32 *value);

static asynInt32 ifaceInt32Sim = { WriteInt32Sim, ReadInt32Sim, NULL, NULL, NULL};

/*** asynFloat64 methods **************************************************************************/

static asynStatus WriteFloat64Sim(void *drvPvtSim, asynUser *pasynUser, epicsFloat64 value);
static asynStatus ReadFloat64Sim(void *drvPvtSim, asynUser *pasynUser, epicsFloat64 *value);

static asynFloat64 ifaceFloat64Sim = { WriteFloat64Sim, ReadFloat64Sim, NULL, NULL};

/*** asynInt16Array methods ***********************************************************************/

static asynStatus WriteInt16ArraySim(void *drvPvtSim, asynUser *pasynUser, epicsInt16 *value, size_t nelements);
static asynStatus ReadInt16ArraySim(void *drvPvtSim, asynUser *pasynUser, epicsInt16 *value, size_t nelements, size_t *nIn);

static asynInt16Array ifaceInt16ArraySim = { WriteInt16ArraySim, ReadInt16ArraySim, NULL, NULL};

/*** asynInt32Array methods ***********************************************************************/

static asynStatus WriteInt32ArraySim(void *drvPvtSim, asynUser *pasynUser, epicsInt32 *value, size_t nelements);
static asynStatus ReadInt32ArraySim(void *drvPvtSim, asynUser *pasynUser, epicsInt32 *value, size_t nelements, size_t *nIn);

static asynInt32Array ifaceInt32ArraySim = { WriteInt32ArraySim, ReadInt32ArraySim, NULL, NULL};

/*** asynFloat32Array *****************************************************************************/

static asynStatus WriteFloat32ArraySim(void *drvPvtSim, asynUser *pasynUser, epicsFloat32 *value, size_t nelements);
static asynStatus ReadFloat32ArraySim(void *drvPvtSim, asynUser *pasynUser, epicsFloat32 *value, size_t nelements, size_t *nIn);

static asynFloat32Array ifaceFloat32ArraySim = { WriteFloat32ArraySim, ReadFloat32ArraySim, NULL, NULL};

/*** asynDrvUser **********************************************************************************/

static asynStatus drvUserCreateSim (void *drvPvtSim, asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
static asynStatus drvUserGetTypeSim(void *drvPvtSim, asynUser *pasynUser, const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroySim(void *drvPvtSim, asynUser *pasynUser);

static asynDrvUser ifaceDrvUserSim = { drvUserCreateSim, drvUserGetTypeSim, drvUserDestroySim};

/**************************************************************************************************/

/* double linked list for Multiple Memory reads */

static ELLLIST mmListSim;

/**************************************************************************************************/

int finsSIMInit(const char *portName)
{
	asynStatus status;
	asynStandardInterfaces *pInterfaces;
	drvPvtSim *pdrvPvtSim = callocMustSucceed(1, sizeof(drvPvtSim), __func__);

    printf("SIMULATED FINS using portName '%s'\n", portName);

	pdrvPvtSim->portName = epicsStrDup(portName);
	pdrvPvtSim->tLast = -1.0;

	pdrvPvtSim->pasynUser = pasynManager->createAsynUser(0, 0);
	pdrvPvtSim->pasynUserCommon = pasynManager->createAsynUser(0, 0);

	status = pasynManager->registerPort(portName, ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0);

	if (status != asynSuccess)
	{
		printf("%s: registerPort failed\n", __func__);
		return (-1);
	}

/* Create asyn interfaces and register with asynManager */

	pInterfaces = &pdrvPvtSim->asynStdInterfaces;
	
	pInterfaces->common.pinterface = (void *) &ifacecommonSim;
	pInterfaces->drvUser.pinterface = (void *) &ifaceDrvUserSim;
	pInterfaces->octet.pinterface = (void *) &ifaceOctetSim;
	pInterfaces->int32.pinterface = (void *)&ifaceInt32Sim;
	pInterfaces->float64.pinterface = (void *) &ifaceFloat64Sim;
	pInterfaces->int16Array.pinterface = (void *) &ifaceInt16ArraySim;
	pInterfaces->int32Array.pinterface = (void *) &ifaceInt32ArraySim;
	pInterfaces->float32Array.pinterface = (void *) &ifaceFloat32ArraySim;

	status = pasynStandardInterfacesBase->initialize(pdrvPvtSim->portName, pInterfaces, pdrvPvtSim->pasynUser, pdrvPvtSim);
	
	if (status != asynSuccess)
	{
		errlogPrintf("%s: port %s can't register standard interfaces: %s\n", __func__, 
                     pdrvPvtSim->portName, pdrvPvtSim->pasynUser->errorMessage);
		return (-1);
	}

	// initialise the simulator's data (used to store simulator's value of DM addresses)
	memset(pdrvPvtSim->simData, '\0', sizeof(epicsUInt16) * SIM_DATA_SIZE);

	return 0;
}

/**************************************************************************************************/

static void reportSim(void *pvt, FILE *fp, int details)
{
	const drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
    // uncomment following if display sim data - see below
    //int i;
	
	fprintf(fp, "%s: connected to FINS simulator %s \n", pdrvPvtSim->portName, (pdrvPvtSim->connected ? "Yes" : "No"));

    // uncomment following to display current sim data values when asynReport is called
/*    fprintf(fp, "Sim data:\n");
    for(i=0; i < SIM_DATA_SIZE; i++) {
        fprintf(fp, "[%05d]%04x ", i ,pdrvPvtSim->simData[i]);
    }
    fprintf(fp, "\n");
*/	
	fprintf(fp, "    Min: %.4fs  Max: %.4fs  Last: %.4fs\n", pdrvPvtSim->tMin, pdrvPvtSim->tMax, pdrvPvtSim->tLast);
}

/**************************************************************************************************/

static asynStatus aconnectSim(void *pvt, asynUser *pasynUser)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
	asynStatus status;
	int addr;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s connect addr %d\n", pdrvPvtSim->portName, addr);
	
	if (addr >= 0)
	{
		pasynManager->exceptionConnect(pasynUser);
		return (asynSuccess);
	}
	
	if (pdrvPvtSim->connected)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s already connected\n", pdrvPvtSim->portName);
		return (asynError);
	}

	pdrvPvtSim->connected = 1;
	pasynManager->exceptionConnect(pasynUser);
	return (asynSuccess);
}

/**************************************************************************************************/

static asynStatus adisconnectSim(void *pvt, asynUser *pasynUser)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
	asynStatus status;
	int addr;
	
	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s disconnect addr %d\n", pdrvPvtSim->portName, addr);

	if (addr >= 0)
	{
		pasynManager->exceptionDisconnect(pasynUser);
		return (asynSuccess);
	}
	
	if (!pdrvPvtSim->connected)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "port %s already disconnected\n", pdrvPvtSim->portName);
		return (asynError);
	}
	
	pdrvPvtSim->connected = 0;
	pasynManager->exceptionDisconnect(pasynUser);
	
	return (asynSuccess);
}

/**************************************************************************************************/

static asynStatus octetFlushSim(void *pvt, asynUser *pasynUser)
{
	const drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s\n", __func__, pdrvPvtSim->portName);

 	return (asynSuccess);
}
		
/**************************************************************************************************/

static void UpdateTimesSim(drvPvtSim * const pdrvPvtSim, epicsTimeStamp *ets, epicsTimeStamp *ete)
{
	epicsTimeGetCurrent(ete);

	{
		const double diff = epicsTimeDiffInSeconds(ete, ets);
	
		if (pdrvPvtSim->tLast == -1)
		{
			pdrvPvtSim->tMax = diff;
			pdrvPvtSim->tMin = diff;
		}
		else
		{
			if (diff > pdrvPvtSim->tMax) pdrvPvtSim->tMax = diff;
			if (diff < pdrvPvtSim->tMin) pdrvPvtSim->tMin = diff;
		}
		
		pdrvPvtSim->tLast = diff;
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

static int simRead(drvPvtSim * const pdrvPvtSim, asynUser *pasynUser, void *data, const size_t nelements, const epicsUInt16 address, size_t *transferred, size_t asynSize)
{
	size_t sendlen = 0, sentlen = 0, recvlen = 0, recdlen = 0;
	epicsTimeStamp ets, ete;

	if (nelements < 1)
	{
		return (0);
	}

/* using %lu and casting to unsigned long instead of using %zu because of our old PPC/vxWorks gcc compiler */

	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, (char *) pdrvPvtSim->message, sendlen, "%s: port %s, sending %lu bytes, expecting %lu bytes.\n", __func__, pdrvPvtSim->portName, (unsigned long) sendlen, (unsigned long) recvlen);
	
	if (pasynUser->timeout <= 0.0)
	{
		 pasynUser->timeout = FINS_TIMEOUT;
	}
	
	epicsTimeGetCurrent(&ets);
	
	UpdateTimesSim(pdrvPvtSim, &ets, &ete);
	
	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, (char *) pdrvPvtSim->message, recdlen, "%s: port %s, received %lu bytes.\n", __func__, pdrvPvtSim->portName, (unsigned long) recdlen);

	if (sentlen != sendlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, writeRead() write failed. %lu != %lu\n", __func__, pdrvPvtSim->portName, (unsigned long) sentlen, (unsigned long) sendlen);
		return (-1);
	}
	
	if (recdlen != recvlen)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, writeRead() read failed.\n", __func__, pdrvPvtSim->portName);
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
			int nextAddress = address;
			
		/* asynInt16Array */
		
			if (asynSize == sizeof(epicsUInt16))
			{
				epicsUInt16 *ptrd = (epicsUInt16 *) data;

				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = pdrvPvtSim->simData[nextAddress];
					nextAddress++;
				}
			}
			else
			
		/* asynInt32 * 1 */
		
			{			
				epicsUInt32 *ptrd = (epicsUInt32 *) data;

				for (i = 0; i < nelements; i++)
				{
					ptrd[i] = pdrvPvtSim->simData[nextAddress];
					ptrd[i] <<= 16;
					ptrd[i] += pdrvPvtSim->simData[nextAddress+1];
					nextAddress +=2;
				}
			}

			asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, read %lu 16-bit word(s) from address %d to address %d.\n", __func__, pdrvPvtSim->portName, (unsigned long) nelements, address, (nextAddress-1));
			
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
			epicsUInt32 *ptrd = (epicsUInt32 *) data;
			int nextAddress = address;

			for (i = 0; i < nelements; i++)
			{
				ptrd[i] = pdrvPvtSim->simData[nextAddress];
				ptrd[i] <<= 16;
				ptrd[i] += pdrvPvtSim->simData[nextAddress+1];
				nextAddress +=2;
			}

			asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: port %s, read %lu 32-bit word(s) from address %d to address %d.\n", __func__, pdrvPvtSim->portName, (unsigned long) nelements, address, (nextAddress-2));
			
			break;
		}
		
/* return a string of 20 chars, packed as two characters per word */

		case FINS_MODEL:
		{
			memcpy(data, "", nelements);
            strncpy(data, "FINS-SIM-PLC", nelements);
			
			break;
		}

/* return status - epicsInt32 */

		case FINS_CPU_STATUS:
		{
			*(epicsInt32 *)(data) = pdrvPvtSim->message[RESP + 0];
			
			break;
		}
		
/* return mode - epicsInt32 */

		case FINS_CPU_MODE:
		{
			*(epicsInt32 *)(data) = pdrvPvtSim->message[RESP + 1];
			
			break;
		}

/* return 3 parameters - epicsInt32 */

		case FINS_CYCLE_TIME:
		{
			int i;
			epicsInt32 *rep = (epicsInt32 *) &pdrvPvtSim->message[RESP + 0];
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
			const epicsInt32 *rep = (epicsInt32 *) &pdrvPvtSim->message[RESP + 0];

			*(epicsInt32 *)(data) = BSWAP32(*rep);

			break;
		}
		
/* return max - epicsInt32 */

		case FINS_CYCLE_TIME_MAX:
		{
			const epicsInt32 *rep = (epicsInt32 *) &pdrvPvtSim->message[RESP + 4];

			*(epicsInt32 *)(data) = BSWAP32(*rep);
			
			break;
		}
		
/* return min - epicsInt32 */

		case FINS_CYCLE_TIME_MIN:
		{
			const epicsInt32 *rep = (epicsInt32 *) &pdrvPvtSim->message[RESP + 8];

			*(epicsInt32 *)(data) = BSWAP32(*rep);

			break;
		}

/* asynInt16array */

		case FINS_CLOCK_READ:	/* convert from BCD to dec */
		{
			epicsInt8  *rep = (epicsInt8 *)  &pdrvPvtSim->message[RESP + 0];
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
			MultiMemAreaPair *ptrs = (MultiMemAreaPair *) &pdrvPvtSim->message[RESP];
			epicsUInt16 *ptrd = (epicsUInt16 *) data;
			
			for (i = 0; i < nelements; i++)
			{
				ptrd[i] = (epicsUInt16) BSWAP16(ptrs[i].address);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (-1);
		}
	}
	
	if (transferred) *transferred = nelements;

	return (0);	
}

/*** asynOctet ************************************************************************************/

static asynStatus octetReadSim(void *pvt, asynUser *pasynUser, char *data, size_t maxchars, size_t *nbytesTransferred, int *eomReason)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
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
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, length is not >= %d for FINS_MODEL\n", __func__, pdrvPvtSim->portName, addr, FINS_MODEL_LEN);
				return (asynError);
			}
			
			break;
		}
		
	/* no more reasons for asynoctetReadSim */
	
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);

/* read sim data */

	if (simRead(pdrvPvtSim, pasynUser, (void *) data, maxchars, addr, nbytesTransferred, 0) < 0)
	{
		return (asynError);
	}
	
	if (eomReason)
	{
		*eomReason |= ASYN_EOM_END;
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %lu bytes.\n", __func__, pdrvPvtSim->portName, addr, (unsigned long) *nbytesTransferred);

   	return (asynSuccess);
}

static asynStatus octetWriteSim(void *pvt, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransferred)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
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
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);

    // &&&ajb TODO 

	*nbytesTransferred = numchars;

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %lu bytes.\n", __func__, pdrvPvtSim->portName, addr, (unsigned long) numchars);

   	return (asynSuccess);
}

/*** asynInt32 ************************************************************************************/

static asynStatus ReadInt32Sim(void *pvt, asynUser *pasynUser, epicsInt32 *value)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
	int addr;
	asynStatus status;
	epicsInt32 val;
    int is16bitVal = 0;

	if ((status = pasynManager->getAddr(pasynUser, &addr)) != asynSuccess)
	{
		return (status);
	}

	switch (pasynUser->reason)
	{
		case FINS_DM_READ:
		case FINS_AR_READ:
		case FINS_IO_READ:
        {
            is16bitVal = 1;
            break;
        }

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
			asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, WRITE_NOREAD\n", __func__, pdrvPvtSim->portName, addr);
			return (asynError);
		}

	/* don't try and perform a read to initialise the PV */
	
		case FINS_SET_RESET_CANCEL:
		{
			return (asynError);
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, no such command %d.\n", __func__, pdrvPvtSim->portName, addr, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
	
    if (is16bitVal) {
        // get 16 bit return value from simulator's 16 bit memory structure
        val = pdrvPvtSim->simData[addr] & 0xffff;
    } else {
        // get 32 bit return value from simulator's 16 bit memory structure
	    val = pdrvPvtSim->simData[addr];
	    val <<= 16;
	    val += pdrvPvtSim->simData[addr+1];
    }
    *value = val;

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read 1 %s value (%d).\n", __func__, pdrvPvtSim->portName, addr, ((is16bitVal)?"16bit":"32bit"), *value);

	return (asynSuccess);
}

static asynStatus WriteInt32Sim(void *pvt, asynUser *pasynUser, epicsInt32 value)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
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
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, command %s not supported.\n", __func__, pdrvPvtSim->portName, FINS_names[pasynUser->reason]);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);

	switch (pasynUser->reason)
	{
		case FINS_DM_WRITE:
		case FINS_DM_WRITE_NOREAD:
		case FINS_AR_WRITE:
		case FINS_AR_WRITE_NOREAD:
		case FINS_IO_WRITE:
		case FINS_IO_WRITE_NOREAD:
		{
			epicsInt16 valueStored = 0;
			// store written 16 bit value
			pdrvPvtSim->simData[addr] = value & 0xffff;
			// for debug/display purposes retrieve value just stored to simulator's memory
			valueStored = pdrvPvtSim->simData[addr];

			asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote 1 value. Sim 16bit valueStored=%d.\n", __func__, pdrvPvtSim->portName, addr, valueStored);
			break;
		}
		case FINS_DM_WRITE_32:
		case FINS_DM_WRITE_32_NOREAD:
		case FINS_AR_WRITE_32:
		case FINS_AR_WRITE_32_NOREAD:
		case FINS_IO_WRITE_32:
		case FINS_IO_WRITE_32_NOREAD:
		{
			epicsInt32 valueStored = 0;
			// store written 32 bit value into simulator's 16 bit memory structure
			// MSB of 32 bit value into memory[addr] and LSB into memory[addr+1]
			pdrvPvtSim->simData[addr] = value >> 16;
			pdrvPvtSim->simData[addr+1] = value & 0xffff;
			// for debug/display purposes retrieve value just stored to simulator's memory
			valueStored = pdrvPvtSim->simData[addr];
			valueStored <<= 16;
			valueStored += pdrvPvtSim->simData[addr+1];

			asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote 1 value. Sim 32bit valueStored=%d.\n", __func__, pdrvPvtSim->portName, addr, valueStored);
			break;
		}
		default:
			// do nothing for other 'reasons'
			break;
	}

	return (asynSuccess);
}

/*** asynFloat64 **********************************************************************************/

static asynStatus ReadFloat64Sim(void *pvt, asynUser *pasynUser, epicsFloat64 *value)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
	int addr;
	asynStatus status;
	epicsInt32 val;
	
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
			asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, WRITE_NOREAD\n", __func__, pdrvPvtSim->portName, addr);
			return (asynError);
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, no such command %d.\n", __func__, pdrvPvtSim->portName, addr, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);

    // get 32 bit return value from simulator's 16 bit memory structure
	val = pdrvPvtSim->simData[addr];
	val <<= 16;
	val += pdrvPvtSim->simData[addr+1];
	*value = (epicsFloat64) val;
	
	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read 1 word.\n", __func__, pdrvPvtSim->portName, addr);

	return (asynSuccess);
}

static asynStatus WriteFloat64Sim(void *pvt, asynUser *pasynUser, epicsFloat64 value)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
	int addr;
	asynStatus status;
	// epicsFloat32 val = (epicsFloat32) value;
	
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
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
	
    // &&&ajb TODO

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote 1 word.\n", __func__, pdrvPvtSim->portName, addr);

	return (asynSuccess);
}

/*** asynInt16Array *******************************************************************************/

static asynStatus ReadInt16ArraySim(void *pvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements, size_t *nIn)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
	int addr, i, nextAddress;
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
			if (((pdrvPvtSim->type == FINS_UDP_type) && (nelements > FINS_MAX_UDP_WORDS)) || ((pdrvPvtSim->type == FINS_TCP_type) && (nelements > FINS_MAX_TCP_WORDS)) || ((pdrvPvtSim->type == HOSTLINK_type) && (nelements > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		case FINS_CLOCK_READ:
		{
			if (nelements != FINS_CLOCK_READ_LEN)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_CLOCK_READ size != %d.\n", __func__, pdrvPvtSim->portName, addr, FINS_CLOCK_READ_LEN);
				return (asynError);
			}
			
			break;
		}
		
		case FINS_MM_READ:
		{
			if (nelements > FINS_MM_MAX_ADDRS)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_MM_READ size > %d.\n", __func__, pdrvPvtSim->portName, addr, FINS_MM_MAX_ADDRS);
				return (asynError);
			}
			
			if (addr >= ellCount(&mmListSim))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_MM_READ invalid entry.\n", __func__, pdrvPvtSim->portName, addr);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
	
	if (simRead(pdrvPvtSim, pasynUser, (void *) value, nelements, addr, nIn, sizeof(epicsUInt16)) < 0)
	{
		*nIn = 0;
		return (asynError);
	}
	nextAddress = addr;
	for(i = 0; i < nelements; i++) {
		asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: addr:%d=%d\n", __func__, nextAddress, value[i]);
		nextAddress++;
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %lu 16-bit word(s).\n", __func__, pdrvPvtSim->portName, addr, (unsigned long) *nIn);

	return (asynSuccess);
}

static asynStatus WriteInt16ArraySim(void *pvt, asynUser *pasynUser, epicsInt16 *value, size_t nelements)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
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
			if (((pdrvPvtSim->type == FINS_UDP_type) && (nelements > FINS_MAX_UDP_WORDS)) || ((pdrvPvtSim->type == FINS_TCP_type) && (nelements > FINS_MAX_TCP_WORDS)) || ((pdrvPvtSim->type == HOSTLINK_type) && (nelements > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);

    // &&&ajb TODO 

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %lu 16-bit word(s).\n", __func__, pdrvPvtSim->portName, addr, (unsigned long) nelements);

	return (asynSuccess);
}

/*** asynInt32Array *******************************************************************************/

static asynStatus ReadInt32ArraySim(void *pvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements, size_t *nIn)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
	int addr, i, nextAddress;
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
			if (((pdrvPvtSim->type == FINS_UDP_type) && ((nelements * 2) > FINS_MAX_UDP_WORDS)) || ((pdrvPvtSim->type == FINS_TCP_type) && ((nelements * 2) > FINS_MAX_TCP_WORDS)) || ((pdrvPvtSim->type == HOSTLINK_type) && ((nelements * 2) > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}

		case FINS_CYCLE_TIME:
		{
			if (nelements != FINS_CYCLE_TIME_LEN)
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, FINS_CYCLE_TIME size != %d.\n", __func__, pdrvPvtSim->portName, addr, FINS_CYCLE_TIME_LEN);
				return (asynError);
			}
			
			break;
		}

		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s (nelements=%lu)\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason], (unsigned long) nelements);

	if (simRead(pdrvPvtSim, pasynUser, (void *) value, nelements, addr, nIn, sizeof(epicsUInt32)) < 0)
	{
		*nIn = 0;
		return (asynError);
	}
	nextAddress = addr;
	for(i = 0; i < nelements; i++) {
		asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: addr:%d=%d\n", __func__, nextAddress, value[i]);
		nextAddress += 2;
	}

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %lu 32-bit word(s).\n", __func__, pdrvPvtSim->portName, addr, (unsigned long) *nIn);
	
	return (asynSuccess);
}

static asynStatus WriteInt32ArraySim(void *pvt, asynUser *pasynUser, epicsInt32 *value, size_t nelements)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
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
			if (((pdrvPvtSim->type == FINS_UDP_type) && ((nelements * 2) > FINS_MAX_UDP_WORDS)) || ((pdrvPvtSim->type == FINS_TCP_type) && ((nelements * 2) > FINS_MAX_TCP_WORDS)) || ((pdrvPvtSim->type == HOSTLINK_type) && ((nelements * 2) > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
	
    // &&&ajb TODO 

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %lu 32-bit word(s).\n", __func__, pdrvPvtSim->portName, addr, (unsigned long) nelements);

	return (asynSuccess);
}

/*** asynFloat32Array *****************************************************************************/

/*
	Read 32 bit values from the PLC which are encoded as IEEE floats
*/

static asynStatus ReadFloat32ArraySim(void *pvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements, size_t *nIn)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
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
			if (((pdrvPvtSim->type == FINS_UDP_type) && ((nelements * 2) > FINS_MAX_UDP_WORDS)) || ((pdrvPvtSim->type == FINS_TCP_type) && ((nelements * 2) > FINS_MAX_TCP_WORDS)) || ((pdrvPvtSim->type == HOSTLINK_type) && ((nelements * 2) > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
	
    // &&&ajb TODO 

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, read %lu float(s).\n", __func__, pdrvPvtSim->portName, addr, (unsigned long) *nIn);
	
	return (asynSuccess);
}

static asynStatus WriteFloat32ArraySim(void *pvt, asynUser *pasynUser, epicsFloat32 *value, size_t nelements)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;
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
			if (((pdrvPvtSim->type == FINS_UDP_type) && ((nelements * 2) > FINS_MAX_UDP_WORDS)) || ((pdrvPvtSim->type == FINS_TCP_type) && ((nelements * 2) > FINS_MAX_TCP_WORDS)) || ((pdrvPvtSim->type == HOSTLINK_type) && ((nelements * 2) > FINS_MAX_HOST_WORDS)))
			{
				asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, addr %d, request too big for %s.\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
				return (asynError);
			}
			
			break;
		}
		
		default:
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, no such command %d.\n", __func__, pdrvPvtSim->portName, pasynUser->reason);
			return (asynError);
		}
	}
	
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: port %s, addr %d, %s\n", __func__, pdrvPvtSim->portName, addr, FINS_names[pasynUser->reason]);
	
    // &&&ajb TODO 

	asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "%s: port %s, addr %d, wrote %lu float(s).\n", __func__, pdrvPvtSim->portName, addr, (unsigned long) nelements);

	return (asynSuccess);
}

/*** asynDrvUser **********************************************************************************/

static asynStatus drvUserDestroySim(void *drvPvtSim, asynUser *pasynUser)
{
	return (asynSuccess);
}

static asynStatus drvUserGetTypeSim(void *drvPvtSim, asynUser *pasynUser, const char **pptypeName, size_t *psize)
{
	*psize = 0;
	return (asynSuccess);
}

asynStatus drvUserCreateSim(void *pvt, asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize)
{
	drvPvtSim * const pdrvPvtSim = (drvPvtSim *) pvt;

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

		asynPrint(pasynUser, ASYN_TRACEIO_DEVICE, "drvUserCreateSim: port %s, %s = %d\n", pdrvPvtSim->portName, drvInfo, pasynUser->reason);

		return (asynSuccess);
	}

	return (asynError);
}


/*** ioc shell ************************************************************************************/

static const iocshArg finsSIMInitArg0 = { "port name", iocshArgString };

static const iocshArg *finsSIMInitArgs[] = { &finsSIMInitArg0 };
static const iocshFuncDef finsSIMInitFuncDef = { "finsSIMInit", 1, finsSIMInitArgs};

static void finsSIMInitCallFunc(const iocshArgBuf *args)
{
	finsSIMInit(args[0].sval);
}

static void finsSIMRegister(void)
{
	static int firstTime = 1;
	
	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&finsSIMInitFuncDef, finsSIMInitCallFunc);
	}
}

epicsExportRegistrar(finsSIMRegister);

/**************************************************************************************************/

