/*
	This program creates an asyn interpose layer for asynOctet methods. It converts between FINS
	binary and HOSTLINK ASCII message types.
*/

#include <stdio.h>
#include <string.h>

#include <cantProceed.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsTypes.h>
#include <iocsh.h>
#include <osiUnistd.h>
#include <osiSock.h>

#include "asynDriver.h"
#include "asynOctet.h"
#include "asynOption.h"
#include "asynStandardInterfaces.h"
#include <epicsExport.h>

#include "FINS.h"

/* asynOctet methods */

static asynStatus interposeWrite(void *ppvt, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransfered);
static asynStatus interposeRead(void *ppvt, asynUser *pasynUser, char *data,size_t maxchars, size_t *nbytesTransfered, int *eomReason);
static asynStatus interposeFlush(void *ppvt, asynUser *pasynUser);

static asynStatus setInputEos(void *ppvt, asynUser *pasynUser, const char *eos,int eoslen);
static asynStatus getInputEos(void *ppvt, asynUser *pasynUser, char *eos, int eossize, int *eoslen);
static asynStatus setOutputEos(void *ppvt, asynUser *pasynUser, const char *eos,int eoslen);
static asynStatus getOutputEos(void *ppvt, asynUser *pasynUser, char *eos, int eossize, int *eoslen);

static asynOctet octet =
{
	interposeWrite, interposeRead, interposeFlush,
	NULL, NULL,
	setInputEos, getInputEos, setOutputEos, getOutputEos
};

typedef struct interposePvt
{
	char *portName;
	asynInterface octet;
	asynOctet *pasynOctet;
	void *octetPvt;
	asynUser *pasynUser;

	unsigned int numWritten;
	char buffer[FINS_MAX_MSG];
	char fins_header[FINS_HEADER_LEN];
	size_t bufferSize;
	
} interposePvt;

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

static int extractAndCompareChecksum(interposePvt * const pdrvPvt, const size_t pos)
{
#ifdef vxWorks
	unsigned short kcalc, krecv;
#else
	unsigned char kcalc, krecv;
#endif

/* extract the received checksum */

#ifdef vxWorks	
	sscanf(&pdrvPvt->buffer[pos], "%2hx", &krecv);
#else
	sscanf(&pdrvPvt->buffer[pos], "%2hhx", &krecv);
#endif

	pdrvPvt->buffer[pos] = 0;
	kcalc = checksum(pdrvPvt->buffer);
	
	if (kcalc != krecv)
	{
		return (-1);
	}
	
	return (0);
}

/******************************************************************************/

epicsShareFunc int HostlinkInterposeInit(const char *portName)
{
	interposePvt *pPvt;
	asynInterface *poctetasynInterface;
	asynStatus status;
	asynUser *pasynUser;
	int addr = 0;
	
	pPvt = callocMustSucceed(1, sizeof(*pPvt), __func__);
	pPvt->portName = epicsStrDup(portName);
	
	pPvt->octet.interfaceType = asynOctetType;
	pPvt->octet.pinterface = &octet;
	pPvt->octet.drvPvt = pPvt;
	
	pasynUser = pasynManager->createAsynUser(0, 0);
	pPvt->pasynUser = pasynUser;
	pPvt->pasynUser->userPvt = pPvt;

  	status = pasynManager->connectDevice(pasynUser, portName, addr);

  	if (status != asynSuccess)
	{
		printf("%s connectDevice failed\n", portName);
		goto bad;
	}

/* Find the asynOctet interface */

	poctetasynInterface = pasynManager->findInterface(pasynUser, asynOctetType, 1);

 	if (!poctetasynInterface)
	{
		printf("%s findInterface error for asynOctetType %s\n", portName, pasynUser->errorMessage);
		goto bad;
	}
    
/* add our interpose layer */
   
	status = pasynManager->interposeInterface(portName, addr, &pPvt->octet, &poctetasynInterface);

 	if (status != asynSuccess)
	{
		printf("%s interposeInterface failed\n", portName);
		goto bad;
	}
	
	pPvt->pasynOctet = (asynOctet *) poctetasynInterface->pinterface;
	pPvt->octetPvt = poctetasynInterface->drvPvt;

/* specify the terminating character */

	status = pPvt->pasynOctet->setInputEos(pPvt, pasynUser, "\r", 1);
	status = pPvt->pasynOctet->flush(pPvt, pasynUser);

	return (0);

bad:
	pasynManager->freeAsynUser(pasynUser);
	free(pPvt);
	
	return (-1);
}

/**************************************************************************************************/
/*

	80 00 02 00 6c 00 00 fe 00 04 06 01 
	@00FA000000000                06 01 70 *
	
	@00FA0040000000               06 01 00 00 05 02 00 00 00 00 00 00 00 00 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 43 *
	c0 00 02 00 fe 00 00 6c 00 04 06 01 00 00 05 04 00 00 00 00 00 00 00 00 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 
*/

static asynStatus interposeWrite(void *ppvt, asynUser *pasynUser, const char *data, size_t numchars, size_t *nbytesTransfered)
{
	interposePvt * const pPvt = (interposePvt *) ppvt;
	asynStatus status = asynSuccess;
	size_t bytesTransfered;
	int i;
	
	memcpy(pPvt->fins_header, data, FINS_HEADER_LEN);	/* save the fins heaader */
	
	memset(pPvt->buffer, 0, sizeof(pPvt->buffer));		/* clear buffer */
	strcpy(pPvt->buffer, "@00FA000000000");			/* short hostlink/fins header */
	
	for (i = 0; i < numchars - FINS_HEADER_LEN; i++)
	{
		sprintf(pPvt->buffer + HOST_HEADER_LEN + 2 * i, "%02X", data[FINS_HEADER_LEN + i]);
	}
	
/* calculate and add checksum */
	
	sprintf(pPvt->buffer + HOST_HEADER_LEN + 2 * (numchars - FINS_HEADER_LEN), "%02X*\r", checksum(pPvt->buffer));

/* convert FINS to HOSTLINK */

	status = pPvt->pasynOctet->write(pPvt->octetPvt, pasynUser, pPvt->buffer, strlen(pPvt->buffer), &bytesTransfered);

	*nbytesTransfered = numchars;
	return (status);
}

/**************************************************************************************************/

static asynStatus interposeRead(void *ppvt, asynUser *pasynUser, char *data, size_t maxchars, size_t *nbytesTransfered, int *eomReason)
{
	interposePvt *pPvt = (interposePvt *) ppvt;
	asynStatus status = asynSuccess;
	size_t bytesTransfered;
	int i;
	
	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: requesting %d bytes\n", __func__, maxchars);

/* Read the response into pPvt->buffer. We'll received all available characters up to the terminator we specified */

	status = pPvt->pasynOctet->read(pPvt->octetPvt, pasynUser, pPvt->buffer, sizeof(pPvt->buffer), &bytesTransfered, eomReason);

	if (status != asynSuccess)
	{
		return (status);
	}
	
/* if the response is complete then it will end with the checksum plus an '*' */

	if (extractAndCompareChecksum(pPvt, bytesTransfered - 3) < 0)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: port %s, checksum error.\n", __func__,pPvt ->portName);
		return (asynError);
	}

/* replace the header */

	memcpy(data, pPvt->fins_header, FINS_HEADER_LEN);
	data[ICF] |= 0x40;
	
/* swap source and destination addresses */

	{
		unsigned char x;
		
		x = data[DA1]; data[DA1] = data[SA1]; data[SA1] = x;
		x = data[DA2]; data[DA2] = data[SA2]; data[SA2] = x;
		x = data[DNA]; data[DNA] = data[SNA]; data[SNA] = x;
	}
	
/* convert ASCII to binary */

	for (i = 0; i < bytesTransfered - HOST_HEADER_LEN_RESP - 2; i++)
	{
		unsigned short aa;
		
		sscanf(pPvt->buffer + HOST_HEADER_LEN_RESP + 2 * i, "%02hx", &aa);
		data[FINS_HEADER_LEN + i] = aa;
	}

	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pPvt->buffer, *nbytesTransfered, "%s: received %d bytes of %d.\n", __func__, bytesTransfered, maxchars);

	*nbytesTransfered	= maxchars;
	
	return (asynSuccess);
}
/**************************************************************************************************/

static asynStatus interposeFlush(void *ppvt, asynUser *pasynUser)
{
	interposePvt *pPvt = (interposePvt *) ppvt;
	asynStatus status = asynSuccess;

	status = pPvt->pasynOctet->flush(pPvt->octetPvt, pasynUser);
	
	return (status);
}

/**************************************************************************************************/

static asynStatus setInputEos(void *ppvt, asynUser *pasynUser, const char *eos, int eoslen)
{
	interposePvt *pPvt = (interposePvt *) ppvt;
	return pPvt->pasynOctet->setInputEos(pPvt->octetPvt, pasynUser, eos, eoslen);
}

/**************************************************************************************************/

static asynStatus getInputEos(void *ppvt, asynUser *pasynUser, char *eos, int eossize, int *eoslen)
{
	interposePvt *pPvt = (interposePvt *) ppvt;
	return pPvt->pasynOctet->getInputEos(pPvt->octetPvt, pasynUser, eos, eossize, eoslen);
}

/**************************************************************************************************/

static asynStatus setOutputEos(void *ppvt, asynUser *pasynUser, const char *eos, int eoslen)
{
	interposePvt *pPvt = ( interposePvt*) ppvt;
	return pPvt->pasynOctet->setOutputEos(pPvt->octetPvt, pasynUser, eos, eoslen);
}

/**************************************************************************************************/

static asynStatus getOutputEos(void *ppvt, asynUser *pasynUser, char *eos, int eossize, int *eoslen)
{
	interposePvt *pPvt = (interposePvt *) ppvt;
	return pPvt->pasynOctet->getOutputEos(pPvt->octetPvt, pasynUser, eos, eossize, eoslen);
}

/**************************************************************************************************/
/* register interposeInterfaceInit */

static const iocshArg interposeInterfaceInitArg0 = { "portName", iocshArgString };
static const iocshArg *interposeInterfaceInitArgs[] = { &interposeInterfaceInitArg0};
static const iocshFuncDef interposeInterfaceInitFuncDef = {"HostlinkInterposeInit", 1, interposeInterfaceInitArgs};

static void interposeInterfaceInitCallFunc(const iocshArgBuf *args)
{
	HostlinkInterposeInit(args[0].sval);
}

static void HostlinkInterposeRegister(void)
{
	static int firstTime = 1;

  	if (firstTime)
	{
		firstTime = 0;
		iocshRegister(&interposeInterfaceInitFuncDef, interposeInterfaceInitCallFunc);
	}
}

epicsExportRegistrar(HostlinkInterposeRegister);
