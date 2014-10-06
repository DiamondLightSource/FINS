/* PLC memory  types */

#define DM	0x82
#define IO	0xB0
#define WR	0xB1
#define HR	0xB2
#define AR	0xB3
#define CT	0x89	/* counter */
#define TM	0x89	/* timer */
#define CF	0x09	/* counter completion flags */
#define TF	0x09	/* time completion flags */
#define CP	0x07	/* clock pulses */

/* offsets into the FINS UDP packet */

#define ICF		0
#define RSV		1
#define GCT		2
#define DNA		3
#define DA1		4
#define DA2		5
#define SNA		6
#define SA1		7
#define SA2		8
#define SID		9
#define MRC		10
#define SRC		11
#define COM		12
#define MRES	12
#define SRES	13
#define RESP	14

#define COMMAND_DATA_OFFSET	6

#define MIN_RESP_LEN	14

#define	FINS_HEADER_LEN		10
#define	HOST_HEADER_LEN		14
#define	HOST_HEADER_LEN_RESP	(HOST_HEADER_LEN + 1)

/* constants */

#define FINS_NET_PORT		9600					/* default PLC FINS port */
#define FINS_MAX_UDP_WORDS	950
#define FINS_MAX_TCP_WORDS	FINS_MAX_UDP_WORDS
#define FINS_MAX_HOST_WORDS	268
#define FINS_MAX_MSG		((FINS_MAX_UDP_WORDS) * 2 + 100)
#define FINS_TIMEOUT		1					/* asyn default timeout */
#define FINS_SOURCE_ADDR	(0xFE)				/* default node address 254 */
#define FINS_GATEWAY		0x02

#define FINS_MODEL_LEN		20
#define FINS_CYCLE_TIME_LEN	3
#define FINS_CLOCK_READ_LEN	7
#define FINS_CPU_STATE_LEN	26

#define FINS_MM_MAX_ADDRS	10

#define ONE_ELEMENT	(1)


/*
		IOC			PLC
		00 01 02 03		00 01 02 03
		-----------		-----------
	BE	11 22 33 44		33 44 11 22
	LE	44 33 22 11		33 44 11 22

*/

#define BESWAP32(a)	(((a) & 0x0000ffff) << 16) | (((a) & 0xffff0000) >> 16)
#define LESWAP32(a)	(((a) & 0x00ff00ff) <<  8) | (((a) & 0xff00ff00) >>  8)

#if (EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE)

	#define BSWAP16(a)	(((a) & 0x00ff) << 8) | (((a) & 0xff00) >> 8)
	#define BSWAP32(a)	(((a) & 0x000000ff) << 24) | (((a) & 0x0000ff00) << 8) | (((a) & 0x00ff0000) >> 8) | (((a) & 0xff000000) >> 24)
	#define SWAPT		"swapping"

	#define WSWAP32 LESWAP32

#else

	#define BSWAP16(a)	(a)
	#define BSWAP32(a)	(a)
	#define SWAPT		"copying"

	#define WSWAP32 BESWAP32
	
#endif

enum { FINS_UDP_type, FINS_TCP_type, HOSTLINK_type };

static const char * const asynStatusMessages[] = { "asynSuccess", "asynTimeout", "asynOverflow", "asynError", "asynDisconnected", "asynDisabled"};
static const char * const asynEomMessages[] = { "Request count reached", "End of String detected", "End indicator detected"};

enum FINS_COMMANDS
{
	FINS_NULL,
	FINS_DM_READ, FINS_DM_WRITE, FINS_DM_WRITE_NOREAD,
	FINS_IO_READ, FINS_IO_WRITE, FINS_IO_WRITE_NOREAD,
	FINS_AR_READ, FINS_AR_WRITE, FINS_AR_WRITE_NOREAD,
	FINS_CT_READ, FINS_CT_WRITE, FINS_CT_WRITE_NOREAD,
	FINS_WR_READ,
	FINS_HR_READ,
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
	FINS_SET_RESET_CANCEL,
	FINS_MM_READ,
	FINS_EXPLICIT
};

static const char * const FINS_names[] = {
	"FINS_NULL",
	"FINS_DM_READ", "FINS_DM_WRITE", "FINS_DM_WRITE_NOREAD",
	"FINS_IO_READ", "FINS_IO_WRITE", "FINS_IO_WRITE_NOREAD",
	"FINS_AR_READ", "FINS_AR_WRITE", "FINS_AR_WRITE_NOREAD",
	"FINS_CT_READ", "FINS_CT_WRITE", "FINS_CT_WRITE_NOREAD",
	"FINS_WR_READ",
	"FINS_HR_READ",
	"FINS_DM_READ_32", "FINS_DM_WRITE_32", "FINS_DM_WRITE_32_NOREAD",
	"FINS_IO_READ_32", "FINS_IO_WRITE_32", "FINS_IO_WRITE_32_NOREAD",
	"FINS_AR_READ_32", "FINS_AR_WRITE_32", "FINS_AR_WRITE_32_NOREAD",
	"FINS_CT_READ_32", "FINS_CT_WRITE_32", "FINS_CT_WRITE_32_NOREAD",
	"FINS_READ_MULTI",
	"FINS_WRITE_MULTI",
	"FINS_SET_MULTI_TYPE",
	"FINS_SET_MULTI_ADDR",
	"FINS_CLR_MULTI",
	"FINS_MODEL",
	"FINS_CPU_STATUS",
	"FINS_CPU_MODE",
	"FINS_CYCLE_TIME_RESET",
	"FINS_CYCLE_TIME",
	"FINS_CYCLE_TIME_MEAN",
	"FINS_CYCLE_TIME_MAX",
	"FINS_CYCLE_TIME_MIN",
	"FINS_MONITOR",
	"FINS_CLOCK_READ",
	"FINS_SET_RESET_CANCEL",
	"FINS_MM_READ",
	"FINS_EXPLICIT"
};

/* from asyn/drvAsynSerial/drvAsynIPPort.c */

typedef struct
{
	asynUser *pasynUser;
	char *IPDeviceName;
	char *IPHostName;
	char *portName;
	int socketType;
	int flags;
	SOCKET fd;
	unsigned long nRead;
	unsigned long nWritten;
	int haveAddress;
	osiSockAddr farAddr;
	asynInterface common;
	asynInterface octet;
	
} ttyController_t;

typedef struct MultiMemArea
{
	ELLNODE node;
	
	epicsUInt16 area[10];
	epicsUInt16 address[10];
	
} MultiMemArea;

typedef struct MultiMemAreaPair
{
	epicsUInt8 area;
	epicsUInt16 address __attribute__ ((packed));
	
} MultiMemAreaPair;

typedef struct drvPvt
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

} drvPvt;

/* FINS TCP */

#define FINS_TCP_HEADER		0x46494E53
#define FINS_MODE_HEADER	0
#define FINS_MODE_LENGTH	1
#define FINS_MODE_COMMAND	2
#define FINS_MODE_ERROR		3
#define FINS_MODE_CLIENT	4
#define FINS_MODE_SERVER	5

#define FINS_SEND_FRAME_SIZE	16
#define FINS_MODE_SEND_SIZE	20
#define FINS_MODE_RECV_SIZE	24

#define FINS_NODE_CLIENT_COMMAND	0
#define FINS_NODE_SERVER_COMMAND	1
#define FINS_FRAME_SEND_COMMAND	2
#define FINS_FRAME_SEND_ERROR		3

#define FINS_ERROR_NORMAL		0x00
#define FINS_ERROR_HEADER		0x01
#define FINS_ERROR_TOO_LONG		0x02
#define FINS_ERROR_NOT_SUPPORTED	0x03
#define FINS_ERROR_ALL_USED		0x20
#define FINS_ERROR_CONNECTED		0x21
#define FINS_ERROR_PROTECTED		0x22
#define FINS_ERROR_RANGE		0x23
#define FINS_ERROR_DUPLICATE		0x24
#define FINS_ERROR_FULL			0x25
