TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += FINS

DBD += FINS.dbd

# To build FINSSim uncomment following line
#DBD += FINSSim.dbd

FINS_SRCS += FINS.c
FINS_SRCS += interpose.c

# To build FINSSim uncomment following line
#FINS_SRCS += FINSSim.c

FINS_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

include $(TOP)/configure/RULES
