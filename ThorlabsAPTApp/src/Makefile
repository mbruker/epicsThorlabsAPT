TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += ThorlabsAPT

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install ThorlabsAPT.dbd into <top>/dbd
DBD += ThorlabsAPT.dbd

# specify all source files to be compiled and added to the library
ThorlabsAPT_SRCS += thorlabs_apt_driver.cpp

ThorlabsAPT_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

