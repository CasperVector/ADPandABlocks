TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#USR_CXXFLAGS += -Werror
USR_CXXFLAGS += -Wall -Wextra
USR_CXXFLAGS += -Wno-unused-parameter
USR_CXXFLAGS += -std=c++11
USR_INCLUDES += $(XML2_INCLUDE)

#======== DETECTOR LIBRARY ==============

# The following gets rid of the -fno-implicit-templates flag on vxWorks, 
# so we get automatic template instantiation.
# This is what we want for miscellaneous/asynPortDriver.cpp
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
CODE_CXXFLAGS=
endif

INC += ADPandABlocks.h

LIBRARY_IOC = ADPandABlocks
LIB_SRCS += ADPandABlocks.cpp

DBD += ADPandABlocksSupport.dbd
#DBD += ADPandABlocksMotorSync.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

