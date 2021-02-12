TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CXXFLAGS += -Werror
USR_CXXFLAGS += -Wall -Wextra -Wno-unused-parameter -std=c++0x -pthread

USR_INCLUDES += $(XML2_INCLUDE)

PROD_IOC += tests
tests_SRCS += tests.cpp
tests_SRCS += test_ADPandABlocks.cpp
tests_SRCS += testingutilities.cpp
tests_LIBS += $(EPICS_BASE_IOC_LIBS)
PROD_LIBS += ADBase asyn ADPandABlocks


ifdef BOOST_LIB
	boost_unit_test_framework_DIR=$(BOOST_LIB)
	tests_LIBS += boost_unit_test_framework
else
	tests_SYS_LIBS += boost_unit_test_framework
endif

include $(TOP)/configure/RULES
