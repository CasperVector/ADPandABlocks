TOP = ..
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *src*))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *Src*))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *db*))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *Db*))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *opi*))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard protocol))

ifdef BOOST
  DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard tests))
  tests_DEPEND_DIRS += src
endif

include $(TOP)/configure/RULES_DIRS

