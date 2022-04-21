< envPaths
errlogInit(20000)

dbLoadDatabase "$(TOP)/dbd/pandaApp.dbd"
pandaApp_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("PREFIX", "PANDA1:")
epicsEnvSet("PANDAADDR", "172.23.252.202")
epicsEnvSet("QSIZE", "128")
epicsEnvSet("PORT", "PANDA1")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(ADPANDABLOCKS)/db")

drvAsynIPPortConfigure("$(PORT)_CTRL", "$(PANDAADDR):8888")
drvAsynIPPortConfigure("$(PORT)_DATA", "$(PANDAADDR):8889")
ADPandABlocksConfig("$(PORT)", "$(PANDAADDR)", 0, 0)
dbLoadRecords("ADPandABlocks.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
NDFileHDF5Configure("FileHDF1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("NDFileHDF5.template", "P=$(PREFIX),R=HDF1:,PORT=FileHDF1,ADDR=0,TIMEOUT=1,XMLSIZE=2048,NDARRAY_PORT=$(PORT)")

cd "$(TOP)/iocBoot/$(IOC)"
iocInit()

