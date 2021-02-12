#!../../bin/linux-x86_64/panda

< envPaths

cd "${TOP}"

## Change this to point to your PandA device
epicsEnvSet("PANDA_ADDRESS","192.168.1.11")

## Register all support components
dbLoadDatabase "dbd/panda.dbd"
panda_registerRecordDeviceDriver pdbbase

## Create asyn ports connected to PandA device
drvAsynIPPortConfigure("PANDA1.DRV_CTRL", "${PANDA_ADDRESS}:8888", 100, 0, 0)
drvAsynIPPortConfigure("PANDA1.DRV_DATA", "${PANDA_ADDRESS}:8889", 100, 0, 0)

## Configure ADPandABlocks areaDetector plugin to control acquisition from the PandA
ADPandABlocksConfig("PANDA1.DRV", "${PANDA_ADDRESS}", 100000, 1000, 0, 0)

## Configure NDAttrPlot areaDetector plugin 
NDAttrPlotConfig("PANDA1.PLT", 198, 10000, 9, "PANDA1.DRV", 0, 10000, 0)

## Configure NDPos areaDetector plugin 
NDPosPluginConfigure("PANDA1.POS", 10000, 0, "PANDA1.DRV", 0, 0, 0, 0, 0)

## Configure HDF5 areaDetector plugin 
NDFileHDF5Configure("PANDA1.HDF", 10000, 0, "PANDA1.POS", 0)

## Load record instances
dbLoadRecords "db/panda.db"

cd "${TOP}/iocBoot/${IOC}"
iocInit
