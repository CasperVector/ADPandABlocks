// BK: const modifiers on input parameters and methods are missing
// BK: Prefer const references in params instead of pointers where the passed 
//     objects are not changed, references if the memory position of the object
//     is not changed in method (the reference cannot be null), pointers are 
//     usually reserved for output parameters

#include "ADPandABlocks.h"

#include <stdint.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <stdlib.h>

#include <libxml/xmlreader.h>

#include <epicsEndian.h>
#include "epicsThread.h"

//#include <unistd.h>
#include <cstdlib>


static void pollDataPortC(void *userPvt) {
	ADPandABlocks *pPvt = (ADPandABlocks *) userPvt;
	pPvt->readDataPort();
}

static void pollCommandPortC(void *userPvt) {
	ADPandABlocks *pPvt = (ADPandABlocks *) userPvt;
	pPvt->pollCommandPort();
}

static void callbackC(asynUser *pasynUser, asynException exception){
	ADPandABlocks *pPvt = (ADPandABlocks *) pasynUser->drvUser;
	pPvt->exceptionCallback(pasynUser, exception);
}

void ADPandABlocks::exceptionCallback(asynUser *pasynUser, asynException exception){
	int port_connected = 0;
	pandaResponsive = false;
	setIntegerParam(ADPandABlocksIsResponsive, 0);
	pasynManager->isConnected(pasynUser, &port_connected);
	if(port_connected) {
		// It seems that the PandA doesn't successfuly receive/respond to messages
		// straight away after the AsynPort reconnects; keep trying until it does
		while (!pandaResponsive) {
			sendReceivingFormat();
			epicsThreadSleep(5.0);
		}
	}
}

typedef int static_assert_endianness[EPICS_BYTE_ORDER != EPICS_ENDIAN_BIG ? 1 : -1];

static const char *driverName = "ADPandABlocks";
static std::map<asynStatus, std::string> errorMsg;

ADPandABlocks::ADPandABlocks(const char* portName, const char* pandaAddress, int maxPts, int maxBuffers, int maxMemory) :
		ADDriver(portName, 1 /*maxAddr*/, NUM_PARAMS, maxBuffers, maxMemory,
				 asynInt8ArrayMask | asynFloat64ArrayMask | asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
				 asynInt8ArrayMask | asynFloat64ArrayMask | asynInt32Mask | asynFloat64Mask | asynOctetMask,
				 ASYN_CANBLOCK, /*ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0 */
				 1, /*autoConnect*/ 0, /*default priority */ 0 /*default stack size*/) {

	//private variables
	state = waitHeaderStart; //init state for the data read
	pandaResponsive = false;
	errorMsg[asynSuccess] = "asynSuccess";
	errorMsg[asynTimeout] = "asynTimeout";
	errorMsg[asynOverflow] = "asynOverflow";
	errorMsg[asynError] = "asynError";
	errorMsg[asynDisconnected] = "asynDisconnected";
	errorMsg[asynDisabled] = "asynDisabled";

	captureType["No"] = 0;
	captureType["Value"] = 1;
	captureType["Diff"] = 2;
	captureType["Sum"] = 3;
	captureType["Mean"] = 4;
	captureType["Min"] = 5;
	captureType["Max"] = 6;
	captureType["Min Max"] = 7;
	captureType["Min Max Mean"] = 8;
	captureStrings.push_back("No");
	captureStrings.push_back("Value");
	captureStrings.push_back("Diff");
	captureStrings.push_back("Sum");
	captureStrings.push_back("Mean");
	captureStrings.push_back("Min");
	captureStrings.push_back("Max");
	captureStrings.push_back("Min Max");
	captureStrings.push_back("Min Max Mean");

	const char *functionName = "ADPandABlocks";
	asynStatus status = asynSuccess;
	asynInterface *pasynInterface;

	/* For areaDetector image */
	pArray = NULL;
	arrayCounter = 0;
	numImagesCounter = 0;
	imgMode = ADImageContinuous;
	imgNo = 0;

	/* Connection status */
	createParam("ISCONNECTED", asynParamInt32, &ADPandABlocksIsConnected);
	setIntegerParam(ADPandABlocksIsConnected, 0);
	createParam("ISRESPONSIVE", asynParamInt32, &ADPandABlocksIsResponsive);
	setIntegerParam(ADPandABlocksIsResponsive, 0);

	/*Create a parameter to store the header value /*/
	createParam("HEADER", asynParamOctet, &ADPandABlocksHeader);

	/*Create a parameter to store the end of data string */
	createParam("DATAEND", asynParamOctet, &ADPandABlocksDataEnd);
	setStringParam(ADPandABlocksDataEnd, "");

	/* initialise areaDetector parameters */
	setStringParam(ADManufacturer, "Diamond Light Source Ltd.");
	setStringParam(ADModel, "ADPandABlocks");
	setIntegerParam(ADMaxSizeX, NARRAYS + 1);
	setIntegerParam(ADMaxSizeY, FRAMEHEIGHT);
	//    setIntegerParam(NDDataType, 7);
	setIntegerParam(ADStatus, ADStatusIdle);
	setStringParam(ADStatusMessage, "Idle");

	/* Connect to the device port */
	ctrlPort = std::string(portName).append("_CTRL").c_str();
	//drvAsynIPPortConfigure(ctrlPort, std::string(pandaAddress).append(":").append(CTRL_PORT).c_str(), 100, 0, 0);	
	/* Copied from asynOctecSyncIO->connect */
	pasynUser_ctrl = pasynManager->createAsynUser(0, 0);
	status = pasynManager->connectDevice(pasynUser_ctrl, ctrlPort, 0);
	if (status != asynSuccess) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, ctrlPort, status);
		return;
	}
	pasynInterface = pasynManager->findInterface(pasynUser_ctrl, asynCommonType, 1);
	if (!pasynInterface) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
		return;
	}
	pasynInterface = pasynManager->findInterface(pasynUser_ctrl, asynOctetType, 1);
	if (!pasynInterface) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: %s interface not supported", driverName, functionName, asynOctetType);
		return;
	}

	pasynOctet_ctrl = (asynOctet *) pasynInterface->pinterface;
	octetPvt_ctrl = pasynInterface->drvPvt;
	asynSetOption(ctrlPort, 0, "disconnectOnReadTimeout", "Y");
	pasynUser_ctrl->drvUser = (void *) this;
	pasynManager->exceptionCallbackAdd(pasynUser_ctrl, callbackC);

	/* Set EOS and flush */
	pasynOctet_ctrl->flush(octetPvt_ctrl, pasynUser_ctrl);
	pasynOctet_ctrl->setInputEos(octetPvt_ctrl, pasynUser_ctrl, "\n", 1);
	pasynOctet_ctrl->setOutputEos(octetPvt_ctrl, pasynUser_ctrl, "\n", 1);

	/* Connect to the data port */
	dataPort = std::string(portName).append("_DATA").c_str();
	//drvAsynIPPortConfigure(dataPort, std::string(pandaAddress).append(":").append(DATA_PORT).c_str(), 100, 0, 0);
	/* Copied from asynOctecSyncIO->connect */
	pasynUser_data = pasynManager->createAsynUser(0, 0);
	status = pasynManager->connectDevice(pasynUser_data, dataPort, 0);
	if (status != asynSuccess) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, dataPort, status);
		return;
	}
	pasynInterface = pasynManager->findInterface(pasynUser_data, asynCommonType, 1);
	if (!pasynInterface) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
		return;
	}
	pasynCommon_data = (asynCommon *) pasynInterface->pinterface;
	pcommonPvt_data = pasynInterface->drvPvt;
	pasynInterface = pasynManager->findInterface(pasynUser_data, asynOctetType, 1);
	if (!pasynInterface) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: %s interface not supported", driverName, functionName, asynOctetType);
		return;
	}
	pasynOctet_data = (asynOctet *) pasynInterface->pinterface;
	octetPvt_data = pasynInterface->drvPvt;

	/* Set EOS and flush */
	pasynOctet_data->flush(octetPvt_data, pasynUser_data);
	pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "\n", 1);
	pasynOctet_data->setOutputEos(octetPvt_data, pasynUser_data, "\n", 1);
	pasynUser_data->timeout = LONGWAIT;

	/*set the receiving format on the data channel*/
	sendReceivingFormat();
	/*Get the labels for the bitmask*/
	int numBitMasks = 0;
	for(int n=0; n<4; n++)
	{
		std::stringstream bitmaskCmd;
		bitmaskCmd << "PCAP.BITS"<< n << ".BITS?";
		sendCtrl(bitmaskCmd.str());
		bitMasks.push_back(readFieldNames(&numBitMasks));
	}

	/*Get the POSITION fields*/
	std::stringstream fieldCmd;
	fieldCmd << "*POSITIONS?";
	sendCtrl(fieldCmd.str());
	int numPosFields = 0;
	posFields.push_back(readFieldNames(&numPosFields));
	while (numPosFields < 32)
	{
		std::stringstream fieldNum;
		fieldNum << "POSBUS" << numPosFields+1;
		posFields[0].push_back(fieldNum.str());
		numPosFields++;
	}

	/*Make params for each of the POSITION fields*/
	char str[NBUFF];
	for(int a = 0; a < 32; a++)
	{
		if(a == 0 || a < numPosFields){
			epicsSnprintf(str, NBUFF, "POSBUS%d", a);
			createParam(str, asynParamOctet, &ADPandABlocksPosFields[a]);
			setStringParam(ADPandABlocksPosFields[a], posFields[0][a].c_str());
		}
		else{
			epicsSnprintf(str, NBUFF, "POSBUS%d", a);
			createParam(str, asynParamOctet, &ADPandABlocksPosFields[a]);
			setStringParam(ADPandABlocksPosFields[a], "UNUSED");
		}
	}

	/*Make the params for the Motor fields*/
	for(int a = 0; a < 4; a++)
	{
		epicsSnprintf(str, NBUFF, "INENC%d:SCALE", a+1);
		createParam(str, asynParamFloat64, &ADPandABlocksMScale[a]);
		epicsSnprintf(str, NBUFF, "INENC%d:OFF", a+1);
		createParam(str, asynParamFloat64, &ADPandABlocksMOffset[a]);
		epicsSnprintf(str, NBUFF, "INENC%d:UNITS", a+1);
		createParam(str, asynParamOctet, &ADPandABlocksMUnits[a]);
		epicsSnprintf(str, NBUFF, "INENC%d:SETPOS", a+1);
		createParam(str, asynParamInt32, &ADPandABlocksMSetpos[a]);
		epicsSnprintf(str, NBUFF, "INENC%d:SCREENTYPE", a+1);
		createParam(str, asynParamInt32, &ADPandABlocksMScreenType[a]);
		epicsSnprintf(str, NBUFF, "INENC%d:CALIBRATE", a+1);
		createParam(str, asynParamInt32, &ADPandABlocksMCalibrate[a]);
		epicsSnprintf(str, NBUFF, "INENC%d:MOTORNAME", a+1);
		createParam(str, asynParamOctet, &ADPandABlocksMMotorName[a]);
	}

	// Create the lookup table parameters for the position bus
	int posBusInd = 0;
	for(std::vector<std::string>::iterator it = posFields[0].begin(); it != posFields[0].end(); ++it)
	{
		createLookup(*it, "VAL", &ADPandABlocksPosVals[posBusInd], posBusInd);
		createLookup(*it, "SCALE", &ADPandABlocksScale[posBusInd], posBusInd);
		createLookup(*it, "OFFSET", &ADPandABlocksOffset[posBusInd], posBusInd);
		createLookup(*it, "UNITS", &ADPandABlocksUnits[posBusInd], posBusInd);
		createLookup(*it, "CAPTURE", &ADPandABlocksCapture[posBusInd], posBusInd);
		createLookup(*it, "UNSCALEDVAL", &ADPandABlocksPosUnscaledVals[posBusInd], posBusInd);
		createLookup(*it, "SCREENTYPE", &ADPandABlocksScreenType[posBusInd], posBusInd);
		createLookup(*it, "CALIBRATE", &ADPandABlocksCalibrate[posBusInd], posBusInd);
		createLookup(*it, "SETPOS", &ADPandABlocksSetpos[posBusInd], posBusInd);
		createLookup(*it, "MOTORNAME", &ADPandABlocksMotorName[posBusInd], posBusInd);
		posBusInd++;
	}

	// Initialise position bus values
	processChanges("*CHANGES.ATTR?", false);
	processChanges("*CHANGES.POSN?", true);

	/* Create thread to monitor command port for position bus changes */
	if (epicsThreadCreate("ADPandABlocksPollCommandPort", epicsThreadPriorityMedium,
						  epicsThreadGetStackSize(epicsThreadStackMedium),
						  (EPICSTHREADFUNC) pollCommandPortC, this) == NULL) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
		return;
	}

	/* Create thread to monitor data port  */
	if (epicsThreadCreate("ADPandABlocksPollDataPort", epicsThreadPriorityMedium,
						  epicsThreadGetStackSize(epicsThreadStackMedium),
						  (EPICSTHREADFUNC) pollDataPortC, this) == NULL) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
		return;
	}
};

/*
ADPandABlocks::~ADPandABlocks () {
	pandaResponsive = false;
}
*/

/*
 * Update PandA parameter using lookup table
 * \param[in] name Bus name
 * \param[in] field Bus field
 * \param[in] value New value
 */
template<typename T>
void ADPandABlocks::updatePandAParam(std::string name, std::string field, T value)
{
}
template<>
void ADPandABlocks::updatePandAParam<double>(std::string name, std::string field, double value)
{
	if(posBusLookup.find(name) != posBusLookup.end())
	{
		if(posBusLookup[name].find(field) != posBusLookup[name].end())
		{
			setDoubleParam(*posBusLookup[name][field], value);
			if (field == "SCALE" || field == "OFFSET")
			{
				updateScaledPositionValue(name);
			}
		}
	}
}
template<>
void ADPandABlocks::updatePandAParam<std::string>(std::string name, std::string field, std::string value)
{
	if(posBusLookup.find(name) != posBusLookup.end())
	{
		if(posBusLookup[name].find(field) != posBusLookup[name].end())
		{
			setStringParam(*posBusLookup[name][field], value);
		}
	}
}
template<>
void ADPandABlocks::updatePandAParam<int>(std::string name, std::string field, int value)
{
	if(posBusLookup.find(name) != posBusLookup.end())
	{
		if(posBusLookup[name].find(field) != posBusLookup[name].end())
		{
			if (field == "VAL")
			{
				setIntegerParam(*posBusLookup[name]["UNSCALEDVAL"], value);
				updateScaledPositionValue(name);
			}
			else setIntegerParam(*posBusLookup[name][field], value);
		}
	}
}
template<>
void ADPandABlocks::updatePandAParam<ADPandABlocks::embeddedScreenType>(std::string name, std::string field, ADPandABlocks::embeddedScreenType value)
{
	if(posBusLookup.find(name) != posBusLookup.end())
	{
		if(posBusLookup[name].find(field) != posBusLookup[name].end())
		{
			setIntegerParam(*posBusLookup[name]["SCREENTYPE"], static_cast<int>(value));
		}
	}
}

/*
 * Update motor field params
 *   \param[in] motorIndex Index of motor whose field has changed (1..4)
 *   \param[in] motorField Parameter to be updated
 *   \param[in] value New value of motor parameter
 */
template<typename T>
void ADPandABlocks::updatePandAMotorParam(int motorIndex, motorField field, T value)
{
}
template<>
void ADPandABlocks::updatePandAMotorParam<int>(int motorIndex, motorField field, int value)
{
	std::stringstream posBusName, posBusField;
	posBusName << "INENC" << motorIndex << ".VAL";
	switch(field)
	{
		case setpos: {
			// Send command to PandA to calibrate motor position
			posBusField << "SETPOS";
			setPandASetPos(posBusName.str(), value);
			break;
		}
	}
	updatePandAParam(posBusName.str(), posBusField.str(), value);
}
template<>
void ADPandABlocks::updatePandAMotorParam<ADPandABlocks::embeddedScreenType>(int motorIndex, motorField field, ADPandABlocks::embeddedScreenType value)
{
	std::stringstream posBusName, posBusField;
	posBusName << "INENC" << motorIndex << ".VAL";
	switch(field)
	{
		case screen: {
			// Change screen type to use readOnly
			posBusField << "SCREENTYPE";
			break;
		}
	}
	updatePandAParam(posBusName.str(), posBusField.str(), value);
}
template<>
void ADPandABlocks::updatePandAMotorParam<float>(int motorIndex, motorField field, float value)
{
	std::stringstream posBusName, posBusField;
	posBusName << "INENC" << motorIndex << ".VAL";
	switch(field)
	{
		case offset: {
			posBusField << "OFFSET";
			break;
		}
		case scale: {
			posBusField << "SCALE";
			break;
		}
	}
	updatePandAParam(posBusName.str(), posBusField.str(), value);
}
template<>
void ADPandABlocks::updatePandAMotorParam<std::string>(int motorIndex, motorField field, std::string value)
{
	std::stringstream posBusName, posBusField;
	posBusName << "INENC" << motorIndex << ".VAL";
	switch(field)
	{
		case units: {
			posBusField << "UNITS";
			break;
		}
		case motorName: {
			posBusField << "MOTORNAME";
			break;
		}
	}
	updatePandAParam(posBusName.str(), posBusField.str(), value);
}

std::string ADPandABlocks::getPosBusField(std::string posbus, const char* paramName){
	std::string field;
	std::stringstream cmdStr;
	cmdStr << posbus << "." << paramName << "?";
	sendCtrl(cmdStr.str());
	readPosBusValues(&field);
	return field;
}

void ADPandABlocks::createPosBusParam(const char* paramName, asynParamType paramType, int* paramIndex, int paramNo){
	char str[NBUFF];
	epicsSnprintf(str, NBUFF, "POSBUS%d:%s", paramNo, paramName);
	createParam(str, paramType, paramIndex);
}

bool ADPandABlocks::posBusInUse(std::string posBusName) {
	// Unused position buses will have the name POSBUS<N>
	std::size_t found = posBusName.find("POSBUS");
	if (found == std::string::npos) return true;
	else return false;
}

// TODO: consider renaming paramName to position bus name
void ADPandABlocks::createLookup(std::string paramName, std::string paramNameEnd, int* paramInd, int posBusInd)
{
	std::map<std::string, int*> lpMap2;
	if(paramNameEnd == "CAPTURE" || paramNameEnd == "UNSCALEDVAL" || paramNameEnd == "SCREENTYPE" || paramNameEnd == "CALIBRATE" || paramNameEnd == "SETPOS"){
		createPosBusParam(paramNameEnd.c_str(), asynParamInt32, paramInd, posBusInd);
		posBusLookup.insert(std::pair<std::string, std::map<std::string, int*> >(paramName, lpMap2));
		posBusLookup[paramName].insert(std::pair<std::string, int*>(paramNameEnd, paramInd));
		// Set screen type based on whether PosBus is in use
		if (paramNameEnd == "SCREENTYPE")
		{
			if (posBusInUse(paramName)){
				setIntegerParam(*posBusLookup[paramName][paramNameEnd], writeable);
			}
			else setIntegerParam(*posBusLookup[paramName][paramNameEnd], empty);
		}
	}
	else if(paramNameEnd == "UNITS" || paramNameEnd == "MOTORNAME")
	{
		createPosBusParam(paramNameEnd.c_str(), asynParamOctet, paramInd, posBusInd);
		posBusLookup.insert(std::pair<std::string, std::map<std::string, int*> >(paramName, lpMap2));
		posBusLookup[paramName].insert(std::pair<std::string, int*>(paramNameEnd, paramInd));
	}
		// VAL, SCALE and OFFSET
	else
	{
		createPosBusParam(paramNameEnd.c_str(), asynParamFloat64, paramInd, posBusInd);
		posBusLookup.insert(std::pair<std::string, std::map<std::string, int*> >(paramName, lpMap2));
		posBusLookup[paramName].insert(std::pair<std::string, int*>(paramNameEnd, paramInd));
	}
}

/* This is the function that will be run for the read thread */
std::vector<std::string> ADPandABlocks::readFieldNames(int* numFields) {
	const char *functionName = "readFieldNames";
	char rxBuffer[N_BUFF_CTRL];
	size_t nBytesIn;
	int eomReason;
	asynStatus status = asynSuccess;
	asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_ctrl, 0, 0);
	std::vector<std::string> fieldNameStrings;
	pasynUserRead->timeout = 3.0;
	status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, N_BUFF_CTRL - 1,
								   &nBytesIn, &eomReason);
	int i = 0;
	while(rxBuffer[0] != '.')
	{
		if (strlen(rxBuffer) == 0) break;
		i++;
		if (eomReason & ASYN_EOM_EOS) {
			// Replace the terminator with a null so we can use it as a string
			rxBuffer[nBytesIn] = '\0';
			asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
					  "%s:%s: Message: '%s'\n", driverName, functionName, rxBuffer);
		} else {
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
					  "%s:%s: Bad message 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
		}
		if(rxBuffer[0] == '!')
		{
			char* strippedLabel = rxBuffer + 1;
			fieldNameStrings.push_back(strippedLabel);
		}
		// Push the whole bitmask for 'n' to the vector of vectors
		status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, N_BUFF_CTRL - 1,
									   &nBytesIn, &eomReason);
	}
	*numFields = i;
	return fieldNameStrings;
}

/* This is the function that will be run for the read thread */
asynStatus ADPandABlocks::readPosBusValues(std::string* posBusValue) {
	const char *functionName = "readPosBusValues";
	char rxBuffer[N_BUFF_CTRL];
	size_t nBytesIn;
	int eomReason;
	asynStatus status = asynSuccess;
	asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_ctrl, 0, 0);
	pasynUserRead->timeout = 3.0;
	status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, N_BUFF_CTRL - 1,
								   &nBytesIn, &eomReason);
	if (eomReason & ASYN_EOM_EOS) {
		// Replace the terminator with a null so we can use it as a string
		rxBuffer[nBytesIn] = '\0';
		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
				  "%s:%s: Message: '%s'\n", driverName, functionName, rxBuffer);
	} else {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: Bad message 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
	}

	if(rxBuffer[0] == 'O' && rxBuffer[1] == 'K')
	{
		char* readValue = rxBuffer + 4;
		posBusValue->assign(readValue);
	} else {
		//we didn't recieve OK so something went wrong
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s Bad response: %d, %s\n", driverName, functionName, (int)nBytesIn, rxBuffer);
		status = asynError;
	}
	return status;
}

void ADPandABlocks::pollCommandPort(){
	/*  This will check if anything has changed on the panda and update the
        Readback values */
	while(true)
	{
		epicsTimeGetCurrent(&pollStartTime);
		this->lock();
		processChanges("*CHANGES.ATTR?", false);
		processChanges("*CHANGES.POSN?", true);
		this->unlock();
		callParamCallbacks();
		epicsTimeGetCurrent(&pollEndTime);
		epicsThreadSleep(0.1-epicsTimeDiffInSeconds(&pollEndTime, &pollStartTime));
	}
}

void ADPandABlocks::processChanges(std::string cmd, bool posn)
{
	std::vector<std::vector<std::string> > changed;
	std::vector<std::string> changedField;
	int numChanges = 0;
	sendCtrl(cmd);
	changed.clear();
	changed.push_back(readFieldNames(&numChanges));
	for(std::vector<std::string>::iterator it = changed[0].begin(); it != changed[0].end(); ++it)
	{
		std::vector<std::string> changedParts = stringSplit(*it, '=');
		std::stringstream  posBusName;
		std::string posBusNameStr;
		changedField.clear();
		changedField = stringSplit(changedParts[0], '.');
		std::string fieldName = changedField[1];
		std::string fieldVal = "";
		if(posn){
			posBusName << changedParts[0];
			fieldName = "VAL";
		}
		else{
			posBusName << changedField[0] << "." << changedField[1];
			fieldName = changedField[2];
		}
		if(changedParts.size() == 2)
		{
			fieldVal = changedParts[1];
		}
		posBusNameStr = posBusName.str();

		// Update asyn parameter
		if (fieldName == "CAPTURE")
		{
			updatePandAParam(posBusNameStr, fieldName, captureType[fieldVal.c_str()]);
		}
		else if (fieldName == "VAL")
		{
			updatePandAParam(posBusNameStr, fieldName, stringToInteger(fieldVal));
		}
		else if (fieldName == "SCALE" || fieldName == "OFFSET")
		{
			updatePandAParam(posBusNameStr, fieldName, stringToDouble(fieldVal));
		}
		else
		{
			updatePandAParam(posBusNameStr, fieldName, fieldVal);
		}

	}
}

void ADPandABlocks::updateScaledPositionValue(std::string posBusName)
{
	double scale, offset;
	int counts;
	getDoubleParam(*posBusLookup[posBusName]["SCALE"], &scale);
	getDoubleParam(*posBusLookup[posBusName]["OFFSET"], &offset);
	getIntegerParam(*posBusLookup[posBusName]["UNSCALEDVAL"], &counts);
	setDoubleParam(*posBusLookup[posBusName]["VAL"], counts*scale + offset);
}

double ADPandABlocks::stringToDouble(std::string str)
{
	std::stringstream strStream(str);
	double value;
	strStream >> value;
	return value;
}

int ADPandABlocks::stringToInteger(std::string str)
{
	std::stringstream strStream(str);
	int value;
	strStream >> value;
	return value;
}

std::string ADPandABlocks::doubleToString(double value)
{
	std::stringstream strStream;
	strStream << value;
	return strStream.str();
}

std::vector<std::string> ADPandABlocks::stringSplit(const std::string& s, char delimiter)
{
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(s);
	while (std::getline(tokenStream, token, delimiter))
	{
		tokens.push_back(token);
	}
	return tokens;
}

asynStatus ADPandABlocks::sendReceivingFormat() {

	return sendData("XML FRAMED SCALED\n");
}

asynStatus ADPandABlocks::readHeaderLine(char* rxBuffer, const size_t buffSize) const {
	/*buffSize is the size fo rxBuffer*/
	const char *functionName = "readHeaderLine";
	int eomReason;
	size_t nBytesIn;
	asynStatus status = asynTimeout;
	//check to see if rxBuffer is
	while (status == asynTimeout) {
		status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer,
									   buffSize, &nBytesIn, &eomReason);
	}
	if(status && pandaResponsive) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %s'\n",
				  driverName, functionName, errorMsg[status].c_str());
	}
	if (eomReason != ASYN_EOM_EOS && pandaResponsive) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: failed on 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
		return asynError;
	}
	return status;
}

asynStatus ADPandABlocks::readDataBytes(char* rxBuffer, const size_t nBytes, bool &responsive) const {
	const char *functionName = "readDataBytes";
	int eomReason;
	size_t nBytesIn = 0;
	asynStatus status = asynTimeout;

	while (status == asynTimeout) {
		status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer,
									   nBytes, &nBytesIn, &eomReason);
	}

	if(status) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %s'\n",
				  driverName, functionName, errorMsg[status].c_str());
	}
	if(nBytes != nBytesIn) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "%s:%s: Only got %d bytes, not %d bytes with EOM reason %d\n",
				  driverName, functionName, (int)nBytesIn, (int)nBytes, eomReason);
		return asynError;
	}
	if (!responsive && nBytesIn != 0) {
		responsive = true;
	}
	return status;
}

/*this function reads from the data port*/
void ADPandABlocks::readDataPort() {
	asynStatus status = asynSuccess;
	char rxBuffer[N_BUFF_DATA];
	std::string header;
	try{
		while (true) {
			switch(state) {
				case waitHeaderStart:
					status = readHeaderLine(rxBuffer, N_BUFF_DATA);
					if(status == asynError){
						state = waitHeaderStart;
						setIntegerParam(ADPandABlocksIsResponsive, 0);
						break;
					} else {
						setIntegerParam(ADPandABlocksIsResponsive, 1);
					}
					if (strcmp(rxBuffer, "<header>\0") == 0) {
						//we have a header so we have started acquiring
						setIntegerParam(ADAcquire, 1);
                        setIntegerParam(ADStatus, ADStatusAcquire);
                        setStringParam(ADStatusMessage, "Acquiring");
						header.append(rxBuffer);
						header.append("\n");
						callParamCallbacks();
						state = waitHeaderEnd;
					}
					break;

				case waitHeaderEnd:
					status = readHeaderLine(rxBuffer, N_BUFF_DATA);
					if(status == asynError){
						header.clear();
						state = dataEnd;
						setIntegerParam(ADPandABlocksIsResponsive, 0);
						break;
					} else {
						setIntegerParam(ADPandABlocksIsResponsive, 1);
					}
					/*accumulate the header until we reach the end, then process*/
					header.append(rxBuffer);
					header.append("\n");
					if (strcmp(rxBuffer, "</header>\0") == 0) {
						headerValues = parseHeader(header);
						// Read the last line of the header
						status = readHeaderLine(rxBuffer, N_BUFF_DATA);
						if(status == asynError){
							header.clear();
							state = dataEnd;
							setIntegerParam(ADPandABlocksIsResponsive, 0);
							break;
						} else {
							setIntegerParam(ADPandABlocksIsResponsive, 1);
						}
						//change the input eos as the data isn't terminated with a newline
						pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "", 0);
						state = waitDataStart;
					}
					break;

				case waitDataStart:
					// read "BIN " or "END "
					status = readDataBytes(rxBuffer, 4, pandaResponsive);
					if(status == asynError){
						setIntegerParam(ADPandABlocksIsResponsive, 0);
						state = dataEnd;
						break;
					} else {
						setIntegerParam(ADPandABlocksIsResponsive, 1);
					}

					if (strncmp(rxBuffer, "BIN ", 4) == 0) {
						state = receivingData;
					}
					else if (strncmp(rxBuffer, "END ", 4) == 0) {
						state = dataEnd;
					}
					break;

				case receivingData:
					// read next four bytes to get the packet size
				{
					uint32_t message_length;
					status = readDataBytes(reinterpret_cast<char *>(&message_length), 4, pandaResponsive);
					if(status == asynError){
						setIntegerParam(ADPandABlocksIsResponsive, 0);
						state = dataEnd;
						break;
					} else {
						setIntegerParam(ADPandABlocksIsResponsive, 1);
					}
					uint32_t dataLength = message_length - 8; // size of the packet prefix information is 8
					// read the rest of the packet
					status = readDataBytes(rxBuffer, dataLength, pandaResponsive);
					if(status == asynError){
						setIntegerParam(ADPandABlocksIsResponsive, 0);
						state = dataEnd;
						break;
					} else {
						setIntegerParam(ADPandABlocksIsResponsive, 1);
					}

					std::vector<char> dataPacket;
					dataPacket.insert(dataPacket.begin(), rxBuffer, rxBuffer + dataLength);
					parseData(dataPacket, dataLength);
					state = waitDataStart;
				}
					break;

				case dataEnd:
					//reset the header string
					header = "";
					//change the input eos back to newline for the header
					pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "\n", 1);
					//set the acquire light to 0
					setIntegerParam(ADAcquire, 0);
                    setIntegerParam(ADStatus, ADStatusIdle);
                    setStringParam(ADStatusMessage, "Idle");
					status = readHeaderLine(rxBuffer, N_BUFF_DATA);
					setStringParam(ADPandABlocksDataEnd, rxBuffer);
					callParamCallbacks();
					state = waitHeaderStart;
					break;
			}
		}
	}
	catch(const std::runtime_error& e){
		//return to beginning state if there is an exception
		state = waitHeaderStart;
		status = asynError;
	}
	callParamCallbacks();
}


ADPandABlocks::headerMap ADPandABlocks::parseHeader(const std::string& headerString)
{
	/**return a map containing the header data corresponding to each xml node
	 * the first map will always be the 'data' node,
	 * then each field will be pushed onto the vector sequentially
	 */
	std::map<std::string, std::string> tmpValues;
	headerMap tmpHeaderValues;

	//set the header parameter
	setStringParam(ADPandABlocksHeader, headerString.c_str());

	asynStatus status = asynSuccess;
	xmlTextReaderPtr xmlreader = xmlReaderForMemory(headerString.c_str(), (int)headerString.length(), NULL, NULL, 0);

	if (xmlreader == NULL){
		//do some error handling
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
				  "ERROR PARSING HEADER\n");
		status = asynError;
	}
	if(status == asynSuccess)
	{
		/*walk the xml nodes*/
		while ((xmlTextReaderRead(xmlreader)) == 1)
		{
			/*get the node names*/
			const xmlChar* xmlNodeName = xmlTextReaderConstName(xmlreader);
			if (xmlNodeName != NULL)
			{
				std::string name((const char*)xmlNodeName);
				/*get the attributes for the data and field nodes and put on vector*/
				if( name == "data" || name == "field")
				{
					extractHeaderData(xmlreader, tmpValues);
					tmpHeaderValues.push_back(tmpValues);
				}
			}
		}
	}
	xmlFreeTextReader(xmlreader);

	callParamCallbacks();
	return tmpHeaderValues;
}

asynStatus ADPandABlocks::extractHeaderData(const xmlTextReaderPtr xmlreader, std::map<std::string, std::string>& values)const
{
	/*Get the attribute values for a node and place in the map values*/
	xmlNodePtr node= xmlTextReaderCurrentNode(xmlreader);
	if (xmlTextReaderNodeType(xmlreader)==1 && node && node->properties) {
		xmlAttr* attribute = node->properties;
		while(attribute && attribute->name && attribute->children)
		{
			xmlChar* xvalue = xmlNodeListGetString(node->doc, attribute->children, 1);

			/*Insert the values into the data_value map*/
			std::string value((const char*)xvalue);
			std::string name((const char*)attribute->name);
			values[name] = value;

			xmlFree(xvalue);
			attribute = attribute->next;
		}
	}
	return asynSuccess;
}

std::string ADPandABlocks::getHeaderValue(const int index, const std::string attribute)const
{
	/*return the value of an attribute of a given element*/
	//first check index (do find on headerValues
	if (headerValues[index].find(attribute) != headerValues[index].end())
	{
		return headerValues[index].find(attribute)->second;
	}
	else
	{
		throw std::out_of_range("attribute not in map");
	}
}

void ADPandABlocks::getAllData(std::vector<char>& inBuffer, const int dataLen, const int buffLen)const
{
	const char *functionName = "getAllData";
	size_t nBytesIn;
	size_t readBytes = dataLen - buffLen;
	int eomReason;
	asynStatus status = asynSuccess;
	asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_data, 0, 0);
	char rxBuffer[readBytes];
	status = pasynOctet_data->read(octetPvt_data, pasynUserRead, rxBuffer, readBytes,
								   &nBytesIn, &eomReason);

	inBuffer.insert(inBuffer.end(), rxBuffer, rxBuffer+nBytesIn);
	if(status)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %d'\n",
				  driverName, functionName, status);
	}
}

void ADPandABlocks::parseData(std::vector<char> dataBuffer, const int dataLen){
	int buffLen = dataBuffer.size(); //actual length of received input data stream (could be multiple lines)
	int dataNo = headerValues.size() - 1; //number of received data points (total header fields - 'data' = number of captured fields)
	//check to see if we have read all the data in, and do another read if we haven't
	if(dataLen > buffLen)
	{
		getAllData(dataBuffer, dataLen, buffLen);
	}
	outputData(dataLen, dataNo, dataBuffer);
}

void ADPandABlocks::outputData(const int dataLen, const int dataNo, const std::vector<char> data)
{
	try{
		int linecount = 0; //number of lines of data received and parsed
		//get the length of an individual dataSet
		int setLen = 0;
		for(size_t i = 0; i < headerValues.size()-1; ++i)
		{
			if(getHeaderValue(i+1, "type") == "double")
			{
				setLen += sizeof(double);
			}
			else if (getHeaderValue(i+1, "type") == "uint32")
			{
				setLen += sizeof(uint32_t);
			}
		}
		const char* ptridx = &data.front();
		int noDataSets = data.size() / setLen; //number of data sets in the received binary data
		std::string dataType;
		//find other possible data types..

		//loop over the data sets in the received data
		for(int j = 0; j < noDataSets; ++j)
		{
			// are we still acquiring?
			int acquiring;
			getIntegerParam(ADAcquire, &acquiring);
			if (!acquiring) {
				return;
			}

			// allocate a frame for each data set
			allocateFrame();
			if(pArray != NULL) {
				//loop over each data point in the data set
				for(int i = 0; i < dataNo; ++i)
				{

					// NDAttributes are used to store the actual captured data
					std::string desc("sample value");
					//find out what type the individual point is
					//from the header and assign the appropriate pointer.
					dataType = getHeaderValue(i+1, "type");
					if(dataType == "double")
					{

						// Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
						pArray->pAttributeList->add(
								(getHeaderValue(i+1, "name") + "." + getHeaderValue(i+1, "capture")).c_str(),
								desc.c_str(),
								NDAttrFloat64,
								(double*)ptridx);
						((double *)pArray->pData)[i] = *(double*)ptridx;
						ptridx += sizeof(double);
					}
					else if(dataType == "uint32")
					{
						// Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
						pArray->pAttributeList->add(
								(getHeaderValue(i+1, "name") + "." + getHeaderValue(i+1, "capture")).c_str(),
								desc.c_str(),
								NDAttrUInt32,
								(uint32_t*)ptridx);
						uint32_t value = *(uint32_t*)ptridx;
						((double*)pArray->pData)[i] = (double)value;

						//Determine if we need to populate the Bit Mask values
						std::string headerLabel = getHeaderValue(i+1, "name");
						size_t bitsFound = headerLabel.find("BITS");
						if(bitsFound != std::string::npos)
						{
							int blockNum = atoi(headerLabel.substr(bitsFound + 4, 1).c_str());
							uint8_t maskPtr;
							std::string desc("bit mask");
							if(pArray != NULL) {
								for(int maski = 0; maski <32; maski++)
								{
									//shift and mask the value and push into individual NDAttrs
									maskPtr = (value >> maski) & 0x01;
									pArray->pAttributeList->add(
											bitMasks[blockNum][maski].c_str(),
											desc.c_str(),
											NDAttrUInt8,
											&maskPtr);
								}
							}
						}
						ptridx += sizeof(uint32_t);
					};
				}
			}
			/* Ship off the NDArray*/
			wrapFrame();

			/* Increment number of lines processed*/
			linecount++;
			callParamCallbacks();
		}
	}
	catch(const std::out_of_range& e){
		//if attribute is not in header map, go back to beginning ?
	}
}

void ADPandABlocks::allocateFrame() {
	// Release the old NDArray if it exists
	if (pArray != NULL) {
		pArray->release();
		pArray = NULL;
	}
	// Allocate a new NDArray
	int arraysize = headerValues.size() -1;
	size_t dims[2];
	int nDims = 2;
	dims[0] = arraysize;
	dims[1] = FRAMEHEIGHT;
	pArray = pNDArrayPool->alloc(nDims, dims, NDFloat64, 0, NULL);
	//clear the attribute list to get rid of previous scans
	if (pArray != NULL) {
		pArray->pAttributeList->clear();
	}
}

void ADPandABlocks::wrapFrame() {
	this->lock();
	getIntegerParam(NDArrayCounter, &(arrayCounter));
	getIntegerParam(ADNumImagesCounter, &(numImagesCounter));
	// Set the time stamp
	epicsTimeStamp arrayTime;
	epicsTimeGetCurrent(&arrayTime);
	if (pArray != NULL) {
		pArray->timeStamp = arrayTime.secPastEpoch;
		// Save the NDAttributes if there are any
		getAttributes(pArray->pAttributeList);
	}
	// Update statistics
	arrayCounter++;
	numImagesCounter++;
	//send disarm signal if we are in a mode that requires it
	if ((imgMode == ADImageSingle && arrayCounter == 1) ||
		(imgMode == ADImageMultiple && numImagesCounter == imgNo)) {
		sendCtrl("*PCAP.DISARM=");
		setIntegerParam(ADAcquire, 0);
	}
	// Set the unique ID
	if (pArray != NULL) {
		pArray->uniqueId = arrayCounter;
	}
	// Update the counters
	setIntegerParam(NDArrayCounter, arrayCounter);
	setIntegerParam(ADNumImagesCounter, numImagesCounter);
	callParamCallbacks();
	this->unlock();
	if (pArray != NULL) {
		// Ship the array off
		doCallbacksGenericPointer(pArray, NDArrayData, 0);
	}
}

/* Send helper function
 * called with lock taken
 */
asynStatus ADPandABlocks::sendData(const std::string txBuffer){
	asynStatus status = send(txBuffer, pasynOctet_data, octetPvt_data, pasynUser_data);
	return status;
}

asynStatus ADPandABlocks::sendCtrl(const std::string txBuffer){
	asynStatus status = send(txBuffer, pasynOctet_ctrl, octetPvt_ctrl, pasynUser_ctrl);
	return status;
}

asynStatus ADPandABlocks::send(const std::string txBuffer, asynOctet *pasynOctet, void* octetPvt, asynUser* pasynUser) {
	const char *functionName = "send";
	asynStatus status = asynSuccess;
	int connected;
	size_t nBytesOut;
	pasynUser->timeout = TIMEOUT;
	status = pasynOctet->write(octetPvt, pasynUser, txBuffer.c_str(), txBuffer.length(),
							   &nBytesOut);
	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
			  "%s:%s: Send: '%.*s'\n", driverName, functionName, (int)txBuffer.length(), txBuffer.c_str());
	getIntegerParam(ADPandABlocksIsConnected, &connected);
	if (status != asynSuccess && connected) {
		// Can't write, port probably not connected
		setIntegerParam(ADPandABlocksIsConnected, 0);
        setIntegerParam(ADStatus, ADStatusDisconnected);
        setStringParam(ADStatusMessage, "Disconnected");
		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
				  "%s:%s: Can't write to ADPandABlocks: '%.*s'\n", driverName, functionName, (int)txBuffer.length(), txBuffer.c_str());
	} else if (status == asynSuccess && !connected) {
		setIntegerParam(ADPandABlocksIsConnected, 1);
        setIntegerParam(ADStatus, ADStatusIdle);
        setStringParam(ADStatusMessage, "Idle");
		asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
				  "Reconnected to ADPandABlocks'\n");
	}
	return status;
}

// Get encoder number from name of position bus
int ADPandABlocks::getEncoderNumberFromName(std::string posBusName)
{
	// Check if encoder
	std::string encoderStringName("INENC");
	if (posBusName.find(encoderStringName) != std::string::npos)
	{
		// Parse and return encoder number
		int encoderNumber;
		std::stringstream encoderNumberSS;
		encoderNumberSS << posBusName[5];
		encoderNumberSS >> encoderNumber;
		return encoderNumber;
	}
	else
	{
		return -1;
	}

}

// Calibrate encoder position by number (1..4)
void ADPandABlocks::calibrateEncoderPosition(int encoderNumber)
{
	if(encoderNumber > 0 && encoderNumber < NENC)
	{
		// Increment value to cause record to be processed
		int currentValue;
		getIntegerParam(ADPandABlocksMCalibrate[encoderNumber-1], &currentValue);
		setIntegerParam(ADPandABlocksMCalibrate[encoderNumber-1], currentValue + 1);
	}
}

// Set position of encoder (1..4) to custom value
void ADPandABlocks::setEncoderPosition(int encoderNumber, int value)
{
	if(encoderNumber > 0 && encoderNumber < NENC)
	{
		setIntegerParam(ADPandABlocksMSetpos[encoderNumber-1], value);
	}
}

// Erase substring from string in place
void ADPandABlocks::removeSubString(std::string &string, std::string &subString)
{
	// Search and only remove if it exists in string
	size_t pos = string.find(subString);
	if (pos != std::string::npos)
	{
		string.erase(pos, subString.length());
	}
}


// Send setpos (SETP) command to PandA for homing a position bus
void ADPandABlocks::setPandASetPos(std::string posBusName, int value){

	// Remove .VAL from posBusName for correct command
	std::string valSuffix(".VAL");
	removeSubString(posBusName, valSuffix);

	// Build and send command
	std::stringstream setPosCmd;
	setPosCmd << posBusName << ".SETP=" << value;
	sendCtrl(setPosCmd.str());
}

// Check motor offset and scale
bool ADPandABlocks::checkIfMotorFloatParams(int reason, double value)
{
	bool motorParamChanged = false;
	if (checkIfReasonIsMotorOffset(reason, value))
	{
		motorParamChanged = true;
	}
	else if (checkIfReasonIsMotorScale(reason, value))
	{
		motorParamChanged = true;
	}
	return motorParamChanged;
}

// Check if motor offset has changed
bool ADPandABlocks::checkIfReasonIsMotorOffset(int reason, double value)
{
	for (int i=0; i<4; i++)
	{
		if (reason == ADPandABlocksMOffset[i])
		{
			updatePandAMotorParam(i+1, offset, value);
			return true;
		}
	}
	return false;
}

// Check if motor record scaling has changed
bool ADPandABlocks::checkIfReasonIsMotorScale(int reason, double value)
{
	for (int i=0; i<4; i++)
	{
		if (reason == ADPandABlocksMScale[i])
		{
			updatePandAMotorParam(i+1, scale, value);
			return true;
		}
	}
	return false;
}

// Check if motor record units have changed
bool ADPandABlocks::checkIfReasonIsMotorUnit(int reason, std::string value)
{
	for (int i=0; i<4; i++)
	{
		if (reason == ADPandABlocksMUnits[i])
		{
			updatePandAMotorParam(i+1, units, value);
			return true;
		}
	}
	return false;
}

// Check if motor record setpos have changed
bool ADPandABlocks::checkIfReasonIsMotorSetpos(int reason, int value)
{
	for (int i=0; i<4; i++)
	{
		if (reason == ADPandABlocksMSetpos[i])
		{
			updatePandAMotorParam(i+1, setpos, value);
			return true;
		}
	}
	return false;
}

// Change embedded window to readOnly if using motorSync template
bool ADPandABlocks::checkIfReasonIsMotorScreenType(int reason, int value)
{
	for (int i=0; i<4; i++)
	{
		if (reason == ADPandABlocksMScreenType[i])
		{
			embeddedScreenType screenType = static_cast<embeddedScreenType>(value);
			updatePandAMotorParam(i+1, screen, screenType);
			return true;
		}
	}
	return false;
}

// Set motor name param
bool ADPandABlocks::checkIfReasonIsMotorName(int reason, std::string name)
{
	for (int i=0; i<4; i++)
	{
		if (reason == ADPandABlocksMMotorName[i])
		{
			updatePandAMotorParam(i+1, motorName, name);
			return true;
		}
	}
	return false;
}

// Search and update lookup table parameter, sending command to PandA if required
template<typename T>
asynStatus ADPandABlocks::UpdateLookupTableParamFromWrite(int param, T value)
{
	return asynSuccess;
}
template<>
asynStatus ADPandABlocks::UpdateLookupTableParamFromWrite<int>(int param, int value)
{
	const char *functionName = "UpdateLookupTableParamFromWrite<int>";
	if(posBusLookup.empty() == true) return asynSuccess;
	for(std::map<std::string, std::map<std::string, int*> >::iterator it = posBusLookup.begin();
			it != posBusLookup.end(); ++it)
	{
		for(std::map<std::string, int*>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
		{
			if(param == *it2->second)
			{
				if(it2->first == "CAPTURE")
				{
					// Update Capture enum
					std::stringstream cmdStr;
					cmdStr << it->first <<"."<< it2->first <<"="<<captureStrings[value];
					sendCtrl(cmdStr.str());
					std::string field;
					asynStatus status = readPosBusValues(&field);
					if(status == asynError)
					{
						asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting: %s '\n",
								  driverName, functionName, cmdStr.str().c_str());
					}
					return status;
				}
				else if(it2->first == "CALIBRATE")
				{
					// Trigger manual copy of motor encoder position to PandA position
					// Only calibrate once per button press
					if(value == 1)
					{
						int encoderNumber = getEncoderNumberFromName(it->first);
						calibrateEncoderPosition(encoderNumber);
					}
					return asynSuccess;
				}
				else if(it2->first == "SETPOS")
				{
					// Update PandA setpos to new value
					setPandASetPos(it->first, value);
				}
					// Regular parameter update
				else
				{
					std::stringstream cmdStr;
					cmdStr << it->first <<"."<< it2->first <<"="<<value;
					sendCtrl(cmdStr.str());
					std::string field;
					asynStatus status = readPosBusValues(&field);
					if(status == asynError)
					{
						asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting: %s '\n",
								  driverName, functionName, cmdStr.str().c_str());
					}
					return status;
				}
			}
		}
	}
	return asynSuccess;
}
template<>
asynStatus ADPandABlocks::UpdateLookupTableParamFromWrite<float>(int param, float value)
{
	const char *functionName = "UpdateLookupTableParamFromWrite<float>";
	if(posBusLookup.empty() == true) return asynSuccess;
	for(std::map<std::string, std::map<std::string, int*> >::iterator it = posBusLookup.begin();
			it != posBusLookup.end(); ++it)
	{
		for(std::map<std::string, int*>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
		{
			if(param == *it2->second)
			{
				std::stringstream cmdStr;
				cmdStr << it->first <<"."<< it2->first <<"="<<value;
				sendCtrl(cmdStr.str());
				std::string field;
				asynStatus status = readPosBusValues(&field);
				if(status == asynError)
				{
					asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting: %s '\n",
							  driverName, functionName, cmdStr.str().c_str());
				}
				callParamCallbacks();
				return status;
			}
		}
	}
	return asynSuccess;
}
template<>
asynStatus ADPandABlocks::UpdateLookupTableParamFromWrite<std::string>(int param, std::string value)
{
	const char *functionName = "UpdateLookupTableParamFromWrite<std::string>";
	if(posBusLookup.empty() == true) return asynSuccess;
	for(std::map<std::string, std::map<std::string, int*> >::iterator it = posBusLookup.begin();
			it != posBusLookup.end(); ++it)
	{
		for(std::map<std::string, int*>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
		{
			if(param == *it2->second)
			{
				std::stringstream cmdStr;
				cmdStr << it->first <<"."<< it2->first <<"="<<value;
				sendCtrl(cmdStr.str());
				std::string field;
				asynStatus status = readPosBusValues(&field);
				if(status == asynError)
				{
					asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting: %s '\n",
							  driverName, functionName, cmdStr.str().c_str());
				}
				callParamCallbacks();
				return status;
			}
		}
	}
	return asynSuccess;
}

/** Called when asyn clients call pasynFloat64->write().
 * This function performs actions for some parameters
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ADPandABlocks::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	const char *functionName = "writeFloat64";
	asynStatus status = asynError;
	/* Any work we need to do */
	int param = pasynUser->reason;
	status = setDoubleParam(param, value);

	// Check if motor parameters have changed
	if (checkIfMotorFloatParams(param, value));
		// Otherwise check lookup table
	else status = UpdateLookupTableParamFromWrite(param, value);

	if(status)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting values'\n",
				  driverName, functionName);
	}

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();
	return status;

}

/** Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ADPandABlocks::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	const char *functionName = "writeInt32";
	asynStatus status = asynError;

	// Update parameter value
	int param = pasynUser->reason;
	status = setIntegerParam(param, value);

	// Check if image configuration has been updated
	if(param == ADImageMode)
	{
		imgMode = value;
	}
	else if(param == ADNumImages)
	{
		imgNo = value;
	}
	else if(param == ADAcquire)
	{
		if(value)
		{
			//set the current array number
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
					  "SEND ARM CMD:\n");
			sendCtrl("*PCAP.ARM=");
			setIntegerParam(ADNumImagesCounter, 0);
		}
		else
		{
			asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
					  "SEND DISARM CMD:\n");
			sendCtrl("*PCAP.DISARM=");
		}
	}
		// Check if setpos has been called due to homing motor
	else if(checkIfReasonIsMotorSetpos(param, value));
		// Check if need to change embedded screen type if using motorsync
	else if(checkIfReasonIsMotorScreenType(param, value));
		// Otherwise update lookup table
	else status = UpdateLookupTableParamFromWrite(param, value);

	if(status)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting values'\n",
				  driverName, functionName);
	}

	callParamCallbacks();
	return status;
}

/** Called when asyn clients call pasynOctet->write().
 * This function performs actions for some parameters
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ADPandABlocks::writeOctet(asynUser *pasynUser, const char* value, size_t nChars, size_t* nActual) {
	/* This will check if a user has changed a value and attempt to update the
	 * panda. It will also update the readback values */
	const char *functionName = "writeOctet";
	asynStatus status = asynError;
	/* Any work we need to do */
	int param = pasynUser->reason;
	status = setStringParam(param, value);
	*nActual = nChars;

	// Parse to stringStream for string conversion
	std::stringstream valueStream;
	valueStream << value;

	// Check if motor units have changed
	if (checkIfReasonIsMotorUnit(param, valueStream.str()));
		// Check if motor name is being set
	else if (checkIfReasonIsMotorName(param, valueStream.str()));
		// Otherwise check lookup table
	else status = UpdateLookupTableParamFromWrite(param, valueStream.str());

	if(status)
	{
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting values'\n",
				  driverName, functionName);
	}

	callParamCallbacks();
	return status;
}


extern "C" int ADPandABlocksConfig(const char *portName, const char* pandaAddress, int maxPts, int maxBuffers, int maxMemory) {
	new ADPandABlocks(portName, pandaAddress, maxPts, maxBuffers, maxMemory);
	return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg ADPandABlocksConfigArg0 = { "Port name", iocshArgString };
static const iocshArg ADPandABlocksConfigArg1 = { "Panda address", iocshArgString };
static const iocshArg ADPandABlocksConfigArg2 = { "Max number of points to capture in position compare", iocshArgInt };
static const iocshArg ADPandABlocksConfigArg3 = { "maxBuffers for areaDetector", iocshArgInt };
static const iocshArg ADPandABlocksConfigArg4 = { "maxMemory for areaDetector", iocshArgInt };
static const iocshArg* const ADPandABlocksConfigArgs[] = { &ADPandABlocksConfigArg0,
														   &ADPandABlocksConfigArg1, &ADPandABlocksConfigArg2, &ADPandABlocksConfigArg3, &ADPandABlocksConfigArg4 };
static const iocshFuncDef configADPandABlocks = { "ADPandABlocksConfig", 5, ADPandABlocksConfigArgs };
static void configADPandABlocksCallFunc(const iocshArgBuf *args) {
	ADPandABlocksConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void ADPandABlocksRegister(void) {
	iocshRegister(&configADPandABlocks, configADPandABlocksCallFunc);
}

extern "C" {
epicsExportRegistrar(ADPandABlocksRegister);
}
