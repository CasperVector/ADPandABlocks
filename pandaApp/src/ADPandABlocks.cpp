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


ADPandABlocks::ADPandABlocks(const char* portName, const char* pandaAddress, int maxBuffers, int maxMemory) :
        ADDriver(portName, 1 /*maxAddr*/, NUM_PARAMS, maxBuffers, maxMemory,
                 asynInt8ArrayMask | asynFloat64ArrayMask | asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
                 asynInt8ArrayMask | asynFloat64ArrayMask | asynInt32Mask | asynFloat64Mask | asynOctetMask,
                 ASYN_CANBLOCK, /*ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0 */
                 1, /*autoConnect*/ 0, /*default priority */ 0 /*default stack size*/) {

    //private variables
    state = waitHeaderStart; //init state for the data read
    pandaResponsive = false;
    epicsTimeGetCurrent(&lastHeaderErrorTime);

    errorMsg[asynSuccess] = "asynSuccess";
    errorMsg[asynTimeout] = "asynTimeout";
    errorMsg[asynOverflow] = "asynOverflow";
    errorMsg[asynError] = "asynError";
    errorMsg[asynDisconnected] = "asynDisconnected";
    errorMsg[asynDisabled] = "asynDisabled";

    const char *functionName = "ADPandABlocks";
    asynStatus status = asynSuccess;
    asynInterface *pasynInterface;

    /* For areaDetector image */
    pArray = NULL;
    arrayCounter = 0;
    numImagesCounter = 0;
    numExposures = 1;
    imgMode = ADImageContinuous;
    imgNo = 0;

    /* Connection status */
    createParam("ISCONNECTED", asynParamInt32, &ADPandABlocksIsConnected);
    setIntegerParam(ADPandABlocksIsConnected, 0);
    createParam("ISRESPONSIVE", asynParamInt32, &ADPandABlocksIsResponsive);
    setIntegerParam(ADPandABlocksIsResponsive, 0);

    /*Create a parameter to store the header value */
    createParam("HEADER", asynParamOctet, &ADPandABlocksHeader);

    /*Create a parameter to store the end of data string */
    createParam("DATAEND", asynParamOctet, &ADPandABlocksDataEnd);
    setStringParam(ADPandABlocksDataEnd, "");

    /* initialise areaDetector parameters */
    setStringParam(ADManufacturer, "Diamond Light Source Ltd.");
    setStringParam(ADModel, "ADPandABlocks");
    setIntegerParam(ADMaxSizeX, NPOSBUS);
    setIntegerParam(ADMaxSizeY, 1);
    //    setIntegerParam(NDDataType, 7);
    setIntegerParam(ADStatus, ADStatusIdle);
    setStringParam(ADStatusMessage, "Idle");

    /* Connect to the device port */
    ctrlPortName = std::string(portName).append("_CTRL");
    ctrlPort = ctrlPortName.c_str();

    /* Copied from asynOctecSyncIO->connect */
    pasynUser_ctrl_tx = pasynManager->createAsynUser(0, 0);
    status = pasynManager->connectDevice(pasynUser_ctrl_tx, ctrlPort, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, ctrlPort, status);
        return;
    }
    pasynInterface = pasynManager->findInterface(pasynUser_ctrl_tx, asynCommonType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
        return;
    }
    pasynInterface = pasynManager->findInterface(pasynUser_ctrl_tx, asynOctetType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: %s interface not supported", driverName, functionName, asynOctetType);
        return;
    }

    pasynOctet_ctrl = (asynOctet *) pasynInterface->pinterface;
    octetPvt_ctrl = pasynInterface->drvPvt;
    asynSetOption(ctrlPort, 0, "disconnectOnReadTimeout", "Y");
    pasynUser_ctrl_tx->drvUser = (void *) this;
    pasynManager->exceptionCallbackAdd(pasynUser_ctrl_tx, callbackC);

    /* Set EOS and flush */
    pasynOctet_ctrl->flush(octetPvt_ctrl, pasynUser_ctrl_tx);
    pasynOctet_ctrl->setInputEos(octetPvt_ctrl, pasynUser_ctrl_tx, "\n", 1);
    pasynOctet_ctrl->setOutputEos(octetPvt_ctrl, pasynUser_ctrl_tx, "\n", 1);
    
    // duplicate port & set timeout
    
    pasynUser_ctrl_rx = pasynManager->duplicateAsynUser(pasynUser_ctrl_tx, 0, 0);
    pasynUser_ctrl_rx->timeout = 3.0;

    /* Connect to the data port */
    dataPortName = std::string(portName).append("_DATA");
    dataPort = dataPortName.c_str();

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

    /* Create thread to monitor data port */ 
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
 * Create position bus parameter
 * \param[in] paramName Parameter name
 * \param[in] paramType Parameter type
 * \param[in] paramIndex Parameter index
 * \param[in] paramNo Parameter number
 */
void ADPandABlocks::createPosBusParam(const char* paramName, asynParamType paramType, int* paramIndex, int paramNo){
    char str[NBUFF];
    epicsSnprintf(str, NBUFF, "POSBUS%d:%s", paramNo, paramName);
    createParam(str, paramType, paramIndex);
}


/*
 * Check if the position bus is in use
 * \param[in] posBusName Position bus name
 * \param[out] boolean Bus is in use
 */
bool ADPandABlocks::posBusInUse(std::string posBusName) {
    // Unused position buses will have the name POSBUS<N>
    std::size_t found = posBusName.find("POSBUS");
    if (found == std::string::npos) return true;
    else return false;
}


/*
 * Create lookup table parameter
 * \param[in] paramName Parameter name prefix
 * \param[in] paramNameEnd Parameter name suffix
 * \param[in] paramInd Parameter index
 * \param[in] posBusInd Position bus index
 */
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


/*
 * Read field names (called from read thread)
 * \param[in] numFields Number of fields updated
 * \param[out] fieldNameStrings Vector of updated field names
 */
std::vector<std::string> ADPandABlocks::readFieldNames(int* numFields) {
    const char *functionName = "readFieldNames";
    char rxBuffer[N_BUFF_CTRL];
    size_t nBytesIn;
    int eomReason;
    asynStatus status = asynSuccess;
    std::vector<std::string> fieldNameStrings;
    status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUser_ctrl_rx, rxBuffer, N_BUFF_CTRL - 1,
                                   &nBytesIn, &eomReason);

    // We failed to read the field names, return empty object
    if (status != asynSuccess) {

        // Check if we are connected
        int connected;
        getIntegerParam(ADPandABlocksIsConnected, &connected);
        // Only print error if we expect a response
        if (connected) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: readFieldNames read failed, error=%d\n", driverName, functionName, status);
        }
        return fieldNameStrings;
    }

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
        status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUser_ctrl_rx, rxBuffer, N_BUFF_CTRL - 1,
                                       &nBytesIn, &eomReason);
    }
    *numFields = i;
    return fieldNameStrings;
}


asynStatus ADPandABlocks::sendReceivingFormat() {

    return sendData("XML FRAMED SCALED\n");
}


asynStatus ADPandABlocks::readHeaderLine(char* rxBuffer, const size_t buffSize, epicsTimeStamp &lastErrorTime) const {
    /*buffSize is the size fo rxBuffer*/
    const char *functionName = "readHeaderLine";
    int eomReason;
    bool threw = false;
    size_t nBytesIn;
    asynStatus status = asynTimeout;
    //check to see if rxBuffer is
    while (status == asynTimeout) {
        status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer,
                                       buffSize, &nBytesIn, &eomReason);
    }
    epicsTimeStamp currentTime;
    epicsTimeGetCurrent(&currentTime);
    if(status && pandaResponsive) {
        if (epicsTimeDiffInSeconds(&currentTime, &lastErrorTime) > 0.5) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Error reading data: %s'\n",
                      driverName, functionName, errorMsg[status].c_str());
            threw = true;
        }
    }
    if (eomReason != ASYN_EOM_EOS && pandaResponsive) {
        if (epicsTimeDiffInSeconds(&currentTime, &lastErrorTime) > 0.5) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: failed on 'bt%.*s'\n", driverName, functionName, (int) nBytesIn, rxBuffer);
        }
        lastErrorTime = currentTime;
        return asynError;
    }
    if (threw)  lastErrorTime = currentTime;
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
                    status = readHeaderLine(rxBuffer, N_BUFF_DATA, lastHeaderErrorTime);
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
                    status = readHeaderLine(rxBuffer, N_BUFF_DATA, lastHeaderErrorTime);
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
                        status = readHeaderLine(rxBuffer, N_BUFF_DATA, lastHeaderErrorTime);
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
                    setIntegerParam(ADStatus, ADStatusIdle);
                    setStringParam(ADStatusMessage, "Idle");
                    callParamCallbacks();
                    setIntegerParam(ADAcquire, 0);
                    status = readHeaderLine(rxBuffer, N_BUFF_DATA, lastHeaderErrorTime);
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

    /*
     *  As this method is only called before the first frame it makes sense to any map travsering here, once.
     *  What it in the map is stored in arrays for fast access on every frame.
     */
    typeArray[0]=0;
    nameCaptArray[0]=(char * )"";
    setLen=0;
    headerArraySize=tmpHeaderValues.size();


    /*
     * Depending on the data type populate typeArray with a 1 for a double and a 2 for an int.
     * Also keep setLen up to date with the total size so far.
     */
    for(unsigned int j = 0; j < tmpHeaderValues.size()-1; j++){;
        if(tmpHeaderValues[j+1].find("type")->second=="double") {
            typeArray[j+1]=1;
            setLen += sizeof(double);
        }
        else {
            typeArray[j+1]=2;
            setLen += sizeof(uint32_t);
        }

        nameCaptArray[j+1]=strdup(tmpHeaderValues[j+1].find("name")->second.c_str());
        nameCaptArray[j+1]=strcat(nameCaptArray[j+1],".");
        nameCaptArray[j+1]=strcat(nameCaptArray[j+1],tmpHeaderValues[j+1].find("capture")->second.c_str());
    }

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


void ADPandABlocks::getAllData(std::vector<char>& inBuffer, const int dataLen, const int buffLen)const
{
    const char *functionName = "getAllData";
    size_t nBytesIn;
    size_t readBytes = dataLen - buffLen;
    int eomReason;
    asynStatus status = asynSuccess;
    // asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_data, 0, 0);
    char rxBuffer[readBytes];
    status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer, readBytes,
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
        int writeAttributes = 0;
        int endOfFrame = 0;

        const char* ptridx = &data.front();
        int noDataSets = data.size() / setLen; //number of data sets in the received binary data

        //find other possible data types..

        //loop over the data sets in the received data

        for (int j = 0; j < noDataSets; ++j) {
            writeAttributes = 0;
            endOfFrame = 0;
            if(numExposuresCounter == 0){
                endOfFrame = 0;
                writeAttributes = 1;
            }else{
                writeAttributes = 0;
            }

            // are we still acquiring?
            int acquiring;
            getIntegerParam(ADAcquire, &acquiring);
            if (!acquiring) {
                return;
            }

            // allocate a frame for each data set
            if(writeAttributes) {
                allocateFrame();
            }
            if (pArray != NULL) {
                //loop over each data point in the data set
                for (int i = 0; i < dataNo; ++i) {

                    // NDAttributes are used to store the actual captured data

                    //find out what type the individual point is
                    //from the header and assign the appropriate pointer.

                    if(typeArray[i+1] == 1) {
                        // Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
                        if(writeAttributes) {
                            pArray->pAttributeList->add(
                                (nameCaptArray[i+1]),
                                "sample value",
                                NDAttrFloat64,
                                (double *) ptridx);
                        }
                        // Write data for every trigger but if using multiple exposures don't overwrite data from a previous exposure
                        ((double *) pArray->pData)[i+(dataNo*numExposuresCounter)] = *(double *) ptridx;
                        ptridx += sizeof(double);
                    } else if (typeArray[i+1] == 2) {
                        // Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
                        if(writeAttributes) {
                            pArray->pAttributeList->add(
                                    (nameCaptArray[i+1]),
                                    "sample value",
                                    NDAttrUInt32,
                                    (uint32_t *) ptridx);
                        }
                        uint32_t value = *(uint32_t *) ptridx;
                        // Write data for every trigger but if using multiple exposures don't overwrite data from a previous exposure
                        ((double *) pArray->pData)[i+(dataNo*numExposuresCounter)] = (double) value;

                        //Determine if we need to populate the Bit Mask value
                        std::string headerLabel = nameCaptArray[i+1];
                        size_t bitsFound = headerLabel.find("BITS");
                        if (bitsFound != std::string::npos) {
                            int blockNum = atoi(headerLabel.substr(bitsFound + 4, 1).c_str());
                            uint8_t maskPtr;
                            if (pArray != NULL) {
                                for (int maski = 0; maski < 32; maski++) {
                                    //shift and mask the value and push into individual NDAttrs
                                    maskPtr = (value >> maski) & 0x01;
                                    pArray->pAttributeList->add(
                                            bitMasks[blockNum][maski].c_str(),
                                            "bit mask",
                                            NDAttrUInt8,
                                            &maskPtr);
                                }
                            }
                        }
                        ptridx += sizeof(uint32_t);
                    };
                }
            }

            numExposuresCounter++;
            if(numExposuresCounter == numExposures){
                numExposuresCounter = 0;
                endOfFrame = 1;
            }
            /* Ship off the NDArray*/
            if(endOfFrame) {
                wrapFrame();
            }

            /* Increment number of lines processed*/
            linecount++;
            //callParamCallbacks();
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
    dims[1] = numExposures;
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
        pArray->timeStamp +=0.000000001*arrayTime.nsec;
        // Required as of ADCore 2-0 - https://cars.uchicago.edu/software/epics/areaDetectorTimeStampSupport.html
        updateTimeStamp(&pArray->epicsTS);
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
        setIntegerParam(ADStatus, ADStatusIdle);
        setStringParam(ADStatusMessage, "Idle");
        callParamCallbacks();
        setIntegerParam(ADAcquire, 0);
        callParamCallbacks();
    }
    // Set the unique ID
    if (pArray != NULL) {
        pArray->uniqueId = arrayCounter;
    }
    // Update the counters
    setIntegerParam(NDArrayCounter, arrayCounter);
    setIntegerParam(ADNumImagesCounter, numImagesCounter);
    //callParamCallbacks();
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
    asynStatus status = send(txBuffer, pasynOctet_ctrl, octetPvt_ctrl, pasynUser_ctrl_tx);
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Can't write to ADPandABlocks: '%.*s'\n", driverName, functionName, (int)txBuffer.length(), txBuffer.c_str());
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "PandA box appears to be disconnected. Polling errors will be suppressed.\n");
    } else if (status == asynSuccess && !connected) {
        setIntegerParam(ADPandABlocksIsConnected, 1);
        setIntegerParam(ADStatus, ADStatusIdle);
        setStringParam(ADStatusMessage, "Idle");
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "Reconnected to ADPandABlocks\n");
    }
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
            getIntegerParam(ADNumExposures,&numExposures);
                        if (numExposures < 1) numExposures = 1;
            numExposuresCounter=0;
        }
        else
        {
            asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
                      "SEND DISARM CMD:\n");
            sendCtrl("*PCAP.DISARM=");
        }
    }

    if(status)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting values'\n",
                  driverName, functionName);
    }

    callParamCallbacks();
    return status;
}


extern "C" int ADPandABlocksConfig(const char *portName, const char* pandaAddress, int maxBuffers, int maxMemory) {
    new ADPandABlocks(portName, pandaAddress, maxBuffers, maxMemory);
    return (asynSuccess);
}


/** Code for iocsh registration */
static const iocshArg ADPandABlocksConfigArg0 = { "Port name", iocshArgString };
static const iocshArg ADPandABlocksConfigArg1 = { "Panda address", iocshArgString };
static const iocshArg ADPandABlocksConfigArg2 = { "maxBuffers for areaDetector", iocshArgInt };
static const iocshArg ADPandABlocksConfigArg3 = { "maxMemory for areaDetector", iocshArgInt };
static const iocshArg* const ADPandABlocksConfigArgs[] = { &ADPandABlocksConfigArg0,
                                                           &ADPandABlocksConfigArg1, &ADPandABlocksConfigArg2, &ADPandABlocksConfigArg3 };
static const iocshFuncDef configADPandABlocks = { "ADPandABlocksConfig", 4, ADPandABlocksConfigArgs };
static void configADPandABlocksCallFunc(const iocshArgBuf *args) {
    ADPandABlocksConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}


static void ADPandABlocksRegister(void) {
    iocshRegister(&configADPandABlocks, configADPandABlocksCallFunc);
}


extern "C" {
epicsExportRegistrar(ADPandABlocksRegister);
}
