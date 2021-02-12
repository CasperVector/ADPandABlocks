#ifndef ADPandABlocks_H
#define ADPandABlocks_H

#include <cstring>
#include <string>
#include <map>
#include <vector>

#include <libxml/xmlreader.h>

#include "asynOctetSyncIO.h"
#include "asynCommonSyncIO.h"
#include "ADDriver.h"
#include "drvAsynIPPort.h"
#include "asynShellCommands.h"
#include "epicsTime.h"

/*Port numbers for connection*/
#define CTRL_PORT "8888"
#define DATA_PORT "8889"

/* This is the number of messages on our queue */
#define NQUEUE 10000

/* The size of our transmit and receive buffers,
 * max filename length and string param buffers */
#define NBUFF 255
#define N_BUFF_CTRL 255
#define N_BUFF_DATA 65536

/* This is the number of positions on the posbus*/
#define NPOSBUS 32

/* This is the number of encoders (motors) */
#define NENC 4

/* The timeout waiting for a response from ADPandABlocks */
#define TIMEOUT 1.0

/* We want to block while waiting on an asyn port forever.
 * Unfortunately putting 0 or a large number causes it to
 * poll and take up lots of CPU. This number seems to work
 * and causes it to block for a reasonably long time (in seconds)
 */
#define LONGWAIT 1000.0

class ADPandABlocks: public ADDriver {
private:
    /**Typedefs**/
    //vector of maps to store header data. Each map is for an individual xml node
    typedef std::vector<std::map<std::string, std::string> > headerMap;

public:
    // Embedded screen type for position bus
    enum embeddedScreenType {writeable, readOnly, empty};

    // Enum for tracking updates to motor fields from GeoBrick
    enum motorField {scale, offset, units, setpos, screen, motorName};

public:
    ADPandABlocks(const char *portName, const char* pandaAddress, int maxPts, int maxBuffers, int maxMemory);
    // virtual ~ADPandABlocks();
    /** These should be private, but get called from C, so must be public */
    void readDataPort();
    void pollCommandPort();

    const char *ctrlPort;
    const char *dataPort;
    std::string ctrlPortName;
    std::string dataPortName;

    bool pandaResponsive;
    epicsTimeStamp lastHeaderErrorTime;
    /** These functions are used in the tests, so they are public */
    asynStatus sendCtrl(const std::string txBuffer);
    asynStatus sendData(const std::string txBuffer);
    asynStatus send(const std::string txBuffer, asynOctet *pasynOctet, void* octetPvt, asynUser* pasynUser);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char* value, size_t nChars, size_t* nActual);

    void exceptionCallback(asynUser *pasynUser, asynException exception);
protected:

#define FIRST_PARAM ADPandABlocksIsConnected
    int ADPandABlocksIsConnected;        // int32 read  - is ADPandABlocks connected?
    int ADPandABlocksIsResponsive;       // int32 read  - is ADPandABlocks responsive ?
    int ADPandABlocksHeader;             // string read - data header
    int ADPandABlocksDataEnd;            // string read - end of data string
    int ADPandABlocksPCTime;             // float64array read - position compare timestamps
#define LAST_PARAM ADPandABlocksPCTime
    int ADPandABlocksPosFields[NPOSBUS]; // string read     - position field names
    int ADPandABlocksPosVals[NPOSBUS];   // string read     - position field scaled values
    int ADPandABlocksPosUnscaledVals[NPOSBUS];// int32 read - position field unscaled values
    int ADPandABlocksScale[NPOSBUS];     // float64 write   - motor scale
    int ADPandABlocksSetpos[NPOSBUS];     // float64 write  - motor setpos
    int ADPandABlocksOffset[NPOSBUS];    // float64 write   - motor offset
    int ADPandABlocksUnits[NPOSBUS];     // string write    - motor units
    int ADPandABlocksCapture[NPOSBUS];   // string write    - pcap capture type
    int ADPandABlocksScreenType[NPOSBUS];// int32 write     - embedded screen to use for each bus
    int ADPandABlocksCalibrate[NPOSBUS]; // int32 write     - Used for calibrating encoders via MCalibrate param
    int ADPandABlocksMotorName[NPOSBUS]; // string write    - motor name
    int ADPandABlocksMScale[NENC];       // float64 write   - motor scale from GeoBrick
    int ADPandABlocksMSetpos[NENC];      // int32 write     - motor setpos from GeoBrick (Calibrate encoder when homed)
    int ADPandABlocksMOffset[NENC];      // float64 write   - motor offset from GeoBrick
    int ADPandABlocksMUnits[NENC];       // string write    - motor units from GeoBrick
    int ADPandABlocksMScreenType[NENC];  // int32 write     - Applies writeOnly embedded screen if using MotorSync
    int ADPandABlocksMCalibrate[NENC];   // int32 read      - Calibrates encoder position
    int ADPandABlocksMMotorName[NENC]; // string write  - motor name from GeoBrick
#define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1 + NPOSBUS*9 + NENC*6)

private:
    headerMap parseHeader(const std::string& headerString);
    void parseData(std::vector<char> dataBuffer, const int dataLen);
    void allocateFrame();
    void wrapFrame();
    std::vector<std::string> readFieldNames(int* numFields);
    asynStatus readPosBusValues(std::string* posBusValue);
    asynStatus extractHeaderData(const xmlTextReaderPtr xmlreader, std::map<std::string, std::string>& values)const;
    std::string getHeaderValue(const int index, const std::string attribute)const;
    void getAllData(std::vector<char>& inBuffer, const int dataLen,const  int buffLen)const;
    void outputData(const int dataLen, const int dataNo, const std::vector<char> data);
    asynStatus readHeaderLine(char* rxBuffer, const size_t buffSize, epicsTimeStamp &lastErrorTime)const;
    asynStatus readDataBytes(char* rxBuffer, const size_t nBytes, bool &responsive)const;
    void createPosBusParam(const char* paramName, asynParamType paramType, int* paramIndex, int paramNo);
    std::string getPosBusField(std::string posbus, const char* paramName);
    bool posBusInUse(std::string posBusName);
    void createLookup(std::string paramName, std::string paramNameEnd, int* paramInd, int posBusInd);
    std::vector<std::string> stringSplit(const std::string& s, char delimiter);
    void processChanges(std::string cmd, bool posn);
    void updateScaledPositionValue(std::string posBusName);
    int getEncoderNumberFromName(std::string posBusName);
    void calibrateEncoderPosition(int encoderNumer);
    void setEncoderPosition(int encoderNumer, int value);
    void setPandASetPos(std::string posBusName, int value);
    bool checkIfMotorFloatParams(int reason, double value);
    bool checkIfReasonIsMotorOffset(int reason, double value);
    bool checkIfReasonIsMotorScale(int reason, double value);
    bool checkIfReasonIsMotorUnit(int reason, std::string value);
    bool checkIfReasonIsMotorSetpos(int reason, int value);
    bool checkIfReasonIsMotorScreenType(int reason, int value);
    bool checkIfReasonIsMotorName(int reason, std::string name);
    template<typename T>
    void updatePandAParam(std::string name, std::string field, T value);
    template<typename T>
    void updatePandAMotorParam(int motorIndex, motorField field, T value);
    template<typename T>
    asynStatus UpdateLookupTableParamFromWrite(int param, T value);
    double stringToDouble(std::string str);
    int stringToInteger(std::string str);
    std::string doubleToString(double value);
    void removeSubString(std::string &string, std::string &subString);    
private:
    NDArray *pArray;
    asynUser *pasynUser_ctrl_tx;
    asynUser *pasynUser_ctrl_rx;
    asynOctet *pasynOctet_ctrl;
    void *octetPvt_ctrl; // BK: is there a good reason for this to be part of the global state?
    asynUser *pasynUser_data;
    asynCommon *pasynCommon_data;
    void *pcommonPvt_data;
    asynOctet *pasynOctet_data;
    void *octetPvt_data;
    int arrayCounter, numImagesCounter, numExposures,numExposuresCounter, imgMode, imgNo;
    //vector of maps for the header values
    headerMap headerValues;
    // Important header information stored in arrays for efficiency.
    int typeArray[1024];
    char* nameCaptArray[1024];
    int headerArraySize;
    int setLen;
    //Vector containing vector of strings for bit mask values
    std::vector<std::vector<std::string> > bitMasks;

    //Vector containing vector of strings for position fields
    std::vector<std::vector<std::string> > posFields;

    //Lookup table for posbus params
    std::map<std::string, std::map<std::string, int*> > posBusLookup;

    //Capture type map
    std::map<std::string, int> captureType;
    std::vector<std::string> captureStrings;

    //states for readDataTask state machine
    enum readState{waitHeaderStart, waitHeaderEnd, waitDataStart, receivingData, dataEnd,};
    readState state; //init state for the data read
    asynStatus sendReceivingFormat() ;

    // Polling
    epicsTimeStamp pollStartTime, pollEndTime;

};
#endif
