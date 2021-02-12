#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <boost/test/unit_test.hpp>
#include <unistd.h>
#include <signal.h>
#include <thread>

#include "testingutilities.h"

#include "NDArray.h"
#include "asynNDArrayDriver.h"
#include "NDAttributeList.h"

//#include <libxml/xmlreader.h>

#include "ADPandABlocks.h"

//path to the ADPandABlocks sim server
//#define Z2SIMSERVER "/home/fwf58757/git/zebra2/zebra2-server/simserver -Pserver.pid 2>&1"
#define Z2SIMSERVER "/dls_sw/work/targetOS/zebra2-server/simserver -- -Pserver.pid "

/*---------------
 *COMS CONSTANTS*/
#define PORTNAME "ADPandABlocks"
#define CMDIP "127.0.0.1"
#define MAXPTS 10000
#define MAXBUFFERS 10000
#define MAXMEMORY 0

/*--------------*/

/*GLOBAL VARIABLES*/
ADPandABlocks *z2;
/*----------------*/

using namespace std;


FILE* startSim(){
    FILE *z2SimProcess;
    if(!(z2SimProcess = popen(Z2SIMSERVER, "r"))){
        cout<<"failed to start process"<<endl;
        return NULL;
    }
    else{
        cout<<"started ADPandABlocks simulation"<<endl;
    }

    return z2SimProcess;
}

void closeSim(FILE* simProcess){
    //Now we're done, ask the server to close.
    FILE *file = fopen("server.pid", "r");
    int server_pid;

    fscanf(file, "%d", &server_pid);
    fclose(file);

    printf("About to kill %d\n", server_pid);
    kill(server_pid, SIGINT);
    pclose(simProcess);

}


void connectToADPandABlocks(){
    unsigned int microseconds = 3e6;
    //need a better way to do this wait - if it isn't here we don't connect
    usleep(microseconds);
    z2 = new ADPandABlocks("ADPandABlocks", CMDIP, MAXPTS, MAXBUFFERS, MAXMEMORY);
    cout << "NEW PANDABOX" << endl;
}

void setParams(const char* paramsFile){
        ifstream file(paramsFile);
        string str;
        while (std::getline(file, str))
        {
            z2->sendCtrl(str);
        }
}

void setSequenceParams(double countStep, double countStart, double prescale, int frames){
/* Temporary function in absence of web interface for setup for testing*/
    std::ostringstream sscntstp, sscntst, ssps, ssfr;
    sscntstp << "COUNTER1.STEP=" << countStep;
    sscntst  << "COUNTER1.START=" << countStart;
    ssps     << "SEQ1.PRESCALE=" << prescale;
    ssfr     << frames;
    std::string str1 = sscntstp.str();
    std::string str2 = sscntst.str();
    std::string str3 = ssps.str();
    std::string str4 = ssfr.str();

        std::string cmds[22]= {
                "*PCAP.DISARM=",
                "*CAPTURE=",
                str1,
                str2,
                "COUNTER1.ENABLE=SEQ1.ACTIVE",
                "COUNTER1.TRIG=SEQ1.OUTA",
                "COUNTER1.OUT.CAPTURE=Triggered",
                str3,
                "SEQ1.TABLE_CYCLE=1",
                "SEQ1.TABLE<",
                str4,
                "520109824",
                "1",
                "1",
                "",
                "SEQ1.ENABLE=PCAP.ACTIVE",
                "SEQ1.INPA=BITS.ONE",
                "PCAP.ENABLE=SEQ1.ACTIVE",
                "PCAP.FRAME=BITS.ZERO",
                "PCAP.CAPTURE=SEQ1.OUTA",
                "PCAP.CAPTURE_TS.CAPTURE=Trigger",
                "*PCAP.ARM="};
        for(int i = 0; i < 22; i ++)
        {
            cout << "SENDING: " << cmds[i] << endl;
            z2->sendCtrl(cmds[i]);
        }
}

struct ADPandABlocksGlobalFixture
{
    FILE* simProcess;

    ADPandABlocksGlobalFixture()
    {
        simProcess = startSim();
        //instead of waiting here, we should somehow read 'Server started' from the output
        unsigned int microseconds = 3e6;
        usleep(microseconds);
        connectToADPandABlocks(); // BK: flatten this function here so it is clearly
                             // visible that z2 is created here with new

    }
    ~ADPandABlocksGlobalFixture()
    {
        delete z2;
        closeSim(simProcess);

    }
};

struct ADPandABlocksFixture
{
    NDArrayPool *arrayPool;
    asynPortDriver *dummy_driver;
    asynNDArrayDriver *dummy_NDArrayDriver;
    TestingPlugin *ds;

    ADPandABlocksFixture()
    {

        std::string dummy_port("simPort"), testport("testPort"), dummy_NDArrayport("testNDArrayPort");

        // Asyn manager doesn't like it if we try to reuse the same port name for multiple drivers (even if only one is ever instantiated at once), so
        // change it slightly for each test case.
        uniqueAsynPortName(dummy_port);
        uniqueAsynPortName(testport);

        // We need some upstream driver for our test plugin so that calls to connectToArrayPort don't fail, but we can then ignore it and send
        // arrays by calling processCallbacks directly.
        // Thus we instansiate a basic asynPortDriver object which is never used.
        dummy_driver = new asynPortDriver(dummy_port.c_str(), 0, asynGenericPointerMask, asynGenericPointerMask, 0, 0, 0, 2000000);


        dummy_NDArrayDriver = new asynNDArrayDriver(dummy_NDArrayport.c_str(), 0, 1, 0, asynGenericPointerMask, asynGenericPointerMask, 0, 0, 0, 2000000);

        arrayPool = new NDArrayPool(dummy_NDArrayDriver, 0);

        // This is the mock downstream plugin
        ds = new TestingPlugin("ADPandABlocks", 0);

    }
    ~ADPandABlocksFixture()
    {
        delete dummy_driver;
        delete arrayPool;
        delete dummy_NDArrayDriver;
        // BK are we leaking ds?
    }
};

BOOST_GLOBAL_FIXTURE(ADPandABlocksGlobalFixture);

BOOST_AUTO_TEST_SUITE(ADPandABlocksTests)

BOOST_FIXTURE_TEST_SUITE(ReframeBufferingTests, ADPandABlocksFixture)

BOOST_AUTO_TEST_CASE(sequence_test_short_slow)
{
    //these should come from the actual test script
    double count, ts;
    double prescale = 0.05;
    double timer_step = prescale *2;
    double count_start = 100;
    double count_step = 50;
    double counter = count_start + count_step;
    double timer = prescale;
    int frames = 50;
    setSequenceParams(count_step, count_start, prescale, frames);

//    wait till all the arrays are there
    cout << "WAITING.." << endl;
    while(ds->arrays.size() != (unsigned int) frames){
        cout << "acquired frames: " << ds->arrays.size() << "\r" << std::flush;
        // BK: std::endl := end of line + flush
    }
    cout << endl;
    cout << "RECEIVED ALL ARRAYS: "<< ds->arrays.size() << endl;

    for(unsigned int i = 0; i< ds->arrays.size(); i++)
    {
        ds->arrays[i]->pAttributeList->find("COUNTER1.OUT")->getValue(NDAttrFloat64, &count );
        ds->arrays[i]->pAttributeList->find("PCAP.CAPTURE_TS")->getValue(NDAttrFloat64, &ts );
        BOOST_REQUIRE_CLOSE( timer, ts, 0.001 );
        BOOST_REQUIRE_CLOSE(counter, count, 0.001 );
        counter += count_step;
        timer += timer_step;
    }

    BOOST_REQUIRE_EQUAL((size_t)frames, ds->arrays.size()); // BK if this is not true the test hangs before this test, so this will always be true

    //wait to give a chance to send the next set of data
    unsigned int waitTime = 2e6;
    usleep(waitTime);
}

BOOST_AUTO_TEST_CASE(sequence_test_short_fast)
{
       //these should come from the actual test script
        double count, ts;
        double prescale = 0.01;
        double timer_step = prescale *2;
        double count_start = 100;
        double count_step = 50;
        double counter = count_start + count_step;
        double timer = prescale;
        int frames = 50;
        setSequenceParams(count_step, count_start, prescale, frames);

    //    wait till all the arrays are there
        cout << "WAITING.." << endl;
        while(ds->arrays.size() != (unsigned int) frames){
            cout << "acquired frames: " << ds->arrays.size() << "\r" << std::flush;
        }
        cout << endl;
        cout << "RECEIVED ALL ARRAYS: "<< ds->arrays.size() << endl;

        for(unsigned int i = 0; i< ds->arrays.size(); i++)
        {
            ds->arrays[i]->pAttributeList->find("COUNTER1.OUT")->getValue(NDAttrFloat64, &count );
            ds->arrays[i]->pAttributeList->find("PCAP.CAPTURE_TS")->getValue(NDAttrFloat64, &ts );
            BOOST_REQUIRE_CLOSE( timer, ts, 0.001 );
            BOOST_REQUIRE_CLOSE(counter, count, 0.001 );
            counter += count_step;
            timer += timer_step;
        }

        BOOST_REQUIRE_EQUAL((size_t)frames, ds->arrays.size()); // BK: same error as before

        //wait to give a chance to send the next set of data
        unsigned int waitTime = 2e6;
        usleep(waitTime);
}

BOOST_AUTO_TEST_CASE(sequence_test_long_slow)
{
    //these should come from the actual test script
    double count, ts;
    double prescale = 0.05;
    double timer_step = prescale *2;
    double count_start = 100;
    double count_step = 50;
    double counter = count_start + count_step;
    double timer = prescale;
    int frames = 500;
    setSequenceParams(count_step, count_start, prescale, frames);

//    wait till all the arrays are there
    cout << "WAITING.." << endl;
    while(ds->arrays.size() != (unsigned int) frames){
        cout << "acquired frames: " << ds->arrays.size() << "\r" << std::flush;
    }
    cout << endl;
    cout << "RECEIVED ALL ARRAYS: "<< ds->arrays.size() << endl;

    for(unsigned int i = 0; i< ds->arrays.size(); i++)
    {
        ds->arrays[i]->pAttributeList->find("COUNTER1.OUT")->getValue(NDAttrFloat64, &count );
        ds->arrays[i]->pAttributeList->find("PCAP.CAPTURE_TS")->getValue(NDAttrFloat64, &ts );
        BOOST_REQUIRE_CLOSE( timer, ts, 0.001 );
        BOOST_REQUIRE_CLOSE(counter, count, 0.001 );
        counter += count_step;
        timer += timer_step;
    }

    BOOST_REQUIRE_EQUAL((size_t)frames, ds->arrays.size());

    //wait to give a chance to send the next set of data
    unsigned int waitTime = 2e6;
    usleep(waitTime);
}

BOOST_AUTO_TEST_CASE(sequence_test_long_fast)
{
    //these should come from the actual test script
    double count, ts;
    double prescale = 0.01;
    double timer_step = prescale *2;
    double count_start = 100;
    double count_step = 50;
    double counter = count_start + count_step;
    double timer = prescale;
    int frames = 50; //5000
    setSequenceParams(count_step, count_start, prescale, frames);

//    wait till all the arrays are there
    cout << "WAITING.." << endl;
    while(ds->arrays.size() != (unsigned int) frames){
        cout << "acquired frames: " << ds->arrays.size() << "\r" << std::flush;
    }
    cout << endl;
    cout << "RECEIVED ALL ARRAYS: "<< ds->arrays.size() << endl;

    for(unsigned int i = 0; i< ds->arrays.size(); i++)
    {
        ds->arrays[i]->pAttributeList->find("COUNTER1.OUT")->getValue(NDAttrFloat64, &count );
        ds->arrays[i]->pAttributeList->find("PCAP.CAPTURE_TS")->getValue(NDAttrFloat64, &ts );
        BOOST_REQUIRE_CLOSE( timer, ts, 0.001 );
        BOOST_REQUIRE_CLOSE(counter, count, 0.001 );
        counter += count_step;
        timer += timer_step;
    }

    BOOST_REQUIRE_EQUAL((size_t)frames, ds->arrays.size());

    //wait to give a chance to send the next set of data
    unsigned int waitTime = 2e6;
    usleep(waitTime);
}

//test sending n frames
BOOST_AUTO_TEST_CASE(sequence_test)
{

    //these should come from the actual test script
    double count, ts;
    double prescale = 0.01;
    double timer_step = prescale *2;
    double count_start = 100;
    double count_step = 50;
    double counter = count_start + count_step;
    double timer = prescale;
    int frames = 100;
    setSequenceParams(count_step, count_start, prescale, frames);

//    setParams("testseqlong");
//

//    unsigned int waitTime = 100e6;
//    //other test scenarios -
//
//    //this wait should really be waiting for the 'END' from the data
//    usleep(waitTime);


//    wait till all the arrays are there
    cout << "WAITING.." << endl;
    while(ds->arrays.size() != (unsigned int) frames){
        cout << "acquired frames: " << ds->arrays.size() << "\r" << std::flush;
    }
    cout << endl;
    cout << "RECEIVED ALL ARRAYS: "<< ds->arrays.size() << endl;

    for(unsigned int i = 0; i< ds->arrays.size(); i++)
    {
//        cout << "ARRAY[0]       : " << ((double *)ds->arrays[i]->pData)[0];
//        cout << ", ARRAY[1]    : " << ((double *)ds->arrays[i]->pData)[1] << endl;
        ds->arrays[i]->pAttributeList->find("COUNTER1.OUT")->getValue(NDAttrFloat64, &count );
        ds->arrays[i]->pAttributeList->find("PCAP.CAPTURE_TS")->getValue(NDAttrFloat64, &ts );
//        cout << "PCAP.CAPTURE_TS: " << ts << ", COUNTER1.OUT: "<< count <<  endl;
//        cout << "TIMER          : " << timer << ", COUNTER      :" << counter << endl;
        BOOST_REQUIRE_CLOSE( timer, ts, 0.001 );
        BOOST_REQUIRE_CLOSE(counter, count, 0.001 );
        counter += count_step;
        timer += timer_step;
    }

    BOOST_REQUIRE_EQUAL((size_t)frames, ds->arrays.size());

    //wait to give a chance to send the next set of data
    unsigned int waitTime = 2e6;
    usleep(waitTime);
}

//other test scenarios -


//parse header test
BOOST_AUTO_TEST_CASE(parse_header_test)
{
    /* INPUT : xml header
     * OUTPUT: std::map<std::string, std::string>
     */ // BK: Missing?
}

//parse data test
BOOST_AUTO_TEST_CASE(parse_data_test)
{

}

BOOST_AUTO_TEST_SUITE_END()
}
