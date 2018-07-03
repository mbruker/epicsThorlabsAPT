/*
	EPICS asyn driver for Thorlabs APT motor controllers
	June 2018, M. W. Bruker

    Multi-channel hardware is currently unsupported because I don't have any.
    Support for it could be added easily, though, and additions to the code are welcome.
*/

#include "thorlabs_apt_driver.h"
#include <asynDriver.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <stdint.h>
#include <string.h>
#include <cantProceed.h>

#define MGMSG_HW_REQ_INFO               0x0005
#define MGMSG_HW_GET_INFO               0x0006
#define MGMSG_HW_NO_FLASH_PROGRAMMING   0x0018
#define MGMSG_HW_RESPONSE               0x0080
#define MGMSG_HW_RICHRESPONSE           0x0081
#define MGMSG_MOD_SET_CHANENABLESTATE   0x0210
#define MGMSG_MOD_REQ_CHANENABLESTATE   0x0211
#define MGMSG_MOD_GET_CHANENABLESTATE   0x0212
#define MGMSG_MOT_SET_VELPARAMS         0x0413
#define MGMSG_MOT_REQ_VELPARAMS         0x0414
#define MGMSG_MOT_GET_VELPARAMS         0x0415
#define MGMSG_MOT_SET_GENMOVEPARAMS     0x043A
#define MGMSG_MOT_REQ_GENMOVEPARAMS     0x043B
#define MGMSG_MOT_GET_GENMOVEPARAMS     0x043C
#define MGMSG_MOT_MOVE_HOME             0x0443
#define MGMSG_MOT_MOVE_ABSOLUTE         0x0453
#define MGMSG_MOT_MOVE_STOP             0x0465
#define MGMSG_MOT_REQ_DCSTATUSUPDATE    0x0490
#define MGMSG_MOT_GET_DCSTATUSUPDATE    0x0491

// yet to be implemented:
// S. 55
#define MGMSG_MOT_MOVE_HOMED            0x0444
// S. 58
#define MGMSG_MOT_MOVE_COMPLETED        0x0464
// S. 59
// S. 63
#define MGMSG_MOT_MOVE_STOPPED          0x0466
// S. 97
//#define MGMSG_MOT_GET_STATUSUPDATE      0x0481
//#define MGMSG_MOT_REQ_STATUSUPDATE      0x0480
// S. 100
#define MGMSG_MOT_ACK_DCSTATUSUPDATE    0x0492


void ThorlabsAPTPollThreadC(void *drvPvt)
{
    ((ThorlabsAPTDriver *) drvPvt)->pollThread();
}

ThorlabsAPTDriver::ThorlabsAPTDriver(const char *portName, const char *serialPortName)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynInt32Mask | asynUInt32DigitalMask | asynOctetMask | asynDrvUserMask,
                    asynInt32Mask | asynUInt32DigitalMask | asynOctetMask,
                    ASYN_CANBLOCK, /* asynFlags */
                    1, /* autoConnect */
                    0, /* default priority */
                    0) /* default stack size */
{
    asynStatus status;
    asynInterface *pasynInterface;
    struct ioPvt *pioPvt;
    
    pioPvt = (struct ioPvt *) callocMustSucceed(1, sizeof(struct ioPvt), "ThorlabsAPT");
    asynUserSerial = pasynManager->createAsynUser(0, 0);
    asynUserSerial->userPvt = pioPvt;
    status = pasynManager->connectDevice(asynUserSerial, serialPortName, 0);
    if (status != asynSuccess) {
        printf("Cannot connect to port %s: %s\n", serialPortName, asynUserSerial->errorMessage);
        return;
    }
    pasynInterface = pasynManager->findInterface(asynUserSerial, asynOctetType, 1);
    if (!pasynInterface) {
        printf("%s interface not supported\n", asynOctetType);
        return;
    }
    pioPvt->pasynOctet = (asynOctet *) pasynInterface->pinterface;
    pioPvt->octetPvt = pasynInterface->drvPvt;

    pasynManager->lockPort(asynUserSerial);

    sendShortCommand(MGMSG_HW_NO_FLASH_PROGRAMMING);

    unsigned char *data;
    size_t dataLen;
    sendShortCommand(MGMSG_HW_REQ_INFO);
    status = waitForReply(MGMSG_HW_GET_INFO, (char **) &data, &dataLen);
    if (status == asynTimeout) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: timeout waiting for message MGMSG_HW_GET_INFO\n");
        return;
    }
    if (dataLen != 84) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: malformed message MGMSG_HW_GET_INFO\n");
        return;
    }
    
    createParam(P_NumEvents_String, asynParamInt32, &P_NumEvents);
    setIntegerParam(P_NumEvents, 0);
    createParam(P_LastEvent_String, asynParamInt32, &P_LastEvent);
    createParam(P_LastEventNotes_String, asynParamOctet, &P_LastEventNotes);
    
    unsigned long int serialNumber = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
    createParam(P_SerialNumber_String, asynParamInt32, &P_SerialNumber);
    setIntegerParam(P_SerialNumber, serialNumber);

    char modelNumber[9];
    memcpy(modelNumber, data + 4, 8);
    modelNumber[8] = 0;
    createParam(P_ModelNumber_String, asynParamOctet, &P_ModelNumber);
    setStringParam(P_ModelNumber, modelNumber);
    
    createParam(P_FirmwareVersionMinor_String, asynParamInt32, &P_FirmwareVersionMinor);
    setIntegerParam(P_FirmwareVersionMinor, data[14]);
    createParam(P_FirmwareVersionInterim_String, asynParamInt32, &P_FirmwareVersionInterim);
    setIntegerParam(P_FirmwareVersionInterim, data[15]);
    createParam(P_FirmwareVersionMajor_String, asynParamInt32, &P_FirmwareVersionMajor);
    setIntegerParam(P_FirmwareVersionMajor, data[16]);

    char notes[49];
    memcpy(notes, data + 18, 48);
    notes[48] = 0;
    createParam(P_Notes_String, asynParamOctet, &P_Notes);
    setStringParam(P_Notes, notes);

    unsigned short int hardwareVersion = (data[79] << 8) | data[78];
    createParam(P_HardwareVersion_String, asynParamInt32, &P_HardwareVersion);
    setIntegerParam(P_HardwareVersion, hardwareVersion);
    
    unsigned short int modState = (data[81] << 8) | data[80];
    createParam(P_ModState_String, asynParamInt32, &P_ModState);
    setIntegerParam(P_ModState, modState);

    unsigned short int numberChannels = (data[83] << 8) | data[82];
    createParam(P_NumberChannels_String, asynParamInt32, &P_NumberChannels);
    setIntegerParam(P_NumberChannels, numberChannels);

    free(data);

    printf("ThorlabsAPT: model %s; fw ver %u.%u.%u; hw ver %u; %u channels\n", modelNumber, data[16], data[15], data[14], hardwareVersion, numberChannels);

    // Channel 1: Get enabled state
    sendShortCommand(MGMSG_MOD_REQ_CHANENABLESTATE, 1, 0);
    status = waitForReply(MGMSG_MOD_GET_CHANENABLESTATE, (char **) &data, &dataLen);
    if (status == asynTimeout) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: timeout waiting for message MGMSG_MOD_GET_CHANENABLESTATE\n");
        return;
    }
    if (dataLen != 2) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: malformed message MGMSG_MOD_GET_CHANENABLESTATE\n");
        return;
    }
    createParam(P_ChEnabled_String, asynParamInt32, &P_ChEnabled);
    setIntegerParam(P_ChEnabled, data[1] == 1 ? 1 : 0);
    free(data);

    // Channel 1: Get velocity parameters
    sendShortCommand(MGMSG_MOT_REQ_VELPARAMS, 1, 0);
    status = waitForReply(MGMSG_MOT_GET_VELPARAMS, (char **) &data, &dataLen);
    if (status == asynTimeout) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: timeout waiting for message MGMSG_MOT_GET_VELPARAMS\n");
        return;
    }
    if (dataLen != 14) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: malformed message MGMSG_MOT_GET_VELPARAMS\n");
        return;
    }
    createParam(P_MinVelocity_String, asynParamInt32, &P_MinVelocity);
    setIntegerParam(P_MinVelocity, (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
    createParam(P_Acceleration_String, asynParamInt32, &P_Acceleration);
    setIntegerParam(P_Acceleration, (data[9] << 24) | (data[8] << 16) | (data[7] << 8) | data[6]);
    createParam(P_MaxVelocity_String, asynParamInt32, &P_MaxVelocity);
    setIntegerParam(P_MaxVelocity, (data[13] << 24) | (data[12] << 16) | (data[11] << 8) | data[10]);
    free(data);
    
    // Channel 1: Get general move parameters (backlash)
    sendShortCommand(MGMSG_MOT_REQ_GENMOVEPARAMS, 1, 0);
    status = waitForReply(MGMSG_MOT_GET_GENMOVEPARAMS, (char **) &data, &dataLen);
    if (status == asynTimeout) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: timeout waiting for message MGMSG_MOT_GET_GENMOVEPARAMS\n");
        return;
    }
    if (dataLen != 6) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: malformed message MGMSG_MOT_GET_GENMOVEPARAMS\n");
        return;
    }
    createParam(P_Backlash_String, asynParamInt32, &P_Backlash);
    setIntegerParam(P_Backlash, (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
    free(data);
    
    pasynManager->unlockPort(asynUserSerial);
    
    createParam(P_CurrentPosition_String, asynParamInt32, &P_CurrentPosition);
    createParam(P_CurrentVelocity_String, asynParamInt32, &P_CurrentVelocity);
    createParam(P_StatusBits_String, asynParamUInt32Digital, &P_StatusBits);
    requestStatusUpdate();
    
    createParam(P_MoveAbsolute_String, asynParamInt32, &P_MoveAbsolute);
    createParam(P_MoveStop_String, asynParamInt32, &P_MoveStop);
    createParam(P_MoveHome_String, asynParamInt32, &P_MoveHome);
    
    status = (asynStatus) (epicsThreadCreate("ThorlabsAPTPollThread",
                                             epicsThreadPriorityMedium,
                                             epicsThreadGetStackSize(epicsThreadStackMedium),
                                             (EPICSTHREADFUNC) ThorlabsAPTPollThreadC,
                                             this) == NULL);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "epicsThreadCreate failed\n");
    }
}

void ThorlabsAPTDriver::pollThread()
{
    for (;;) {
        requestStatusUpdate();
        epicsThreadSleep(0.1);
    }
}

asynStatus ThorlabsAPTDriver::sendPacket(unsigned char *dataToSend, size_t sendLen)
{
    asynStatus status = asynSuccess;
    size_t numBytes = 0;
    struct ioPvt *pioPvt = (struct ioPvt *) asynUserSerial->userPvt;

    // send message to device and read reply
    asynUserSerial->timeout = 1.0;
    status = pioPvt->pasynOctet->write(pioPvt->octetPvt, asynUserSerial, (char *) dataToSend, sendLen, &numBytes);

    return status;
}

asynStatus ThorlabsAPTDriver::sendLongCommand(unsigned short int commandId, unsigned char *extraData, size_t extraDataLen)
{
    unsigned char message[256];
    unsigned char *pMessage = message;

    if (extraDataLen > 250)
        return asynError;

    // command id, little endian
    *pMessage++ = commandId;
    *pMessage++ = commandId >> 8;

    // size of extra data
    *pMessage++ = extraDataLen;
    *pMessage++ = extraDataLen >> 8;

    // device address
    *pMessage++ = extraDataLen == 0 ? deviceAddress : (deviceAddress | 0x80);
    
    // source address
    *pMessage++ = 0x01;

    // extra data
    for (unsigned char i = 0; i < extraDataLen; ++i)
        *pMessage++ = extraData[i];
    
    return sendPacket(message, extraDataLen + 6);
}

asynStatus ThorlabsAPTDriver::sendShortCommand(unsigned short int commandId, unsigned char p1, unsigned char p2)
{
    unsigned char message[6];
    message[0] = commandId;
    message[1] = commandId >> 8;
    message[2] = p1;
    message[3] = p2;
    message[4] = deviceAddress;
    message[5] = 0x01;
    
    return sendPacket(message, 6);
}

unsigned short int ThorlabsAPTDriver::recvPacket(char **extraData, size_t *extraDataLen)
{
    size_t numBytes = 0, totalBytes = 0;
    struct ioPvt *pioPvt = (struct ioPvt *) asynUserSerial->userPvt;
    unsigned char message[50];
    
    // Find out if there is data to be read. If so, read the whole message, otherwise return immediately.
    // This is done to prevent the polling thread from locking the port unnecessarily.
    asynUserSerial->timeout = 0.0;
    pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, (char *) message, 6, &numBytes, 0);
    if (numBytes == 0)
        return 0;

    totalBytes = numBytes;
    while (totalBytes < 6) {
        asynUserSerial->timeout = 1.0;
        pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, (char *) &message[totalBytes], 6 - totalBytes, &numBytes, 0);

        if (!numBytes) {
            printf("ThorlabsAPT: timeout after reading %lu/6 bytes; message incomplete\n", totalBytes);
            return 0;
        }
        totalBytes += numBytes;
    }
    
    unsigned short int messageId = message[1] << 8 | message[0];
    
    size_t _extraDataLen = 0;
    char *_extraData = 0;
    if (message[4] & 0x80) {
        _extraDataLen = message[3] << 8 | message[2];
        _extraData = (char *) malloc(_extraDataLen < 2 ? 2 : _extraDataLen);
        
        totalBytes = 0;
        while (totalBytes < _extraDataLen) {
            asynUserSerial->timeout = 1.0;
            pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, _extraData + totalBytes, _extraDataLen - totalBytes, &numBytes, 0);
    
            if (!numBytes) {
                printf("ThorlabsAPT: message id 0x%x: timeout after reading %lu/%lu bytes of extra data; message incomplete\n", messageId, totalBytes, _extraDataLen);
                free(_extraData);
                return 0;
            }
            totalBytes += numBytes;
        }
    } else {
        _extraData = (char *) malloc(2);
        _extraData[0] = message[2];
        _extraData[1] = message[3];
        _extraDataLen = 2;
    }
    if (extraData)
        *extraData = _extraData;
    else
        free(_extraData);
    if (extraDataLen)
        *extraDataLen = _extraDataLen;

    return messageId;
}

void ThorlabsAPTDriver::processUnsolicitedMessage(unsigned short int messageId, unsigned char *extraData, size_t extraDataLen)
{
    switch (messageId) {
        case MGMSG_HW_RESPONSE: {
            int numEvents;
            getIntegerParam(P_NumEvents, &numEvents);
            setIntegerParam(P_NumEvents, numEvents + 1);
            setIntegerParam(P_LastEvent, extraData[1] << 8 | extraData[0]);
            callParamCallbacks();
            break;
        }
        case MGMSG_HW_RICHRESPONSE: {
            int numEvents;
            getIntegerParam(P_NumEvents, &numEvents);
            setIntegerParam(P_NumEvents, numEvents + 1);
            setIntegerParam(P_LastEvent, extraData[2] << 8 | extraData[1]);
            char buffer[65];
            buffer[64] = 0;
            memcpy(buffer, extraData + 4, 64);
            setStringParam(P_LastEventNotes, buffer);
            callParamCallbacks();
            break;
        }
        default: {
        }
    }
}

asynStatus ThorlabsAPTDriver::waitForReply(unsigned short int expectedReplyId, char **extraData, size_t *extraDataLen)
{
    unsigned short int replyId = 0;
    char *_extraData;
    size_t _extraDataLen;
    int timeoutCounter = 0;

    for (;;) {
        replyId = recvPacket(&_extraData, &_extraDataLen);
        for (timeoutCounter = 0; (timeoutCounter < 10) && !replyId; ++timeoutCounter) {
            epicsThreadSleep(0.05);
            replyId = recvPacket(&_extraData, &_extraDataLen);
        }
        if (timeoutCounter == 10)
            return asynTimeout;
        
        if (replyId == expectedReplyId) {
            *extraData = _extraData;
            *extraDataLen = _extraDataLen;
            return asynSuccess;
        }
    
        processUnsolicitedMessage(replyId, (unsigned char *) _extraData, _extraDataLen);
        free(_extraData);
    }
}

void ThorlabsAPTDriver::requestStatusUpdate()
{
    asynStatus status;
    unsigned char *data;
    size_t dataLen;
    
    lock();
    pasynManager->lockPort(asynUserSerial);
    sendShortCommand(MGMSG_MOT_REQ_DCSTATUSUPDATE, 1, 0);
    status = waitForReply(MGMSG_MOT_GET_DCSTATUSUPDATE, (char **) &data, &dataLen);
    pasynManager->unlockPort(asynUserSerial);
    
    if (status == asynTimeout) {
    	unlock();
        printf("ThorlabsAPT: timeout waiting for message MGMSG_MOT_GET_DCSTATUSUPDATE\n");
        return;
    }
    if (dataLen != 14) {
    	unlock();
        free(data);
        printf("ThorlabsAPT: malformed message MGMSG_MOT_GET_DCSTATUSUPDATE\n");
        return;
    }
    setIntegerParam(P_CurrentPosition, (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
    setIntegerParam(P_CurrentVelocity, (data[7] << 8) | data[6]);
    setUIntDigitalParam(P_StatusBits, (data[13] << 24) | (data[12] << 16) | (data[11] << 8) | data[10], 0xffffffff);
    callParamCallbacks(0, 0);
    unlock();
    free(data);
}

asynStatus ThorlabsAPTDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    
    if (function == P_MoveAbsolute) {
    	unsigned char data[6];
    	data[0] = 1;
    	data[1] = 0;
    	data[2] = value;
    	data[3] = value >> 8;
    	data[4] = value >> 16;
    	data[5] = value >> 24;
        return sendLongCommand(MGMSG_MOT_MOVE_ABSOLUTE, data, 6);
    } else if (function == P_MoveStop) {
    	return sendShortCommand(MGMSG_MOT_MOVE_STOP, 1, 2);
    } else if (function == P_MoveHome) {
        return sendShortCommand(MGMSG_MOT_MOVE_HOME, 1, 0);
    } else
    	return asynPortDriver::writeInt32(pasynUser, value);
}


extern "C" {

int ThorlabsAPTConfigure(const char *portName, const char *serialPortName)
{
    new ThorlabsAPTDriver(portName, serialPortName); // scary but apparently the usual way
    return asynSuccess;
}

static const iocshArg initArg0 = { "portName", iocshArgString };
static const iocshArg initArg1 = { "serialPortName", iocshArgString };
static const iocshArg * const initArgs[] = {&initArg0, &initArg1};
static const iocshFuncDef initFuncDef = { "ThorlabsAPTConfigure", 2, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{
    ThorlabsAPTConfigure(args[0].sval, args[1].sval);
}

void ThorlabsAPTDriverRegister()
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(ThorlabsAPTDriverRegister);

}

