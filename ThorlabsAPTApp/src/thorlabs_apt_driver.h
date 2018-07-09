/*
	EPICS asyn driver for Thorlabs APT motor controllers
	June 2018, M. W. Bruker
*/

#ifndef _THORLABS_APT_DRIVER_H
#define _THORLABS_APT_DRIVER_H

#include "asynPortDriver.h"
#include "stdint.h"

class asynUser;

#define P_NumEvents_String "NUM_EVENTS"
#define P_LastEvent_String "LAST_EVENT"
#define P_LastEventNotes_String "LAST_EVENT_NOTES"

#define P_SerialNumber_String "SERIAL_NUMBER"
#define P_ModelNumber_String "MODEL_NUMBER"
#define P_FirmwareVersionMinor_String "FIRMWARE_VERSION_MINOR"
#define P_FirmwareVersionInterim_String "FIRMWARE_VERSION_INTERIM"
#define P_FirmwareVersionMajor_String "FIRMWARE_VERSION_MAJOR"
#define P_Notes_String "NOTES"
#define P_HardwareVersion_String "HARDWARE_VERSION"
#define P_ModState_String "MOD_STATE"
#define P_NumberChannels_String "NUMBER_CHANNELS"

#define P_ChEnabled_String "CH_ENABLED"
#define P_MinVelocity_String "MIN_VELOCITY"
#define P_Acceleration_String "ACCELERATION"
#define P_MaxVelocity_String "MAX_VELOCITY"
#define P_Backlash_String "BACKLASH"
#define P_CurrentPosition_String "CURRENT_POSITION"
#define P_CurrentVelocity_String "CURRENT_VELOCITY"
#define P_StatusBits_String "STATUS_BITS"

#define P_MoveAbsolute_String "MOVE_ABSOLUTE"
#define P_MoveStop_String "MOVE_STOP"
#define P_MoveHome_String "MOVE_HOME"

class ThorlabsAPTDriver : public asynPortDriver {
public:
    ThorlabsAPTDriver(const char *portName, const char *serialPortName);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    void pollThread();
protected:
    struct ioPvt {
        asynOctet *pasynOctet;
        void *octetPvt;
    };
    asynStatus sendPacket(unsigned char *dataToSend, size_t sendLen);
    asynStatus sendLongCommand(unsigned short int commandId, unsigned char *extraData, size_t extraDataLen);
    asynStatus sendShortCommand(unsigned short int commandId, unsigned char p1 = 0, unsigned char p2 = 0);
    unsigned short int recvPacket(char **extraData, size_t *extraDataLen);
    void processUnsolicitedMessage(unsigned short int messageId, unsigned char *extraData, size_t extraDataLen);
    asynStatus waitForReply(unsigned short int expectedReplyId, char **extraData = 0, size_t *extraDataLen = 0);
    
    void requestStatusUpdate();
    asynStatus setVelocityParams();
    
    static const unsigned char deviceAddress = 0x50;
    
    int P_NumEvents;
    int P_LastEvent;
    int P_LastEventNotes;
    
    int P_SerialNumber;
    int P_ModelNumber;
    int P_FirmwareVersionMinor;
    int P_FirmwareVersionInterim;
    int P_FirmwareVersionMajor;
    int P_Notes;
    int P_HardwareVersion;
    int P_ModState;
    int P_NumberChannels;

    // The following quantities correspond to one motor channel.
    // For multi-channel support, they should be declared as arrays and handled accordingly.
    int P_ChEnabled;
    int P_MinVelocity;
    int P_Acceleration;
    int P_MaxVelocity;
    int P_Backlash;
    int P_CurrentPosition;
    int P_CurrentVelocity;
    int P_StatusBits;
    
    int P_MoveAbsolute;
    int P_MoveStop;
    int P_MoveHome;

    asynUser *asynUserSerial;
};


#endif

