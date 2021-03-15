#ifndef HCHEAD_H
#define HCHEAD_H

#include <set>
#include <chrono>
#include <algorithm>
#include <vector>
#include <list>
#include <iostream>

#define SDK_VER                    (char*)"3.0.6"

#define SHARK_ENABLE               1

typedef unsigned char              UCHAR;
typedef unsigned char              UINT8;
typedef unsigned short             USHORT;
typedef unsigned short             UINT16;
typedef unsigned long              ULONG;
typedef unsigned int               UINT;
typedef unsigned long              DWORD;
typedef int                        BOOL;
typedef unsigned char              BYTE;
typedef unsigned short             WORD;
typedef unsigned int               UINT32;
typedef unsigned long long int     UINT64;



#define TRUE                            1
#define FALSE                           0


#define  DEFAULT_ID                     (char*)"00000000000000000000000000000000"
#define  DEFAULT_FACTORY                (char*)"CS"
#define  DEFAULT_FIRMWARE               (char*)"00.00.00.00"
#define  DEFAULT_HARDWARE               (char*)"00.00.00.00"

#define  X1B                            "X1B"
#define  X1D                            "X1D"
#define  X1E                            "X1E"
#define  X1F                            "X1F"
#define  X1M                            "X1M"
#define  X2A                            "X2A"
#define  X2B                            "X2B"
#define  X2C                            "X2C"
#define  X2D                            "X2D"
#define  X2E                            "X2E"
#define  X2F                            "X2F"
#define  X2M                            "X2M"
#define  X2N                            "X2N"

#define  PI_HC                          3.141592653589793

//#define  PACKAGESIZE                    128
#define  READ_BUFF_SIZE                 256
#define  READ_TIMEOUT_MS                10


#define  FACT_NEW_LEN                   6
#define  FACT_NEW_RESERVE_LEN           6
#define  FACT_NEW_CAL_LEN               2
#define  FACT_NEW_HARD_LEN              3

#define  FAC_INFO_LEN                   12
#define  RESERVER_LEN                   4
#define  VER_LEN                        3
#define  ID_LEN                         4
#define  DIST_BYTES                     2

#define  MSG_ID                         1
#define  MSG_CMD                        2
#define  MSG_POINTCLOUD                 3

#define  FPS_2000_NOR                   2085
#define  FPS_3000_NOR                   3000
#define  FPS_2000_RANGE                 35
#define  FPS_3000_RANGE                 50
#define  FPS_2000_MAX                   (FPS_2000_NOR+FPS_2000_RANGE)
#define  FPS_2000_MIN                   (FPS_2000_NOR-FPS_2000_RANGE)
#define  FPS_3000_MAX                   (FPS_3000_NOR+FPS_3000_RANGE)
#define  FPS_3000_MIN                   (FPS_3000_NOR-FPS_3000_RANGE)

#define  SPEED_2000_NOR                 312
#define  SPEED_3000_NOR                 360
#define  SPEED_2000_RANGE               6
#define  SPEED_3000_RANGE               6
#define  SPEED_2000_MAX                 (SPEED_2000_NOR+SPEED_2000_RANGE)
#define  SPEED_2000_MIN                 (SPEED_2000_NOR-SPEED_2000_RANGE)
#define  SPEED_3000_MAX                 (SPEED_3000_NOR+SPEED_3000_RANGE)
#define  SPEED_3000_MIN                 (SPEED_3000_NOR-SPEED_3000_RANGE)

#define  MCU_BLOCK_TIME_MS              1500
#define  INIT_TIMEOUT_MS                2000
#define  LESS_THAN_NUMBER               32  
#define  VALID_NUMBER_COUNT             50 
#define  NUMBER_CONTINUE_CIRCLE         50  

typedef struct tsNodeInfo
{
    UINT16    syn_quality;
    UINT16    angle_q6_checkbit;
    UINT16    distance_q2;
    UINT16    isValid;// 1Valid,  0 invalid
   /* bool operator<(const tsNodeInfo& _Left) const
    {
        if (_Left.angle_q6_checkbit < this->angle_q6_checkbit)
        {
            return false;
        }

        return true;
    }*/
}tsNodeInfo;

typedef std::list<tsNodeInfo> LstNodeDistQ2;

typedef struct tsSDKPara
{
    int           iNoDataMS;
    int           iDisconnectMS;
    int           iFPSContinueMS;
    int           iSpeedContinueMS;
    int           iCoverContinueMS;
    int           iBlockContinueMS;
    int           iCoverPoints;
    int           iPollBuffSize;
    int           iCallbackBuffSize;

    tsSDKPara()
    {
        iNoDataMS=1000;
        iDisconnectMS=3000;
        iFPSContinueMS=5000;
        iSpeedContinueMS=3500;
        iCoverContinueMS=3500;
        iBlockContinueMS=3500;
        iCoverPoints = 100;
        iPollBuffSize=1000;
        iCallbackBuffSize = 50;
    }
}tsSDKPara;


typedef enum enSDKStatus
{
    SDK_UNINIT = -1,
    SDK_INIT = 0,
    SDK_ID_TIMEOUT,
    SDK_NO_DATA,
    SDK_DISCONNECT,
    SDK_WORKING
}enSDKStatus;


enum LiDarErrorCode
{
    LIDAR_SUCCESS = 0,

    ERR_MODE_NOT_EXISTS               = -1000,//Lidar mode not exists
    ERR_SERIAL_INVALID_HANDLE         = -1001,//COM handle error
    ERR_SERIAL_SETCOMMTIMEOUTS_FAILED = -1002,//set COM timeouts
    ERR_SERIAL_READFILE_FAILED        = -1003,//read COM Failed
    ERR_SERIAL_READFILE_ZERO          = -1004,//read COM NULL
    ERR_FIND_HEAD_TIMEOUT             = -1005,//Find packet header error
    ERR_CHECKDATA_INVALID             = -1006,//packet cal failed
    ERR_GETPACKAGE_FAILED             = -1007,//get packet failed
    ERR_DATABYTELENGTH_INVALID        = -1008,//Lidar packet error
    ERR_RECEIVE_BUFFER_SMALL          = -1009,//rx buffer too small
    ERR_NOT_ID                        = -1010,//init failed ,get ID Failed
    ERR_NO_DATA                       = -1011,//init failed ,no data rx
    ERR_MOTOR_BLOCKED                 = -1012,//motor blocked
    ERR_REBOOT_LIDAR                  = -1013,//lidar reboot
    ERR_SDK_HAD_BEEN_INIT             = -1014,//SDK had been init
    ERR_SDK_INIT_PARA                 = -1015,//SDK init parameter error
    ERR_DISCONNECTED                  = -1016,//lidar disconnect
    ERR_CALLBACK_FUN                  = -1017,//Not register callback fun
    ERR_POLL_MODE                     = -1018,//poll/callback mode error
    ERR_BUFF_FULL                     = -1019,//poll  mode  pointclouds buff full
    ERR_START_INFO                    = -1020,//Start info error
    ERR_DEV_MODEL                     = -1021,//device model error

    ERR_SHARK_MOTOR_BLOCKED           = -1515,//motor blocked for shark
    ERR_SHARK_INVALID_POINTS          = -1516,//invalid points for shark


    //Lidar erro
    ERR_LIDAR_FPS_INVALID = -3001,//fps 
    ERR_LIDAR_SPEED_LOW = -3002,//speed low
    ERR_LIDAR_SPEED_HIGH = -3003,//speed high
    ERR_LIDAR_NUMBER_INVALID = -3004,//pointcloud too little
    ERR_LIDAR_SAME_ANGLE = -3005, //continue same a angle

};



#pragma pack(push)
#pragma pack(1)

typedef union unDevID
{
    UINT32       u32ID;
    UCHAR        u8ID[ID_LEN];
}unDevID;


typedef struct tsIDNew
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
    UCHAR        u8FacInfo[FACT_NEW_LEN];
    UCHAR        u8FacReserve[FACT_NEW_RESERVE_LEN];
    UINT16       u16Ang;
    UCHAR        u8Direction;
    UCHAR        u8AngleCorrection;
    UCHAR        u8CalVer[FACT_NEW_CAL_LEN];
    UCHAR        u8HardVer[FACT_NEW_HARD_LEN];
    UCHAR        u8ID[ID_LEN];
}tsIDNew;

typedef struct tsIDX2
{
    UINT16       u16Head;
    UCHAR        u8Ver[VER_LEN];
    UCHAR        u8ID[ID_LEN];
    UCHAR        u8Cal;
}tsIDX2;

typedef struct tsIDX1
{
    UINT16       u16Head;
    UCHAR        u8ID[ID_LEN];
    UINT16       u16Cal;
}tsIDX1;

typedef struct tsCmdInfo
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
    UCHAR        u8FacInfo[FAC_INFO_LEN];
    UINT16       u16Ang;
    UCHAR        u8Direction;
    UCHAR        u8AngleCorrection;
    UCHAR        u8ReserverInfo[RESERVER_LEN];
}tsCmdInfo;

typedef struct tsCmdStart
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
}tsCmdStart;

typedef struct tsPointCloudHead
{
    UINT16       u16Head;
    UCHAR        u8Info;
    UCHAR        u8Num;
    UINT16       u16Speed;
    UINT16       u16FirstAng;
}tsPointCloudHead;

typedef struct tsBlockMessage
{
    UINT16       u16Head;
    UCHAR        u8Info;
    UCHAR        u8Len;
    UCHAR        u8MsgID;
    UCHAR        u8Code;
    UCHAR        u8Reserve;
    UCHAR        u8CheckSum;
}tsBlockMessage;


typedef struct tsPointCloudTail
{
    UINT16       u16LastAng;
    UINT16       u16CheckSum;
}tsPointCloudTail;

/*
typedef struct tsPointCloud
{
    UCHAR        u8Dist[DIST_BYTES];
    UINT16       u16Signal;
}tsPointCloud;
*/


#pragma pack(pop)


typedef struct tsSDKStatistic
{
    UINT64        u64TimeStampS;
    UINT64        u64TSRxPacketFirst;
    UINT64        u64RxPacketCount;
    UINT64        u64ErrorPacketCount;
    int           iPortReadCount;
    UINT64        u64FPS;
    int           iPacketPerSecond;
    int           iNumPerPacket;
    int           iValid;
    int           iInvalid;
    UINT32        dRMS;
    UINT64        u64CurrentS;
    int           iGrayBytes;    //
    tsSDKStatistic()
    {
        reset();
        iGrayBytes = 0;
    }
    void reset()
    {
        u64TimeStampS = 0;
        u64TSRxPacketFirst=0;
        u64RxPacketCount=0;
        u64ErrorPacketCount=0;
        iPortReadCount=0;
        u64FPS=0;
        iPacketPerSecond=0;
        iNumPerPacket=0;
        iValid=0;
        iInvalid=0;
        dRMS=0;
        u64CurrentS=0;
    }
}tsSDKStatistic;

//pointcloud
typedef struct tsPointCloud
{
    bool         bValid;       // true Valid point, false  invalid            
    double       dAngle;       // compensate angle    ,degree     
    double       dAngleRaw;    // raw angle     ,degree 
    double       dAngleDisp;   // don't care    
    UINT16       u16Dist;      // compensate distance  ,mm  
    UINT16       u16DistRaw;   // raw distance,mm
    UINT16       u16Speed;     // motor speed, RPM
    UINT16       u16Gray;      // luminance
    bool         bGrayTwoByte; // true  u16Gray 2byte ,false u16Gray 1byte 
    UINT64       u64TimeStampMs;    // timestamp ,ms  
    tsPointCloud() :
        bValid(true),
        dAngle(0.),
        dAngleRaw(0.),
        dAngleDisp(0.),
        u16Dist(0),
        u16DistRaw(0),
        u16Speed(0),
        u16Gray(0),
        bGrayTwoByte(false),
        u64TimeStampMs(0)
    {}
}tsPointCloud;
typedef std::vector<tsPointCloud> LstPointCloud;

class HCHead
{
public:
    HCHead();

    static UINT64 getCurrentTimestampUs();

    static void eraseBuff(std::vector<UCHAR>& lstG,int iLen);
    static void eraseRangeData(LstPointCloud& lstG,int iLen);
};

bool nodeComparator(const tsNodeInfo& s1, const tsNodeInfo& s2);
bool newComparator(const tsPointCloud& s1, const tsPointCloud& s2);

#ifdef __linux__
#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)
#endif
#if defined (_WIN32) || defined( _WIN64)
#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1):__FILE__)
#endif
#define LOG_WARNING (printf("HCSDK Warning:%s:%s %s:%u:\t", __DATE__, __TIME__, __FILENAME__, __LINE__), printf) 
#define LOG_INFO    (printf("HCSDK Info:%s:%s %s:%u:\t", __DATE__, __TIME__, __FILENAME__, __LINE__), printf) 
#define LOG_ERROR   (printf("HCSDK Error:%s:%s %s:%u:\t", __DATE__, __TIME__, __FILENAME__, __LINE__), printf) 

#endif // HCHEAD_H
