#ifndef HCLIDAR_H
#define HCLIDAR_H


#include "hchead.h"
#include "rOc_serial.h"


#include <thread>
#include <mutex>
#include <cmath>
#include <list>
#include <vector>
#include <chrono>
#include <condition_variable>
#include <atomic>
#include <functional>


typedef std::function<void(int)>              CallBackFunErroCode;
typedef std::function<void(tsSDKStatistic)>   CallBackFunSecondInfo;
typedef std::function<void(LstPointCloud)>    CallBackFunPointCloud;
typedef std::function<void(LstNodeDistQ2)>    CallBackFunDistQ2;

typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> MicroClock_type;



class HCLidar
{
public:
    HCLidar();
    ~HCLidar();

    BOOL initialize(const char* chPort, const char* chLidarModel, int iBaud, int iReadTimeoutMs, bool bDistQ2,bool bGetLoopData, bool bPollMode,tsSDKPara& sSDKPara);
    BOOL unInit();

    void setCallBackFunErrorCode(CallBackFunErroCode fun)
    {
        m_funErrorCode = fun;
    }
    void setCallBackFunSecondInfo(CallBackFunSecondInfo fun)
    {
        m_funSecondInfo = fun;
    }
    void setCallBackFunPointCloud(CallBackFunPointCloud fun)
    {
        m_funPointCloud = fun;
    }
    void setCallBackFunDistQ2(CallBackFunDistQ2 fun)
    {
        m_funDistQ2 = fun;
    }
    int getLastErrCode()
    {
        int err = m_iLastErrorCode;
        if (m_iLastErrorCode != 0)
            m_iLastErrorCode = 0;
        return err;
    }

    int getSDKStatus()
    {
        return m_iSDKStatus;
    }

    char* getSDKVersion() const
    {
        return SDK_VER;
    }

    char* getLidarID() const
    {
        return (char*)m_strDevID.c_str();
    }

    char* getFactoryInfo() const
    {
        return (char*)m_strFactoryInfo.c_str();
    }

    char* getLidarModel() const
    {
        return (char*)m_strLidarModel.c_str();
    }

    char* getFirmwareVersion() const
    {
        return (char*)m_strFirmwareVer.c_str();
    }

    char* getHardwareVersion() const
    {
        return (char*)m_strHardwareVer.c_str();
    }


    void getRxPointClouds(LstPointCloud& lstG);
    void getScanData(std::list<tsNodeInfo>& dataList, bool bReverse=true);

private:
    void threadWork();
    void threadParse();
    void readData();

    void setReadCharsError(int errCode);


private:
    CallBackFunErroCode      m_funErrorCode=nullptr;
    CallBackFunSecondInfo    m_funSecondInfo=nullptr;
    CallBackFunPointCloud    m_funPointCloud=nullptr;
    CallBackFunDistQ2        m_funDistQ2=nullptr;

    int                      m_iBaud = 115200;
    std::string              m_strLidarModel = X1B;
    std::string              m_strFirmwareVer = DEFAULT_FIRMWARE;
    std::string              m_strHardwareVer = DEFAULT_HARDWARE;
    std::vector<UCHAR>       m_lstBuff;
    std::vector<UCHAR>       m_lstTemp;
    UCHAR                   *m_p8Buff;

    bool                     m_bScanning = false;


    std::thread              m_threadWork;
    std::thread              m_threadParse;
    std::mutex               m_mtxBuff;

    std::mutex               m_mtxData;
    std::list<tsNodeInfo>    m_sNodeList;
    std::list<tsNodeInfo>    m_sLoopNodeList;
	LstPointCloud            m_resultRange;

    bool                     m_bGetLoopData = false;
    bool                     m_bPollMode=false;
    bool                     m_bDistQ2=false;
    int                      m_iInvalidNumberContinue = 0;
    bool                     m_bGreaterThan = false;
    rOc_serial               m_serial ;
    int                      m_iReadTimeOutms = 30;
    std::atomic<int>         m_iLastErrorCode ;
    std::atomic<int>         m_iSDKStatus ;

    double                   m_dAngleCur = 0;
    double                   m_dAnglePre = 0;
    bool                     m_bTurn = false;
    bool                     m_bFirsLoop = false;


	std::atomic<bool>        m_bInitTimeout;
    std::atomic<bool>        m_bDisconnect;
    std::atomic<bool>        m_bHadID;
    std::atomic<bool>        m_bGetIDTimeOut;
    std::atomic<bool>        m_bHadFact;
    std::atomic<bool>        m_bGetFactTimeOut;

    bool                     m_bHadInfo;
    bool                     m_bX2ID = true;
    std::string              m_strDevID=DEFAULT_ID;
    std::string              m_strFactoryInfo=DEFAULT_FACTORY;

    tsSDKStatistic           m_sStatistic;
    std::mutex               m_mtxStatistic;

    double                   m_dAngleOffsetD=21;    
    bool                     m_bThreadStart;
    bool                     m_bCompensate = true;
    double                   m_dBaseline_mm=20;
    double                   m_dTheta_d = 10;

    std::mutex               m_mtxInit;
    std::condition_variable  m_cvInit;
    bool                     m_bReady = false;
    tsSDKPara                m_sSDKPara;
    int                      m_iReadTimeoutCount=0;
    int                      m_iFPSMax = 2120;
    int                      m_iFPSMin = 2050;
    int                      m_iSpeedMax = 420;
    int                      m_iSpeedMin = 300;
    int                      m_iCircleNumberMAX=415;

    int                      m_iInvalidFPSSecond=0;
    UINT64                   m_u64StartTimeLowSpeed=0;
    UINT64                   m_u64StartTimeHighSpeed=0;
    UINT64                   m_u64StartTimeSharkBlock=0;
    int                      m_iSharkBlockCount=0;
    UINT64                   m_u64StartTimeInvalidPoints=0;
    int                      m_iValidNumber = 0;

    UINT64                   m_u64StartMS = 0;
    inline bool bIntervalOneSecond(UINT64& u64StartUS)
    {
        UINT64 u64EndUs = HCHead::getCurrentTimestampUs();
        UINT64 u64Int= u64EndUs - u64StartUS ;

        if(u64Int  >=1000000 )
        {
            u64StartUS = u64EndUs;
            return true;
        }
        return false;
    }


private:
    bool setLidarPara(const char* chLidarModel);
    void lidarReConnect();
    void processMain();
    bool processData();
    bool calIDX2(char* ch,int iLen);
    bool calIDX1(char* ch,int iLen);
    bool calStartInfo(char* ch,int iLen);
    bool calMCUFrame(char* ch,int iLen);
    bool getDevID(std::vector<UCHAR>& lstBuff);
    bool getStartInfo(std::vector<UCHAR>& lstBuff);
    bool getPointCloud(std::vector<UCHAR>& lstBuff);
    bool getMCUCmd(std::vector<UCHAR>& lstBuff);

    bool parserRangeEX(LstPointCloud &resultRange,const char * chBuff, int iIndex, int in_numData,int iPointSize);
    void compensate(double &angle, UINT16 &dist, const double theta_d, const double baseline_mm);
    bool checkDataCal(std::vector<UCHAR>& lstBuff, int iIndex);
    void checkReadPacketData();
    void sendGetIDInfoSignal(bool bGetID);
    void sendGetFactoryInfoSignal(bool bGetFact);

    void checkInvalidFPS(int iFPS);
    void checkInvalidLowSpeed(UINT16 u16Speed);
    void checkInvalidHighSpeed(UINT16 u16Speed);
    void checkSharkBlocked();
    void checkSharkInvalidPoints(tsPointCloud& sData);
    void checkHadInitSuccess(bool bTimeout);


    void grabScanDataWithLoop(std::list<tsNodeInfo>& nodeList, tsNodeInfo* nodebuffer, size_t buffLen);
    void pushValidData2Buffer(tsNodeInfo& nodeInfo, int index, tsNodeInfo* nodebuffer, int len);
    bool checkBufferIsSorted(tsNodeInfo* nodebuffer, int len);
    void grabScanDataWithNoLoop(std::list<tsNodeInfo>& nodeList, tsNodeInfo* nodebuffer, size_t buffLen);
    void grabScanData(tsNodeInfo * nodebuffer, size_t buffLen, size_t &count);
    void grabScanData(std::list<tsNodeInfo>& dataList);
    void convertDistQ2(LstPointCloud& lstG);
    void pushDataWithNoLoopMode(tsNodeInfo& node_cur);
    void pushDataWithLoopMode(bool& isTurn, std::list<tsNodeInfo>& loopNodeList, tsNodeInfo& node_cur);
    void checkInvalidLidarNumber(int validNumber);
    void callbackDistQ2();

};

#endif // HCLIDAR_H
