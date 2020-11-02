#include "rOc_serial.h"
#include "TimeOut.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <list>
#include <atomic>

typedef unsigned char BYTE;

#define PI 3.141592653589793

struct rangedata
{
    bool flag;                          // 0->valid, 1->invalid
    float angle;                       // degree
    unsigned int dist;            // millimeter
    unsigned int speed;
    unsigned int syn_quality;     //亮度信息  2020-10-29
    rangedata() :
        flag(false),
        angle(0.),
        dist(0),
        speed(0),
        syn_quality(0){}
};

struct node_info
{
    uint8_t    sync_quality;
    uint16_t   angle_q6_checkbit;
    uint16_t   distance_q2;
    uint64_t   isValid;// 1有效  0 无效
    bool operator<(const node_info& _Left) const
    {
        if (_Left.angle_q6_checkbit < this->angle_q6_checkbit)
        {
            return false;
        }

        return true;
    }
};

enum LiDarErrorCode
{
    LIDAR_SUCCESS = 0,
    //软件和串口错误码
    ERR_SERIAL_INVALID_HANDLE = -1001,//串口句柄为空
    ERR_SERIAL_SETCOMMTIMEOUTS_FAILED = -1002,//设置串口读取超时函数失败
    ERR_SERIAL_READFILE_FAILED = -1003,//读取串口失败
    ERR_SERIAL_READFILE_ZERO = -1004,//串口读取到零字节
    ERR_FIND_HEAD_TIMEOUT = -1005,//查找包头失败
    ERR_CHECKDATA_INVALID = -1006,//校验和失败
    ERR_GETPACKAGE_FAILED = -1007,//获取包失败
    ERR_DATABYTELENGTH_INVALID = -1008,//一个雷达数据的位数异常
    ERR_RECEIVE_BUFFER_SMALL = -1009,//传入的buffer参数大小太小

    //雷达内部错误码
    ERR_LIDAR_FPS_INVALID = -3001,//fps异常
    ERR_LIDAR_SPEED_LOW = -3002,//速度低速
    ERR_LIDAR_SPEED_HIGH = -3003,//速度高速
    ERR_LIDAR_NUMBER_INVALID = -3004,//有效点数异常
};

#define PACKAGESIZE 128

class Dev
{
public:
    Dev();
    ~Dev();

	void initialize();
    
	unsigned long long GetScanData(node_info * nodebuffer, size_t &count, const double angle_vel = 0, const bool is_reverse = true);

   

    ///直接解析数据的接口  步骤：打开串口，读取数据，解析数据，关闭串口
    int openSerial(char* port, unsigned int baud);
    void closeSerial();
    int ReadData_serial(unsigned int TimeOut_ms = 60);
    int ParseData_serial(rangedata* dataPack, int& fps);
    ///--------------------直接解析数据的接口
    
    //获取错误码
    int GetLastErrCode()
    {
        int err = m_lastErrorCode;
        if (m_lastErrorCode != 0)
            m_lastErrorCode = 0;
        return err;
    }

private:

	//打包数据的所有接口 步骤：初始化，获取数据，结束
	//isGetLoopData：true,表示获取一圈数据再输出；false,表示数据实时输出。默认是一圈输出
   //返回值:0，表示成功  非零表示失败
	int Initialize(char* port, unsigned int baud, bool isGetLoopData = true);
	//非阻塞  返回值:0，表示成功; 非零表示失败

	//需要判断count是否获取到数据
	void GetScanData(node_info * nodebuffer, size_t buffLen, size_t &count);

	int Uninit();
	//---------------------------------------打包数据的所有接口

    //isGetLoopData is false
    void PushDataWithNoLoopMode(node_info& node_cur);

    //isGetLoopData is true
    void PushDataWithLoopMode(bool& isTurn, std::list<node_info>& loopNodeList, node_info& node_cur);

    void cacheScanData();
    
    bool CheckData(BYTE* buffer, int len);
    int readByte(char* out_byte, unsigned int TimeOut_ms);
    
    int parse(BYTE* in_buffer, const int in_numData, rangedata* out_dataPack);
    void grabScanData(node_info* nodebuffer, size_t buffLen, size_t& count);

    void GrabScanDataWithNoLoop(std::list<node_info>& nodeList, node_info* nodebuffer, size_t buffLen);

    int GetDataByteLength(char cInfo);

    void GrabScanDataWithLoop(std::list<node_info>& nodeList, node_info* nodebuffer, size_t buffLen);
    void PushValidData2Buffer(node_info& nodeInfo, int index, node_info* nodebuffer, int len);
    bool CheckBufferIsSorted(node_info* nodebuffer, int len);

    void SetReadCharsError(int errCode);

    void CheckInvalidLidarNumber(int validNumber);
    void CheckInvalidFPS(int validNumber);

    void CheckInvalidLowSpeed(unsigned int speed);
    void CheckInvalidHighSpeed(unsigned int speed);
    
    int data_num_per_pack_;
    int unitDataSize_;

    int data_num_total_;
    int data_rate_hz_;
    TimeOut data_timer_;
    rOc_serial serial_;
    BYTE rcvbuffer_[PACKAGESIZE];
    
    std::list<node_info> m_nodeList;
    int m_lastErrorCode;
    
    
    bool is_scanning_;
    
    std::thread thread_;
    std::mutex mutex_;

    bool isGetLoopData_;

    

    int invalidNumberContinue_ = 0;
    int invalidFPSSecond_ = 0;

    int64_t startTimeLowSpeed_ = 0;
    int64_t startTimeHighSpeed_ = 0;

    int64_t GetTimeStamp();

};
