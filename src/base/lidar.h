#include "rOc_serial.h"
#include "TimeOut.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <list>

typedef unsigned char BYTE;

#define PI 3.141592653589793

struct rangedata
{
    bool flag;                          // 0->valid, 1->invalid
    float angle;                       // degree
    unsigned int dist;            // millimeter
    unsigned int speed;
    rangedata() :
        flag(false),
        angle(0.),
        dist(0),
        speed(0) {}
};

struct node_info
{
    uint8_t    sync_quality;
    uint16_t   angle_q6_checkbit;
    uint16_t   distance_q2;
    uint64_t   stamp;
};

enum LiDarErrorCode
{
    LIDAR_SUCCESS = 0,
    ERR_SERIAL_INVALID_HANDLE = 1,
    ERR_SERIAL_SETCOMMTIMEOUTS_FAILED = 2,
    ERR_SERIAL_READFILE_FAILED = 3,
    ERR_SERIAL_READFILE_ZERO = 4,
    ERR_FIND_HEAD_TIMEOUT = 5,
    ERR_CHECKDATA_INVALID = 6,
    ERR_GETPACKAGE_FAILED = 7,
};

class Dev
{
public:
    Dev();
    ~Dev();

    int ReadData_serial(unsigned int TimeOut_ms = 60);
    void ParseData_serial(rangedata *dataPack, int &fps);
    bool CheckData(BYTE *buffer);
    int readByte(char *out_byte, unsigned int TimeOut_ms);
    int openSerial(char *port, unsigned int baud);
    void closeSerial();
    void parse(BYTE *in_buffer, const int in_numData, rangedata *out_dataPack);
    void initialize();
    static void startScan();
    void cacheScanData();
    unsigned long long grabScanData(node_info * nodebuffer, size_t &count);
    unsigned long long GetScanData(node_info * nodebuffer, size_t &count, const double angle_vel = 0, const bool is_reverse = true);

    static Dev *dev_ptr;

private:
    const static unsigned int kBufferSize = 2048;
    static unsigned long long g_data_stamp;
    int data_num_per_pack_;
    int data_num_total_;
    int data_rate_hz_;
    TimeOut data_timer_;
    rOc_serial serial_;
    BYTE rcvbuffer_[kBufferSize];
    node_info node_info_buffer_[kBufferSize];
    std::list<node_info> m_nodeList;
    int m_lastErrorCode;
    unsigned int node_info_count_;
    double speed_mean_;
    bool is_scanning_;
    std::thread thread_;
    std::mutex mutex_;
};

