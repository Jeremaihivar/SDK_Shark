#include "lidar.h"
#include <vector>
#include <set>
#include <chrono>
#include <algorithm>

//大雷达参数配置（2080Hz）

#define CENTER_BASE_ANGLE 21.0  //线性补偿角度，小雷达30度
#define CENTER_BASE_LINE 20.0   //非线性补偿时的基准距离

#define FPS_MAX 2160    //最大fps，小雷达3150
#define FPS_MIN 2000    //最小fps，小雷达3000
#define FPS_CONTINUE_SECOND 5    //fps异常的持续时间

#define SPEED_MAX 400     //最大速度
#define SPEED_MIN 240     //最小速度
#define SPEED_CONTINUE_SECOND 5   //最大速度或者最小速度异常的持续时间

#define VALID_NUMBER_COUNT 50    //一圈有效点数的阈值
#define NUMBER_CONTINUE_CIRCLE 50    //一圈有效点数异常的持续圈数

//在判断是否是一圈出现异常情况时【即角度未出现反转】，该参数生效
#define CIRCLE_NUMBER_MAX 415  //注意:2000转设置415  3000转设置515

Dev::Dev()
{
}

Dev::~Dev()
{
}

int Dev::ParseData_serial(rangedata *dataPack, int &fps)
{
    parse(rcvbuffer_, data_num_per_pack_, dataPack);

    fps = data_rate_hz_;

    return 0;
}

int Dev::GetDataByteLength(char cInfo)
{
    switch(cInfo)
    {
    case 0x02:
        return 2;
    case 0x03:
        return 3;
    case 0x07:
        return 4;
    default:
        return -1;
    }
}

int Dev::parse(BYTE* in_buffer, const int in_numData,rangedata *out_dataPack)
{
    // The size of each data is 3 bytes
    int data_size = unitDataSize_;

	//printf("data_size:%d\n", data_size);
    
    // in_numData: sampling number in every data package
    int id_start = 2;
    float FA = (in_buffer[id_start+1] - 0xA0 + in_buffer[id_start]/256.0) * 4;

    int id_LA_start = id_start + in_numData * data_size + 2;
    float LA = (in_buffer[id_LA_start+1] - 0xA0 + in_buffer[id_LA_start]/256.0) * 4;

    if (LA < FA) { LA += 360; }

    int len = in_numData - 1;
    if (len == 0)
    {
        return -1;
    }

    float dAngle = (LA - FA) / (in_numData - 1);        // angle info for each sampling

    unsigned char *data = new unsigned char[data_size];
    int pre_bytes = 4;          // 4 bytes before sampling data in each data package
    // calc speed (rpm)
    unsigned int speed = (in_buffer[1] << 8 | in_buffer[0]) / 64;

    CheckInvalidLowSpeed(speed);
    CheckInvalidHighSpeed(speed);

    double angle_offset = CENTER_BASE_ANGLE;
    for (int i=0; i < in_numData; ++i)
    {
        //printf("%.4f\n", FA + dAngle * i);
        double angle_cur = FA + dAngle * i + angle_offset;
        if (angle_cur > 360)
        {
            angle_cur -= 360;
        }

        out_dataPack[i].angle = angle_cur;

        memcpy(data, in_buffer + pre_bytes + i * data_size, sizeof(unsigned char) * data_size);
        out_dataPack[i].flag = (data[1] >> 7) & 0x01;
        out_dataPack[i].dist = ((data[1] & 0x3F) << 8) | data[0];
        out_dataPack[i].speed = speed;
        if (data_size == 3)
        {
            out_dataPack[i].syn_quality = data[2];
        }
        else if (data_size == 4)
        {
            out_dataPack[i].syn_quality = (data[3]<< 8) | data[2];
			//printf("%d\n", out_dataPack[i].syn_quality);
        }
        else
        {
            out_dataPack[i].syn_quality = 0;
        }

        if (0 == out_dataPack[i].dist)
        {
            out_dataPack[i].flag = true;
        }
    }

    delete[] data;
    return 0;
}
//isGetLoopData 是否输出一圈的数据
int Dev::Initialize(char* port, unsigned int baud, bool isGetLoopData /*false*/)
{
    if (openSerial(port, baud) != 1)
    {
        printf("open serial failed.\n");
        return -1;
    }

    is_scanning_ = true;
    isGetLoopData_ = isGetLoopData;

    thread_ = std::thread(&Dev::cacheScanData, this);

    return 0;
}

void  Dev::initialize()
{
    is_scanning_ = true;
    isGetLoopData_ = false;

    thread_ = std::thread(&Dev::cacheScanData, this);

}

unsigned long long Dev::GetScanData(node_info * nodebuffer, size_t &count, const double angle_vel /*= 0*/, const bool is_reverse /*= true*/)
{
    int bufferLen = count;
    if (bufferLen <= 0)
    {
        bufferLen = 2048;
    }

    std::list<node_info> nodeList;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        nodeList.swap(m_nodeList);
    }

    count = nodeList.size();
    if (count == 0)
    {
        return 0;
    }

    GrabScanDataWithNoLoop(nodeList, nodebuffer, bufferLen);

    for (int i = 0; i < count; i++)
    {
        double angle_cur = nodebuffer[i].angle_q6_checkbit;
        angle_cur = angle_cur / 64.0;
        if (angle_cur > 360)
        {
            angle_cur -= 360;
        }
        else if (angle_cur < 0)
        {
            angle_cur += 360;
        }

        if (is_reverse)
        {
            angle_cur = 360 - angle_cur;
        }

        nodebuffer[i].angle_q6_checkbit = angle_cur * 64;
    }

    return count;

}

void Dev::PushDataWithNoLoopMode(node_info& node_cur)
{
    std::lock_guard<std::mutex> guard(mutex_);
    if (m_nodeList.size() >= 2048)
    {
        m_nodeList.pop_front();
    }

    m_nodeList.push_back(node_cur);
}

//isGetLoopData is false
void Dev::PushDataWithLoopMode(bool& isTurn, std::list<node_info>& loopNodeList, node_info& node_cur)
{
    if (isTurn || (loopNodeList.size() > CIRCLE_NUMBER_MAX))
    {
        
        {
            std::lock_guard<std::mutex> guard(mutex_);
            m_nodeList.clear();
            m_nodeList.swap(loopNodeList);
        }

        isTurn = false;
    }

    loopNodeList.push_back(node_cur);
}

void Dev::cacheScanData()
{
    int fps = 0, rtn = 0;
    
    double angle_cur = 0;
    //double m = 20;
    double preAngle = 0;
    bool isTurn = false;
    std::list<node_info> loopNodeList;
    bool isFirsLoop = false;

    int validNumber = 0;

    while (is_scanning_)
    {
        rtn = ReadData_serial();                    // Read data from serial port
        if (rtn <= 0)
        {
            printf("Warning: Read serial data time out! %d\n", rtn);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        rangedata* dataPack = new rangedata[data_num_per_pack_];
        ParseData_serial(dataPack, fps);        // Parse data

        for (int i = 0; i < data_num_per_pack_; ++i)
        {
            node_info node_cur;
            angle_cur = dataPack[i].angle;
            // Compensate angle & dist
            unsigned int dist = dataPack[i].dist;
            if (0 != dist && !dataPack[i].flag)//&& !dataPack[i].flag
            {
                //printf("%.4f\n", atan(dist / m));
                double angle_correct_cur = angle_cur + 80.0 - atan(dist / CENTER_BASE_LINE) / PI * 180;
                if (angle_correct_cur > 360)
                {
                    angle_correct_cur -= 360;
                }
                else if (angle_correct_cur < 0)
                {
                    angle_correct_cur += 360;
                }

                dataPack[i].angle = angle_correct_cur;
                dataPack[i].dist = sqrt(CENTER_BASE_LINE * CENTER_BASE_LINE + 1.0 * dist * dist);

            }

            if (!dataPack[i].flag)
            {
                // valid data
                node_cur.distance_q2 = (uint16_t)(dataPack[i].dist * 4);                    // dist (Q2)

                node_cur.isValid = 1;
                ++validNumber;
            }
            else
            {
                // invalid data
                node_cur.distance_q2 = 0;
                node_cur.isValid = 0;
                //dataPack[i].angle = 0;
            }

            node_cur.angle_q6_checkbit = (uint16_t)(dataPack[i].angle * 64);
            node_cur.sync_quality = (uint16_t)dataPack[i].syn_quality;
			node_cur.speed = dataPack[i].speed;
            if (preAngle > angle_cur)
            {
                isTurn = true;
                isFirsLoop = true;

                CheckInvalidLidarNumber(validNumber);
                validNumber = 0;
            }
            //printf("%.4f,%.4f\n", preAngle, angle_cur);
            if (!isFirsLoop)
            {
                preAngle = angle_cur;
                continue;
            }

            if (!isGetLoopData_)
            {
                PushDataWithNoLoopMode(node_cur);
            }
            else
            {
                PushDataWithLoopMode(isTurn, loopNodeList, node_cur);
            }

            preAngle = angle_cur;
            //printf("%.3f\n", node_cur.angle_q6_checkbit /  64.0f);
        }
        
        delete[] dataPack;
    } // while()
}

void Dev::GrabScanDataWithLoop(std::list<node_info>& nodeList, node_info* nodebuffer, size_t buffLen)
{
    int len = nodeList.size();
    
    double perAngle = 360.0 / nodeList.size();
    for (int i = 0; i < len; ++i)
    {
        nodebuffer[i].angle_q6_checkbit = perAngle * (i + 1.0);
        nodebuffer[i].distance_q2 = 0;
        nodebuffer[i].isValid = 0;
        nodebuffer[i].sync_quality = 0;
		nodebuffer[i].speed = 0;
        //printf("%.3f,%d,%d\n", nodebuffer[i].angle_q6_checkbit, nodebuffer[i].distance_q2, nodebuffer[i].isValid);
    }

    for (auto it = nodeList.begin(); it != nodeList.end(); ++it)
    {
        if (it->isValid == 1)
        {
            int index = (it->angle_q6_checkbit / perAngle) + 0.5 - 1;
            //printf("index:%d\n", index);
            PushValidData2Buffer(*it, index, nodebuffer, len);
        }
    }

    if (!CheckBufferIsSorted(nodebuffer, len))
    {
        //printf("data is not sorted.\n");
        std::stable_sort(nodebuffer, nodebuffer + len);
    }
}

void Dev::PushValidData2Buffer(node_info& nodeInfo, int index, node_info* nodebuffer, int len)
{
    bool isExit = false;
    while (!isExit)
    {
        //printf("valid:%d\n", nodebuffer[index % len].isValid);
        index = index % len;
        if (nodebuffer[index].isValid == 0)
        {
            //nodebuffer[index].isValid = 1;
            //nodebuffer[index].angle_q6_checkbit = nodeInfo.angle_q6_checkbit;
            //nodebuffer[index].distance_q2 = nodeInfo.distance_q2;
            //nodebuffer[index].sync_quality = nodeInfo.sync_quality;
			nodebuffer[index] = nodeInfo;
            isExit = true;
        }

        ++index;
    };
}

bool Dev::CheckBufferIsSorted(node_info* nodebuffer, int len)
{
    if (len < 2)
    {
        return true;
    }

    for (int i = 1; i < len; ++i)
    {
        if (nodebuffer[i - 1].angle_q6_checkbit > nodebuffer[i].angle_q6_checkbit)
        {
            return false;
        }
    }

    return true;
}

void Dev::GrabScanDataWithNoLoop(std::list<node_info>& nodeList, node_info* nodebuffer, size_t buffLen)
{
    size_t index = 0;
    size_t indexCount = 0;
    size_t startIndex = 0;
    size_t nodeLen = nodeList.size();
    if (nodeLen > buffLen)
    {
        startIndex = nodeLen - buffLen;
    }

    for (auto it = nodeList.begin(); it != nodeList.end(); ++it, ++indexCount)
    {
        if (indexCount >= startIndex)
        {
            nodebuffer[index] = *it;
            ++index;
        }
    }
}

void Dev::grabScanData(node_info * nodebuffer, size_t buffLen, size_t &count)
{
    std::list<node_info> nodeList;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        nodeList.swap(m_nodeList);
    }

    count = nodeList.size();
    if (count == 0)
    {
        return;
    }

    if (isGetLoopData_)
    {
        if (count > buffLen)
        {
            printf("buffer is too small");
            m_lastErrorCode = ERR_RECEIVE_BUFFER_SMALL;
            count = 0;
            return;
        }

        GrabScanDataWithLoop(nodeList, nodebuffer, buffLen);
    }
    else
    {
        GrabScanDataWithNoLoop(nodeList, nodebuffer, buffLen);
    }

}

void Dev::GetScanData(node_info * nodebuffer, size_t buffLen, size_t &count)
{
    if (nodebuffer == nullptr || buffLen == 0)
    {
        count = 0;
        return;
    }

    grabScanData(nodebuffer, buffLen, count);
}

void Dev::SetReadCharsError(int errCode)
{
    if (errCode == -5)
    {
        m_lastErrorCode = ERR_SERIAL_INVALID_HANDLE;
    }
    else if (errCode == -1)
    {
        m_lastErrorCode = ERR_SERIAL_SETCOMMTIMEOUTS_FAILED;
    }
    else if (errCode == -2)
    {
        m_lastErrorCode = ERR_SERIAL_READFILE_FAILED;
    }
    else if (errCode == 0)
    {
        m_lastErrorCode = ERR_SERIAL_READFILE_ZERO;
    }
    else if (errCode == -3)
    {
        m_lastErrorCode = ERR_GETPACKAGE_FAILED;
    }
}

int Dev::ReadData_serial(unsigned int TimeOut_ms)
{
    if (data_timer_.Duation_ms() >= 1000)
    {
        data_rate_hz_ = data_num_total_;
        data_num_total_ = 0;
        data_timer_.InitTimer();

        CheckInvalidFPS(data_rate_hz_);
    }

    char rtn;                                                // Returned value from Read
    bool found= false;
    int find_times = 0;
    int numDataByte;
    BYTE curByte = NULL, preByte = NULL;

    while(! found)
    {
        rtn=serial_.readChar(&curByte, TimeOut_ms);
        if (rtn==1)                                                 // If a byte has been read
        {
            found = (preByte == BYTE(0x55)) && (curByte == BYTE(0xaa));
            numDataByte = 2;
            preByte = curByte;
        }else
        {
            SetReadCharsError(rtn);
            printf("k1\n");
            return m_lastErrorCode;
        }

        ++find_times;
        if (find_times >= 100)
        {
            // Find data header time out!
            m_lastErrorCode = ERR_FIND_HEAD_TIMEOUT;
            printf("k2\n");
            return m_lastErrorCode;
        }
    }

    //printf("read head success\n");
    BYTE buff[PACKAGESIZE] = { 0 };
    rtn = serial_.readChars(buff, 2, TimeOut_ms);
    if (rtn != 2)
    {
        SetReadCharsError(rtn);
        printf("k3\n");
        return -1;
    }
    //Infomation
    unitDataSize_ = GetDataByteLength(buff[0]);
    if (unitDataSize_ < 0)
    {
        m_lastErrorCode = ERR_DATABYTELENGTH_INVALID;
        printf("k4\n");
        return m_lastErrorCode;
    }

    //data number
    data_num_per_pack_ = buff[1];
    //printf("data_num_per_pack_:%d, %d\n", data_num_per_pack_, unitDataSize_);

    int buff_length_target = data_num_per_pack_ * unitDataSize_ + 8;
    
    rtn = serial_.readChars(buff + 2, buff_length_target, TimeOut_ms);
    if (rtn != buff_length_target)
    {
        SetReadCharsError(rtn);
        printf("k5\n");
        return m_lastErrorCode;
    }

    bool is_equal = CheckData(buff, buff_length_target + 2);
    if (!is_equal)
    {
        m_lastErrorCode = ERR_CHECKDATA_INVALID;
        printf("k6\n");
        return m_lastErrorCode;
    }

    memset(rcvbuffer_, 0, PACKAGESIZE);
    memcpy(rcvbuffer_, buff + 2, buff_length_target);
    
    data_num_total_ += data_num_per_pack_;

    return buff_length_target + 4;
}

bool Dev::CheckData(BYTE *buffer, int len)
{
    if (len / 2 == 1)
    {
        printf("it is error.\n");
        return false;
    }

    int* temp = new int[ len / 2 ];//[17] = {0};
    //printf("len:%d\n", len / 2 );
    temp[0] = 0x55 + (0xAA << 8);
    for (int i = 1; i < len / 2 ; i++)
    {
        temp[i] = buffer[2 * (i-1)] + (buffer[2 * (i-1) + 1] << 8);
    }

    int chk32 = 0;
    for (int i = 0; i < len / 2 ; i++)
    {
        chk32 = (chk32 << 1) + temp[i];
    }

    int checksum_target = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum_target = checksum_target & 0x7FFF;
    int checksum_cur = buffer[len - 2] + (buffer[len - 1] << 8);
    bool is_equal = (checksum_target == checksum_cur);

    delete[] temp;
    return is_equal;
}

int Dev::readByte(char *out_byte, unsigned int TimeOut_ms)
{
    int rtn = serial_.readChar(out_byte, TimeOut_ms);
    return rtn;
}

int Dev::openSerial(char *port, unsigned int bauds)
{
    return serial_.openDevice(port, bauds);
}

void Dev::closeSerial()
{
    is_scanning_ = false;
    thread_.join();
    serial_.closeDevice();
}

int Dev::Uninit()
{
   

    closeSerial();

    return 0;
}

void Dev::CheckInvalidLidarNumber(int validNumber)
{
    if (validNumber < VALID_NUMBER_COUNT)
    {
        ++invalidNumberContinue_;
        if (invalidNumberContinue_ >= NUMBER_CONTINUE_CIRCLE)
        {
            m_lastErrorCode = ERR_LIDAR_NUMBER_INVALID;
            invalidNumberContinue_ = 0;
        }
    }
    else
    {
        invalidNumberContinue_ = 0;
    }
}

void Dev::CheckInvalidFPS(int validNumber)
{
    if (validNumber < FPS_MIN || validNumber > FPS_MAX)
    {
        ++invalidFPSSecond_;
        if (invalidFPSSecond_ > FPS_CONTINUE_SECOND)
        {
            printf("FPS:%d\n", validNumber);
            m_lastErrorCode = ERR_LIDAR_FPS_INVALID;
            
            invalidFPSSecond_ = 0;
        }
    }
    else
    {
        invalidFPSSecond_ = 0;
    }
}

int64_t Dev::GetTimeStamp()
{
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::nanoseconds ns = now.time_since_epoch();

    return ns.count();
}

void Dev::CheckInvalidLowSpeed(unsigned int speed)
{
    if (speed < SPEED_MIN)
    {
        if (startTimeLowSpeed_ != 0)
        {
            int64_t endTime = GetTimeStamp();
            if ((endTime - startTimeLowSpeed_) / 1e9 >= SPEED_CONTINUE_SECOND)
            {
                m_lastErrorCode = ERR_LIDAR_SPEED_LOW;
                startTimeLowSpeed_ = 0;
            }
        }
        else
        {
            startTimeLowSpeed_ = GetTimeStamp();
        }
    }
    else
    {
        startTimeLowSpeed_ = 0;
    }
}

void Dev::CheckInvalidHighSpeed(unsigned int speed)
{
    if (speed > SPEED_MAX)
    {
        if (startTimeHighSpeed_ != 0)
        {
            int64_t endTime = GetTimeStamp();
            if ((endTime - startTimeHighSpeed_) / 1e9 >= SPEED_CONTINUE_SECOND)
            {
                m_lastErrorCode = ERR_LIDAR_SPEED_HIGH;
                startTimeHighSpeed_ = 0;
            }
        }
        else
        {
            startTimeHighSpeed_ = GetTimeStamp();
        }
    }
    else
    {
        startTimeHighSpeed_ = 0;
    }
}