#include "lidar.h"

Dev * Dev::dev_ptr = 0;
unsigned long long Dev::g_data_stamp = 0;

Dev::Dev()
{
}

Dev::~Dev()
{
}

void Dev::ParseData_serial(rangedata *dataPack, int &fps)
{
    parse(rcvbuffer_, data_num_per_pack_, dataPack);

    fps = data_rate_hz_;
}

void Dev::parse(unsigned char *in_buffer, const int in_numData,rangedata *out_dataPack)
{
    // The size of each data is 3 bytes
    const int data_size = 3;
    // in_numData: sampling number in every data package
    int id_start = 2;
    float FA = (in_buffer[id_start+1] - 0xA0 + in_buffer[id_start]/256.0) * 4;

    int id_LA_start = id_start + in_numData * data_size + 2;
    float LA = (in_buffer[id_LA_start+1] - 0xA0 + in_buffer[id_LA_start]/256.0) * 4;

    if (LA < FA) { LA += 360; }

    float dAngle = (LA - FA) / (in_numData - 1);        // angle info for each sampling

    unsigned char data[3];
    int pre_bytes = 4;          // 4 bytes before sampling data in each data package
    // calc speed (rpm)
    unsigned int speed = (in_buffer[1] << 8 | in_buffer[0]) / 64;
    double angle_offset = 21;
    for (int i=0; i<in_numData; ++i)
    {
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
        if (0 == out_dataPack[i].dist)
        {
            out_dataPack[i].flag = true;
        }
    }
}

void Dev::initialize()
{
    dev_ptr = this;
    is_scanning_ = true;
    thread_ = std::thread(startScan);
}

void Dev::startScan()
{
    dev_ptr->cacheScanData();
}

void Dev::cacheScanData()
{
    int fps = 0, rtn = 0;
    const int data_num = 8;
    rangedata dataPack[data_num];

    double speed_sum = 0;
    float angle_cur = 0;
    double m = 20;
    while (is_scanning_)
    {
        rtn = ReadData_serial();                    // Read data from serial port
        if (rtn <= 0)
        {
            printf("Warning: Read serial data time out!\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        ParseData_serial(dataPack, fps);        // Parse data

        for (int i = 0; i < data_num; ++i)
        {
            node_info node_cur;
            //angle_pre = angle_cur;
            angle_cur = dataPack[i].angle;
            
            // Compensate angle & dist
            unsigned int dist = dataPack[i].dist;
            if (0 != dist && !dataPack[i].flag)
            {
                
                double angle_correct_cur = angle_cur + 80.0 - atan(dist / m) / PI * 180;
                if (angle_correct_cur > 360)
                {
                    angle_correct_cur -= 360;
                } else if (angle_correct_cur < 0)
                {
                    angle_correct_cur += 360;
                }
                dataPack[i].angle = angle_correct_cur;
                dataPack[i].dist = sqrt(m * m + dist * dist);
            }

            if (!dataPack[i].flag)
            {
                // valid data
                node_cur.distance_q2 = (uint16_t)(dataPack[i].dist * 4);                    // dist (Q2)
            } else
            {
                // invalid data
                node_cur.distance_q2 = 0;
            }

            node_cur.angle_q6_checkbit = (uint16_t)(dataPack[i].angle * 64);    // angle (Q6)
            //node_buffer_temp[node_idx] = node_cur;
            speed_sum += dataPack[i].speed;
            {
                std::lock_guard<std::mutex> guard(mutex_);
                if (m_nodeList.size() >= 2048)
                {
                    m_nodeList.pop_front();
                }

                m_nodeList.push_back(node_cur);

                this->speed_mean_ = speed_sum / m_nodeList.size();
            }

            //printf("%.3f\n", node_cur.angle_q6_checkbit /  64.0f);
        }
    } // while()
}

unsigned long long Dev::grabScanData(node_info * nodebuffer, size_t &count)
{
    std::list<node_info> nodeList;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        nodeList.swap(m_nodeList);
    }

    count = nodeList.size();
    if (count == 0)
    {
        return m_lastErrorCode;
    }

    int i = 0;
    for (auto it = nodeList.begin(); it != nodeList.end(); ++it, ++i)
    {
        nodebuffer[i] = *it;
    }
    //count = node_info_count_;
    //memset(nodebuffer, 0, kBufferSize * sizeof(node_info));
    //memcpy(nodebuffer, node_info_buffer_, node_info_count_ * sizeof(node_info));
    
    return 0;
}

unsigned long long Dev::GetScanData(node_info * nodebuffer, size_t &count, const double angle_vel /*= 0*/, const bool is_reverse /*= true*/)
{
    const unsigned long long data_stamp = grabScanData(nodebuffer, count);
    const double angle_vel_cur = -angle_vel;                       // reverse angle
    const double scan_time = 60.0 / speed_mean_;            // single-turn scan time
    // Apply angle offset to each data package
    const double angle_delta = (angle_vel_cur / PI * 180.0) * scan_time / count;       // degree
    for (int i = 0; i < count; i++)
    {
        double angle_cur = nodebuffer[i].angle_q6_checkbit;
        angle_cur = angle_cur / 64.0 + angle_delta * i;
        if (angle_cur > 360)
        {
            angle_cur -= 360;
        } else if (angle_cur < 0)
        {
            angle_cur += 360;
        }

        if (is_reverse)
        {
            angle_cur = 360 - angle_cur;
        }

        nodebuffer[i].angle_q6_checkbit = angle_cur * 64;
    }

    return data_stamp;
}

int Dev::ReadData_serial(unsigned int TimeOut_ms)
{
    if (data_timer_.Duation_ms() >= 1000)
    {
        data_rate_hz_ = data_num_total_;
        data_num_total_ = 0;
        data_timer_.InitTimer();
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
            if (rtn == -5)
            {
                m_lastErrorCode = ERR_SERIAL_INVALID_HANDLE;
            }
            else if (rtn == -1)
            {
                m_lastErrorCode = ERR_SERIAL_SETCOMMTIMEOUTS_FAILED;
            }
            else if (rtn == -2)
            {
                m_lastErrorCode = ERR_SERIAL_READFILE_FAILED;
            }
            else if (rtn == 0)
            {
                m_lastErrorCode = ERR_SERIAL_READFILE_ZERO;
            }
            return rtn;
        }

        ++find_times;
        if (find_times >= 100)
        {
            // Find data header time out!
            m_lastErrorCode = ERR_FIND_HEAD_TIMEOUT;
            return -1;
        }
    }

    BYTE buff[128] = {0};
    int buff_length_target = 34;
    const int data_num_target = 8;
    rtn = serial_.readChars(buff, buff_length_target, TimeOut_ms);
    if (rtn != buff_length_target)
    {
        if (rtn == -5)
        {
            m_lastErrorCode = ERR_SERIAL_INVALID_HANDLE;
        }
        else if (rtn == -1)
        {
            m_lastErrorCode = ERR_SERIAL_SETCOMMTIMEOUTS_FAILED;
        }
        else if (rtn == -2)
        {
            m_lastErrorCode = ERR_SERIAL_READFILE_FAILED;
        }
        else if (rtn == 0)
        {
            m_lastErrorCode = ERR_SERIAL_READFILE_ZERO;
        }
        else
        {
            m_lastErrorCode = ERR_GETPACKAGE_FAILED;
        }

        return rtn;
    }

    bool is_equal = CheckData(buff);
    if (!is_equal || data_num_target != buff[1])
    {
        m_lastErrorCode = ERR_CHECKDATA_INVALID;
        return -1;
    }

    unsigned int counts = data_num_target * 3 + 8;
    memset(rcvbuffer_, 0, 2048);
    memcpy(rcvbuffer_, buff + numDataByte, counts);
    data_num_per_pack_ = data_num_target;
    data_num_total_ += data_num_per_pack_;
    return data_num_target;
}

bool Dev::CheckData(BYTE *buffer)
{
    int temp[17] = {0};
    temp[0] = 0x55 + (0xAA << 8);
    for (int i = 1; i < 17; i++)
    {
        temp[i] = buffer[2 * (i-1)] + (buffer[2 * (i-1) + 1] << 8);
    }

    int chk32 = 0;
    for (int i = 0; i < 17; i++)
    {
        chk32 = (chk32 << 1) + temp[i];
    }

    int checksum_target = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum_target = checksum_target & 0x7FFF;
    int checksum_cur = buffer[32] + (buffer[33] << 8);
    bool is_equal = (checksum_target == checksum_cur);
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