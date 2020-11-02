#include "base/lidar.h"
#include <stdio.h>

Dev device;

int main(int argc, char **argv)
{
    int com_id = 0;             // Open serial port using valid COM id
#ifdef _WIN32
    com_id = 3;
#else
    com_id = 0;
#endif
    if (argc >=2)
    {
        com_id = atoi(argv[1]);
    }

    char buff[32];
#ifdef _WIN32
    sprintf(buff, "//./com%d", com_id);
#else
    sprintf(buff, "/dev/ttyUSB%d", com_id);
#endif

    int rtn = device.openSerial(buff, 115200);                       // For windows OS
    if (rtn != 1)
    {
        printf("Error: Unable to open serial port!\n");
        getchar();
        exit(0);
    }

    bool isExit = false;
    //press any key exit
    auto threadExitFunc = [&] {
        getchar();
        isExit = true;
    };

    std::thread thr(threadExitFunc);
    thr.detach();

    device.initialize();
    double angle_vel = 0;       // angle velocity (radian)
    bool is_reverse = true;     // whether reverse the data packages
    TimeOut timer;
    unsigned long long data_stamp_old = 0, data_stamp_new = 0;
    while (!isExit)
    {
        node_info nodebuffer[2048];
        size_t count = 0;
        data_stamp_new = device.GetScanData(nodebuffer, count, angle_vel, is_reverse);
        for (int i = 0;i < count; ++i)
        {
            // Comment the print sentence if necessary
            // FD->First Degree, LD->Last Degree, TC: Time Cost
            printf("data size=%d, FD=[%.2f], TC=%.3fms, stamp: %d\n",
                count,
                nodebuffer[i].angle_q6_checkbit / 64.0f,
                timer.Duation_ms(),
				nodebuffer[i].stamp);
            //data_stamp_old = data_stamp_new;
            timer.InitTimer();
            // Perform other data processing...

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    device.closeSerial();
    return 0;
}
