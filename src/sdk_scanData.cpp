#include "base/lidar.h"
#include <stdio.h>

#include <fstream>

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

    Dev device;
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
    printf("press any key exited\n");

    while (!isExit)
    {
        node_info nodebuffer[2048];
        size_t count = 2048;
        
        device.GetScanData(nodebuffer, count, angle_vel, is_reverse);
        
        int errcode = device.GetLastErrCode();
        if (errcode != LIDAR_SUCCESS)
        {
            printf("errcode:%d\n", errcode);
        }
            
        if (count == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        for (size_t i = 0;i < count; ++i)
        {
            
            printf("angle:%d, distance:%d, sync_quality:%d, isValid:%d, speed:%d\n",
                nodebuffer[i].angle_q6_checkbit, 
                nodebuffer[i].distance_q2, 
                nodebuffer[i].sync_quality,
                nodebuffer[i].isValid,
			    nodebuffer[i].speed);
           
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    device.closeSerial();

    return 0;
}
