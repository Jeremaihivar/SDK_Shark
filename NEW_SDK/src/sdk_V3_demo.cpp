
#include "base/hclidar.h"
#include "LidarTest.h"
#include <stdio.h>


void sdkCallBackFunErrorCode(int iErrorCode)
{
    std::cout << "Main: sdkCallBackFunErrorCode ErrorCode=" << iErrorCode << std::endl;
}

void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
    std::cout << "Main: sdkCallBackFunSecondInfo time=" << sInfo.u64TimeStampS << "s,points=" << sInfo.iNumPerPacket
              << ",GrayBytes=" << sInfo.iGrayBytes << ",FPS=" << sInfo.u64FPS
              << ",speed=" << sInfo.dRMS << ",PPS=" << sInfo.iPacketPerSecond
              << ",valid=" << sInfo.iValid << ",invalid=" << sInfo.iInvalid
              << ",ErrorPacket=" << sInfo.u64ErrorPacketCount <<std::endl;
}

void sdkCallBackFunPointCloud(LstPointCloud lstG)
{
    std::cout << "Main: sdkCallBackFunPointCloud Rx Points=" << lstG.size() <<std::endl;
    for(auto sInfo : lstG)
    {
        //std::cout << "Main: Angle=" << sInfo.dAngle  << ",AngleRaw=" << sInfo.dAngleRaw << ",Dist=" << sInfo.u16Dist << std::endl;
    }
}

void sdkCallBackFunDistQ2(LstNodeDistQ2 lstG)
{
    std::cout << "Main: sdkCallBackFunDistQ2 Rx Points=" << lstG.size() <<std::endl;
    for(auto sInfo : lstG)
    {
        std::cout << "Main: Angle=" << sInfo.angle_q6_checkbit/64.0f  << ",Dist=" << sInfo.distance_q2/4 << std::endl;
    }
}


int main()
{

    HCLidar device;
    int fps = 0, rtn = 0;

    bool bPollMode = true;
    bool bDistQ2 = false;
    bool bLoop = false;

    std::cout << "Main: SDK verion=" << device.getSDKVersion().c_str() << std::endl;

    //auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
    //device.setCallBackFunErrorCode(funErrorCode);

    //auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
    //device.setCallBackFunSecondInfo(funSecondInfo);

    if(!bPollMode)//call back
    {
        auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1);
        device.setCallBackFunPointCloud(funPointCloud);

        auto funDistQ2 = std::bind(sdkCallBackFunDistQ2, std::placeholders::_1);
        device.setCallBackFunDistQ2(funDistQ2);
    }


	int iBaud = 115200;
	int iReadTimeoutms = 10;//
    // ##### 1. Open serial port using valid COM id #####
#ifdef _WIN32
    rtn = device.initialize("//./com3", "X2M", iBaud, iReadTimeoutms, bDistQ2,bLoop, bPollMode) ;                     // For windows OS
#else
    rtn = device.initialize("/dev/ttyPort1", "X2M", iBaud, iReadTimeoutms, bDistQ2,bLoop, bPollMode) ;               // For Linux OS
#endif

    if (rtn != 1)
    {
		device.unInit();
		printf("Main: Init sdk failed!\n");
		getchar();
		exit(0);
		return 0;
        
    }

	//device.unInit();
	//device.unInit();
//#ifdef _WIN32
//	rtn = device.initialize("//./com5", "X2M", iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);                     // For windows OS
//#else
//	rtn = device.initialize("/dev/ttyPort1", "X2M", iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);               // For Linux OS
//#endif


	printf( "Lidar ID=%s\n" , device.getLidarID().c_str());
	printf( "Factory Info:%s\n" , device.getFactoryInfo().c_str());
	printf( "Main: Firmware ver:%s\n", device.getFirmwareVersion().c_str() );
	printf( "Main: Hardware ver:%s\n", device.getHardwareVersion().c_str());
	printf( "Main: Lidar model:%s\n" , device.getLidarModel().c_str() );

    while (true)
    {

        if(bPollMode)
        {
            if(bDistQ2)
            {
                LstNodeDistQ2 lstG;
                device.getScanData(lstG, false);
				printf( "Main: Poll DistQ2 Rx Points=%d\n" ,lstG.size() );
                for(auto sInfo : lstG)
                {
					//printf("Main: Angle=%0.2f,Dist=%d\n" ,(double)sInfo.angle_q6_checkbit/64.0f  , sInfo.distance_q2/4 );
                }
            }
            else
            {
                LstPointCloud lstG;
				if (device.getRxPointClouds(lstG))
				{
					//printf("Main: Poll Rx Points=%d\n",lstG.size());
					for (auto sInfo : lstG)
					{
						printf( "Main: Angle=%0.4f,Dist=%d\n", sInfo.dAngle , sInfo.u16Dist );
					}
				}
				else
				{
					int iError = device.getLastErrCode();
					if (iError != LIDAR_SUCCESS)
					{
						printf( "Main: Poll Rx Points error code=%d\n", iError );
						switch (iError)
						{
						case ERR_SHARK_MOTOR_BLOCKED:
							break;
						case ERR_SHARK_INVALID_POINTS:
							break;
						case ERR_LIDAR_SPEED_LOW:
							break;
						case ERR_LIDAR_SPEED_HIGH:
							break;
						case ERR_DISCONNECTED:
							break;
						case ERR_LIDAR_FPS_INVALID:
							break;
						default:
							break;
						}
					}
				}
				                
            }
        }
        int iSDKStatus = device.getSDKStatus();
		//printf("Main: SDK Status=%d\n" ,iSDKStatus );

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::this_thread::yield();
        //printf("main....\n");
    }

    // ##### 4. Close serial #####
    device.unInit();
    return 0;

}
