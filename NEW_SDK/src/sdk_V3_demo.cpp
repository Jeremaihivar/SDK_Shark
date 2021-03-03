
#include "base/hclidar.h"

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
    const int data_num = 8;
    //rangedata dataPack[data_num];
    HCLidar device;
    int fps = 0, rtn = 0;

    bool bPollMode = true;
    bool bDistQ2 = false;
    bool bLoop = false;

    std::cout << "Main: SDK verion=" << device.getSDKVersion() << std::endl;

    auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
    device.setCallBackFunErrorCode(funErrorCode);

    auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
    device.setCallBackFunSecondInfo(funSecondInfo);

    if(!bPollMode)//call back
    {
        auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1);
        device.setCallBackFunPointCloud(funPointCloud);

        auto funDistQ2 = std::bind(sdkCallBackFunDistQ2, std::placeholders::_1);
        device.setCallBackFunDistQ2(funDistQ2);
    }

    tsSDKPara sPara;
    sPara.iNoDataMS = 1000;
    sPara.iDisconnectMS = 3000;
    sPara.iFPSContinueMS = 5000;
    sPara.iSpeedContinueMS = 3500;
    sPara.iCoverContinueMS = 3500;
    sPara.iBlockContinueMS = 3500;
    sPara.iCoverPoints = 100;
    sPara.iPollBuffSize = 1000;
    sPara.iCallbackBuffSize = 50;

	int iBaud = 115200;
	int iReadTimeoutms = 10;//
    // ##### 1. Open serial port using valid COM id #####
#ifdef _WIN32
    rtn = device.initialize("//./com3", "X2M", iBaud, iReadTimeoutms, bDistQ2,bLoop, bPollMode,sPara) ;                     // For windows OS
#else
    rtn = device.initialize("/dev/ttyPort1", "X2M", iBaud, iReadTimeoutms, bDistQ2,bLoop, bPollMode,sPara) ;               // For Linux OS
#endif

    if (rtn != 1)
    {
		int iErrorCode = device.getLastErrCode();
		if (iErrorCode == ERR_SERIAL_INVALID_HANDLE || iErrorCode == ERR_SERIAL_READFILE_FAILED)
		{
			device.unInit();
			std::cout << "Main: Init sdk failed!\n" << std::endl;
			getchar();
			exit(0);
			return 0;
		}
        
    }


    std::cout << "Main: Lidar ID=" << device.getLidarID() << std::endl;
    std::cout << "Main: Factory Info:" << device.getFactoryInfo() << std::endl;
    std::cout << "Main: Firmware ver:" << device.getFirmwareVersion() << std::endl;
    std::cout << "Main: Hardware ver:" << device.getHardwareVersion() << std::endl;
	std::cout << "Main: Lidar model:" << device.getLidarModel() << std::endl;

    while (true)
    {

        if(bPollMode)
        {
            if(bDistQ2)
            {
                LstNodeDistQ2 lstG;
                device.getScanData(lstG, false);
                std::cout << "Main: Poll DistQ2 Rx Points=" << lstG.size() <<std::endl;
                for(auto sInfo : lstG)
                {
                    //std::cout << "Main: Angle=" << (double)sInfo.angle_q6_checkbit/64.0f  << ",Dist=" << sInfo.distance_q2/4 << std::endl;
                }
            }
            else
            {
                LstPointCloud lstG;
                device.getRxPointClouds(lstG);
                std::cout << "Main: Poll Rx Points=" << lstG.size() <<std::endl;
                for(auto sInfo : lstG)
                {
                    //std::cout << "Main: Angle=" << sInfo.dAngle  << ",AngleRaw=" << sInfo.dAngleRaw << ",Dist=" << sInfo.u16Dist << std::endl;
                }
            }
        }
        int iSDKStatus = device.getSDKStatus();
        //std::cout << "Main: SDK Status=" << iSDKStatus <<std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::this_thread::yield();
        //printf("\n");
    }

    // ##### 4. Close serial #####
    device.unInit();
    return 0;

}
