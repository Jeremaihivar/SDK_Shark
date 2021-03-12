#include "LidarTest.h"
#include <chrono>



LidarTest::LidarTest()
{
	m_device = nullptr;

	m_threadWork = std::thread(&LidarTest::initLidar, this);
}
LidarTest::~LidarTest()
{
	if (m_device)
	{
		m_device->unInit();

		delete m_device;
		m_device = nullptr;
	}
}


void LidarTest::sdkCallBackFunErrorCode(int iErrorCode)
{
	printf("HCSDK Main: sdkCallBackFunErrorCode ErrorCode=%d" , iErrorCode);
}

void LidarTest::sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
	printf("HCSDK Main: sdkCallBackFunSecondInfo time=%lld s,points=%d,GrayBytes=%d,FPS=%lld,speed=%0.2f,PPS=%d,valid=%d,invalid=%d,ErrorPacket=%d\n" , 
		sInfo.u64TimeStampS , sInfo.iNumPerPacket, sInfo.iGrayBytes , sInfo.u64FPS
		,sInfo.dRMS , sInfo.iPacketPerSecond,sInfo.iValid , sInfo.iInvalid
		, sInfo.u64ErrorPacketCount);
}

void LidarTest::sdkCallBackFunPointCloud(LstPointCloud lstG)
{
	printf("HCSDK Main: sdkCallBackFunPointCloud Rx Points=%d\n",lstG.size());
	for (auto sInfo : lstG)
	{
		//std::cout << "Main: Angle=" << sInfo.dAngle  << ",AngleRaw=" << sInfo.dAngleRaw << ",Dist=" << sInfo.u16Dist << std::endl;
	}
}

void LidarTest::sdkCallBackFunDistQ2(LstNodeDistQ2 lstG)
{
	std::cout << "Main: sdkCallBackFunDistQ2 Rx Points=" << lstG.size() << std::endl;
	for (auto sInfo : lstG)
	{
		std::cout << "Main: Angle=" << sInfo.angle_q6_checkbit / 64.0f << ",Dist=" << sInfo.distance_q2 / 4 << std::endl;
	}
}


void LidarTest::initLidar()
{
	HCLidar device;
	int fps = 0, rtn = 0;

	bool bPollMode = true;
	bool bDistQ2 = false;
	bool bLoop = false;

	std::cout << "Main: SDK verion=" << device.getSDKVersion().c_str() << std::endl;

	auto funErrorCode = std::bind(&LidarTest::sdkCallBackFunErrorCode, this,std::placeholders::_1);
	device.setCallBackFunErrorCode(funErrorCode);

	auto funSecondInfo = std::bind(&LidarTest::sdkCallBackFunSecondInfo, this,std::placeholders::_1);
	device.setCallBackFunSecondInfo(funSecondInfo);

	if (!bPollMode)//call back
	{
		auto funPointCloud = std::bind(&LidarTest::sdkCallBackFunPointCloud, this,std::placeholders::_1);
		device.setCallBackFunPointCloud(funPointCloud);

		auto funDistQ2 = std::bind(&LidarTest::sdkCallBackFunDistQ2, this,std::placeholders::_1);
		device.setCallBackFunDistQ2(funDistQ2);
	}

	/*tsSDKPara sPara;
	sPara.iNoDataMS = 1000;
	sPara.iDisconnectMS = 3000;
	sPara.iFPSContinueMS = 5000;
	sPara.iSpeedContinueMS = 3500;
	sPara.iCoverContinueMS = 3500;
	sPara.iBlockContinueMS = 3500;
	sPara.iCoverPoints = 100;
	sPara.iPollBuffSize = 1000;
	sPara.iCallbackBuffSize = 50;*/

	int iBaud = 115200;
	int iReadTimeoutms = 10;//
	// ##### 1. Open serial port using valid COM id #####
#ifdef _WIN32
	rtn = device.initialize("//./com5", "X2M", iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);                     // For windows OS
#else
	rtn = device.initialize("/dev/ttyPort1", "X2M", iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);               // For Linux OS
#endif

	if (rtn != 1)
	{
		int iErrorCode = device.getLastErrCode();
		if (iErrorCode == ERR_SERIAL_INVALID_HANDLE || iErrorCode == ERR_SERIAL_READFILE_FAILED)
		{
			device.unInit();
			std::cout << "Main: Init sdk failed!\n" << std::endl;
			return ;
		}

	}

	//device.unInit();
	//device.unInit();
#ifdef _WIN32
	rtn = device.initialize("//./com5", "X2M", iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);                     // For windows OS
#else
	rtn = device.initialize("/dev/ttyPort1", "X2M", iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);               // For Linux OS
#endif


	std::cout << "Main: Lidar ID=" << device.getLidarID().c_str() << std::endl;
	std::cout << "Main: Factory Info:" << device.getFactoryInfo().c_str() << std::endl;
	std::cout << "Main: Firmware ver:" << device.getFirmwareVersion().c_str() << std::endl;
	std::cout << "Main: Hardware ver:" << device.getHardwareVersion().c_str() << std::endl;
	std::cout << "Main: Lidar model:" << device.getLidarModel().c_str() << std::endl;

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
				if (device.getRxPointClouds(lstG))
				{
					std::cout << "Main: Poll Rx Points=" << lstG.size() << std::endl;
					for (auto sInfo : lstG)
					{
						//std::cout << "Main: Angle=" << sInfo.dAngle  << ",AngleRaw=" << sInfo.dAngleRaw << ",Dist=" << sInfo.u16Dist << std::endl;
					}
				}
				else
				{
					int iError = device.getLastErrCode();
					if (iError != LIDAR_SUCCESS)
					{
						std::cout << "Main: Poll Rx Points error code=" << iError << std::endl;
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
		//std::cout << "Main: SDK Status=" << iSDKStatus <<std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		std::this_thread::yield();
		//printf("\n");
	}

	device.unInit();
}