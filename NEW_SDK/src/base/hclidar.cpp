#include "hclidar.h"
#include <vector>
#include <set>

#include <algorithm>
#include <stdio.h>




HCLidar::HCLidar()
{
	initPara();
    m_p8Buff = new UCHAR[READ_BUFF_SIZE];
}

HCLidar::~HCLidar()
{
    m_lstBuff.clear();
    m_lstTemp.clear();
    unInit();

    delete [] m_p8Buff;
    m_p8Buff = nullptr;
}

void HCLidar::initPara()
{
	m_iSDKStatus = SDK_UNINIT;
	m_bScanning = false;
	m_iLastErrorCode = 0;
	m_bDisconnect = true;
	m_bHadID = false;
	m_bGetIDTimeOut = false;
	m_bHadFact = false;
	m_bGetFactTimeOut = false;
	m_bInitTimeout = false;
	m_sStatistic.reset();
}


void HCLidar::lidarReConnect()
{
	m_bInitTimeout = false;
    m_bHadID=false;
    m_bGetIDTimeOut=false;
    m_bHadFact=false;
    m_bGetFactTimeOut=false;
}

void HCLidar::setWorkPara(tsSDKPara& sSDKPara)
{
	m_sSDKPara = sSDKPara;
}

bool HCLidar::setLidarPara(const char* chLidarModel)
{
    m_strLidarModel = chLidarModel;
    //m_iBaud= 115200;
    if(m_strLidarModel == X1B)
    {
        m_dAngleOffsetD = 21;
        m_dBaseline_mm = 20;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=415;
    }
    else if(m_strLidarModel == X1D)
    {
        m_dAngleOffsetD = 21;
        m_dBaseline_mm = 20;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=415;
    }
    else if(m_strLidarModel == X1E)
    {
        m_dAngleOffsetD = 21;
        m_dBaseline_mm = 20;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=415;
    }
    else if(m_strLidarModel == X1F)
    {
        m_dAngleOffsetD = 21;
        m_dBaseline_mm = 20;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=415;
    }
    else if(m_strLidarModel == X1M)
    {
        m_dAngleOffsetD = 21;
        m_dBaseline_mm = 20;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=415;
    }
    else if(m_strLidarModel == X2A)
    {
        m_dAngleOffsetD = 28.5;
        m_dBaseline_mm = 17.92;
        m_iFPSMax = FPS_3000_MAX;
        m_iFPSMin = FPS_3000_MIN;
        m_iSpeedMax = SPEED_3000_MAX;
        m_iSpeedMin = SPEED_3000_MIN;
        m_iCircleNumberMAX=515;
    }
    else if(m_strLidarModel == X2B)
    {
        m_dAngleOffsetD = 28.5;
        m_dBaseline_mm = 17.92;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=415;
    }
    else if(m_strLidarModel == X2C)
    {
        m_dAngleOffsetD = 28.5;
        m_dBaseline_mm = 17.92;
        m_iFPSMax = FPS_3000_MAX;
        m_iFPSMin = FPS_3000_MIN;
        m_iSpeedMax = SPEED_3000_MAX;
        m_iSpeedMin = SPEED_3000_MIN;
        m_iCircleNumberMAX=515;
    }
    else if(m_strLidarModel == X2E)
    {
        m_dAngleOffsetD = 28.5;
        m_dBaseline_mm = 17.92;
        m_iFPSMax = FPS_3000_MAX;
        m_iFPSMin = FPS_3000_MIN;
        m_iSpeedMax = SPEED_3000_MAX;
        m_iSpeedMin = SPEED_3000_MIN;
        m_iCircleNumberMAX=515;
    }
    else if(m_strLidarModel == X2F)
    {
        m_dAngleOffsetD = 28.5;
        m_dBaseline_mm = 17.92;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=415;
    }
    else if(m_strLidarModel == X2M)
    {
        //m_iBaud = 230400;
        m_dAngleOffsetD = 28.5;
        m_dBaseline_mm = 17.92;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=515;
    }
    else if(m_strLidarModel == X2N)
    {

        m_dAngleOffsetD = 28.5;
        m_dBaseline_mm = 17.92;
        m_iFPSMax = FPS_2000_MAX;
        m_iFPSMin = FPS_2000_MIN;
        m_iSpeedMax = SPEED_2000_MAX;
        m_iSpeedMin = SPEED_2000_MIN;
        m_iCircleNumberMAX=415;
    }
    else
    {

        return false;
    }
    return true;
}
//bGetLoopData 是否输出一圈的数据
BOOL HCLidar::initialize(const char* chPort, const char* chLidarModel,int iBaud, int iReadTimeoutMs,  bool bDistQ2,bool bGetLoopData, bool bPollMode)
{
	LOG_INFO("Init sdk\n");
    if(m_bScanning)
    {

		LOG_WARNING( "Had been init.\n");
        setReadCharsError(ERR_SDK_HAD_BEEN_INIT);
        return false;
    }
    if(chPort == NULL)
    {
		LOG_ERROR("port is null.\n" );
        setReadCharsError(ERR_SDK_INIT_PARA);
        return false;
    }

    if(chLidarModel == NULL)
    {
		LOG_ERROR("lidar mode is null.\n");
        setReadCharsError(ERR_SDK_INIT_PARA);
        return false;
    }

    m_iReadTimeOutms = iReadTimeoutMs;
    if( m_iReadTimeOutms<= 0 || m_iReadTimeOutms >= 50)
    {
        m_iReadTimeOutms = READ_TIMEOUT_MS;
    }

   


    if(!setLidarPara(chLidarModel))
    {
		LOG_ERROR("Lidar mode does not exists.\n");
        setReadCharsError(ERR_MODE_NOT_EXISTS);
        return false;
    }

	m_iBaud = iBaud;

    if (m_serial.openDevice(chPort, m_iBaud) != 1)
    {

		LOG_ERROR("open serial port failed.\n");
        setReadCharsError(ERR_SERIAL_INVALID_HANDLE);
        return false;
    }

    m_iSDKStatus = SDK_INIT;
    m_bScanning = true;
    m_bGetLoopData = bGetLoopData;
    m_bPollMode = bPollMode;
    m_bDistQ2 = bDistQ2;


    m_threadWork = std::thread(&HCLidar::threadWork, this);
    m_threadParse = std::thread(&HCLidar::threadParse, this);

    std::unique_lock<std::mutex> lck(m_mtxInit);
    while (!m_bReady)
        m_cvInit.wait(lck);

	LOG_INFO("Init complete\n");


	return m_bHadID;
}

BOOL HCLidar::unInit()
{
    if(!m_bScanning)
        return false;

    m_bScanning = false;
    if(m_threadWork.joinable())
        m_threadWork.join();

    if(m_threadParse.joinable())
        m_threadParse.join();

    m_serial.closeDevice();

	initPara();
    return true;
}


void HCLidar::setReadCharsError(int errCode)
{
    switch (errCode)
    {
	case -6:
		m_iLastErrorCode = ERR_SERIAL_READFILE_FAILED;
		break;
    case -5:
        m_iLastErrorCode = ERR_SERIAL_INVALID_HANDLE;
        break;
    case -4:
        m_iLastErrorCode = ERR_SERIAL_SETCOMMTIMEOUTS_FAILED;
        break;
    case -3:
        m_iLastErrorCode = ERR_SERIAL_READFILE_FAILED;
        break;
    case -1:
        m_iLastErrorCode = ERR_SERIAL_READFILE_FAILED;
        break;
    case 0:
        m_iLastErrorCode = ERR_SERIAL_READFILE_ZERO;
        break;
    default:
        m_iLastErrorCode = errCode;
        break;
    }

    if(m_funErrorCode)
    {
        m_funErrorCode(m_iLastErrorCode);
    }
}

void HCLidar::threadWork()
{
    while (m_bScanning)
    {
        readData();
    }
}

void HCLidar::readData()
{

    memset(m_p8Buff,0, READ_BUFF_SIZE);

    //read data number
    int iRC = m_serial.readData(m_p8Buff, READ_BUFF_SIZE, m_iReadTimeOutms);
    if (iRC < 1)
    {

        if(iRC == 0)
        {
            m_iReadTimeoutCount++;
            int iTemp = m_sSDKPara.iNoDataMS / m_iReadTimeOutms;
            int iTemp2 = m_sSDKPara.iDisconnectMS / m_iReadTimeOutms;
            if(m_iReadTimeoutCount%iTemp == 0)
            {
				LOG_WARNING("Lidar read timeout!\n");

                setReadCharsError(iRC);
            }
            else if(m_iReadTimeoutCount >= iTemp2 )
            {
				LOG_ERROR("Lidar disconnect! status=%d\n" , static_cast<int>(m_iSDKStatus) );
                if(m_iSDKStatus == SDK_INIT)
                {
                    checkHadInitSuccess(true);
                    return;
                }
                m_iReadTimeoutCount = 0;
                m_bDisconnect = true;
                lidarReConnect();
                m_iSDKStatus = SDK_DISCONNECT;

                setReadCharsError(ERR_DISCONNECTED);
            }

        }
        else
        {
            setReadCharsError(iRC);
        }

        return;
    }
    m_iReadTimeoutCount=0;
    m_bDisconnect = false;

    std::lock_guard<std::mutex> lock(m_mtxBuff);
    for(int i=0;i<iRC;i++)
        m_lstTemp.push_back(m_p8Buff[i]);


}

void HCLidar::threadParse()
{

    UINT64 u64Start = HCHead::getCurrentTimestampUs();

    while (m_bScanning)
    {
        {
            std::lock_guard<std::mutex> lock(m_mtxBuff);
            if(m_lstTemp.size()>0)
            {
                std::vector<UCHAR>  lstTemp;
                lstTemp.swap(m_lstTemp);

                m_lstBuff.insert(m_lstBuff.end(), lstTemp.begin(), lstTemp.end());
            }

        }

        
		if (!m_bInitTimeout)
		{
			UINT64 u64End = HCHead::getCurrentTimestampUs();
			UINT64 u64Int = (u64End - u64Start) / 1000;

			if (u64Int >= INIT_TIMEOUT_MS)
			{
				if (!m_bHadID && !m_bGetIDTimeOut)
				{
					m_bGetIDTimeOut = true;
					m_bHadID = false;
					LOG_INFO("Get Device ID timeout\n" );
					checkHadInitSuccess(true);
				}

				if (!m_bHadFact && !m_bGetFactTimeOut && m_bHadID)
				{

					LOG_INFO("Get Two CMD info timeout\n" );
					checkHadInitSuccess(false);
				}

				//m_iSDKStatus = SDK_WORKING;
				m_bInitTimeout = true;
			}
		}
		else
		{
			u64Start = HCHead::getCurrentTimestampUs();
		}
        

        if(m_bDisconnect)
        {
            std::vector<UCHAR>  lstTemp;
            lstTemp.swap(m_lstBuff);

            u64Start = HCHead::getCurrentTimestampUs();

            m_sStatistic.reset();
        }
        else
        {
            processMain();

            checkReadPacketData();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        std::this_thread::yield();
    }
}


void HCLidar::checkReadPacketData()
{

    if(m_sStatistic.u64TSRxPacketFirst == 0)
        return;


    if(bIntervalOneSecond(m_u64StartMS))
    {
        m_sStatistic.u64CurrentS++;
        m_sStatistic.u64FPS = m_sStatistic.iPacketPerSecond*m_sStatistic.iNumPerPacket;

        m_sStatistic.u64TimeStampS = HCHead::getCurrentTimestampUs()/1e6;
        checkInvalidFPS(m_sStatistic.u64FPS);

        /*
        std::cout << "Info: time=" << m_sStatistic.u64TimeStampS << "s,points=" << m_sStatistic.iNumPerPacket
                  << ",GrayBytes=" << m_sStatistic.iGrayBytes << ",FPS=" << m_sStatistic.u64FPS
                  << ",speed=" << m_sStatistic.dRMS << ",PPS=" << m_sStatistic.iPacketPerSecond
                  << ",valid=" << m_sStatistic.iValid << ",invalid=" << m_sStatistic.iInvalid
                  << ",ErrorPacket=" << m_sStatistic.u64ErrorPacketCount <<std::endl;
*/

        if(m_funSecondInfo)
        {
            m_funSecondInfo(m_sStatistic);
        }
        m_sStatistic.reset();

    }
}

void HCLidar::processMain()
{
    if(m_lstBuff.size()==0)
        return;
    while (true)
    {

        if(!processData())
            break;
        if(m_lstBuff.size()==0)
            break;

    }
    //printf("Info: buff size = %d\n",m_lstBuff.size());
}

void HCLidar::sendGetIDInfoSignal(bool bGetID)
{
    if(!bGetID)
    {
        setReadCharsError(ERR_NOT_ID);
    }

    m_bHadID = bGetID;
    checkHadInitSuccess(false);
}

void HCLidar::sendGetFactoryInfoSignal(bool bGetFact)
{
    if(!bGetFact)
    {
        setReadCharsError(ERR_NOT_ID);
    }

    m_bHadFact = bGetFact;
    checkHadInitSuccess(false);
}

void HCLidar::checkHadInitSuccess(bool bTimeout)
{
    if(bTimeout)
    {
        m_iSDKStatus = SDK_ID_TIMEOUT;
        std::unique_lock <std::mutex> lck(m_mtxInit);
        m_bReady = true;
        m_cvInit.notify_all();
    }
    else
    {
        if(m_bHadID && m_bHadFact)
        {
            m_iSDKStatus = SDK_WORKING;
            std::unique_lock <std::mutex> lck(m_mtxInit);
            m_bReady = true;
            m_cvInit.notify_all();
        }
		else// if (m_bHadID && !m_bHadFact)
		{
			m_iSDKStatus = SDK_ID_TIMEOUT;
			std::unique_lock <std::mutex> lck(m_mtxInit);
			m_bReady = true;
			m_cvInit.notify_all();
		}
    }
}

bool HCLidar::processData()
{
    if(m_lstBuff.size()<=2)
        return false;

    int iIndex = -1;
    int iMsgID = 0;
    for(int i=0;i<m_lstBuff.size()-1;i++)
    {
        if(iIndex<0)
        {
            if(m_iSDKStatus == SDK_INIT)
            {
                if(!m_bHadID && !m_bGetIDTimeOut)
                {
                    if((UCHAR)m_lstBuff.at(i)== 0x5A && (UCHAR)m_lstBuff.at(i+1) == 0xA5)//ID
                    {
                        iMsgID = MSG_ID;
                        iIndex = i;
                        break;
                    }
                }
                if(!m_bHadFact && !m_bGetFactTimeOut)
                {
                    if((UCHAR)m_lstBuff.at(i) == 0xA5 && (UCHAR)m_lstBuff.at(i+1) == 0x5A)//Two CMD
                    {
                        iMsgID = MSG_CMD;
                        iIndex = i;
                        break;
                    }
                }
            }
            else if((UCHAR)m_lstBuff.at(i) == 0x55 && (UCHAR)m_lstBuff.at(i+1) == 0xAA)//point cloud
            {
                iMsgID = MSG_POINTCLOUD;
                iIndex = i;
                break;
            }
        }
    }
    if(iIndex < 0)
    {
        //printf("HCSDK Error: rx data not mes header\n" );
        m_lstBuff.clear();
        return false;
    }

    if(iIndex>0)
    {
        HCHead::eraseBuff(m_lstBuff,iIndex);
        //printf("HCSDK Error: find mes header,buff size=%d\n" , m_lstBuff.size());
    }


    switch (iMsgID) {
    case MSG_ID:
            return getDevID(m_lstBuff);
        break;
    case MSG_CMD:
            return getStartInfo(m_lstBuff);
        break;
    case MSG_POINTCLOUD:
            return getPointCloud(m_lstBuff);
        break;

    }

    return false;

}

bool HCLidar::calIDX2(char* ch,int iLen)
{
    UINT sum = 0;
    const UINT id_len = iLen-1;
    for (UINT i = 0; i < id_len; i++)
    {
       sum += (UCHAR)ch[i];
    }

    const UINT sum_check = (UCHAR)ch[id_len];
    unsigned char temp_sum = sum & 0xff;
    return (sum_check == temp_sum);
}

bool HCLidar::calIDX1(char* ch,int iLen)
{
    UINT sum = 0;
    const UINT id_len = iLen-2;
    for (UINT i = 0; i < id_len; i++)
    {
        sum += (UCHAR)ch[i];
    }

    const UINT sum_check = ((UCHAR)ch[id_len + 1] << 8) | (UCHAR)ch[id_len];
    return (sum_check == sum);
}

bool HCLidar::getDevID(std::vector<UCHAR>& lstBuff)
{
    char chTemp[128] = { 0 };
    if(m_bX2ID)//X2 series
    {
        int iLen = sizeof(tsIDX2);
        if(lstBuff.size() < iLen)
            return false;


        tsIDX2  sID;
        memcpy(&sID, lstBuff.data(), iLen);
        //memcpy(&m_unDevID.u32ID, sID.u8ID, ID_LEN);

        if(calIDX2((char*)lstBuff.data(),iLen))
        {
#if SHARK_ENABLE
            sprintf(chTemp, "000000000000000000%02X%02X%02X%02X%02X%02X%02X",
                                sID.u8Ver[0], sID.u8Ver[1], sID.u8Ver[2],
                                sID.u8ID[0], sID.u8ID[1], sID.u8ID[2], sID.u8ID[3]);
#else
			sprintf(chTemp, "%02X%02X%02X%02X%02X%02X%02X",
				sID.u8Ver[0], sID.u8Ver[1], sID.u8Ver[2],
				sID.u8ID[0], sID.u8ID[1], sID.u8ID[2], sID.u8ID[3]);
#endif

            m_strDevID = chTemp;

            //m_bHadID = true;

            memset(chTemp,0,128);
            sprintf(chTemp, "00.%02X.%02X.%02X",
                                sID.u8Ver[0], sID.u8Ver[1], sID.u8Ver[2]);

            m_strFirmwareVer = chTemp;


			LOG_INFO("Get ID ok!\n");
            sendGetIDInfoSignal(true);

        }
        else
        {
            m_strDevID = std::string(DEFAULT_ID);

			LOG_ERROR("ID cal error!\n" );


        }
        //lstBuff.erase(lstBuff.begin(), lstBuff.begin() + iLen);
        HCHead::eraseBuff(lstBuff,iLen);

        return true;
    }
    else//X1
    {
        int iLen = sizeof(tsIDX1);
        if(lstBuff.size() < iLen)
            return false;


        tsIDX1  sID;
        memcpy(&sID, lstBuff.data(), iLen);
        //memcpy(&m_unDevID.u32ID, sID.u8ID, ID_LEN);

        if(calIDX1((char*)lstBuff.data(),iLen))
        {
#if SHARK_ENABLE
            sprintf(chTemp, "000000000000000000000000%02X%02X%02X%02X",
                                sID.u8ID[3], sID.u8ID[2], sID.u8ID[1], sID.u8ID[0]);
#else
			sprintf(chTemp, "%02X%02X%02X%02X",
				sID.u8ID[3], sID.u8ID[2], sID.u8ID[1], sID.u8ID[0]);
#endif

            m_strDevID = chTemp;
            //m_bHadID = true;
			LOG_INFO("Get ID ok!\n" );
            sendGetIDInfoSignal(true);

        }
        else
        {
            m_strDevID = std::string(DEFAULT_ID);

			LOG_ERROR("ID cal error!\n");


        }
        //lstBuff.erase(lstBuff.begin(), lstBuff.begin() + iLen);
        HCHead::eraseBuff(lstBuff,iLen);

        return true;
    }

}

bool HCLidar::calStartInfo(char* ch,int iLen)
{
    UINT sum = 0;

    for (UINT i = 0; i < iLen; i++)
    {
        if(i==4 || i==5)
        {

        }
        else
            sum += (UCHAR)ch[i];
    }

    const UINT sum_check = ((UCHAR)ch[5] << 8) | (UCHAR)ch[4];
    return (sum_check == sum);
}

bool HCLidar::getStartInfo(std::vector<UCHAR>& lstBuff)
{

	LOG_INFO("getStartInfo!\n");

    int iLen = sizeof(tsCmdInfo);
    int iMin =  iLen <lstBuff.size() ? iLen : lstBuff.size();
    if(lstBuff.size() < sizeof(tsCmdStart))
        return false;

    tsCmdInfo  sCmd;
    memcpy(&sCmd, lstBuff.data(), iMin);
    if(sCmd.u16Len == 0)
    {

        HCHead::eraseBuff(lstBuff,sizeof(tsCmdStart));

		LOG_ERROR("lidar start message\n");
        setReadCharsError(ERR_START_INFO);
        return true;
    }

    if(sCmd.u16Len == 20)//old
    {
        if(calStartInfo((char*)lstBuff.data(),iLen))
        {
            HCHead::eraseBuff(lstBuff,sizeof(tsCmdStart));

            m_strFactoryInfo = std::string((char*)sCmd.u8FacInfo);


            sendGetFactoryInfoSignal(true);
			LOG_INFO("lidar factory info:%s\n" , (char*)sCmd.u8FacInfo );
            return true;
        }
        else
        {
            HCHead::eraseBuff(lstBuff,sizeof(tsCmdStart));

			LOG_ERROR("lidar start message cal error\n");
            setReadCharsError(ERR_START_INFO);
            return true;

        }
    }
    else if(sCmd.u16Len == 25)//NEW
    {
        if(lstBuff.size() < sizeof(tsIDNew))
            return false;

        tsIDNew sNewInfo;
        memcpy(&sNewInfo, lstBuff.data(), sizeof(tsIDNew));

        if(calStartInfo((char*)lstBuff.data(),sizeof(tsIDNew)))
        {
            HCHead::eraseBuff(lstBuff,sizeof(tsIDNew));

            char chTemp[128] = { 0 };
            sprintf(chTemp, "%c%c",
                                sNewInfo.u8FacInfo[0], sNewInfo.u8FacInfo[1]);
            m_strFactoryInfo = chTemp;

            memset(chTemp,0,128);
            sprintf(chTemp, "%c%d%c",sNewInfo.u8FacInfo[2],sNewInfo.u8FacInfo[3],sNewInfo.u8FacInfo[4]);
            std::string strModel = chTemp;
            if(strModel != m_strLidarModel)
            {
				LOG_WARNING("Lidar model error init:%s,device:%s\n",(char*)m_strLidarModel.c_str(),  (char*)strModel.c_str() );
                //setReadCharsError(ERR_DEV_MODEL);
				setLidarPara(chTemp);
            }

            memset(chTemp,0,128);
#if SHARK_ENABLE
            sprintf(chTemp, "00000000000000000000%02X%02X%02X%02X%02X%02X%02X",
                                sNewInfo.u8FacInfo[3], sNewInfo.u8FacInfo[4], sNewInfo.u8FacInfo[5],
                                sNewInfo.u8ID[0], sNewInfo.u8ID[1], sNewInfo.u8ID[2], sNewInfo.u8ID[3]);
#else
			sprintf(chTemp, "%02X%02X%02X%02X%02X%02X%02X",
				sNewInfo.u8FacInfo[3], sNewInfo.u8FacInfo[4], sNewInfo.u8FacInfo[5],
				sNewInfo.u8ID[0], sNewInfo.u8ID[1], sNewInfo.u8ID[2], sNewInfo.u8ID[3]);
#endif

            m_strDevID = chTemp;

			LOG_INFO("Get ID ok ID:%s\n",chTemp );


            memset(chTemp,0,128);
            sprintf(chTemp, "00.00.%02X.%02X",
                                sNewInfo.u8CalVer[0], sNewInfo.u8CalVer[1]);

            m_strFirmwareVer = chTemp;


            memset(chTemp,0,128);
            sprintf(chTemp, "00.%02X.%02X.%02X",
                                sNewInfo.u8HardVer[0], sNewInfo.u8HardVer[1], sNewInfo.u8HardVer[2]);

            m_strHardwareVer = chTemp;


            sendGetIDInfoSignal(true);

            sendGetFactoryInfoSignal(true);
			LOG_INFO("New lidar factory info:%s,Hardware ver:%s\n" , (char*)m_strFactoryInfo.c_str() , (char*)m_strHardwareVer.c_str());
            return true;
        }
        else
        {
            HCHead::eraseBuff(lstBuff,sizeof(tsIDNew));

			LOG_ERROR("New lidar factory info cal error\n" );
            setReadCharsError(ERR_START_INFO);
            return true;

        }
    }
    else
    {
        HCHead::eraseBuff(lstBuff,sizeof(tsCmdStart));

		LOG_ERROR("lidar start message\n" );
        setReadCharsError(ERR_START_INFO);
        return true;
    }
    return false;
}

bool HCLidar::calMCUFrame(char* ch,int iLen)
{
    UINT iSUM = 0;
    for (UINT i = 0; i < iLen-1; i++)
    {
        iSUM += (UCHAR)ch[i];
    }

    UCHAR u8Sum = (UCHAR)iSUM;
    return (u8Sum == (UCHAR)ch[iLen-1]);
}

bool HCLidar::getMCUCmd(std::vector<UCHAR>& lstBuff)
{

    tsBlockMessage  sBlockMessage;
    memcpy(&sBlockMessage, lstBuff.data(), sizeof(tsBlockMessage));

    int iPackLen = sBlockMessage.u8Len; // u8Num is length
    if(iPackLen <= lstBuff.size())//have a good packet
    {
        if(calMCUFrame((char *)lstBuff.data(),iPackLen))
        {
            if(sBlockMessage.u8MsgID == 0x45) //block message for motor
            {
                if(sBlockMessage.u8Code == 0x01)
                {
					LOG_INFO("MCU message <Motor blocked>\n" );

                    /*if(m_iLastErrorCode != ERR_MOTOR_BLOCKED)
                    {
                        setReadCharsError(ERR_MOTOR_BLOCKED);


                    }*/
                    checkSharkBlocked();
                }
                else
                {

					LOG_INFO("MCU message <Lidar reboot>\n");
                    /*if(m_iLastErrorCode != ERR_REBOOT_LIDAR)
                        setReadCharsError(ERR_REBOOT_LIDAR);*/
                }
            }
        }
        HCHead::eraseBuff(lstBuff,iPackLen);
        return true;
    }
    else
    {
        return false;
    }
}
bool HCLidar::getPointCloud(std::vector<UCHAR>& lstBuff)
{

    //printf("Info: getPointCloud!\n");

    int iLen = sizeof(tsPointCloudHead);
    int iMin =  iLen <lstBuff.size() ? iLen : lstBuff.size();
    if(lstBuff.size() < iLen)
        return false;

    tsPointCloudHead  sPointCloudHead;
    memcpy(&sPointCloudHead, lstBuff.data(), iMin);

    bool bBlockMessage = false;
    int iPointBytes = 3;
    if(sPointCloudHead.u8Info == 0x02)
    {
        m_sStatistic.iGrayBytes = 0;
        iPointBytes = 2;
    }
    else if(sPointCloudHead.u8Info == 0x07)
    {
        m_sStatistic.iGrayBytes = 2;
        iPointBytes = 4;
    }
    else if(sPointCloudHead.u8Info == 0x80)//MCU message
    {
        bBlockMessage = true;
    }
    else
    {
        m_sStatistic.iGrayBytes = 1;
        iPointBytes = 3;
    }

    if(bBlockMessage)
    {
        return getMCUCmd(lstBuff);
    }

    int iPackLen = sPointCloudHead.u8Num * iPointBytes + sizeof(tsPointCloudHead) + sizeof(tsPointCloudTail);

    if(iPackLen <= lstBuff.size())//have a good packet
    {
        if(m_sStatistic.u64TSRxPacketFirst==0)
        {

            m_sStatistic.u64TSRxPacketFirst = HCHead::getCurrentTimestampUs();
            m_u64StartMS = m_sStatistic.u64TSRxPacketFirst;

        }
        if(checkDataCal(lstBuff, iPackLen))//good packet
        {


            m_sStatistic.u64RxPacketCount++;
            m_sStatistic.iNumPerPacket = sPointCloudHead.u8Num ;


            if(m_bPollMode)
            {
                std::lock_guard<std::mutex> lock(m_mtxData);
                parserRangeEX(m_resultRange,(char *)lstBuff.data(), iPackLen, sPointCloudHead.u8Num,iPointBytes);
                if(m_bDistQ2)
                {
                    if(m_resultRange.size()>0)
                    {
                        convertDistQ2(m_resultRange);
                        LstPointCloud tmp;
                        tmp.swap(m_resultRange);
                    }
                }
                else
                {
                    if(m_resultRange.size()>m_sSDKPara.iPollBuffSize)
                    {
                        HCHead::eraseRangeData(m_resultRange,m_resultRange.size()-m_sSDKPara.iPollBuffSize);
                        setReadCharsError(ERR_BUFF_FULL);
                    }
                }

            }
            else//call back mode
            {
                parserRangeEX(m_resultRange,(char *)lstBuff.data(), iPackLen, sPointCloudHead.u8Num,iPointBytes);
                if(m_bDistQ2)
                {

                    if(m_resultRange.size()>0)
                    {
                        convertDistQ2(m_resultRange);
                        LstPointCloud tmp;
                        tmp.swap(m_resultRange);
                    }
                    if(m_sNodeList.size()>=m_sSDKPara.iCallbackBuffSize)
                    {

                        if(m_funDistQ2)
                        {
                            callbackDistQ2();
                        }
                        else
                        {
                            setReadCharsError(ERR_CALLBACK_FUN);
                        }
                    }
                }
                else
                {
                    if(m_resultRange.size()>=m_sSDKPara.iCallbackBuffSize)
                    {
                        if(m_funPointCloud)
                        {
                            m_funPointCloud(m_resultRange);
                        }
                        else
                        {
                            setReadCharsError(ERR_CALLBACK_FUN);
                        }
                        LstPointCloud tmp;
                        tmp.swap(m_resultRange);
                    }
                }
            }

			//LOG_INFO("lidar point cal ok! Point cloud buff size=%d\n" ,m_resultRange.size() );
        }
        else
        {

			LOG_ERROR("lidar point cloud error!\n");
            m_sStatistic.u64ErrorPacketCount++;
            setReadCharsError(ERR_CHECKDATA_INVALID);

        }
        m_sStatistic.iPacketPerSecond++;

        HCHead::eraseBuff(lstBuff,iPackLen);

        return true;
    }
    return false;

}

bool HCLidar::parserRangeEX(LstPointCloud &resultRange,const char * chBuff, int iIndex, int in_numData,int iPointSize)
{

    UINT64 epochTime = HCHead::getCurrentTimestampUs()/1000;// dt.toMSecsSinceEpoch();

    const int data_size = iPointSize;//const int data_size = 3;
    int id_start = 6;//int id_start = 2;
    double FA = ((UCHAR)chBuff[id_start + 1] - 0xA0 + (UCHAR)chBuff[id_start] / 256.0) * 4;
    int id_LA_start = id_start + in_numData * data_size + 2;
    double LA = ((UCHAR)chBuff[id_LA_start + 1] - 0xA0 + (UCHAR)chBuff[id_LA_start] / 256.0) * 4;
    if (LA < FA) { LA += 360; }
    double dAngle = (LA - FA) / (in_numData - 1);        // angle info for each sampling
    UCHAR *data = new UCHAR[data_size];//unsigned char data[3];
    int pre_bytes = 8;          // 4 bytes before sampling data in each data package
    unsigned int speed = ((UCHAR)chBuff[5] << 8 | (UCHAR)chBuff[4]) / 64;
    const double angle_offset = m_dAngleOffsetD;
    for (int i = 0; i < in_numData; ++i)
    {
        double Beforecompensation = FA + dAngle * i;
        double angle_cur = FA + dAngle * i + angle_offset;
        double Aftercompensation = angle_cur;
        if (angle_cur > 360)
        {
            angle_cur -= 360;
        }


        tsPointCloud sData;
        sData.u64TimeStampMs = epochTime;
        memcpy(data, (UCHAR*)chBuff + pre_bytes + i * data_size, sizeof(UCHAR) * data_size);
        if(((data[1] >> 7) & 0x01) == 1)
            sData.bValid = false;//(data[1] >> 7) & 0x01;
        sData.u16DistRaw = ((data[1] & 0x003F) << 8) | data[0];
        sData.u16Dist = sData.u16DistRaw;
        if(data_size==2)
            sData.u16Gray = 0;
        else if(data_size == 3)
            sData.u16Gray = data[2] & 0xff;
        else if (data_size == 4)
        {
            sData.bGrayTwoByte = true;
            sData.u16Gray =  ((data[3]&0x00ff) << 8) | data[2] ;//data[2] + ((data[3]&0x00ff) << 8);
        }
        else
            sData.u16Gray = 0;
        sData.u16Speed = speed;
        sData.dAngleRaw = angle_cur;
        sData.dAngle = sData.dAngleRaw;
        sData.dAngleDisp = sData.dAngle;
        // Compensate angle & dist
        if (0 == sData.u16Dist)
        {
            m_sStatistic.iInvalid++;
            sData.bValid = false;        // invalid point
        }
        else if (m_bCompensate)
        {
            m_sStatistic.iValid++;
            compensate(sData.dAngle, sData.u16Dist, m_dTheta_d, m_dBaseline_mm);
            sData.dAngleDisp = sData.dAngle;
        }

        m_sStatistic.dRMS = speed;
        resultRange.push_back(sData);

        //std::cout << "Info: Angle=" << sData.dAngle  << ",AngleRaw=" << sData.dAngleRaw << ",Dist=" << sData.u16Dist << std::endl;
        checkSharkInvalidPoints(sData);
    }
    if (data)
    {
        delete[] data;
        data = NULL;
    }

    checkInvalidLowSpeed(speed);
    checkInvalidHighSpeed(speed);

    return true;
}
void HCLidar::compensate(double &angle, UINT16 &dist, const double theta_d, const double baseline_mm)
{
    if (0 != dist)
    {
        //const double theta = theta_d / 180. * PI;
        //const double beta = atan(dist / baseline_mm);
        //double angle_correct = PI / 2. - (theta + beta);
        //angle = angle + angle_correct / PI * 180;
        angle = angle + 80 - atan(dist / baseline_mm)/ PI_HC * 180;
        if (angle > 360)
        {
            angle -= 360;
        }
        else if (angle < 0)
        {
            angle += 360;
        }

        dist = sqrt(baseline_mm * baseline_mm + dist * dist);
    }
}

bool HCLidar::checkDataCal(std::vector<UCHAR>& lstBuff, int iIndex)
{
    const int iB = iIndex-2;
    std::vector<int> temp(iB / 2, 0);
    //temp[0] = 0x55 + (0xAA << 8);
    for (int i = 0; i < temp.size(); i++)
    {
        temp[i] = lstBuff.at(2 * i) + (lstBuff.at(2 * i + 1) << 8);
    }

    int chk32 = 0;
    for (int i = 0; i < temp.size(); i++)
    {
        chk32 = (chk32 << 1) + temp[i];
    }

    int checksum_target = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum_target = checksum_target & 0x7FFF;
    int checksum_cur = lstBuff.at(iIndex - 2) + (lstBuff.at(iIndex - 1) << 8);
    bool is_equal = (checksum_target == checksum_cur);
    return is_equal;
}
bool HCLidar::getRxPointClouds(LstPointCloud& lstG)
{
#if SHARK_ENABLE
    if(!m_bPollMode)
    {
        setReadCharsError(ERR_POLL_MODE);
        return false;
    }
    if(lstG.size() > 0)
    {
        LstPointCloud tmp;
        tmp.swap(lstG);
    }
	int iError = m_iLastErrorCode;

	bool bOK = true;
	switch (iError)
	{
	case ERR_SHARK_MOTOR_BLOCKED:

	case ERR_SHARK_INVALID_POINTS:

	case ERR_LIDAR_SPEED_LOW:

	case ERR_LIDAR_SPEED_HIGH:

	case ERR_DISCONNECTED:
	case ERR_LIDAR_FPS_INVALID:
		bOK = false;
		
		break;
	
	default:
		break;
	}

	if (bOK)
	{
		std::lock_guard<std::mutex> lock(m_mtxData);
		if (m_resultRange.size() > 0)
		{
			lstG.swap(m_resultRange);
		}
	}
	else
	{
		std::lock_guard<std::mutex> lock(m_mtxData);
		if (m_resultRange.size() > 0)
		{
			LstPointCloud tmp;
			m_resultRange.swap(tmp);
		}
	}
    
	return bOK;

#else
	if (!m_bPollMode)
	{
		setReadCharsError(ERR_POLL_MODE);
		return false;
	}
	if (lstG.size() > 0)
	{
		LstPointCloud tmp;
		tmp.swap(lstG);
	}
	std::lock_guard<std::mutex> lock(m_mtxData);
	if (m_resultRange.size() > 0)
	{
		lstG.swap(m_resultRange);
	}
	return true;

#endif
}


void HCLidar::checkInvalidFPS(int iFPS)
{
    if (iFPS < m_iFPSMin || iFPS > m_iFPSMax)
    {
        ++m_iInvalidFPSSecond;
		int iTemp = m_sSDKPara.iFPSContinueMS / 1000;
        if (m_iInvalidFPSSecond > iTemp)
        {
			LOG_ERROR("FPS:%d \n" , iFPS );
            setReadCharsError(ERR_LIDAR_FPS_INVALID);

            m_iInvalidFPSSecond = 0;
        }
    }
    else
    {
        m_iInvalidFPSSecond = 0;
    }
}


void HCLidar::checkInvalidLowSpeed(UINT16 u16Speed)
{
    if (u16Speed < m_iSpeedMin)
    {
        if (m_u64StartTimeLowSpeed != 0)
        {
            UINT64 endTime = HCHead::getCurrentTimestampUs();
            if ((endTime - m_u64StartTimeLowSpeed) >= m_sSDKPara.iSpeedContinueMS*1000)
            {
                setReadCharsError(ERR_LIDAR_SPEED_LOW);
                m_u64StartTimeLowSpeed = 0;
            }
        }
        else
        {
            m_u64StartTimeLowSpeed = HCHead::getCurrentTimestampUs();
        }
    }
    else
    {
        m_u64StartTimeLowSpeed = 0;
    }
}

void HCLidar::checkInvalidHighSpeed(UINT16 u16Speed)
{
    if (u16Speed > m_iSpeedMax)
    {
        if (m_u64StartTimeHighSpeed != 0)
        {
            int64_t endTime = HCHead::getCurrentTimestampUs();
            if ((endTime - m_u64StartTimeHighSpeed) >= m_sSDKPara.iSpeedContinueMS*1000)
            {
                setReadCharsError( ERR_LIDAR_SPEED_HIGH);
                m_u64StartTimeHighSpeed = 0;
            }
        }
        else
        {
            m_u64StartTimeHighSpeed = HCHead::getCurrentTimestampUs();
        }
    }
    else
    {
        m_u64StartTimeHighSpeed = 0;
    }
}



void HCLidar::checkSharkBlocked()
{
    if (m_u64StartTimeSharkBlock != 0)
    {
        UINT64 endTime = HCHead::getCurrentTimestampUs();
        UINT64 u64Temp = endTime - m_u64StartTimeSharkBlock;

        if(u64Temp >= (m_sSDKPara.iBlockContinueMS+2000)*1000) //3.5+2S
        {
            m_u64StartTimeSharkBlock = 0;
            m_iSharkBlockCount=0;
        }
        else if(u64Temp >= m_sSDKPara.iBlockContinueMS*1000 && u64Temp < (m_sSDKPara.iBlockContinueMS+2000)*1000)//3.5 - 4.5s
        {
            if(m_iSharkBlockCount>0)
            {
				LOG_ERROR("Shark moto block\n");
                setReadCharsError(ERR_SHARK_MOTOR_BLOCKED);
                m_u64StartTimeSharkBlock = 0;
                m_iSharkBlockCount=0;
            }
            else
            {
                m_u64StartTimeSharkBlock = 0;
                m_iSharkBlockCount=0;
            }

        }
        else if(u64Temp < 100*1000)//100ms
        {
            m_u64StartTimeSharkBlock = endTime;
        }
        else if(u64Temp >= MCU_BLOCK_TIME_MS*1000)
        {
            m_iSharkBlockCount++;
        }
        else
        {
        }
    }
    else
    {

        m_u64StartTimeSharkBlock = HCHead::getCurrentTimestampUs();
        m_iSharkBlockCount=0;

    }
}


void HCLidar::checkSharkInvalidPoints(tsPointCloud& sData)
{
    if (!sData.bValid)
    {
        if (m_u64StartTimeInvalidPoints != 0)
        {
            int64_t endTime = HCHead::getCurrentTimestampUs();
            if ((endTime - m_u64StartTimeInvalidPoints) >= m_sSDKPara.iCoverContinueMS*1000)
            {
                setReadCharsError( ERR_SHARK_INVALID_POINTS);
                m_u64StartTimeInvalidPoints = 0;
            }
        }
        else
        {
            m_u64StartTimeInvalidPoints = HCHead::getCurrentTimestampUs();
        }
    }
    else
    {
        m_u64StartTimeInvalidPoints = 0;
    }
}

/*
void HCLidar::getScanData(tsNodeInfo * nodebuffer, size_t buffLen, size_t &count, bool bReverse)
{
    if (nodebuffer == nullptr || buffLen == 0)
    {
        count = 0;
        return;
    }

    grabScanData(nodebuffer, buffLen, count);

    for (int i = 0; i < count; ++i)
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

        if (bReverse)
        {
            angle_cur = 360 - angle_cur;
        }

        nodebuffer[i].angle_q6_checkbit = angle_cur * 64;
    }
}
*/

bool HCLidar::getScanData(std::list<tsNodeInfo>& dataList, bool bReverse)
{

    grabScanData(dataList);

	if (dataList.size() == 0)
		return false;

    for (auto it = dataList.begin(); it != dataList.end(); ++it)
    {
        double angle_cur = it->angle_q6_checkbit;
        angle_cur = angle_cur / 64.0;
        if (angle_cur > 360)
        {
            angle_cur -= 360;
        }
        else if (angle_cur < 0)
        {
            angle_cur += 360;
        }

        if (bReverse)
        {
            angle_cur = 360 - angle_cur;
        }

        it->angle_q6_checkbit = angle_cur * 64;
    }
	return true;
}



void HCLidar::grabScanDataWithLoop(std::list<tsNodeInfo>& nodeList, tsNodeInfo* nodebuffer, size_t buffLen)
{
    int len = nodeList.size();

	if (len == 0)
		return;

    double perAngle = 360.0 / nodeList.size();
    for (int i = 0; i < len; ++i)
    {
        nodebuffer[i].angle_q6_checkbit = perAngle * (i + 1.0) * 64;
        nodebuffer[i].distance_q2 = 0;
        nodebuffer[i].isValid = 0;
        nodebuffer[i].syn_quality = 0;
        //printf("%.3f,%d,%d\n", nodebuffer[i].angle_q6_checkbit, nodebuffer[i].distance_q2, nodebuffer[i].isValid);
    }

    for (auto it = nodeList.begin(); it != nodeList.end(); ++it)
    {
        if (it->isValid == 1)
        {
            int index = (it->angle_q6_checkbit / (perAngle * 64.0)) + 0.5 - 1;
            //printf("index:%d\n", index);
            pushValidData2Buffer(*it, index, nodebuffer, len);
        }
    }

    if (!checkBufferIsSorted(nodebuffer, len))
    {
        //printf("data is not sorted.\n");
        //std::stable_sort(nodebuffer, nodebuffer + len);
		std::stable_sort(nodebuffer, nodebuffer + len, nodeComparator);
    }
}

void HCLidar::pushValidData2Buffer(tsNodeInfo& nodeInfo, int index, tsNodeInfo* nodebuffer, int len)
{
    bool isExit = false;
    while (!isExit)
    {
        index = index % len;
        if (nodebuffer[index].isValid == 0)
        {
            nodebuffer[index] = nodeInfo;
            isExit = true;
        }

        ++index;
    };
}

bool HCLidar::checkBufferIsSorted(tsNodeInfo* nodebuffer, int len)
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

void HCLidar::grabScanDataWithNoLoop(std::list<tsNodeInfo>& nodeList, tsNodeInfo* nodebuffer, size_t buffLen)
{
    size_t index = 0;
    size_t indexCount = 0;
    size_t startIndex = 0;
    size_t nodeLen = nodeList.size();

	if (nodeLen == 0)
		return;

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

void HCLidar::grabScanData(tsNodeInfo * nodebuffer, size_t buffLen, size_t &count)
{
    std::list<tsNodeInfo> nodeList;
    {
        std::lock_guard<std::mutex> guard(m_mtxData);
        nodeList.swap(m_sNodeList);
    }

    count = nodeList.size();
    if (count == 0)
    {
        return;
    }

    if (m_bGetLoopData)
    {
        if (count > buffLen)
        {
			LOG_ERROR("Rx buffer is too small\n" );
            setReadCharsError(ERR_RECEIVE_BUFFER_SMALL);
            count = 0;
            return;
        }

        grabScanDataWithLoop(nodeList, nodebuffer, buffLen);
    }
    else
    {
        grabScanDataWithNoLoop(nodeList, nodebuffer, buffLen);
    }

}

void HCLidar::grabScanData(std::list<tsNodeInfo>& dataList)
{
    std::list<tsNodeInfo> nodeList;
    {
        std::lock_guard<std::mutex> guard(m_mtxData);
        nodeList.swap(m_sNodeList);
    }

    if (nodeList.empty())
    {
        return;
    }

    size_t count = nodeList.size();
    tsNodeInfo* nodebuffer = new tsNodeInfo[count];
    if (m_bGetLoopData)
    {
        grabScanDataWithLoop(nodeList, nodebuffer, count);
    }
    else
    {
        grabScanDataWithNoLoop(nodeList, nodebuffer, count);
    }

    for (int i = 0; i < count; ++i)
    {
        dataList.push_back(nodebuffer[i]);
    }

    delete[] nodebuffer;

}

void HCLidar::convertDistQ2(LstPointCloud& lstG)
{
    for(auto sInfo:lstG)//for (int i = 0; i < rdPrPack_.data_num_per_pack_; ++i)
    {
        tsNodeInfo node_cur;
        m_dAngleCur = sInfo.dAngleRaw;
        // Compensate angle & dist
        unsigned int dist = sInfo.u16Dist;

        if (sInfo.bValid)
        {
            // valid data
            node_cur.distance_q2 = (uint16_t)(sInfo.u16Dist * 4);

            node_cur.isValid = 1;
            ++m_iValidNumber;
        }
        else
        {
            // invalid data
            node_cur.distance_q2 = 0;
            node_cur.isValid = 0;
        }

        node_cur.angle_q6_checkbit = (uint16_t)(sInfo.dAngle * 64);
        node_cur.syn_quality = sInfo.u16Gray;

        if (m_dAnglePre > m_dAngleCur)
        {
            m_bTurn = true;
            m_bFirsLoop = true;

            checkInvalidLidarNumber(m_iValidNumber);
            m_iValidNumber = 0;
        }
        //printf("%.4f,%.4f\n", preAngle, angle_cur);
        if (!m_bFirsLoop)
        {
            m_dAnglePre = m_dAngleCur;
            continue;
        }

        if (!m_bGetLoopData)
        {
            pushDataWithNoLoopMode(node_cur);
        }
        else
        {
            pushDataWithLoopMode(m_bTurn, m_sLoopNodeList, node_cur);
        }

        m_dAnglePre = m_dAngleCur;
        //printf("%.3f\n", node_cur.angle_q6_checkbit /  64.0f);
    }
}


void HCLidar::checkInvalidLidarNumber(int validNumber)
{
    if (validNumber < VALID_NUMBER_COUNT)
    {
        ++m_iInvalidNumberContinue;
        if (m_iInvalidNumberContinue >= NUMBER_CONTINUE_CIRCLE)
        {
            setReadCharsError(ERR_LIDAR_NUMBER_INVALID);
            m_iInvalidNumberContinue = 0;
        }
    }
    else
    {
        m_iInvalidNumberContinue = 0;
    }
}
void HCLidar::pushDataWithNoLoopMode(tsNodeInfo& node_cur)
{
    //std::lock_guard<std::mutex> guard(mutex_);
    if (m_sNodeList.size() >= m_sSDKPara.iPollBuffSize)
    {
        m_sNodeList.pop_front();
        setReadCharsError(ERR_BUFF_FULL);
    }

    m_sNodeList.push_back(node_cur);
}

//isGetLoopData is false
void HCLidar::pushDataWithLoopMode(bool& bTurn, std::list<tsNodeInfo>& loopNodeList, tsNodeInfo& node_cur)
{
    if (bTurn)
    {
        if (!m_bGreaterThan || (m_bGreaterThan && loopNodeList.size() > LESS_THAN_NUMBER))
        {
            //std::lock_guard<std::mutex> guard(mutex_);
            m_sNodeList.clear();
            m_sNodeList.swap(loopNodeList);
        }
        else
        {
            loopNodeList.clear();
        }

        bTurn = false;
        m_bGreaterThan = false;

    }
    else if (loopNodeList.size() >= m_iCircleNumberMAX)
    {
        {
            //std::lock_guard<std::mutex> guard(mutex_);
            m_sNodeList.clear();
            m_sNodeList.swap(loopNodeList);
        }

        m_bGreaterThan = true;
    }
    loopNodeList.push_back(node_cur);
}

void HCLidar::callbackDistQ2()
{
    LstNodeDistQ2 dataList;
    getScanData(dataList,true);

    if(dataList.size()>0)
        m_funDistQ2(dataList);
}
