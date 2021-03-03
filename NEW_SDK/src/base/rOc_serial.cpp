/*!
\file    serial-O-matic.cpp
\brief   Source file of the class rOc_serial. This class is used for communication over a serial device.
\author  Philippe Lucidarme (University of Angers)
\version 1.2
\date    28 avril 2011

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


This is a licence-free software, it can be used by anyone who try to build a better world.
*/

#include "rOc_serial.h"


#ifdef __linux__
#include <errno.h>
#endif

#include <chrono>
//_____________________________________
// ::: Constructors and destructors :::


/*!
\brief      Constructor of the class rOc_serial.
*/
rOc_serial::rOc_serial()
{}


/*!
\brief      Destructor of the class rOc_serial. It close the connection
*/
// Class desctructor
rOc_serial::~rOc_serial()
{
    closeDevice();
}



//_________________________________________
// ::: Configuration and initialization :::



/*!
\brief Open the serial port
\param Device : Port name (COM1, COM2, ... for Windows ) or (/dev/ttyS0, /dev/ttyACM0, /dev/ttyUSB0 ... for linux)
\param Bauds : Baud rate of the serial port.

\n Supported baud rate for Windows :
- 110
- 300
- 600
- 1200
- 2400
- 4800
- 9600
- 14400
- 19200
- 38400
- 56000
- 57600
- 115200
- 128000
- 256000

\n Supported baud rate for Linux :\n
- 110
- 300
- 600
- 1200
- 2400
- 4800
- 9600
- 19200
- 38400
- 57600
- 115200

\return 1 success
\return -1 device not found
\return -2 error while opening the device
\return -3 error while getting port parameters
\return -4 Speed (Bauds) not recognized
\return -5 error while writing port parameters
\return -6 error while writing timeout parameters
*/
char rOc_serial::openDevice(const char *Device,const unsigned int Bauds)
{
#if defined (_WIN32) || defined( _WIN64)

    // Open serial port
    hSerial = CreateFileA(  Device,GENERIC_READ | GENERIC_WRITE,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
    if(hSerial==INVALID_HANDLE_VALUE) {
        if(GetLastError()==ERROR_FILE_NOT_FOUND)
            return -1;                                                  // Device not found
        return -2;                                                      // Error while opening the device
    }

    // Set parameters
    DCB dcbSerialParams = {0};                                          // Structure for the port parameters
    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))                       // Get the port parameters
        return -3;                                                      // Error while getting port parameters
    switch (Bauds)                                                      // Set the speed (Bauds)
    {
    case 110  :     dcbSerialParams.BaudRate=CBR_110; break;
    case 300  :     dcbSerialParams.BaudRate=CBR_300; break;
    case 600  :     dcbSerialParams.BaudRate=CBR_600; break;
    case 1200 :     dcbSerialParams.BaudRate=CBR_1200; break;
    case 2400 :     dcbSerialParams.BaudRate=CBR_2400; break;
    case 4800 :     dcbSerialParams.BaudRate=CBR_4800; break;
    case 9600 :     dcbSerialParams.BaudRate=CBR_9600; break;
    case 14400 :    dcbSerialParams.BaudRate=CBR_14400; break;
    case 19200 :    dcbSerialParams.BaudRate=CBR_19200; break;
    case 38400 :    dcbSerialParams.BaudRate=CBR_38400; break;
    case 56000 :    dcbSerialParams.BaudRate=CBR_56000; break;
    case 57600 :    dcbSerialParams.BaudRate=CBR_57600; break;
    case 115200 :   dcbSerialParams.BaudRate=CBR_115200; break;
    case 128000 :   dcbSerialParams.BaudRate=CBR_128000; break;
	case 153600:   dcbSerialParams.BaudRate = 153600; break;
    case 230400:   dcbSerialParams.BaudRate = 230400; break;
    case 256000 :   dcbSerialParams.BaudRate=CBR_256000; break;
    case 1500000 :   dcbSerialParams.BaudRate=1500000; break;
    default : return -4;
    }
    dcbSerialParams.ByteSize=8;                                         // 8 bit data
    dcbSerialParams.StopBits=ONESTOPBIT;                                // One stop bit
    dcbSerialParams.Parity=NOPARITY;                                    // No parity
    if(!SetCommState(hSerial, &dcbSerialParams))                        // Write the parameters
        return -5;                                                      // Error while writing

    // Set TimeOut
    timeouts.ReadIntervalTimeout=0;                                     // Set the Timeout parameters
    timeouts.ReadTotalTimeoutConstant=MAXDWORD;                         // No TimeOut
    timeouts.ReadTotalTimeoutMultiplier=0;
    timeouts.WriteTotalTimeoutConstant=MAXDWORD;
    timeouts.WriteTotalTimeoutMultiplier=0;
    if(!SetCommTimeouts(hSerial, &timeouts))                            // Write the parameters
        return -6;                                                      // Error while writting the parameters


#endif
#ifdef __linux__
    struct termios options;                                             // Structure with the device's options


    // Open device
    m_fd = open(Device, O_RDWR | O_NOCTTY | O_NDELAY);                    // Open port
    if (m_fd == -1)
    {
        printf("errno=%d\n",errno);
        return -2; 
    }                                                                   // If the device is not open, return -1
    
    fcntl(m_fd, F_SETFL, FNDELAY);                                        // Open the device in nonblocking mode

    // Set parameters
    tcgetattr(m_fd, &options);                                            // Get the current options of the port
    bzero(&options, sizeof(options));                                   // Clear all the options
    speed_t         Speed;
    switch (Bauds)                                                      // Set the speed (Bauds)
    {
    case 110  :     Speed=B110; break;
    case 300  :     Speed=B300; break;
    case 600  :     Speed=B600; break;
    case 1200 :     Speed=B1200; break;
    case 2400 :     Speed=B2400; break;
    case 4800 :     Speed=B4800; break;
    case 9600 :     Speed=B9600; break;
    case 19200 :    Speed=B19200; break;
    case 38400 :    Speed=B38400; break;
    case 57600 :    Speed=B57600; break;
    case 115200 :   Speed=B115200; break;
    case 230400 :   Speed=B230400; break;
    default : return -4;
    }
    cfsetispeed(&options, Speed);                                       // Set the baud rate at 115200 bauds
    cfsetospeed(&options, Speed);
    options.c_cflag |= ( CLOCAL | CREAD |  CS8);                        // Configure the device : 8 bits, no parity, no control
    options.c_iflag |= ( IGNPAR | IGNBRK );
    options.c_cc[VTIME]=0;                                              // Timer unused
    options.c_cc[VMIN]=0;                                               // At least on character before satisfy reading
    tcsetattr(m_fd, TCSANOW, &options);                                   // Activate the settings


#endif

    flushReceiver();
    mHandlesOpen_ = true;
    return 1;                                                           // Opening successfull
}


/*!
\brief Close the connection with the current device
*/
void rOc_serial::closeDevice()
{
    if (!mHandlesOpen_)
    {
        return;
    }
#if defined (_WIN32) || defined( _WIN64)
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hSerial);
    }
#endif
#ifdef __linux__
    close (m_fd);
    m_fd = 0;
#endif

    mHandlesOpen_ = false;
}

char rOc_serial::writeChar(const char Byte)
{
#if defined (_WIN32) || defined( _WIN64)
    DWORD dwBytesWritten;                                               // Number of bytes written
    if(!WriteFile(hSerial,&Byte,1,&dwBytesWritten,NULL))                // Write the char
        return -1;                                                      // Error while writing
    return 1;                                                           // Write operation successfull
#endif
#ifdef __linux__
    if (write(m_fd,&Byte,1)!=1)                                           // Write the char
        return -1;                                                      // Error while writting
    return 1;                                                           // Write operation successfull
#endif
}


int rOc_serial::readData( unsigned char *Buffer, unsigned int expectedBytes,int iTimeoutms)
{
    #if defined (_WIN32) || defined(_WIN64)
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        return -5;
    }

    timeouts.ReadTotalTimeoutConstant = iTimeoutms;                       // Set the TimeOut
    if (!SetCommTimeouts(hSerial, &timeouts))                            // Write the parameters
        return -4;                                                      // Error while writting the parameters

    DWORD dwBytesRead = 0;
    if (!ReadFile(hSerial, Buffer, expectedBytes, &dwBytesRead, NULL))                 // Read the byte
    {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return -1;                                                      // Error while reading the byte
    }
    return dwBytesRead;
    #endif

    #ifdef __linux__
    
    int nfds;
    int nread = 0 ;
    
    fd_set readfds;
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = iTimeoutms*1000;//10ms
    
    FD_ZERO(&readfds);
    FD_SET(m_fd,&readfds);
    
    nfds = select(m_fd+1,&readfds,NULL,NULL,&tv);

    if(nfds == 0) 
    {
        //printf("timeout!\r\n");
        return 0;
    }
    else
    {
        nread = read(m_fd , Buffer, expectedBytes);//即使不满desire_get_len，也会返回实际读取到的数据量
        return nread;
    }
    
    #endif
}

int rOc_serial::readChar(unsigned char *pByte, unsigned int TimeOut_ms)
{
    return readChars(pByte, 1, TimeOut_ms);
}

int rOc_serial::readChars(unsigned char *Buffer, unsigned int MaxNbBytes, unsigned int TimeOut_ms)
{
    int readBytes = 0;
    
    double timeElased = 0;
    int64_t startElased = GetTimeStamp();
    unsigned int alreadyReadBytes = 0;
    
    while (timeElased < TimeOut_ms && alreadyReadBytes != MaxNbBytes)
    {
        
        readBytes = readData(Buffer + alreadyReadBytes, MaxNbBytes - alreadyReadBytes);
        if (readBytes < 0)
        {
            printf("error:%d\n", readBytes);
            return readBytes;
        }

        alreadyReadBytes += readBytes;
        timeElased = (GetTimeStamp() - startElased) / 1.0e6;
        
    }

    if (alreadyReadBytes != 0 && alreadyReadBytes != MaxNbBytes)
    {
        return -3;
    }

    return alreadyReadBytes;
}

// _________________________
// ::: Special operation :::

/*!
\brief Empty receiver buffer (UNIX only)
*/

void rOc_serial::flushReceiver()
{
#ifdef __linux__
    tcflush(m_fd, TCIOFLUSH);//tcflush(m_fd,TCIFLUSH);//tcflush(m_fd, TCIOFLUSH);
#endif
}

/*!
\brief  Return the number of bytes in the received buffer (UNIX only)
\return The number of bytes in the received buffer
*/
int rOc_serial::peekReceiver()
{
    int Nbytes = 0;

#if defined (_WIN32) || defined(_WIN64)
    DWORD   errors = CE_IOE;
    COMSTAT commStat;

    if(!ClearCommError(hSerial, &errors, &commStat))
        Nbytes = 0;
    else
        Nbytes = commStat.cbInQue;
#endif
#ifdef __linux__
    ioctl(m_fd, FIONREAD, &Nbytes);
#endif
    return Nbytes;
}

// __________________
// ::: I/O Access :::

/*!
\brief      Set or unset the bit DTR
\param      Status=true  set DTR
Status=false unset DTR
*/
void rOc_serial::DTR(bool Status)
{
#if defined (_WIN32) || defined(_WIN64)
    if(Status)
        EscapeCommFunction(hSerial, SETDTR);
    else
        EscapeCommFunction(hSerial, CLRDTR);
#endif
#ifdef __linux__

    int status_DTR=0;
    ioctl(m_fd, TIOCMGET, &status_DTR);
    if (Status)
        status_DTR |= TIOCM_DTR;
    else
        status_DTR &= ~TIOCM_DTR;
    ioctl(m_fd, TIOCMSET, &status_DTR);
#endif
}



/*!
\brief      Set or unset the bit RTS
\param      Status=true  set RTS
Status=false unset RTS
*/
void rOc_serial::RTS(bool Status)
{
#if defined (_WIN32) || defined(_WIN64)
    if(Status)
        EscapeCommFunction(hSerial, SETRTS);
    else
        EscapeCommFunction(hSerial, CLRRTS);
#endif
#ifdef __linux__
    int status_RTS=0;
    ioctl(m_fd, TIOCMGET, &status_RTS);
    if (Status)
        status_RTS |= TIOCM_RTS;
    else
        status_RTS &= ~TIOCM_RTS;
    ioctl(m_fd, TIOCMSET, &status_RTS);
#endif
}




/*!
\brief      Get the CTS's status
\return     Return true if CTS is set otherwise false
*/
bool rOc_serial::isCTS()
{
#if defined (_WIN32) || defined(_WIN64)
    DWORD dwModemStatus;

    if (!GetCommModemStatus(hSerial, &dwModemStatus))
        return false;
    if(MS_CTS_ON & dwModemStatus)
        return true;
    return false;
#endif
#ifdef __linux__
    int status=0;
    //Get the current status of the CTS bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_CTS;
#endif
}



/*!
\brief      Get the CTS's status
\return     Return true if CTS is set otherwise false
*/
bool rOc_serial::isDTR()
{
#if defined (_WIN32) || defined(_WIN64)
    DWORD dwModemStatus;

    if (!GetCommModemStatus(hSerial, &dwModemStatus))
        return false;
    if(MS_DSR_ON & dwModemStatus)
        return true;
    return false;
#endif
#ifdef __linux__
    int status=0;
    //Get the current status of the CTS bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_DTR  ;
#endif
}

/*!
\brief      Get the CTS's status
\return     Return true if CTS is set otherwise false
*/
bool rOc_serial::isRTS()
{
#if defined (_WIN32) || defined(_WIN64)
    DWORD dwModemStatus;

    if (!GetCommModemStatus(hSerial, &dwModemStatus))
        return false;
    if(MS_CTS_ON & dwModemStatus)
        return true;
    return false;
#endif
#ifdef __linux__
    int status=0;
    //Get the current status of the CTS bit
    ioctl(m_fd, TIOCMGET, &status);
    return status & TIOCM_RTS;
#endif
}

int64_t rOc_serial::GetTimeStamp()
{
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::nanoseconds ns = now.time_since_epoch();

    return ns.count();
}
