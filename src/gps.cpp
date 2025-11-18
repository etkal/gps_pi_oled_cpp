/*
 * GPS class
 *
 * Copyright (c) 2023-2025 Erik Tkal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "gps.h"

#include <queue>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

typedef enum eSentenceType
{
    kGPGGA,
    kGPGSA,
    kGPGSV,
    kGPRMC,
    kGPVTG,
    kPGTOP,
    kPCD,
} eSentenceType;

static std::map<std::string, eSentenceType> g_SentenceTypeMap = {
    {"$GPGGA", kGPGGA},
    {"$GPGSA", kGPGSA},
    {"$GPGSV", kGPGSV},
    {"$GPRMC", kGPRMC},
    {"$GPVTG", kGPVTG},
    {"$PGTOP", kPGTOP},
    {"$PCD",   kPCD  },
};

// Static members for RX
char GPS::sm_szBuffer[GPS_BUFSIZE];
size_t GPS::sm_iHead      = 0;
size_t GPS::sm_iNext      = 0;
size_t GPS::sm_nSentences = 0;

GPS::GPS(boost::asio::io_context& ioc, const char* szDevice)
    : m_ioc(ioc),
      m_strDevice(szDevice),
      m_serialPort(-1),
      m_bExit(false),
      m_bGSVInProgress(false),
      m_bSendGpsData(false),
      m_pSentenceCallBack(nullptr),
      m_pSentenceCtx(nullptr),
      m_pGpsDataCallback(nullptr),
      m_pGpsDataCtx(nullptr)
{
}

GPS::~GPS()
{
}

void GPS::SetSentenceCallback(void* pCtx, sentenceCallback pCB)
{
    m_pSentenceCtx      = pCtx;
    m_pSentenceCallBack = pCB;
}

void GPS::SetGpsDataCallback(void* pCtx, gpsDataCallback pCB)
{
    m_pGpsDataCtx      = pCtx;
    m_pGpsDataCallback = pCB;
}

int GPS::Initialize()
{
    // Open the GPS serial port
    m_serialPort = open(m_strDevice.c_str(), O_RDWR | O_NOCTTY);

    // Check for errors
    if (m_serialPort < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }

    // Configure serial port settings
    struct termios tty;
    if (tcgetattr(m_serialPort, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        close(m_serialPort);
        return -1;
    }

    cfsetispeed(&tty, B9600); // Set input baud rate
    cfsetospeed(&tty, B9600); // Set output baud rate

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;  // Clear data size bits
    tty.c_cflag |= CS8;     // 8 data bits

    tty.c_cflag &= ~CRTSCTS;       // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~ICANON; // Disable canonical mode (raw input)
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Disable software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/newline

    tty.c_cc[VTIME] = 1; // Wait for up to 0.1s
    tty.c_cc[VMIN]  = 0; // Minimum number of characters to read

    if (tcsetattr(m_serialPort, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(m_serialPort);
        return -1;
    }
    return 0;
}

void GPS::Run()
{
    std::string strSentence;
    bool bSentAntennaCommands = false;
    while (!m_bExit)
    {
        receiveData();

        // Read sentences from GPS device
        while (getSentence(strSentence))
        {
            bool bValidSentenceRead = processSentence(strSentence);

            if (!bSentAntennaCommands && bValidSentenceRead)
            {
                printf("Sending antenna commands\n");
                // Write commands to enable reporting external vs internal antenna.  We wait
                // until some data is received to ensure the GPS has finished initializing.
                std::string strPGCMD("$PGCMD,33,1*6C\r\n"); // Enable antenna output for PA6H
                std::string strCDCMD("$CDCMD,33,1*7C\r\n"); // Enable antenna output for PA1616S
                write(m_serialPort, strPGCMD.c_str(), strPGCMD.size());
                write(m_serialPort, strCDCMD.c_str(), strCDCMD.size());
                bSentAntennaCommands = true;
            }
        }

        if (m_bSendGpsData)
        {
            m_bSendGpsData = false;
            if (NULL != m_pGpsDataCallback)
            {
                (*m_pGpsDataCallback)(m_pGpsDataCtx, m_spGPSData);
            }
        }
    }
}

bool GPS::processSentence(std::string strSentence)
{
    // Validate the string
    if (!validateSentence(strSentence))
    {
        return false;
    }
    printf("%s\n", strSentence.c_str());

    if (NULL != m_pSentenceCallBack)
    {
        (*m_pSentenceCallBack)(m_pSentenceCtx, strSentence);
    }

    if (!m_spGPSData)
    {
        // Guarantee we have an object to update
        m_spGPSData           = std::make_shared<GPSData>();
        m_spGPSData->mSatList = m_mSatListPersistent; // restore any previous data
    }

    std::vector<std::string> vElems;
    std::stringstream s_stream(strSentence);
    while (s_stream.good())
    {
        std::string substr;
        getline(s_stream, substr, ','); // get first string delimited by comma
        vElems.push_back(substr);
    }

    auto timeDiff = std::chrono::system_clock::now() - m_nSatListTime;
    long lDiff    = static_cast<long>(std::chrono::duration_cast<std::chrono::seconds>(timeDiff).count());
    if (lDiff > 30) // Nothing in 30 seconds, clear vectors
    {
        if (!m_spGPSData->mSatList.empty())
        {
            printf("Clearing vectors\n");
            m_spGPSData->mSatList.clear();
            m_spGPSData->vUsedList.clear();
        }
    }

    if (vElems.size() == 0)
    {
        printf("No elements found\n");
        return false;
    }

    if (g_SentenceTypeMap.find(vElems[0]) == g_SentenceTypeMap.end())
    {
        // printf("Ignoring %s\n", vElems[0].c_str());
        return false;
    }

    auto type = g_SentenceTypeMap.at(vElems[0]);

    if (m_bGSVInProgress && type != kGPGSV) // Did not complete
    {
        m_bGSVInProgress = false;
        m_mSatListIncoming.clear();
    }

    switch (type)
    {
    case kGPGGA: // Global Positioning System Fix Data
    {
        m_bSendGpsData = true;
        if (!vElems[7].empty())
        {
            m_spGPSData->strNumSats = "Sat: " + vElems[7];
        }
        if (!vElems[9].empty())
        {
            double dMeters = std::stod(vElems[9].c_str());
            std::stringstream oss;
            if (dMeters < 1000.0)
            {
                oss << std::fixed << std::setfill(' ') << std::setprecision(1) << dMeters << "m";
            }
            else
            {
                oss << std::setfill(' ') << std::setprecision(0) << dMeters << "m";
            }
            m_spGPSData->strAltitude = oss.str();
        }
        break;
    }
    case kGPGSA: // GPS DOP and active satellites
    {
        m_spGPSData->vUsedList.clear();
        m_spGPSData->strMode3D = vElems[2] + "D";
        if (vElems[2] == "1")
        {
            m_spGPSData->strMode3D = "";
        }
        for (int i = 3; i < 15; ++i)
        {
            if (!vElems[i].empty())
            {
                uint satNum = atoi(vElems[i].c_str());
                if (satNum != 0)
                {
                    m_spGPSData->vUsedList.push_back(satNum);
                }
            }
            else
            {
                break;
            }
        }
        break;
    }
    case kGPGSV: // GPS Satellites in view
    {
        // Multipart, clear any previous data and re-gather
        if (vElems[2] == "1")
        {
            m_mSatListIncoming.clear();
            m_strNumGSV      = vElems[1];
            m_bGSVInProgress = true;
        }
        int nNumSatsInGSV = std::min(4, atoi(vElems[3].c_str()) - 4 * (atoi(vElems[2].c_str()) - 1));
        if (m_bGSVInProgress)
        {
            for (int i = 4; i < 4 + 4 * nNumSatsInGSV; i += 4)
            {
                if (!vElems[i].empty() && !vElems[i + 1].empty() && !vElems[i + 2].empty())
                {
                    uint num  = atoi(vElems[i].c_str());
                    uint el   = atoi(vElems[i + 1].c_str());
                    uint az   = atoi(vElems[i + 2].c_str());
                    uint rssi = vElems[i + 3].empty() ? 0 : atoi(vElems[i + 3].c_str());
                    m_mSatListIncoming.emplace(std::make_pair(num, SatInfo(num, el, az, rssi)));
                }
            }
            if (vElems[2] == m_strNumGSV) // Last one received
            {
                m_bGSVInProgress      = false;
                m_nSatListTime        = std::chrono::system_clock::now();
                m_spGPSData->mSatList = m_mSatListIncoming;
                m_mSatListPersistent  = m_spGPSData->mSatList; // Persist the list
            }
        }
        break;
    }
    case kGPRMC: // Recommended minimum specific GPS/Transit data
    {
        if (!vElems[1].empty())
        {
            std::string& t          = vElems[1];
            m_spGPSData->strGPSTime = t.substr(0, 2) + ":" + t.substr(2, 2) + ":" + t.substr(4, 2) + "Z";
        }
        else
        {
            m_spGPSData->strGPSTime = "";
        }
        if (vElems[2] == "A")
        {
            if (!vElems[3].empty() && !vElems[4].empty() && !vElems[5].empty() && !vElems[6].empty())
            {
                m_spGPSData->bHasPosition = true;
                m_spGPSData->strLatitude  = convertToDegrees(vElems[3], 7) + vElems[4];
                m_spGPSData->strLongitude = convertToDegrees(vElems[5], 8) + vElems[6];
            }
            if (!vElems[7].empty())
            {
                double dKnots = std::stod(vElems[7].c_str());
                double dMph   = dKnots * 1.15078;
                std::stringstream oss;
                if (dMph < 10.0)
                {
                    oss << std::fixed << std::setfill(' ') << std::setprecision(1) << dMph << "mph";
                }
                else
                {
                    oss << std::setfill(' ') << std::setprecision(0) << dMph << "mph";
                }
                m_spGPSData->strSpeed = oss.str();
            }
        }
        else
        {
            m_spGPSData->bHasPosition = false;
        }
        break;
    }
    case kPGTOP: // PA6H External antenna info
    {
        if (vElems[2] == "2")
        {
            m_spGPSData->bExternalAntenna = false;
        }
        if (vElems[2] == "3")
        {
            m_spGPSData->bExternalAntenna = true;
        }
        break;
    }
    case kPCD: // PA1616S External antenna info
    {
        if (vElems[2] == "1")
        {
            m_spGPSData->bExternalAntenna = false;
        }
        if (vElems[2] == "2")
        {
            m_spGPSData->bExternalAntenna = true;
        }
        break;
    }
    default:
        break;
    }
    return true;
}

bool GPS::validateSentence(std::string& strSentence)
{
    // Validate format and remove checksum and CRLF
    size_t nLen = strSentence.size();
    if (nLen < 1 || strSentence[0] != '$')
    {
        return false;
    }
    if (nLen < 6 || strSentence.substr(nLen - 2, 2) != "\r\n" || strSentence[nLen - 5] != '*')
    {
        return false;
    }
    std::string specifiedCheck  = strSentence.substr(nLen - 4, 2);
    std::string calculatedCheck = checkSum(strSentence.substr(1, nLen - 6));
    if (calculatedCheck != specifiedCheck)
    {
        return false;
    }

    strSentence = strSentence.substr(0, nLen - 5);

    return true;
}

std::string GPS::checkSum(const std::string& strSentence)
{
    uint8_t check = 0;
    for (const char& c : strSentence)
    {
        check ^= (uint8_t)c;
    }
    std::stringstream oss;
    oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (unsigned int)check;
    return oss.str();
}

std::string GPS::convertToDegrees(std::string strRaw, int width)
{
    // Convert (D)DDMM.mmmm to decimal degrees
    double dRawAsDouble = stod(strRaw);
    int firstdigits     = int(dRawAsDouble / 100);
    int nexttwodigits   = dRawAsDouble - double(firstdigits * 100);
    double converted    = double(firstdigits) + nexttwodigits / 60.0;
    std::stringstream oss;
    oss << std::fixed << std::setw(width) << std::setfill(' ') << std::setprecision(4) << converted;
    return oss.str();
}

void GPS::receiveData()
{
    char read_buf[1024];
    auto num_bytes = read(m_serialPort, &read_buf, sizeof(read_buf));

    if (num_bytes <= 0)
    {
        // error or no data
        return;
    }

    for (auto i = 0; i < num_bytes; ++i)
    {
        auto ch                 = read_buf[i];
        sm_szBuffer[sm_iNext++] = ch;
        sm_iNext %= GPS_BUFSIZE;
        if (ch == '\n')
        {
            sm_szBuffer[sm_iNext++] = '\0';
            sm_iNext %= GPS_BUFSIZE;
            sm_nSentences += 1;
        }
    }
}

bool GPS::getSentence(std::string& strSentence)
{
    bool bFound = false;
    if (sm_nSentences > 0)
    {
        strSentence.clear();
        for (size_t i = sm_iHead; '\0' != sm_szBuffer[i]; i = (i + 1) % GPS_BUFSIZE)
        {
            strSentence += sm_szBuffer[i];
        }
        sm_iHead = (sm_iHead + strSentence.length() + 1) % GPS_BUFSIZE;
        sm_nSentences -= 1;
        bFound = true;
    }

    return bFound;
}
