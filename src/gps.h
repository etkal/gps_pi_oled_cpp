/*
 * GPS class
 *
 * (c) 2023-2025 Erik Tkal
 *
 */

#pragma once

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>

#define ChronoTimePoint std::chrono::system_clock::time_point

class SatInfo
{
public:
    SatInfo(uint num = 0, uint el = 0, uint az = 0, uint rssi = 0)
    {
        m_num  = num;
        m_el   = el;
        m_az   = az;
        m_rssi = rssi;
    }
    ~SatInfo()
    {
    }

    uint m_num;
    uint m_el;
    uint m_az;
    uint m_rssi;
};

typedef std::map<uint, SatInfo> SatList;
typedef std::vector<uint> UsedList;

class GPSData
{
public:
    typedef std::shared_ptr<GPSData> Shared;

    GPSData()
        : bHasPosition(false),
          bExternalAntenna(false)
    {
    }
    ~GPSData() = default;

    bool bHasPosition;
    bool bExternalAntenna;
    std::string strLatitude;
    std::string strLongitude;
    std::string strAltitude;
    std::string strNumSats;
    std::string strGPSTime;
    std::string strMode3D;
    std::string strSpeed;
    SatList mSatList;
    UsedList vUsedList;
};

typedef void (*sentenceCallback)(void* pCtx, std::string strSentence);
typedef void (*gpsDataCallback)(void* pCtx, GPSData::Shared spGPSData);

auto constexpr GPS_BUFSIZE            = 4096; // Circular buffer size

class GPS
{
public:
    typedef std::shared_ptr<GPS> Shared;

    GPS(const char* szDevice);
    ~GPS();

    void SetSentenceCallback(void* pCtx, sentenceCallback pCB);
    void SetGpsDataCallback(void* pCtx, gpsDataCallback pCB);
    int  Initialize();
    void Run();
    void Stop()
    {
        m_bExit = true;
    }

private:
    bool processSentence(std::string strSentence);
    bool validateSentence(std::string& strSentence);
    std::string checkSum(const std::string& strSentence);
    std::string convertToDegrees(std::string strRaw, int width);
    void receiveData();

    std::string m_strDevice;
    int m_serialPort;

    // RX buffer management
    static char sm_szBuffer[GPS_BUFSIZE];
    static size_t sm_iHead;
    static size_t sm_iNext;
    static size_t sm_nSentences;
    static bool getSentence(std::string& strSentence);

    // GPS object members
    bool m_bExit;
    bool m_bGSVInProgress;
    std::string m_strNumGSV;
    ChronoTimePoint m_nSatListTime;
    bool m_bSendGpsData;
    GPSData::Shared m_spGPSData;
    SatList m_mSatListIncoming;
    SatList m_mSatListPersistent;

    sentenceCallback m_pSentenceCallBack;
    void* m_pSentenceCtx;
    gpsDataCallback m_pGpsDataCallback;
    void* m_pGpsDataCtx;
};
