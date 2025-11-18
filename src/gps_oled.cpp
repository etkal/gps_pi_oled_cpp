/*
 * GPS using OLED display
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

#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <thread>

#include "ssd1306.h"
#include "gps_oled.h"

#include "ws281x/ws2811.h"

#define SAT_ICON_RADIUS 2

#if defined(CHAR_WIDTH)
#undef CHAR_WIDTH
#endif

constexpr uint CHAR_WIDTH  = 8;
constexpr uint CHAR_HEIGHT = 8;
constexpr uint LINE_HEIGHT = (CHAR_HEIGHT + 1);
constexpr uint COL_WIDTH   = (CHAR_WIDTH);
constexpr uint PAD_CHARS   = 0;
constexpr uint X_PAD       = 0;

constexpr double pi = 3.14159265359;

GPS_OLED::GPS_OLED(SSD1306::Shared spDisplay, GPS::Shared spGPS, LED::Shared spLED, float GMToffset)
    : m_spDisplay(spDisplay),
      m_spGPS(spGPS),
      m_spLED(spLED),
      m_GMToffset(GMToffset)
{
}

GPS_OLED::~GPS_OLED()
{
}

void GPS_OLED::Initialize()
{
    m_spDisplay->Reset();
    m_spDisplay->Initialize();

    m_spDisplay->SetContrast(0x10);
    m_spDisplay->Fill(COLOUR_BLACK);
    drawText(0, "Waiting for GPS", COLOUR_WHITE, false, 0);
    m_spDisplay->Show();

    m_spGPS->SetSentenceCallback(this, sentenceCB);
    m_spGPS->SetGpsDataCallback(this, gpsDataCB);
}

void GPS_OLED::Run()
{
    m_spGPS->Run();
}

void GPS_OLED::Stop()
{
    printf("\nCleaning up and exiting...\n");
    m_spGPS->Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    m_spDisplay->Fill(COLOUR_BLACK);
    m_spDisplay->Show();
}

void GPS_OLED::sentenceCB(void* pCtx, std::string strSentence)
{
    // printf("sentenceCB received: %s\n", strSentence.c_str());
}

void GPS_OLED::gpsDataCB(void* pCtx, GPSData::Shared spGPSData)
{
    GPS_OLED* pThis = reinterpret_cast<GPS_OLED*>(pCtx);
    pThis->updateUI(spGPSData);
}

void GPS_OLED::updateUI(GPSData::Shared spGPSData)
{
    m_spGPSData = spGPSData;
    if (m_spLED)
    {
        if (m_spGPSData->bHasPosition)
        {
            m_spLED->SetPixel(0, m_spGPSData->bExternalAntenna ? led_blue : led_green);
        }
        else
        {
            m_spLED->SetPixel(0, led_red);
        }
        m_spLED->Blink_ms(10);
    }

    uint16_t nWidth  = m_spDisplay->Width();
    uint16_t nHeight = m_spDisplay->Height();

    m_spDisplay->Fill(COLOUR_BLACK);

    // Draw satellite grid
    drawSatGrid(nWidth / 4, nHeight / 2, nHeight / 2 - CHAR_HEIGHT / 2, 2);

    // Draw fix and #sats text
    drawText(0, spGPSData->strMode3D, COLOUR_WHITE, false, X_PAD);
    drawText(-1, m_spGPSData->bExternalAntenna ? "*" : " ", COLOUR_WHITE, false, X_PAD);
    drawText(3, spGPSData->strNumSats, COLOUR_WHITE, true, X_PAD);

    if (!spGPSData->strGPSTime.empty())
    {
        drawText(-1, spGPSData->strGPSTime, COLOUR_WHITE, true, X_PAD);
    }
    if (!spGPSData->strLatitude.empty())
    {
        drawText(0, spGPSData->strLatitude, COLOUR_WHITE, true, X_PAD);
        drawText(1, spGPSData->strLongitude, COLOUR_WHITE, true, X_PAD);
        drawText(2, spGPSData->strAltitude, COLOUR_WHITE, true, X_PAD);
        drawText(4, spGPSData->strSpeed, COLOUR_WHITE, true, X_PAD);
    }

    // blit the framebuf to the display
    m_spDisplay->Show();

    m_spGPSData.reset();
}

void GPS_OLED::drawSatGrid(uint xCenter, uint yCenter, uint radius, uint nRings)
{
    for (uint i = 1; i <= nRings; ++i)
    {
        m_spDisplay->Ellipse(xCenter, yCenter, radius * i / nRings, radius * i / nRings, COLOUR_WHITE);
    }

    m_spDisplay->VLine(xCenter, yCenter - radius - 2, 2 * radius + 5, COLOUR_WHITE);
    m_spDisplay->HLine(xCenter - radius - 2, yCenter, 2 * radius + 5, COLOUR_WHITE);
    // m_spDisplay->Text("N", xCenter - CHAR_WIDTH / 2, yCenter - radius - CHAR_HEIGHT, COLOUR_RED);
    m_spDisplay->Text("'", xCenter - 6, yCenter - radius - CHAR_HEIGHT / 2, COLOUR_RED);
    m_spDisplay->Text("`", xCenter - 2, yCenter - radius - CHAR_HEIGHT / 2, COLOUR_RED);

    int satRadius = SAT_ICON_RADIUS / 2;
    if (!m_spGPSData->strLatitude.empty())
    {
        satRadius = SAT_ICON_RADIUS;
    }
    for (auto oEntry : m_spGPSData->mSatList)
    {
        auto oSat    = oEntry.second;
        double elrad = oSat.m_el * pi / 180;
        double azrad = oSat.m_az * pi / 180;
        drawCircleSat(xCenter, yCenter, radius, elrad, azrad, satRadius, COLOUR_WHITE, COLOUR_BLACK);
        for (auto nSat : m_spGPSData->vUsedList)
        {
            if (oSat.m_num == nSat)
            {
                drawCircleSat(xCenter, yCenter, radius, elrad, azrad, satRadius, COLOUR_WHITE, COLOUR_BLUE);
                break;
            }
        }
    }
}

void GPS_OLED::drawCircleSat(uint gridCenterX,
                             uint gridCenterY,
                             uint nGridRadius,
                             float elrad,
                             float azrad,
                             uint satRadius,
                             uint16_t color,
                             uint16_t fillColor)
{
    // Draw satellite (fill first, then draw open circle)
    int dx = (nGridRadius - SAT_ICON_RADIUS) * cos(elrad) * sin(azrad);
    int dy = (nGridRadius - SAT_ICON_RADIUS) * cos(elrad) * -cos(azrad);
    int x  = gridCenterX + dx;
    int y  = gridCenterY + dy;
    m_spDisplay->Ellipse(x, y, satRadius, satRadius, fillColor, true); // Clear area with fill
    m_spDisplay->Ellipse(x, y, satRadius, satRadius, color);           // Draw circle without fill
}

int GPS_OLED::linePos(int nLine)
{
    if (nLine >= 0)
        return nLine * LINE_HEIGHT;
    else
        return m_spDisplay->Height() + 1 + (nLine * LINE_HEIGHT);
}

void GPS_OLED::drawText(int nLine, std::string strText, uint16_t color, bool bRightAlign, uint nRightPad)
{
    int x = (!bRightAlign) ? 0 : m_spDisplay->Width() - (strText.length() * COL_WIDTH);
    int y = linePos(nLine);
    x     = x - nRightPad;
    m_spDisplay->Text(strText.c_str(), x, y, color);
}
