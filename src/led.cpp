/*
 * Pico LED class
 *
 * (c) 2025 Erik Tkal
 *
 */

#include "led.h"
#include "ws281x/ws2811.h"

#include <thread>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>

// defaults for cmdline options
#define TARGET_FREQ WS2811_TARGET_FREQ
#define GPIO_PIN    18
#define DMA         10

static ws2811_t ledstring = {
    .freq   = TARGET_FREQ,
    .dmanum = DMA,
    .channel =
        {
                  [0] =
                {
                    .gpionum    = GPIO_PIN,
                    .invert     = 0,
                    .count      = 1,
                    .strip_type = WS2811_STRIP_GRB,
                    .brightness = 255,
                }, [1] =
                {
                    .gpionum    = 0,
                    .invert     = 0,
                    .count      = 0,
                    .brightness = 0,
                }, },
};

LED::LED(boost::asio::io_context& ioc)
    : m_ioc(ioc),
      m_timer(ioc) {};

void LED::Blink_ms(size_t nMs)
{
    On();
    m_timer.expires_after(std::chrono::milliseconds(nMs));
    m_timer.async_wait([this](const boost::system::error_code& error) {
        if (!error)
        {
            Off();
        }
        else
        {
            fprintf(stderr, "Timer error: %s\n", error.message().c_str());
            Off();
        }
    });
}

LED_neo::LED_neo(boost::asio::io_context& ioc, size_t numLEDs, bool bIsRGBW)
    : LED(ioc),
      m_bInitialized(false),
      m_nNumLEDs(numLEDs),
      m_bIsRGBW(bIsRGBW)
{
}

LED_neo::~LED_neo()
{
    if (!m_bInitialized)
    {
        return;
    }
    Off();
    ws2811_fini(&ledstring);
}

int LED_neo::Initialize()
{
    ledstring.channel[0].count = m_nNumLEDs;
    m_vPixels.resize(m_nNumLEDs);

    auto ret = ws2811_init(&ledstring);
    if (WS2811_SUCCESS != ret)
    {
        fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
        return -1;
    }
    m_bInitialized = true;
    Off();
    return 0;
}

void LED_neo::On()
{
    if (!m_bInitialized)
    {
        return;
    }
    for (size_t i = 0; i < m_nNumLEDs; ++i)
    {
        ledstring.channel[0].leds[i] = m_vPixels[i];
    }
    ws2811_render(&ledstring);
}

void LED_neo::Off()
{
    if (!m_bInitialized)
    {
        return;
    }
    for (size_t i = 0; i < m_nNumLEDs; ++i)
    {
        ledstring.channel[0].leds[i] = 0;
    }
    ws2811_render(&ledstring);
}

void LED_neo::SetPixel(size_t idx, uint32_t color)
{
    m_vPixels[idx] = color;
}
