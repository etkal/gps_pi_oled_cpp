/*
 * Pico LED class
 *
 * (c) 2025 Erik Tkal
 *
 */

#pragma once

#include <vector>
#include <memory>

auto constexpr max_lum = 100;

static inline constexpr uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)(r * max_lum / 256) << 16) | ((uint32_t)(g * max_lum / 256) << 8) | (uint32_t)(b * max_lum / 256);
}

auto constexpr led_white = urgb_u32(0x80, 0x80, 0x80);
auto constexpr led_on    = urgb_u32(0x80, 0x80, 0x80);
auto constexpr led_black = urgb_u32(0, 0, 0);
auto constexpr led_off   = urgb_u32(0, 0, 0);

auto constexpr led_red     = urgb_u32(0x80, 0, 0);
auto constexpr led_green   = urgb_u32(0, 0x80, 0);
auto constexpr led_blue    = urgb_u32(0, 0, 0x80);
auto constexpr led_cyan    = urgb_u32(0, 0x80, 0x80);
auto constexpr led_magenta = urgb_u32(0x80, 0, 0x80);
auto constexpr led_yellow  = urgb_u32(0x80, 0x80, 0);

class LED
{
public:
    typedef std::shared_ptr<LED> Shared;

    LED() {};
    virtual ~LED() {};

    virtual int Initialize()                          = 0;
    virtual void On()                                 = 0;
    virtual void Off()                                = 0;
    virtual void SetPixel(size_t idx, uint32_t color) = 0;
    virtual void SetIgnore(std::vector<uint32_t> vIgnore) {};
    void Blink_ms(size_t nMs = 50);
};

class LED_neo : public LED
{
public:
    LED_neo(size_t numLEDs, bool bIsRGBW = false);
    virtual ~LED_neo();

    int Initialize() override;
    void On() override;
    void Off() override;
    void SetPixel(size_t idx, uint32_t color) override;

private:
    bool m_bInitialized;
    size_t m_nNumLEDs;
    bool m_bIsRGBW;
    std::vector<uint32_t> m_vPixels;
};
