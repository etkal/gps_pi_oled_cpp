
/*
 * Copyright (c) 2025 Erik Tkal
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

#include <iostream>
#include <stdio.h>
#include <vector>
#include <thread>
#include <boost/asio.hpp>

#include "led.h"
#include "gps.h"
#include "gps_oled.h"

#include <csignal> // For sigaction, SIGINT, etc.

// #define GPS_DEVICE "/dev/ttyACM0"
#define GPS_DEVICE "/dev/ttyS0"

#define I2C_BUS    "/dev/i2c-1"
#define I2C_DEVICE 0x3C

#if !defined(GPSD_GMT_OFFSET)
#define GPSD_GMT_OFFSET 0.0
#endif

// Make the main device static so we can terminate it
static GPS_OLED::Shared sg_spDevice;

// Signal handler function
void signalHandler(int signum)
{
    sg_spDevice->Stop();
}

int main()
{
    // Handle Ctrl-C
    struct sigaction sa;
    // Initialize the sigaction struct
    sa.sa_handler = signalHandler; // Set the custom handler function
    sigemptyset(&sa.sa_mask);      // Clear the signal mask
    sa.sa_flags = 0;               // No special flags
    // Register the signal handler for SIGINT/SIGTERM
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGKILL, &sa, NULL);

    // Set up the thread pool
    boost::asio::io_context ioc;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard(ioc.get_executor());
    // Create a pool of worker threads
    const int num_threads = 2;
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i)
    {
        threads.emplace_back([&ioc]() {
            ioc.run(); // Each thread calls run() to process handlers
        });
    }

    // Create the GPS object
    GPS::Shared spGPS = std::make_shared<GPS>(ioc, GPS_DEVICE);
    if (0 != spGPS->Initialize())
    {
        return 1;
    }

    LED::Shared spLED;
#if defined(ENABLE_WS2812)
    // Create the LED object
    spLED = std::make_shared<LED_neo>(ioc, 1);
    if (0 != spLED->Initialize())
    {
        spLED.reset();
    }
#endif

    // Create the display
    SSD1306::Shared spDisplay = std::make_shared<SSD1306_I2C>(128, 64, I2C_BUS, I2C_DEVICE);

    // Create the GPS_OLED display object
    sg_spDevice = std::make_shared<GPS_OLED>(spDisplay, spGPS, spLED, GPSD_GMT_OFFSET);

    sg_spDevice->Initialize();

    // Run the show
    sg_spDevice->Run();
    work_guard.reset();

    // Wait for all worker threads to finish
    for (std::thread& t : threads)
    {
        t.join();
    }

    return 0;
}
