/*******************************************************************************
 *
 * MIT License
 *
 * Copyright 2019-2020 Advanced Micro Devices, Inc.
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *******************************************************************************/

#pragma once

#include <chrono>
#include <cstddef>
#include <future>
#include <thread>
#include <tuple>
#include <vector>

#include <hip/hip_runtime.h>
//#include <rocm_smi/rocm_smi.h>

typedef enum {
  RSMI_CLK_TYPE_SYS = 0x0,            //!< System clock
  RSMI_CLK_TYPE_FIRST = RSMI_CLK_TYPE_SYS,
  RSMI_CLK_TYPE_DF,                   //!< Data Fabric clock (for ASICs
                                      //!< running on a separate clock)
  RSMI_CLK_TYPE_DCEF,                 //!< Display Controller Engine clock
  RSMI_CLK_TYPE_SOC,                  //!< SOC clock
  RSMI_CLK_TYPE_MEM,                  //!< Memory clock

  // Add new clocks to the end (not in the middle) and update
  // RSMI_CLK_TYPE_LAST
  RSMI_CLK_TYPE_LAST = RSMI_CLK_TYPE_MEM,
  RSMI_CLK_INVALID = 0xFFFFFFFF
} rsmi_clk_type_t;

#define RSMI_STATUS_SUCCESS (1)
typedef int rsmi_status_t;
typedef int rsmi_frequencies_t;
#define rsmi_init(...) (RSMI_STATUS_SUCCESS)
#define rsmi_num_monitor_devices(...)
#define rsmi_dev_pci_id_get(...)
#define rsmi_dev_temp_metric_get(...) (RSMI_STATUS_SUCCESS)
#define rsmi_status_string(...)
#define rsmi_dev_gpu_clk_freq_get(...) (RSMI_STATUS_SUCCESS);
#define rsmi_dev_fan_rpms_get(...) (RSMI_STATUS_SUCCESS)

namespace Tensile
{
    namespace Client
    {
        /**
 * Monitors properties of a particular GPU in a separate thread.
 *
 * The thread is manually managed because the thread creation overhead is too
 * high to create a thread every time.
 *
 * The interface to this class is not thread-safe.
 */
        class HardwareMonitor
        {
        public:
            /** Translates the Hip device index into the corresponding device index for
   * ROCm-SMI. */
            static uint32_t GetROCmSMIIndex(int hipDeviceIndex);

            using rsmi_temperature_type_t = int;
            using rsmi_temperature_metric_t = int;
            using rsmi_clk_type_t = int;
            #define RSMI_TEMP_CURRENT (0)
            using clock                   = std::chrono::steady_clock;

            // Monitor at the maximum possible rate.
            HardwareMonitor(int hipDeviceIndex);
            // Limit collection to once per minPeriod.
            HardwareMonitor(int hipDeviceIndex, clock::duration minPeriod);

            ~HardwareMonitor();

            void addTempMonitor(rsmi_temperature_type_t   sensorType = 0,
                                rsmi_temperature_metric_t metric     = RSMI_TEMP_CURRENT);
            void addClockMonitor(rsmi_clk_type_t clockType);
            void addFanSpeedMonitor(uint32_t sensorIndex = 0);

            double getAverageTemp(rsmi_temperature_type_t   sensorIndex = 0,
                                  rsmi_temperature_metric_t metric      = RSMI_TEMP_CURRENT);
            double getAverageClock(rsmi_clk_type_t clockType);
            double getAverageFanSpeed(uint32_t sensorIndex = 0);
            int    getDeviceIndex()
            {
                return m_hipDeviceIndex;
            }
            size_t getSamples()
            {
                return m_dataPoints;
            }

            /// Begins monitoring until stop() is called.
            void start();

            /// Sends a signal to the monitoring thread to end monitoring.
            void stop();

            /// Begins monitoring immediately, until the event has occurred.
            void runUntilEvent(hipEvent_t event);

            /// Monitoring will occur from startEvent until stopEvent.
            void runBetweenEvents(hipEvent_t startEvent, hipEvent_t stopEvent);

            /// Waits until monitoring has finished.
            /// Throws an exception if monitoring was started without a stop event
            /// and stop() has not been called.
            void wait();

        private:
            static void InitROCmSMI();

            void assertActive();
            void assertNotActive();

            void clearValues();
            void collectOnce();
            void sleepIfNecessary();

            void initThread();
            void runLoop();
            void collect(hipEvent_t startEvent, hipEvent_t stopEvent);

            clock::time_point m_lastCollection;
            clock::time_point m_nextCollection;
            clock::duration   m_minPeriod;

            std::thread m_thread;

            std::mutex              m_mutex;
            std::condition_variable m_cv;

            using Task = std::packaged_task<void(void)>;
            Task              m_task;
            std::future<void> m_future;
            std::atomic<bool> m_exit;
            std::atomic<bool> m_stop;
            bool              m_hasStopEvent = false;

            int      m_hipDeviceIndex;
            uint32_t m_smiDeviceIndex;

            size_t m_dataPoints;

            std::vector<std::tuple<rsmi_temperature_type_t, rsmi_temperature_metric_t>>
                                 m_tempMetrics;
            std::vector<int64_t> m_tempValues;

            std::vector<rsmi_clk_type_t> m_clockMetrics;
            std::vector<uint64_t>        m_clockValues;

            std::vector<uint32_t> m_fanMetrics;
            std::vector<int64_t>  m_fanValues;
        };
    } // namespace Client
} // namespace Tensile
