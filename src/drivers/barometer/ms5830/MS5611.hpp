/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <drivers/device/device.h>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>

#include "ms5611.h"

enum MS56XX_DEVICE_TYPES {
	MS5611_DEVICE	= 5611,
	MS5607_DEVICE	= 5607,
};

/* helper macro for handling report buffer indices 用于处理报表缓冲区索引的助手宏*/
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument 返回参数的平方*/
#define POW2(_x)		((_x) * (_x))

/*
 * MS5611/MS5607 internal constants and data structures. MS5611/MS5607内部常量和数据结构。
 */
#define ADDR_CMD_CONVERT_D1_OSR256		0x40	/* write to this address to start pressure conversion 写入此地址以开始压力转换*/
#define ADDR_CMD_CONVERT_D1_OSR512		0x42	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024		0x44	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048		0x46	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256		0x50	/* write to this address to start temperature conversion 写入此地址以开始温度转换*/
#define ADDR_CMD_CONVERT_D2_OSR512		0x52	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024		0x54	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048		0x56	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096		0x58	/* write to this address to start temperature conversion */

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
  使用1024的OSR来减少传感器的自热效应。来自MS的信息告诉我们，一些单独的传感器对这种影响相当敏感，降低OSR会产生很大的影响
 */
// 直接就使用OSR1024的常量
#define ADDR_CMD_CONVERT_D1			ADDR_CMD_CONVERT_D1_OSR1024
#define ADDR_CMD_CONVERT_D2			ADDR_CMD_CONVERT_D2_OSR1024

/*
 * Maximum internal conversion time for OSR 1024 is 2.28 ms. We set an update
 * rate of 100Hz which is be very safe not to read the ADC before the
 * conversion finished OSR 1024的最大内部转换时间为2.28毫秒。我们将更新速率设置为100Hz，这是非常安全的，在转换完成之前不读取ADC
 */
#define MS5611_CONVERSION_INTERVAL	10000	/* microseconds */
#define MS5611_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

class MS5611 : public I2CSPIDriver<MS5611>
{
public:
	MS5611(device::Device *interface, ms5611::prom_u &prom_buf, enum MS56XX_DEVICE_TYPES device_type,
	       I2CSPIBusOption bus_option, int bus);
	~MS5611() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int		init();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 * 执行轮询周期；从以前的测量中收集并开始新的测量一个。
	 * 这个是测量状态机的核心。此函数交替启动测量，或从上一个开始收集数据测量。
	 * 当测量之间的间隔大于最小测量间隔，在采集和测量之间插入一个间隙，以便在下一个间隔提供可能的最新测量。
	 *
	 */
	void			RunImpl();

protected:
	void print_status() override;

	PX4Barometer		_px4_barometer;

	device::Device		*_interface;

	ms5611::prom_s		_prom;

	enum MS56XX_DEVICE_TYPES _device_type;
	bool			_collect_phase{false};
	unsigned		_measure_phase{false};

	/* intermediate temperature values per MS5611/MS5607 datasheet 符合MS5611/MS5607数据表的中间温度值*/
	int64_t			_OFF{0};
	int64_t			_SENS{0};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 *       此函数在打开和出错时调用。在发生错误时，更积极地重置总线可能是有意义的。
	 */
	void			start();

	/**
	 * Issue a measurement command for the current state.
	 * 发出当前状态的测量命令。
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.采集最近的结果
	 */
	int			collect();
};
