/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

/**
 * @file frocectl_example.hpp
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */
#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/init.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <sched.h>
#include <uORB/uORB.h>
#include <uORB/topics/forcectl_forcedata.h>
#include <uORB/topics/forcectl_controldata.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/rc_channels.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <uORB/topics/forcectl_adrc_data.h>

/*这段代码定义了一个名为 Forcectl 的类，其中包含了 main() 方法，用于运行主要逻辑。
代码通过条件编译来处理一些力控制相关的数据（forcectl_adrc_data），并将这些数据通过ORB（PX4的发布/订阅通信机制）发布。*/

// 宏定义 SAVE_FORCECTL_ADRC_DATA 为 1，用于控制是否保存 Forcectl 相关的ADRC（自抗扰控制）数据。
// 该宏将决定代码中某些部分是否会被编译。如果 SAVE_FORCECTL_ADRC_DATA 定义为1，则相关的代码会被包含并编译，否则不会。
#define SAVE_FORCECTL_ADRC_DATA 1

// Forcectl 类是一个非常简洁的类定义，包含一个构造函数、一个析构函数和一个 main() 方法。
class Forcectl
{
public:
	// 构造函数和析构函数都是空的，意味着没有初始化或清理特殊资源的操作。
	// 这些函数在创建和销毁 Forcectl 对象时被自动调用。
	Forcectl() {}
	~Forcectl() {}

	// main() 是 Forcectl 类的主要逻辑函数，它将在 PX4_MAIN 函数中被调用。
	int main();

	// appState 是一个静态成员，用来跟踪应用的状态（例如是否请求退出）。
	// 这是一个PX4常见的方式，用于控制任务的生命周期。
	static px4::AppState appState; /* track requests to terminate app */

private:
// 条件编译部分（只有在 SAVE_FORCECTL_ADRC_DATA 为1时编译）
#if SAVE_FORCECTL_ADRC_DATA
	// 这是一个 forcectl_adrc_data_s 结构体实例，用来存储ADRC相关的数据，初始化为全零。
	forcectl_adrc_data_s forcectl_ADRC_data = {0};
	// 这行代码创建了一个ORB发布器 adrc_data_pub，并将 forcectl_ADRC_data 通过ORB发布出去。
	// ORB是PX4中的发布/订阅机制，允许任务之间进行数据传递。
	// orb_advertise() 函数用于在ORB中创建一个新的话题，并发布 forcectl_adrc_data 数据。
	// ORB_ID(forcectl_adrc_data) 指定了ORB话题的ID，通过它可以标识和发布 forcectl_adrc_data 相关的数据。
	orb_advert_t adrc_data_pub = orb_advertise(ORB_ID(forcectl_adrc_data), &forcectl_ADRC_data);
#endif
};
