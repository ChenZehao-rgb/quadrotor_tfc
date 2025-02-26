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
 * @file forcectl_main.cpp
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */

// #include "forcectl_example.hpp"
#include "forcectl_example_iolc_tmech.hpp"

// 这段代码定义了 PX4_MAIN 函数，这是 forcectl 任务的入口函数，用于初始化系统并运行主控制逻辑。
// argc 和 argv 是命令行参数，分别表示参数的数量和具体的参数内容。
// PX4_MAIN 是任务的入口函数名，通常用于定义任务或应用程序的启动入口点。
// 在PX4系统中，当你通过命令行启动 forcectl 时，这个函数会被调用。
int PX4_MAIN(int argc, char **argv)
{
	// 这行代码通过调用 px4::init() 初始化系统，传入命令行参数 argc 和 argv，并指定任务的名称为 "forcectl"。
	// px4::init() 函数通常用于初始化PX4的运行环境，它可能设置一些必要的资源、调度器、日志系统等。
	px4::init(argc, argv, "forcectl");

	// 打印启动信息 "Force-feedback-control Start!"，向控制台输出一条消息，表示 forcectl 任务已经启动并即将进入主逻辑。
	printf("Force-feedback-control Start!\n");
	// 这两行代码创建了一个 Forcectl 类的实例 forcectl，然后调用它的 main() 方法。
	// Forcectl 是一个实现了具体业务逻辑的类，负责处理与力反馈控制相关的功能。
	Forcectl forcectl;
	// forcectl.main() 会运行 forcectl 的主逻辑，控制传感器、执行器、数据处理等功能。
	forcectl.main();

	// 当 forcectl.main() 方法完成执行后，程序打印 "Exiting!"，表示 forcectl 任务已经结束。
	printf("Exiting!\n");
	return 0;
}
