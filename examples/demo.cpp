// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

int main() {
    try {
        std::cout << "=== OpenArm CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the OpenArm API functionality" << std::endl;

        // Initialize OpenArm with CAN interface and enable CAN-FD
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        // 构造 OpenArm 对象，使用 CAN 接口 "can0"，第二个参数 true 表示启用 CAN‑FD（更宽帧）
        openarm::can::socket::OpenArm openarm("can0", true);  // Use CAN-FD on can0 interface   // 左臂can0总线

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310};    //定义臂上电机类型列表
        std::vector<uint32_t> send_can_ids = {0x01, 0x02};  //定义向电机发送消息时使用的 CAN ID（十六进制）
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12};  //定义接收电机状态/回复的 CAN ID 列表
        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Initialize gripper
        std::cout << "Initializing gripper..." << std::endl;
        //始化夹持器电机，传入电机类型、发送 ID、接收 ID
        openarm.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);  

        // Set callback mode to ignore and enable all motors
        //将所有电机的回调模式设为 IGNORE（不使用参数回调/特殊处理），影响后续通信回调行为
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        // Enable all motors
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        openarm.enable_all();   //使能所有电机（允许驱动器输出）
        // Allow time (2ms) for the motors to respond for slow operations like enabling
        //从 CAN 接收并处理来自所有设备的消息，参数 2000 表示等待/接收超时（2s），用于让使能等慢操作完成并得到回复
        openarm.recv_all(2000);

        // Set device mode to param and query motor id
        std::cout << "\n=== Querying Motor Recv IDs ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);  //将回调模式设为 PARAM，用于参数查询/回复处理
        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::MST_ID));  //向所有电机发送查询命令，查询所有电机的 MST_ID 参数（电机 ID）
        // Allow time (2ms) for the motors to respond for slow operations like querying
        // parameter from register
        openarm.recv_all(2000); //等待并接收参数查询的回复（2s）

        // Access motors through components
        //遍历臂上每个电机并打印其发送 CAN ID（整臂can总线id） 及从参数表读取到的 MST_ID（电机 ID）
        for (const auto& motor : openarm.get_arm().get_motors()) {
            std::cout << "Arm Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }
        //遍历夹持器电机并打印其发送 CAN ID 及从参数表读取到的 MST_ID（电机 ID）
        for (const auto& motor : openarm.get_gripper().get_motors()) {
            std::cout << "Gripper Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }

        // Set device mode to state and control motor
        //将回调模式设为 STATE，进入状态/实时控制模式用于位置/力矩控制与状态更新
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Control arm motors with position control
        // 向臂上所有电机发送 MIT 控制命令（参数结构 MITParam 包含控制类型与数值）
        // MIT参数：{kp, kd, q, dq, tau}，位置/速度/扭矩控制，其中 kp、kd 是位置控制的比例和微分增益，q 和 dq 是目标位置和速度，tau 是目标力矩
        openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{2, 1, 0, 0, 0},
                                           openarm::damiao_motor::MITParam{2, 1, 0, 0, 0}});
        openarm.recv_all(500);  //接收控制命令相关的回复/状态（500ms）

        // Control arm motors with torque control
        openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
                                           openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1}});
        openarm.recv_all(500);

        // Control gripper
        std::cout << "Closing gripper..." << std::endl;
        openarm.get_gripper().close();
        openarm.recv_all(1000);

        //循环 10 次：睡眠 100ms，调用 refresh_all()（刷新/发送周期性请求或更新内部状态），recv_all(300) 接收最新状态，随后打印每个电机的当前位置（motor.get_position()）。
        for (int i = 0; i < 10; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            openarm.refresh_all();
            openarm.recv_all(300);

            // Display arm motor states
            for (const auto& motor : openarm.get_arm().get_motors()) {
                std::cout << "Arm Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
            // Display gripper state
            for (const auto& motor : openarm.get_gripper().get_motors()) {
                std::cout << "Gripper Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
        }

        openarm.disable_all();
        openarm.recv_all(1000);
        /*
        关于recv_all()：
        要保证读到的是最新状态，必须让库处理并更新接收到的 CAN 报文，
        可以启动一个后台线程循环调用 recv_all(...)，让状态持续更新
        比如：
        // ...existing code...
        // 启动后台接收线程（添加到初始化完成后）
        std::atomic<bool> recv_thread_running{true};
        std::thread recv_thread([&openarm,&recv_thread_running](){
            while (recv_thread_running.load()) {
                openarm.recv_all(100); // 100 ms 超时，持续处理 CAN 消息
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
        // ...existing code...

        // 在程序退出前停止并 join 线程（放到关闭/disable 之后）
        recv_thread_running.store(false);
        if (recv_thread.joinable()) recv_thread.join();
        // ...existing code...
        */


    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
