/*
 * Oled Subscribers - Subscribe to topics to update the OLED display
 * 
 * Copyright (C) 2025 Jason Piszcyk
 * Email: Jason.Piszcyk@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program (See file: COPYING). If not, see
 * <https://www.gnu.org/licenses/>.
 */

/*
 * Original code (from which this is derived):
 *   https://github.com/UbiquityRobotics/oled_display_node
 * 
 * Original Copyright Notice:
 * 
 * Copyright (c) 2019, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 */


/******************************************************************************
 *
 * Includes
 *
 ******************************************************************************/
// System
#include <string>
#include <cstdint>

// Local
#include "oled_display_node/oled_subscribers.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "oled_display_node/msg/display_output.hpp"


/******************************************************************************
 *
 * Definitions
 *
 ******************************************************************************/
// Specific definitions for X86/64
#ifdef __x86_64__

#endif // __x86_64__

// Specific definitions for ARM
#if defined(__arm__) || defined(__aarch64__)

#endif // __arm__ || __aarch64__


/******************************************************************************
 *
 * Class Definition
 *
 ******************************************************************************/
//
// Constructor
//
// @brief   Initialisation for the instance
//
// @param   (if none delete)
//
// @return  void
//
OledSubscribers::OledSubscribers()
: Node(OLED_TOPIC_SUBSCRIBER)
{
    // Parameters (in YAML File)
    this->declare_parameter<std::string>("i2c_device", DEFAULT_I2C_DEVICE);
    this->declare_parameter<int>(
        "i2c_slave_address", DEFAULT_I2C_SLAVE_ADDRESS
    );
    this->declare_parameter<uint8_t>(
        "i2c_chip_register_address", DEFAULT_I2C_CHIP_REGISTER_ADDRESS
    );

    // Display topic and we then get callbacks for each message
    // auto sub_display = this->create_subscription<oled_display::DisplayOutput>(
    //   TOPIC_DISPLAY_NODE, SUB_QOS_PROFILE, &displayApiCallback
    // );

    // Battery_state topic and we then get callbacks for each message
    // sub_battery = node->create_subscription<sensor_msgs::BatteryState>(
    //   TOPIC_BATTERY, SUB_QOS_PROFILE, &batteryStateApiCallback
    // );

    // Motor_power_active topic
    // sub_motor = node->create_subscription<std_msgs::Bool>(
    //   TOPIC_MOTOR, SUB_QOS_PROFILE, &motorPowerActiveApiCallback
    // );

    // Firmware state topic
    // sub_firmware = node->create_subscription<std_msgs::String>(
    //   TOPIC_FIRMWARE, SUB_QOS_PROFILE, &firmwareVersionCallback
    // );
}

//
// Receive messages for display output
//
// void displayApiCallback(const oled_display::DisplayOutput::ConstPtr& msg)
// {
//     RCLCPP_INFO(
//       this->get_logger(),
//       "%s heard display output msg: of action_type %d row %d column %d "
//       "num_chars %d attr 0x%x text %s comment %s]",
//       OLED_NODE_NAME, msg->action_type, msg->row, msg->column, msg->num_chars,
//       msg->attributes, msg->text.c_str(), msg->comment.c_str()
//     );

//     int i2cSemLockId = -9;

//      // Now send data to the display
//      switch (msg->action_type) {
//          case oled_display::DisplayOutput::DISPLAY_STARTUP_STRING:
//             displaySetStartupString(msg->row, msg->text.c_str(), i2cSemLockId);
//             break;
//          case oled_display::DisplayOutput::DISPLAY_SET_BRIGHTNESS:
//             displaySetBrightness(msg->attributes, i2cSemLockId);
//             break;
//          case oled_display::DisplayOutput::DISPLAY_ALL:
//          case oled_display::DisplayOutput::DISPLAY_SUBSTRING:
//             displayUpdate(msg->text.c_str(), msg->attributes, msg->row, msg->column, msg->num_chars, i2cSemLockId);
//             break;
//          default:
//             break;
//      }
// }
