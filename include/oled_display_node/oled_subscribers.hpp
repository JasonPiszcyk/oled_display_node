/*
 * OledSubscribers - Subscribe to the topics providing input for the display
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
 * Start Define Guard
 ******************************************************************************/
#ifndef OLED_SUBSCRIBERS_HPP_
#define OLED_SUBSCRIBERS_HPP_

/******************************************************************************
 *
 * Includes
 *
 ******************************************************************************/
// System

// Local
#include "rclcpp/rclcpp.hpp"


/******************************************************************************
 *
 * Definitions
 *
 ******************************************************************************/
//
// Type Defs
//

//
// Constants
//
inline const std::string OLED_NODE_NAME = "oled_display_node";
inline const std::string OLED_TOPIC_SUBSCRIBER = "oled_topic_subscriber";
inline const std::string TOPIC_DISPLAY_NODE = "display_node";
inline const std::string TOPIC_BATTERY = "battery_state";
inline const std::string TOPIC_MOTOR = "motor_power_active";
inline const std::string TOPIC_FIRMWARE = "firmware_version";

inline const rmw_qos_profile_t SUB_QOS_PROFILE = 100;


/******************************************************************************
 *
 * Class Declaration
 *
 ******************************************************************************/
class OledSubscribers : public rclcpp::Node
{
  public:
    // Constructor
    OledSubscribers();

    // Methods
    void exampleMethod();

  private:
    int exampleAttribute; // Data member (attribute)
};


/******************************************************************************
 * End Define Guard
 ******************************************************************************/
#endif /* OLED_SUBSCRIBERS_HPP_ */
