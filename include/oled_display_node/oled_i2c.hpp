/*
 * OLED I2C - Comms to I2C Devices
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
#ifndef OLED_DISPLAY_NODE_OLED_I2C_HPP_
#define OLED_DISPLAY_NODE_OLED_I2C_HPP_

/******************************************************************************
 *
 * Includes
 *
 ******************************************************************************/
// System
#include <cstdint>
#include <utility>

// Local
#include "rclcpp/rclcpp.hpp"


/******************************************************************************
 *
 * Definitions
 *
 ******************************************************************************/
//
// Constants
//
// Some bit-encoded errors for access to the OLED display
inline const int IO_ERR_DEV_OPEN_FAILED  = 0x02;
inline const int IO_ERR_IOCTL_ADDR_SET   = 0x04;
inline const int IO_ERR_WRITE_FAILED     = 0x08;
inline const int IO_ERR_READ_FAILED      = 0x10;
inline const int IO_ERR_READ_LENGTH      = 0x20;
inline const int IO_ERR_BAD_DISP_CONTEXT = 0x80;

// Reading/Writing
inline const int I2C_BUFFER_SIZE = 16;

//
// Type Defs
//


/******************************************************************************
 *
 * Class Declaration
 *
 ******************************************************************************/
class OLEDI2C
{
  public:
    // Constructor
    OLEDI2C(
      const char * i2c_device,
      int i2c_slave_address,
      uint8_t i2c_chip_reg_addr,
      rclcpp::Logger rclcpp_logger
    );

    // Methods
    std::tuple<int, uint8_t*> read();
    int write(uint8_t *buffer, int num_bytes);

  private:
    const char * dev;
    int slave_address;
    uint8_t chip_register_address;
    rclcpp::Logger logger;
};


/******************************************************************************
 * End Define Guard
 ******************************************************************************/
#endif /* OLED_DISPLAY_NODE_OLED_I2C_HPP_ */
