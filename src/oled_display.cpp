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
 *
 * Definitions
 *
 ******************************************************************************/
//
// Constants
//


/******************************************************************************
 *
 * Includes
 *
 ******************************************************************************/
// System
#include <stdexcept>
// #include <system_error>
#include <memory>
#include <chrono>

// Local
#include "oled_display_node/oled_display.hpp"
#include "oled_display_node/oled_i2c.hpp"
#include "rclcpp/rclcpp.hpp"


/******************************************************************************
 *
 * Class Definition
 *
 ******************************************************************************/
//
// @brief   Initialisation/constructor for the instance
//
// @param   i2c_device - The device to use for I2C comms
//
// @return  void
//
OLEDDisplay::OLEDDisplay(
  std::string i2c_device,
  int i2c_slave_address,
  uint8_t i2c_chip_reg_addr,
  rclcpp::Logger rclcpp_logger
) : 
  dev(i2c_device),
  slave_address(i2c_slave_address),
  chip_register_address(i2c_chip_reg_addr),
  logger(rclcpp_logger)
{
  // Create the instance of the I2C device
  i2cdev = std::make_shared<OLEDI2C>(
        dev, slave_address, chip_register_address, logger
  );
}


//
// @brief   Read from the I2C Device
//
// @return  int- The display type
//
// @note    Throws 'std::invalid_argument' when I2C device cannot be opened
//          Throws 'std::runtime_error' when write to I2C device fails
//
int OLEDDisplay::detect()
        // std::string devName, uint8_t i2c7bitAddr, int *dispType)
{
  std::array<uint8_t, I2C_BUFFER_SIZE> buffer;
  uint8_t display_type = DISPLAY_TYPE_AUTO;

  // Different devices being voted for
  int vote1106 = 0;
  int vote1306 = 0;

  // We have seen incorrect values read sometimes and because this chip relies
  // on a status register in the SH1106 we better read a few times and 'vote'
  for (int i=0 ; i < 5 ; i++) {
    // Read the status register at chip addr 0 to decide on chip type - set flag to true
    try {
      buffer = this->i2cdev->read();

      RCLCPP_INFO(
        this->logger,
        "Read OLED status register as 0x%02x on pass %d", buffer, i
      );

      if ((buffer[0] & 0x07) == 0x06) {
        // We found lower 3 bit as a 6 but datasheet does not spec it
        vote1306++;
      } else {
        // Data sheet guarentees lower 3 bits as 0
        // We are going to vote by assumption that this is a SSD1106
        vote1106++;
      }
    }
    catch(const std::invalid_argument& exc) {
      // The device cannot be found
      display_type = DISPLAY_TYPE_NONE;
    }
    catch(const std::runtime_error& exc) {
      // Problem reading from the device
      display_type = DISPLAY_TYPE_NONE;
    }
  }

  rclcpp::sleep_for(std::chrono::milliseconds(DISPLAY_DETECT_SLEEP_TIME)); 

  // count the votes and set display type
  if (vote1106 > vote1306) {
    display_type = DISPLAY_TYPE_SH1106;
  } else {
    display_type = DISPLAY_TYPE_SSD1306;
  }

  return display_type;
}
