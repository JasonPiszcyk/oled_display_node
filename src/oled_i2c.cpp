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
// #include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <utility>
#include <cstdint>
// #include <cstdio>
// #include <cstdlib>
// #include <cstring>

// #include <sys/types.h>
// #include <sys/time.h>
// #include <sys/stat.h>

// Local
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
// @param   i2c_slave_address - The I2C slave address
// @param   i2c_chip_reg_addr - The I2C Chip register address
// @param   rclcpp_logger - A logger from rclcpp
//
// @return  void
//
OLEDI2C::OLEDI2C(
  const char * i2c_device,
  int i2c_slave_address,
  uint8_t i2c_chip_reg_addr,
  rclcpp::Logger rclcpp_logger
) : 
  dev(i2c_device),
  slave_address(i2c_slave_address),
  chip_register_address(i2c_chip_reg_addr),
  logger(rclcpp_logger)
{
  // nothing to do!
}


//
// @brief   Read from the I2C Device
//
// @return  tuple<int, uint8_t*> - A return code and a buffer containing the
//          data that was read
//
std::tuple<int, uint8_t*> OLEDI2C::read()
{
  bool i2c_open = false;
  int fd;
  int ret_code = 0;
  static uint8_t buffer[I2C_BUFFER_SIZE];

  // Internal Chip Register Address
  uint8_t chip_reg_addr = this->chip_register_address;

  // Low level representation of one segment of an I2C transaction
  struct i2c_msg msgs[2];

  // Set of transaction segments
  struct i2c_rdwr_ioctl_data msgset[1];

  // Clear the buffer
  for (int i=0 ; i < I2C_BUFFER_SIZE ; i++) {
    buffer[i] = 0;
  }

  if ((fd = open(this->dev, O_RDWR)) < 0) {
    RCLCPP_ERROR(
      this->logger,
      "Cannot open I2C def of %s with error %s", this->dev, strerror(errno)
    );
    ret_code = IO_ERR_DEV_OPEN_FAILED;
  } else {
    i2c_open = true;
  }

  if (i2c_open) {
    msgs[0].addr = this->slave_address;
    // Write bit
    msgs[0].flags = 0;
    // Slave Address/byte written to I2C slave address                            
    msgs[0].len = 1;
    msgs[0].buf = &chip_reg_addr;
       
    msgs[1].addr = this->slave_address;
    // Read bit or Combined transaction bit
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    // Number of bytes read
    msgs[1].len = 1;
    // Output read buffer
    msgs[1].buf = &buffer[0];

    msgset[0].msgs = msgs;
    // number of transaction segments (write and read)
    msgset[0].nmsgs = 2;

    // The ioctl here will execute I2C transaction with kernel enforced lock
    if (ioctl(fd, I2C_RDWR, &msgset) < 0) {
      RCLCPP_ERROR(
        this->logger,
        "Failed to get bus access to I2C device %s!  ERROR: %s",
        this->dev, strerror(errno)
      );
      ret_code = IO_ERR_IOCTL_ADDR_SET;
    }
  }

  if (i2c_open) {
    close(fd);
  } else {
    // Read is odd in that + is num bytes read so make errors negative
    if (ret_code == 0) {
      ret_code = 1;
    } else {
      ret_code = ret_code * -1;
    }
  }

  return std::make_tuple(ret_code, buffer);
}


//
// @brief   Write to the I2c Device
//
// @param   buffer - Buffer containing the data to write
// @param   num_bytes - Number of bytes to be written
//
// @return  int - Returns 0 for ok. Non-zero are bit-encoded failures
//
int OLEDI2C::write(uint8_t *buffer, int num_bytes)
{
  int fd;
  int ret_code = 0;
  // const int  slaveAddress = i2c7bitAddr;      // Address of the I2C device

  // Open port for writing
  if ((fd = open(this->dev, O_WRONLY)) < 0) {
      RCLCPP_ERROR_ONCE(
        this->logger,
        "Cannot open I2C def of %s with error %s", this->dev, strerror(errno)
      );
      ret_code = IO_ERR_DEV_OPEN_FAILED;
  } else {

    if (ioctl(fd, I2C_SLAVE, this->slave_address) != 0) {
        RCLCPP_ERROR_ONCE(
          this->logger,
          "Failed to get bus access to I2C device %s!  ERROR: %s",
          this->dev, strerror(errno)
        );
        ret_code = IO_ERR_IOCTL_ADDR_SET;
    } else {

      if (::write(fd, buffer, num_bytes) != num_bytes) {
            RCLCPP_ERROR_ONCE(
              this->logger,
              "Failed to write to I2C device %s!  ERROR: %s",
              this->dev, strerror(errno)
            );
            ret_code = IO_ERR_WRITE_FAILED;
      }
    }

    close(fd);
  }

  return ret_code;
}
