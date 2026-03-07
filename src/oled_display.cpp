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
#include <cstring>

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
// @param   i2c_slave_address - The I2C slave address
// @param   i2c_chip_reg_addr - The I2C Chip register address
// @param   rclcpp_logger - A logger from rclcpp
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

  uint8_t display_type = DISPLAY_TYPE_AUTO;

  // Display parameters
  uint16_t max_line = 0;
  uint16_t max_column = 0;
  uint16_t max_vertical_pixel = 0;
  uint16_t max_horizontal_pixel = 0;
  uint16_t horizontal_offset = 0;
  uint16_t end_horizontal_pixel = 0;
}


//
// @brief   Detect the OLED display type
//
// @return  void
//
void OLEDDisplay::detect()
{
  std::vector<uint8_t> buffer(I2C_READ_BUFFER_SIZE, 0);

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
      this->display_type = DISPLAY_TYPE_NONE;
    }
    catch(const std::runtime_error& exc) {
      // Problem reading from the device
      this->display_type = DISPLAY_TYPE_NONE;
    }
  }

  rclcpp::sleep_for(std::chrono::milliseconds(DISPLAY_DETECT_SLEEP_TIME)); 

  // count the votes and set display type
  if (vote1106 > vote1306) {
    this->display_type = DISPLAY_TYPE_SH1106;
  } else {
    this->display_type = DISPLAY_TYPE_SSD1306;
  }
}


//
// @brief   Initialise the OLED display
//
// @return  void
//
// @note    Throws 'std::invalid_argument' when unable to determine device type
//
void OLEDDisplay::init()
{
  // Auto-detect display. Detects if display present and type of OLED display
  if (this->display_type == DISPLAY_TYPE_AUTO) {
    this->detect();
  }

  switch (this->display_type) {
    case DISPLAY_TYPE_SSD1306:
      // We treat the 1st byte sort of like a 'register' but it is really a
      // command stream mode to the chip
      RCLCPP_INFO(
        this->logger,
        "Setup for SSD1306 controller on the OLED display."
      );

      this->i2cdev->write(ssd1306_init_bytes);

      // Set characteristics for this display
      this->max_line = SSD1306_MAX_LINE;
      this->max_column = SSD1306_MAX_COLUMN;
      this->max_vertical_pixel = SSD1306_MAX_VERTICAL_PIXEL;
      this->max_horizontal_pixel = SSD1306_MAX_HORIZONTAL_PIXEL;
      this->horizontal_offset = SSD1306_HORIZONTAL_OFFSET;
      this->end_horizontal_pixel = SSD1306_END_HORIZONTAL_PIXEL;

      break;

    case DISPLAY_TYPE_SH1106:
      // We treat the 1st byte sort of like a 'register' but it is really a 
      // command stream mode to the chip
      RCLCPP_INFO(
        this->logger,
        "Setup for SH1106 controller on the OLED display."
      );

      this->i2cdev->write(sh1106_init_bytes);

      // Set characteristics for this display
      this->max_line = SH1106_MAX_LINE;
      this->max_column = SH1106_MAX_COLUMN;
      this->max_vertical_pixel = SH1106_MAX_VERTICAL_PIXEL;
      this->max_horizontal_pixel = SH1106_MAX_HORIZONTAL_PIXEL;
      this->horizontal_offset = SH1106_HORIZONTAL_OFFSET;
      this->end_horizontal_pixel = SH1106_END_HORIZONTAL_PIXEL;

      break;

    default:
      throw std::invalid_argument("Unable to determine display type");
      break;
  }
}


//
// @brief   Move cursor to a given horizontal column and line
//
// @param   line - The line number where 0 is top line
// @param   column - The pixel resolution column from 0 to 127
//
// @return  void
//
// @note    Throws 'std::invalid_argument' when I2C device cannot be opened
//          Throws 'std::runtime_error' when write to I2C device fails
//
void OLEDDisplay::set_cursor(uint16_t line, uint16_t column)
{
  std::vector<uint8_t> cursor_setup = {0};

  switch (this->display_type) {
    case DISPLAY_TYPE_SSD1306:
      cursor_setup.assign(7, 0);

      cursor_setup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
      cursor_setup[1] = OLED_CMD_SET_COLUMN_RANGE;
      // Start of printing from left seg as 0
      cursor_setup[2] = column;
      // last index of printing segments
      cursor_setup[3] = this->max_horizontal_pixel;
      cursor_setup[4] = OLED_CMD_SET_PAGE_RANGE;
      // We assume only one line written to at a time
      cursor_setup[5] = line;
      // We assume only one line written to at a time
      cursor_setup[6] = line;

      // We treat the 1st byte sort of like a 'register' but it is really a
      // command stream mode to the chip
      this->i2cdev->write(cursor_setup);

      break;

    case DISPLAY_TYPE_SH1106:
      cursor_setup.assign(4, 0);

      // SH1106 has different addressing than SSD1306
      cursor_setup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
      cursor_setup[1] = 0xB0 | (line & 0xf);
      // Upper column address
      cursor_setup[2] = 0x10 | 
          (((column + this->horizontal_offset) & 0xf0) >> 4);
      // Lower column address
      cursor_setup[3] = 0x00 | ((column + this->horizontal_offset)  & 0xf);

      this->i2cdev->write(cursor_setup);

      break;

    default:
      throw std::invalid_argument("Unable to determine display type");
      break;
  }
}


//
// @brief   Clear the display
//
// @return  void
//
// @note    Throws 'std::invalid_argument' when I2C device cannot be opened
//          Throws 'std::runtime_error' when write to I2C device fails
//
void OLEDDisplay::clear_display()
{
  uint16_t display_width = this->max_horizontal_pixel +
          this->horizontal_offset + this->end_horizontal_pixel;
  std::vector<uint8_t> zero(display_width, 0);

  zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;

  for (uint16_t line = 0; line <= this->max_line; line++) {
    // Clear the current line
    this->set_cursor(line, 0);
    this->i2cdev->write(zero);
  }
}


//
// @brief   Write text to the display
//
// @param   line - The line number to write the text on
// @param   column - The pixel resolution column from 0 to 127
//
// @return  void
//
// @note    Throws 'std::invalid_argument' when line number is out of range
//          Throws 'std::invalid_argument' when starting column doesn't allow
//            text to fit
//          Throws 'std::invalid_argument' when I2C device cannot be opened
//          Throws 'std::runtime_error' when write to I2C device fails
//
void OLEDDisplay::write_text(
    int16_t line,
    uint16_t column,
    std::string text,
    bool center = false
)
  // dispCtx_t *dispCtx, uint8_t line, uint8_t segment, uint8_t center, const char *textStr)
{
  uint16_t display_width = this->max_horizontal_pixel +
          this->horizontal_offset + this->end_horizontal_pixel;
  std::vector<uint8_t> data(display_width, 0);
  uint16_t start_column = column;

  // const char    *text = textStr;
  //       uint8_t       text_len = strlen(text);
  //       uint8_t       dispData[DISPLAY_MAX_HORZ_PIXEL];
  //       int           dispDataIdx = 1;

  if (line > this->max_line) {
    throw std::invalid_argument("Line number is out of range");
  }

  if ((column + (text.size() * DEFAULT_FONT_WIDTH))
        > this->max_horizontal_pixel) {
    // out of range starting column to end of the print with 8x8 font
    throw std::invalid_argument(
      "Text will not fit starting at column " + std::to_string(column)
    );
  }

  // Data starts on byte after this 1st one
  data[0] = OLED_CONTROL_BYTE_DATA_STREAM;

  if (center) {
    start_column = (this->max_horizontal_pixel - 
          (text.size() * DEFAULT_FONT_WIDTH)) / 2;
  }

  this->set_cursor(line, start_column);


        // Form pixels to send as data by lookup in font table
        for (uint8_t i = 0; i < text_len; i++) {
                        // For each column of pixels for this char send a data byte which is one vertical column of pixels
                        for (uint8_t charCol = 0; charCol < DISPLAY_CHAR_WIDTH; charCol++) {
                                        dispData[dispDataIdx++] = font8x8_basic_tr[(int)(text[i])][charCol];
                        }
        }

  // Write the data to display
        retCode |= i2c_write(&dispCtx->devName[0], dispCtx->i2cAddr, &dispData[0], dispDataIdx);
  this->i2cdev->write(zero);

}
