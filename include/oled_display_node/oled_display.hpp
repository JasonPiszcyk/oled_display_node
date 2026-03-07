/*
 * OLED Display - Mange the OLED Display
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
#ifndef OLED_DISPLAY_NODE_OLED_DISPLAY_HPP_
#define OLED_DISPLAY_NODE_OLED_DISPLAY_HPP_

/******************************************************************************
 *
 * Includes
 *
 ******************************************************************************/
// System
#include <cstring>
#include <cstdint>
#include <utility>
#include <memory>

// Local
#include "oled_display_node/oled_i2c.hpp"
#include "rclcpp/rclcpp.hpp"


/******************************************************************************
 *
 * Definitions
 *
 ******************************************************************************/
//
// Constants
//
inline const uint8_t DISPLAY_TYPE_NONE = 0;
inline const uint8_t DISPLAY_TYPE_AUTO = 1;
inline const uint8_t DISPLAY_TYPE_SSD1306 = 2;    // 0.96" 128x64
inline const uint8_t DISPLAY_TYPE_SH1106 = 3;     // 1.3" diagonal 128x64

inline const uint64_t DISPLAY_DETECT_SLEEP_TIME = 30; // milliseconds

// Font Info
inline const uint16_t DEFAULT_FONT_WIDTH = 8;
inline const uint16_t DEFAULT_FONT_HEIGHT = 8;


//
// Characteristics of each supported display
//
// SH1106
inline const uint16_t SH1106_MAX_HORIZONTAL_PIXEL = 127;
inline const uint16_t SH1106_MAX_VERTICAL_PIXEL = 64;
inline const uint16_t SH1106_MAX_LINE = 7;
inline const uint16_t SH1106_MAX_COLUMN = 15;
inline const uint16_t SH1106_HORIZONTAL_OFFSET = 2;
inline const uint16_t SH1106_END_HORIZONTAL_PIXEL = 0;

// SSD1306
inline const uint16_t SSD1306_MAX_HORIZONTAL_PIXEL = 127;
inline const uint16_t SSD1306_MAX_VERTICAL_PIXEL = 64;
inline const uint16_t SSD1306_MAX_LINE = 7;
inline const uint16_t SSD1306_MAX_COLUMN = 15;
inline const uint16_t SSD1306_HORIZONTAL_OFFSET = 0;
inline const uint16_t SSD1306_END_HORIZONTAL_PIXEL = 4;

//
// OLED Commands
//
// Control byte
inline const uint8_t OLED_CONTROL_BYTE_CMD_SINGLE = 0x80;
inline const uint8_t OLED_CONTROL_BYTE_CMD_STREAM = 0x00;
inline const uint8_t OLED_CONTROL_BYTE_DATA_STREAM = 0x40;

// Fundamental commands
inline const uint8_t OLED_CMD_SET_CONTRAST = 0x81;      // follow with 0x7F
inline const uint8_t OLED_CMD_DISPLAY_RAM = 0xA4;
inline const uint8_t OLED_CMD_DISPLAY_ALLON = 0xA5;
inline const uint8_t OLED_CMD_DISPLAY_NORMAL = 0xA6;
inline const uint8_t OLED_CMD_DISPLAY_INVERTED = 0xA7;

// Must have display off and must follow thiswith 0x8B
inline const uint8_t OLED_CMD_DC_DC_CTRL_MODE = 0xAD;
inline const uint8_t OLED_CMD_DISPLAY_OFF = 0xAE;
inline const uint8_t OLED_CMD_DISPLAY_ON = 0xAF;

// Addressing Command Table
// follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
inline const uint8_t OLED_CMD_SET_MEMORY_ADDR_MODE = 0x20;
// can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
inline const uint8_t OLED_CMD_SET_COLUMN_RANGE = 0x21;
// can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7
inline const uint8_t OLED_CMD_SET_PAGE_RANGE = 0x22;

// Hardware Config
inline const uint8_t OLED_CMD_SET_DISPLAY_START_LINE = 0x40;
inline const uint8_t OLED_CMD_SET_SEGMENT_REMAP = 0xA1;
// follow with 0x3F = 64 MUX
inline const uint8_t OLED_CMD_SET_MUX_RATIO = 0xA8;
inline const uint8_t OLED_CMD_SET_COM_SCAN_MODE = 0xC8;
inline const uint8_t OLED_CMD_SET_DISPLAY_OFFSET = 0xD3;  // follow with 0x00
inline const uint8_t OLED_CMD_SET_COM_PIN_MAP = 0xDA;     // follow with 0x12
inline const uint8_t OLED_CMD_NOP = 0xE3;                 // NOP

// Timing and Driving Scheme
inline const uint8_t OLED_CMD_SET_DISPLAY_CLK_DIV = 0xD5; // follow with 0x80
inline const uint8_t OLED_CMD_SET_PRECHARGE = 0xD9;       // follow with 0xF1
inline const uint8_t OLED_CMD_SET_VCOMH_DESELCT = 0xDB;   // follow with 0x30

// Charge Pump
inline const uint8_t OLED_CMD_SET_CHARGE_PUMP = 0x8D;    // follow with 0x14


// These commands are sent to initialize SSD1306.
// There is not 'internal chip register' for this device and instead it uses
// a funky protocol. A control byte is followed by a single byte or multiple
// bytes depending on the control byte. The chip interprets these 'packets'
// as command(s) or data byte(s) based on the leading control byte
inline const std::vector<uint8_t> ssd1306_init_bytes = {
  OLED_CONTROL_BYTE_CMD_STREAM,
  OLED_CMD_SET_CHARGE_PUMP,       0x14,
  OLED_CMD_SET_SEGMENT_REMAP,
  OLED_CMD_SET_COM_SCAN_MODE,
  OLED_CMD_DISPLAY_ON
};

// These commands are sent to initialize SH1106.
inline const std::vector<uint8_t> sh1106_init_bytes = {
  OLED_CONTROL_BYTE_CMD_STREAM,
  0x30,                           // Charge pump default
  0x40,                           // RAM display line of 0
  OLED_CMD_DISPLAY_OFF,
  OLED_CMD_SET_SEGMENT_REMAP,
  OLED_CMD_SET_COM_SCAN_MODE,
  OLED_CMD_SET_DISPLAY_OFFSET, 0, // Sets mapping of display start line
  OLED_CMD_DC_DC_CTRL_MODE, 0x8B, // Must have DISPLAY_OFF and follow tih 0x8B
  0x81, 0x80,                     // Display contrast set to second byte
  OLED_CMD_DISPLAY_RAM,
  OLED_CMD_DISPLAY_NORMAL,
  OLED_CMD_SET_MUX_RATIO, 0x3F,   // Init multiplex ratio to standard value
  OLED_CMD_DISPLAY_ON
};


//
// Type Defs
//


/******************************************************************************
 *
 * Class Declaration
 *
 ******************************************************************************/
class OLEDDisplay
{
  public:
    // Constructor
    OLEDDisplay(
      std::string i2c_device,
      int i2c_slave_address,
      uint8_t i2c_chip_reg_addr,
      rclcpp::Logger rclcpp_logger
    );

    // Methods
    void OLEDDisplay::detect();
    void OLEDDisplay::init();
    void OLEDDisplay::set_cursor(uint16_t line, uint16_t column);
    void OLEDDisplay::clear_display();
    void OLEDDisplay::write_text(
      int16_t line, uint16_t column, std::string text,
      std::vector<std::vector<uint8_t>> font, bool center = false
    );

  private:
    std::string dev;
    int slave_address;
    uint8_t chip_register_address;
    rclcpp::Logger logger;

    std::shared_ptr<OLEDI2C> i2cdev;
    uint8_t display_type;

    uint16_t max_line;
    uint16_t max_column;
    uint16_t max_vertical_pixel;
    uint16_t max_horizontal_pixel;
    uint16_t horizontal_offset;
    uint16_t end_horizontal_pixel;
};


/******************************************************************************
 * End Define Guard
 ******************************************************************************/
#endif /* OLED_DISPLAY_NODE_OLED_DISPLAY_HPP_ */
