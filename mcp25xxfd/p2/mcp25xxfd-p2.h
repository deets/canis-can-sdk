// Copyright 2020-2023 Canis Automotive Labs (canislabs.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef MCP25xxFD_RP2_H
#define MCP25xxFD_RP2_H

#include <sys/types.h>
#include <stdbool.h>

#include "../../../../spi.h"

// Target-specific type defining the interface to the CAN controller
// This is SPI for the MCP25xxFD (and MCP251863) and on the RP2040 there are
// certain parameters for the SPI. These are included in this structure.

typedef struct {
  spi_t spi_device;                 // Set to SPI_DEVICE
  uint32_t crc_errors;                    // Count of CRC errors
  uint32_t magic;                         // Magic number to indicate the interface is set
  uint8_t spi_rx;                         // MISO pin
  uint8_t spi_tx;                         // MOSI pin
  uint8_t spi_sck;                        // SPI clock
  uint8_t spi_irq;                        // IRQ pin
  uint8_t spi_cs;                         // SPI device chip select
  int spi_lock;
} can_interface_t;

// Binds the drivers to the CANPico hardware
#if defined(REDQUEEN2)

#define CAN_SPI_TX 0
#define CAN_SPI_RX 1
#define CAN_SCK 2
#define CAN_CS 34
#define CAN_IRQ 35


// This binds an SPI interface for a specific board. For other devices this will change.
// For multiple devices on the same SPI channel, spi_cs and spi_irq will be different
// but the other fields the same.
static inline void mcp25xxfd_spi_bind_redqueen2(can_interface_t *interface, int spi_lock)
{
  interface->spi_device.start(CAN_SPI_RX, CAN_SPI_TX, CAN_SCK, 1700, 0, 0);
  interface->magic = 0x1e5515f0U;
  interface->spi_lock = spi_lock;
}

#else
// Other boards might have different SPI controllers, pins, etc.
#error "Unknown board"
#endif

// Code on the RP2040 goes into RAM if it's time critical: the XIP flash is way too slow
#ifndef TIME_CRITICAL
#define TIME_CRITICAL
#endif

#ifndef CONST_STORAGE
#define CONST_STORAGE
#endif

#ifndef WEAK
#define WEAK
#endif

#ifndef INLINE
#define INLINE                              static inline
#endif

// This is called to convert 4 bytes in memory to 32-bits where the lowest address byte is
// at bits 7:0 of the word, which will then be transmitted to the MCP25xxFD in little endian
// format. For a little-endian CPU, this is already in the right format.
static inline uint32_t mcp25xxfd_convert_bytes(uint32_t w)
{
    return w;
}

static inline void mcp25xxfd_spi_gpio_enable_irq(can_interface_t *interface)
{
}

static inline bool mcp25xxfd_spi_gpio_irq_asserted(can_interface_t *interface)
{
  return _pinread(CAN_IRQ) == 0;
}

static inline void mcp25xxfd_spi_gpio_disable_irq(can_interface_t *interface)
{
}

static inline void mcp25xxfd_spi_select(can_interface_t *interface)
{
  while(_locktry(interface->spi_lock) == 0) {}
  _pinl(CAN_CS);
}

static inline void mcp25xxfd_spi_deselect(can_interface_t *interface)
{
  _pinh(CAN_CS);
  _lockrel(interface->spi_lock);
}

static inline void mcp25xxfd_spi_write(can_interface_t *interface, const uint8_t *src, size_t len)
{
  for(size_t i=0; i < len; ++i)
  {
    interface->spi_device.shiftout(interface->spi_device.MSBFIRST, src[i], 8);
  }
}

static inline void mcp25xxfd_spi_read_write(can_interface_t *interface, const uint8_t *cmd, uint8_t *resp, size_t len)
{
  for(size_t i=0; i < len; ++i)
  {
    uint8_t res = interface->spi_device.shiftio(interface->spi_device.MSBFIRST, cmd[i], 8);
    resp[i] = res;
  }
}

static inline void mcp25xxfd_spi_read(can_interface_t *interface, uint8_t *dst, size_t len)
{
  for(size_t i=0; i < len; ++i)
  {
    dst[i] = interface->spi_device.shiftin(interface->spi_device.MSBFIRST, 8);
  }

}

static inline void mcp25xxfd_spi_pins_init(can_interface_t *interface) {
}

#endif // MCP25xxFD_RP2_H
