// $DISCLAIMER$

// $Id$

/*****************************************************************//**
 * @file i2c_core.h
 *
 * @brief access MMIO i2c core
 *
 * Detailed description:
 * - 5 basic commands: start, read, write, stop, restart
 * - i2c transaction can be "assembled" with commands
 *   e.g., start, write, write, stop
 *
 * $Author$
 * $Date$
 * $Revision$
 *********************************************************************/

#ifndef _I2C_CORE_H_INCLUDED
#define _I2C_CORE_H_INCLUDED

#include "chu_init.h"

/**
 * i2c core driver:
 * - access MMIO i2c core
 * - 5 basic i2c commands: start, read, write, stop, restart
 * - i2c transaction can be "assembled" with commands;
 *   e.g., start, write, write, stop
 *
 */
class I2cCore {
   /**
    * Register map
    *
    * write data reg in write operation:
    * bits 7-0: data
    * bits 10-8: command
    * write data reg in read operation:
    * bits 7-0: data
    * bits 8: ready
    */
   enum {
      DVSR_REG = 0, /**< i2c clock divisor register */
      WR_REG = 1,   /**< write data/command register */
      RD_REG = 0    /**< read data/status register */
   };
   /**
    * Symbolic commands
    *
    */
   enum {
      I2C_START_CMD = 0x00 << 8,
      I2C_WR_CMD = 0x01 << 8,
      I2C_RD_CMD = 0x02 << 8,
      I2C_STOP_CMD = 0x03 << 8,
      I2C_RESTART_CMD = 0x04 << 8
   };
public:
   /* methods */
   /**
    * Constructor
    *
	* @note set default i2c clock rate to 100K Hz
    */
   I2cCore(uint32_t core_base_addr);
   ~I2cCore();                  // not used

   /**
    * set i2c clock (sclk) frequency
    *
    * @param freq i2c clock frequency
    *
    */
   void set_freq(int freq);

   /**
    * indicate whether i2c core is ready to take a command
    *
    */
   int ready();

   /**
    * issue a start command
    *
    */
   void start();

   /**
    * issue a restart command
    *
    */
   void restart();

   /**
    * issue a stop command
    *
    */
   void stop();

   /**
    * issue a write command
    *
    * @param data 8-bit data
    * @return device ack status (0: ok; -1: failed)
    *
    */
   int write_byte(uint8_t data);

   /**
    * issue a read command
    *
    * @param last indicates the last byte in read cycle (0: no; 1:yes)
    * @return 8-bit read data
    *
    * note: last byte in read cycle forces i2c master generating NACK
    *
    */
   int read_byte(int last);


   /**
    * perform a read transaction
    *
    * @param dev device id
    * @param bytes pointer to read data array
    * @param num number of bytes to be read
    * @param restart 1:issue "restart" command in the end; 0:issue "stop" command
    *
    * @return device ack status (0: ok; -1: # failed ack)
    * @return retrived data store in bytes array
    *
    * @note command sequence: start, write dev, read, .. read, stop/restart
    *
    */
   int read_transaction(uint8_t dev, uint8_t *bytes, int num,
         int restart);

   /**
    * perform a write transaction
    *
    * @param dev device id
    * @param bytes pointer to write data array
    * @param num number of bytes to be written
    * @param restart 1:issue "restart" command in the end; 0:issue "stop" command
    *
    * @return device ack status (0: ok; negative: # failed acks)
    *
    * @note command sequence: start, write dev, write, .. write, stop/restart
    *
    */
   int write_transaction(uint8_t dev, uint8_t *bytes, int num,
         int restart);

private:
   /* variable to keep track of current status */
   uint32_t base_addr;

};

#endif  //_I2C_CORE_H_INCLUDED


   /*
    * write one data byte (3 writes: device id, register, data)
    *
    * @param dev device id
    * @param reg device register
    * @param data one byte data
    *
    * @return device ack status (0: ok; -1/-2/-3: # failed ack)
    *

   int write_dev_reg_data(uint8_t dev, uint8_t reg, uint8_t data);
    */

   /*
    * read one data byte
    *
    * @param dev device id
    * @param reg device register
    * @param byte pointer to read data byte
    *
    * @return device ack status (0: ok; -1/-2/-3: # failed ack)
    *
    * note: seq: start, write dev, write reg,
    *            restart, write dev, read, stop
    * note: will not work with device using stop/start rather than restart
    *

   int read_dev_reg_byte(uint8_t dev, uint8_t reg, uint8_t *byte);
    */


   /*
    * read multiple data bytes
    *
    * @param dev device id
    * @param reg device register
    * @param bytes pointer to read data bytes
    * @param num number of byte to be read
    *
    * @return device ack status (0: ok; -1/-2/-3: # failed ack)
    *
    * note: seq: start, write dev, write reg,
    *            restart, write dev, read, .. read, stop
    * note: will not work with device using stop/start rather than restart
    *

   int read_dev_reg_bytes(uint8_t dev, uint8_t reg, uint8_t *bytes,
         int num);
    */