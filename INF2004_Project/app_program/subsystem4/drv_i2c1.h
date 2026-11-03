/*
 *	drv_i2c1.h
 *	Subsystem 4 -- minimal polled I2C1 master driver (RP2040 DW_apb_i2c).
 *
 *	Replaces the Pico SDK's hardware_i2c, which this firmware does not link.
 *	Call ONLY from S4's imu_task (TEAM.md: I2C1 is owned by imu_task).
 *	The kernel's own I2C driver (DEVCNF_USE_IIC) stays OFF: it would also
 *	claim I2C0 on GP8/GP9, which are S2's left-motor pins.
 *
 *	Return values follow the SDK: number of bytes transferred, or
 *	DRV_I2C1_ERR_NACK / DRV_I2C1_ERR_TIMEOUT (negative).
 */

#ifndef DRV_I2C1_H
#define DRV_I2C1_H

#include <stdbool.h>
#include <stdint.h>

#define DRV_I2C1_ERR_NACK	(-1)
#define DRV_I2C1_ERR_TIMEOUT	(-2)

/* Reset I2C1, route it to sda_gpio/scl_gpio (with pull-ups), set the bus
   speed.  Returns the actual baud rate in Hz (0 on bad arguments). */
uint32_t drv_i2c1_init(unsigned int sda_gpio, unsigned int scl_gpio,
		       uint32_t baud_hz);

int drv_i2c1_write_timeout_us(uint8_t addr, const uint8_t *src, unsigned int len,
			      bool nostop, uint32_t timeout_us);
int drv_i2c1_read_timeout_us(uint8_t addr, uint8_t *dst, unsigned int len,
			     bool nostop, uint32_t timeout_us);

#endif /* DRV_I2C1_H */
