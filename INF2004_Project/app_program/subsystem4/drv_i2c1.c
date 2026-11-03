/*
 *	drv_i2c1.c
 *	Subsystem 4 -- minimal polled I2C1 master driver (RP2040 DW_apb_i2c).
 *
 *	Register-level port of the Pico SDK's hardware_i2c blocking functions
 *	(same sequencing, same error mapping) because this firmware does not
 *	link the SDK.  Polled, no interrupts, no DMA.  See drv_i2c1.h.
 */

#include "drv_i2c1.h"
#include "../app_hw.h"

#define I2C1_BASE		(0x40048000u)
#define I2C_REG(off)		(I2C1_BASE + (off))
#define IC_CON			I2C_REG(0x00u)
#define IC_TAR			I2C_REG(0x04u)
#define IC_DATA_CMD		I2C_REG(0x10u)
#define IC_FS_SCL_HCNT		I2C_REG(0x1Cu)
#define IC_FS_SCL_LCNT		I2C_REG(0x20u)
#define IC_INTR_MASK		I2C_REG(0x30u)
#define IC_RAW_INTR_STAT	I2C_REG(0x34u)
#define IC_RX_TL		I2C_REG(0x38u)
#define IC_TX_TL		I2C_REG(0x3Cu)
#define IC_CLR_TX_ABRT		I2C_REG(0x54u)
#define IC_CLR_STOP_DET		I2C_REG(0x60u)
#define IC_ENABLE		I2C_REG(0x6Cu)
#define IC_TXFLR		I2C_REG(0x74u)
#define IC_RXFLR		I2C_REG(0x78u)
#define IC_SDA_HOLD		I2C_REG(0x7Cu)
#define IC_TX_ABRT_SOURCE	I2C_REG(0x80u)
#define IC_FS_SPKLEN		I2C_REG(0xA0u)

#define CON_MASTER_MODE		(1u << 0)
#define CON_SPEED_FAST		(2u << 1)
#define CON_SPEED_MASK		(3u << 1)
#define CON_RESTART_EN		(1u << 5)
#define CON_SLAVE_DISABLE	(1u << 6)
#define CON_TX_EMPTY_CTRL	(1u << 8)

#define RAW_TX_EMPTY		(1u << 4)
#define RAW_TX_ABRT		(1u << 6)
#define RAW_STOP_DET		(1u << 9)

#define DATA_CMD_READ		(1u << 8)
#define DATA_CMD_STOP		(1u << 9)
#define DATA_CMD_RESTART	(1u << 10)

#define ABRT_7B_ADDR_NOACK	(1u << 0)
#define ABRT_TXDATA_NOACK	(1u << 3)

#define I2C_TX_FIFO_DEPTH	(16u)

static bool s_restart_on_next = false;

static bool timed_out(uint32_t start_us, uint32_t timeout_us)
{
	return (uint32_t)(app_time_us_32() - start_us) >= timeout_us;
}

uint32_t drv_i2c1_init(unsigned int sda_gpio, unsigned int scl_gpio,
		       uint32_t baud_hz)
{
	const uint32_t freq_in = APP_CLK_SYS_HZ;	/* DW_apb_i2c runs from clk_sys */
	uint32_t period, lcnt, hcnt, hold;

	if ((baud_hz == 0u) || (baud_hz > 1000000u) ||
	    (sda_gpio >= GPIO_NUM) || (scl_gpio >= GPIO_NUM)) {
		return 0u;
	}

	/* Full reset of I2C1: S4 is its only user (TEAM.md section 3). */
	set_w(RESETS_RESET, RESETS_RESET_I2C1);
	app_release_reset(RESETS_RESET_I2C1);
	s_restart_on_next = false;

	out_w(IC_ENABLE, 0u);
	out_w(IC_CON, CON_SPEED_FAST | CON_MASTER_MODE | CON_SLAVE_DISABLE |
		      CON_RESTART_EN | CON_TX_EMPTY_CTRL);
	out_w(IC_TX_TL, 0u);
	out_w(IC_RX_TL, 0u);
	out_w(IC_INTR_MASK, 0u);			/* polled: no interrupts */

	period = (freq_in + baud_hz / 2u) / baud_hz;
	lcnt = period * 3u / 5u;
	hcnt = period - lcnt;
	hold = (baud_hz < 1000000u) ? ((freq_in * 3u) / 10000000u) + 1u
				    : ((freq_in * 3u) / 25000000u) + 1u;
	if ((hcnt < 8u) || (lcnt < 8u) || (hcnt > 0xFFFFu) || (lcnt > 0xFFFFu)) {
		return 0u;
	}
	out_w(IC_CON, (in_w(IC_CON) & ~CON_SPEED_MASK) | CON_SPEED_FAST);
	out_w(IC_FS_SCL_HCNT, hcnt);
	out_w(IC_FS_SCL_LCNT, lcnt);
	out_w(IC_FS_SPKLEN, (lcnt < 16u) ? 1u : (lcnt / 16u));
	out_w(IC_SDA_HOLD, (in_w(IC_SDA_HOLD) & 0xFFFF0000u) | (hold & 0xFFFFu));
	out_w(IC_ENABLE, 1u);

	/* Pins: I2C function, input enabled, pull-up, Schmitt trigger.
	   The GY-511 board normally has its own pull-ups as well. */
	out_w(GPIO(sda_gpio), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
	out_w(GPIO(scl_gpio), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
	out_w(GPIO_CTRL(sda_gpio), GPIO_CTRL_FUNCSEL_I2C);
	out_w(GPIO_CTRL(scl_gpio), GPIO_CTRL_FUNCSEL_I2C);

	return freq_in / period;
}

static int map_abort(bool timeout, uint32_t abort_reason, int byte_ctr,
		     bool is_write)
{
	if (timeout) {
		return DRV_I2C1_ERR_TIMEOUT;
	}
	if ((abort_reason == 0u) || ((abort_reason & ABRT_7B_ADDR_NOACK) != 0u)) {
		return DRV_I2C1_ERR_NACK;		/* nothing answered the address */
	}
	if (is_write && ((abort_reason & ABRT_TXDATA_NOACK) != 0u)) {
		return byte_ctr;			/* address ACKed, data NACKed */
	}
	return DRV_I2C1_ERR_NACK;
}

int drv_i2c1_write_timeout_us(uint8_t addr, const uint8_t *src, unsigned int len,
			      bool nostop, uint32_t timeout_us)
{
	const uint32_t start = app_time_us_32();
	bool abort = false, timeout = false;
	uint32_t abort_reason = 0u;
	int byte_ctr;
	int rval;
	const int ilen = (int)len;

	if ((src == NULL) || (len == 0u) || (addr >= 0x80u)) {
		return DRV_I2C1_ERR_NACK;
	}

	out_w(IC_ENABLE, 0u);
	out_w(IC_TAR, addr);
	out_w(IC_ENABLE, 1u);

	for (byte_ctr = 0; byte_ctr < ilen; ++byte_ctr) {
		const bool first = (byte_ctr == 0);
		const bool last = (byte_ctr == ilen - 1);

		out_w(IC_DATA_CMD, ((first && s_restart_on_next) ? DATA_CMD_RESTART : 0u) |
				   ((last && !nostop) ? DATA_CMD_STOP : 0u) |
				   (uint32_t)*src++);

		/* wait until the byte has left the shift register */
		do {
			timeout = timed_out(start, timeout_us);
			abort |= timeout;
		} while (!timeout && ((in_w(IC_RAW_INTR_STAT) & RAW_TX_EMPTY) == 0u));

		if (!timeout) {
			abort_reason = in_w(IC_TX_ABRT_SOURCE);
			if (abort_reason != 0u) {
				(void)in_w(IC_CLR_TX_ABRT);	/* read-to-clear */
				abort = true;
			}
			if (abort || (last && !nostop)) {
				do {
					timeout = timed_out(start, timeout_us);
					abort |= timeout;
				} while (!timeout &&
					 ((in_w(IC_RAW_INTR_STAT) & RAW_STOP_DET) == 0u));
				if (!timeout) {
					(void)in_w(IC_CLR_STOP_DET);
				}
			}
		}
		if (abort) {
			break;
		}
	}

	rval = abort ? map_abort(timeout, abort_reason, byte_ctr, true) : byte_ctr;
	s_restart_on_next = nostop;
	return rval;
}

int drv_i2c1_read_timeout_us(uint8_t addr, uint8_t *dst, unsigned int len,
			     bool nostop, uint32_t timeout_us)
{
	const uint32_t start = app_time_us_32();
	bool abort = false, timeout = false;
	uint32_t abort_reason = 0u;
	int byte_ctr;
	int rval;
	const int ilen = (int)len;

	if ((dst == NULL) || (len == 0u) || (addr >= 0x80u)) {
		return DRV_I2C1_ERR_NACK;
	}

	out_w(IC_ENABLE, 0u);
	out_w(IC_TAR, addr);
	out_w(IC_ENABLE, 1u);

	for (byte_ctr = 0; byte_ctr < ilen; ++byte_ctr) {
		const bool first = (byte_ctr == 0);
		const bool last = (byte_ctr == ilen - 1);

		while ((I2C_TX_FIFO_DEPTH - in_w(IC_TXFLR)) == 0u) {
			if (timed_out(start, timeout_us)) {
				timeout = true;
				break;
			}
		}
		if (timeout) {
			abort = true;
			break;
		}

		out_w(IC_DATA_CMD, ((first && s_restart_on_next) ? DATA_CMD_RESTART : 0u) |
				   ((last && !nostop) ? DATA_CMD_STOP : 0u) |
				   DATA_CMD_READ);

		do {
			abort_reason = in_w(IC_TX_ABRT_SOURCE);
			if ((in_w(IC_RAW_INTR_STAT) & RAW_TX_ABRT) != 0u) {
				abort = true;
				(void)in_w(IC_CLR_TX_ABRT);
			}
			timeout = timed_out(start, timeout_us);
			abort |= timeout;
		} while (!abort && (in_w(IC_RXFLR) == 0u));

		if (abort) {
			break;
		}
		*dst++ = (uint8_t)in_w(IC_DATA_CMD);
	}

	rval = abort ? map_abort(timeout, abort_reason, byte_ctr, false) : byte_ctr;
	s_restart_on_next = nostop;
	return rval;
}
