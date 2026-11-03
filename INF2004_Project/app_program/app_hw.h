/*
 *	app_hw.h
 *	Small RP2040 hardware helpers shared by the team (no Pico SDK needed).
 *
 *	This firmware does NOT link the Pico SDK runtime, so SDK calls such as
 *	time_us_64(), clock_get_hz(), pwm_set_gpio_level() or i2c_init() are not
 *	available to application code.  Use these helpers and libbsp instead.
 */

#ifndef APP_HW_H
#define APP_HW_H

#include <stdint.h>
#include "app.h"

/* clk_sys and clk_peri as set up by kernel/sysdepend/pico_rp2040/cpu_clock.c */
#define APP_CLK_SYS_HZ		(125000000u)
#define APP_CLK_PERI_HZ		(125000000u)

/* RP2040 TIMER: free-running microsecond counter (non-latching raw reads). */
#define APP_TIMER_RAWH		(*(volatile uint32_t *)0x40054024u)
#define APP_TIMER_RAWL		(*(volatile uint32_t *)0x40054028u)

static inline uint32_t app_time_us_32(void)
{
	return APP_TIMER_RAWL;
}

static inline uint64_t app_time_us_64(void)
{
	uint32_t hi, lo, hi2;

	do {
		hi  = APP_TIMER_RAWH;
		lo  = APP_TIMER_RAWL;
		hi2 = APP_TIMER_RAWH;
	} while (hi != hi2);		/* retry if the low word wrapped */
	return ((uint64_t)hi << 32) | lo;
}

/*
 * Take peripheral blocks out of reset WITHOUT resetting them first.
 * Safe to call from several subsystems that share one block (e.g. the PWM
 * block is shared by S2's motors and S5's servo): it never wipes the other
 * subsystem's configuration.  'mask' is RESETS_RESET_xxx bits.
 */
static inline void app_release_reset(uint32_t mask)
{
	clr_w(RESETS_RESET, mask);
	while ((in_w(RESETS_RESET_DONE) & mask) != mask) {
		/* wait for the block to come out of reset */
	}
}

/*
 * Critical section that is safe against BOTH an interrupt handler on this
 * core AND a task on the other core (replaces the Pico SDK's
 * save_and_disable_interrupts(), which only masks the current core).
 * Not recursive: never take the same lock twice.  Keep the section short.
 */
#if TK_SUPPORT_SMP
#include <tk/smp.h>
typedef T_SPLOCK app_lock_t;
#define APP_LOCK_INIT(l)	((void)InitSpinLock(&(l)))
static inline UINT app_lock(app_lock_t *l)
{
	UINT s;
	(void)ISpinLock(l, &s);
	return s;
}
static inline void app_unlock(app_lock_t *l, UINT s)
{
	(void)ISpinUnlock(l, s);
}
#else
typedef UW app_lock_t;
#define APP_LOCK_INIT(l)	((void)(l))
static inline UINT app_lock(app_lock_t *l)
{
	UINT s;
	(void)l;
	DI(s);
	return s;
}
static inline void app_unlock(app_lock_t *l, UINT s)
{
	(void)l;
	EI(s);
}
#endif

#endif /* APP_HW_H */
