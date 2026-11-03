/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing you create runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The IS33 robot: one subsystem per folder
 *	-----------------------------------------
 *	  subsystem1/  LED heartbeat on GP19 (template example, kept)  prio 12  any
 *	  subsystem2/  motion: motors GP8-11, encoders GP6/GP7          prio 14  core 0
 *	  subsystem3/  line GP2/GP4 + Code 39 barcode GP16              prio 16  core 1
 *	               line following + barcode turns                   prio 17  core 0
 *	  subsystem4/  GY-511 LSM303DLHC on I2C1 GP26/GP27              prio 18  core 0
 *	  subsystem5/  HC-SR04 GP0/GP1 + SG90 servo GP12                prio 20  core 0
 *	  integration/ vehicle_ctrl: team glue between the subsystems   prio 11  core 0
 *
 *	How they talk to each other (TEAM.md section 5):
 *	  IF-01  S3 -> S3   message buffer mbf_barcode   (owned by S3)
 *	  IF-02  S5 -> VC   message buffer mbf_s5_events (owned by S5)
 *	  IF-03  VC, S3, S5 -> S2   b2_stop() / b2_submit_command() function API
 *	  IF-04  S4 -> S2   b2_emergency_brake() on impact / sensor fault
 *	  IF-05  S2 -> S4   b2_get_status() odometry -> b4_update_travel_mm()
 *	  IF-06  VC -> S5   subsystem5_confirm_robot_stopped()
 *	  IF-07  S5 -> S3   subsystem5_get_status(): S3 holds while S5 is busy
 *	  IF-08  S3 -> S5   get_line_barcode_data_safe(): bypass finds the line
 *
 *	Build with the defaults (SMP=1, CRASH_REPORT=0): GP0/GP1 carry the
 *	HC-SR04, and the S3 barcode task needs core 1 to itself.
 *
 *	micro T-Kernel APIs used, and where to find them:
 *	  tasks       tk_cre_tsk tk_sta_tsk tk_slp_tsk tk_wup_tsk tk_can_wup
 *	              tk_dly_tsk tk_sus_tsk tk_rsm_tsk tk_chg_pri tk_ref_tsk
 *	              tk_get_tid tk_get_prc                         app_main.c, S1-S4
 *	  semaphore   tk_cre_sem tk_wai_sem tk_sig_sem tk_ref_sem       S2, S3, S4
 *	  mem. pool   tk_cre_mpf tk_get_mpf tk_rel_mpf tk_ref_mpf       S2, S3, S4
 *	  mailbox     tk_cre_mbx tk_snd_mbx tk_rcv_mbx                  S2, S3
 *	  mutex       tk_cre_mtx tk_loc_mtx tk_unl_mtx                  S3, S4
 *	  msg buffer  tk_cre_mbf tk_snd_mbf tk_rcv_mbf                  S3, S4
 *	  event flag  tk_cre_flg tk_set_flg tk_wai_flg                  S3, S4
 *	  cyclic      tk_cre_cyc tk_ref_cyc                             S1
 *	  alarm       tk_cre_alm tk_sta_alm tk_stp_alm                  S4
 *	  time        tk_get_tim                                        S2, S4
 *	  system      tk_ref_ver                                        app_main.c
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "integration/vehicle_ctrl.h"

#if !TK_SUPPORT_SMP
#warning "SMP=0: the S3 barcode task never sleeps and will starve S4 and S5. Build with SMP=1."
#endif
#if TM_CONSOLE_UART
#warning "CRASH_REPORT=1 / CONSOLE=uart puts UART0 on GP0/GP1, which carry the S5 HC-SR04: unplug the HC-SR04 before flashing this image."
#endif

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

EXPORT INT usermain(void)
{
	T_RVER	ver;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif

	/* Phase 1: every subsystem creates its kernel objects */
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");
	CHECK(vc_init(), "vc_init");

	/* Phase 2: every subsystem starts its tasks */
	CHECK(subsystem1_start(), "subsystem1_start");
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem5_start(), "subsystem5_start");
	CHECK(vc_start(), "vc_start");

	tm_printf((UB*)"all subsystems started\n\n");

#if APP_TEST
	/* Test build (make TEST=1): run every subsystem's tests in turn,
	   here in the initial task, while the system is running.  Tests may
	   call tk_dly_tsk() to let the other tasks work for a while. */
	tm_printf((UB*)"[TEST] === test run start ===\n");
	subsystem1_test();
	subsystem2_test();
	subsystem3_test();
	subsystem4_test();
	subsystem5_test();
	tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
