/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3 (line sensors + barcode reader).
 *	Built only with: make TEST=1
 *	Record results in testing/subsystem3/TEST_LOG.md.  IDs: S3-U01, ...
 *
 *	Tests run in the initial task while the barcode task is already
 *	running (see app_main.c).
 */

#include "line_barcode.h"
#include "../test_report.h"

#if APP_TEST

/* How long S3-M01 waits for you to scan a barcode, in ms.
   Set to 0 to skip that test (e.g. when no barcode is available). */
#define S3_TEST_SCAN_MS		10000

/* S3-M02 live line-sensor diagnostic. These values are kernel ticks.
   With a 1 ms system tick, 100 samples x 100 ticks is about 10 seconds. */
#define S3_TEST_LINE_SAMPLES	100
#define S3_TEST_LINE_DELAY	100

EXPORT void subsystem3_test(void)
{
	LineBarcodeData_t d;
	T_RMBF rmbf;

	TEST_INIT();

	/* S3-U01: action mapping returns the configured movement */
	set_mqtt_action_mapping('F', 'U', 'L', 'R');
	TEST_CHECK("S3-U01",
		   get_action_for_character('F') == MOVE_FORWARD &&
		   get_action_for_character('U') == MOVE_UTURN &&
		   get_action_for_character('L') == MOVE_LEFT &&
		   get_action_for_character('R') == MOVE_RIGHT,
		   "mapped characters give the right movement");

	/* S3-U02: unmapped / NUL characters give MOVE_NONE */
	TEST_CHECK("S3-U02",
		   get_action_for_character('Z') == MOVE_NONE &&
		   get_action_for_character('\0') == MOVE_NONE,
		   "unmapped characters give MOVE_NONE");
	set_mqtt_action_mapping('\0', '\0', '\0', '\0');	/* restore */

	/* Let the barcode task take many samples before reading its data */
	tk_dly_tsk(100);
	d = get_line_barcode_data_safe();

	/* S3-U03: one snapshot is internally consistent (the sensors do not
	   have to be on a line; the derived flags must match the raw ones) */
	TEST_CHECK("S3-U03",
		   d.line_detected  == (d.ir_left_raw || d.ir_center_raw) &&
		   d.cross_junction == (d.ir_left_raw && d.ir_center_raw),
		   "line/junction flags match the raw sensor bits");

	/* S3-U04: the line-loss fault flag follows line_detected */
	TEST_CHECK("S3-U04",
		   (d.line_detected != 0) ==
		   ((d.fault_mask & FAULT_LINE_LOSS) == 0),
		   "FAULT_LINE_LOSS is set exactly when no line is seen");

	/* S3-U05: the barcode message buffer (IF-01) exists */
	TEST_CHECK("S3-U05",
		   tk_ref_mbf(mbf_barcode, &rmbf) == E_OK,
		   "mbf_barcode (IF-01) is valid");

#if S3_TEST_SCAN_MS > 0
	/* S3-M02: live GP2/GP4 line-tracking diagnostic.
	   Move the left and centre sensors between white and black areas.
	   This reads the task's shared snapshot; it does not access GPIO
	   directly and does not alter the production driver. */
	{
		uint8_t last_left = 0xFFu;
		uint8_t last_center = 0xFFu;
		uint8_t last_line = 0xFFu;
		uint8_t last_junction = 0xFFu;
		int i;

		tm_printf((UB*)"[TEST] S3-M02: MOVE GP2/GP4 OVER BLACK AND WHITE (%d samples)\n",
			  S3_TEST_LINE_SAMPLES);

		for (i = 0; i < S3_TEST_LINE_SAMPLES; ++i) {
			d = get_line_barcode_data_safe();
			if (d.ir_left_raw != last_left ||
			    d.ir_center_raw != last_center ||
			    d.line_detected != last_line ||
			    d.cross_junction != last_junction) {
				tm_printf((UB*)"[S3-LINE] GP2 Left=%u | GP4 Centre=%u | Line=%u | Junction=%u | Fault=0x%02X\n",
					  (unsigned)d.ir_left_raw,
					  (unsigned)d.ir_center_raw,
					  (unsigned)d.line_detected,
					  (unsigned)d.cross_junction,
					  (unsigned)d.fault_mask);
				last_left = d.ir_left_raw;
				last_center = d.ir_center_raw;
				last_line = d.line_detected;
				last_junction = d.cross_junction;
			}
			tk_dly_tsk(S3_TEST_LINE_DELAY);
		}
	}

	/* S3-M01: needs a person.  Pass a printed Code 39 barcode (A-Z) in
	   front of the right sensor (GP16) when the message appears. */
	{
		char c = 0;
		INT  n;

		while (tk_rcv_mbf(mbf_barcode, &c, TMO_POL) > 0) {
			/* discard anything decoded before the test started */
		}
		tm_printf((UB*)"[TEST] S3-M01: SCAN A BARCODE NOW (%d s)\n",
			  S3_TEST_SCAN_MS / 1000);
		n = tk_rcv_mbf(mbf_barcode, &c, S3_TEST_SCAN_MS);
		TEST_CHECK("S3-M01", n == 1 && c >= 'A' && c <= 'Z',
			   "a scanned barcode is decoded to a letter A-Z");
	}
#endif

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */