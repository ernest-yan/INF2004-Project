/*
 *	mathf.h
 *	Subsystem 4 -- the four float maths functions S4 needs.
 *
 *	PORT: <math.h> cannot be used next to the kernel headers (newlib's
 *	size_t clashes with tk/syslib.h) and the firmware is not linked with
 *	libm.  These are small, self-contained replacements, accurate to well
 *	under 0.01 degree for the pitch calculation.
 */

#ifndef B4_MATHF_H
#define B4_MATHF_H

#define B4M_PI		(3.14159265f)
#define B4M_HALF_PI	(1.57079633f)

static inline float b4_fabsf(float x)
{
	return (x < 0.0f) ? -x : x;
}

/* Newton-Raphson square root; exact enough for |a| in mg. */
static inline float b4_sqrtf(float x)
{
	float r;
	int i;

	if (x <= 0.0f) {
		return 0.0f;
	}
	r = (x > 1.0f) ? (x * 0.5f) : 1.0f;
	for (i = 0; i < 20; i++) {
		const float next = 0.5f * (r + x / r);
		if (b4_fabsf(next - r) <= (next * 1.0e-7f)) {
			return next;
		}
		r = next;
	}
	return r;
}

/* atan on [-1, 1]: minimax polynomial, |error| < 2e-5 rad */
static inline float b4_atan_unit(float z)
{
	const float z2 = z * z;
	return z * (0.99997726f + z2 * (-0.33262347f + z2 * (0.19354346f +
		    z2 * (-0.11643287f + z2 * (0.05265332f + z2 * -0.01172120f)))));
}

static inline float b4_atan2f(float y, float x)
{
	float a;

	if ((x == 0.0f) && (y == 0.0f)) {
		return 0.0f;
	}
	if (b4_fabsf(x) >= b4_fabsf(y)) {
		a = b4_atan_unit(y / x);
		if (x < 0.0f) {
			a += (y >= 0.0f) ? B4M_PI : -B4M_PI;
		}
	} else {
		a = ((y > 0.0f) ? B4M_HALF_PI : -B4M_HALF_PI) - b4_atan_unit(x / y);
	}
	return a;
}

/* sin for |x| <= pi/2 (S4 only needs pitch angles), Taylor to x^9 */
static inline float b4_sinf(float x)
{
	float x2;

	if (x > B4M_HALF_PI) {
		x = B4M_PI - x;
	} else if (x < -B4M_HALF_PI) {
		x = -B4M_PI - x;
	}
	x2 = x * x;
	return x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f *
		    (1.0f - x2 / 72.0f))));
}

#endif /* B4_MATHF_H */
