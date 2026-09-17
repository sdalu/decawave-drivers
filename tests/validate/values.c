/*
 * Copyright (c) 2026
 * Stephane D'Alu, Inria Chroma / Inria Agora, INSA Lyon, CITI Lab.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * <dw1000/dw1000_validate.h> against what _dw1000_radio_is_valid()
 * accepts in dw1000.c. The two have to agree field for field, and the
 * reason this test exists is that the hand-written copy this API replaced
 * disagreed in three places:
 *
 *   - it refused preamble codes 13, 14, 15 and 16 as "not supported in
 *     this implementation", when lde_repc_tunning[] has an entry for
 *     every code from 1 to 24
 *   - it refused codes 21..24 outright
 *   - it accepted all eight preamble lengths whether the build had
 *     DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH or not, so it passed
 *     lengths dw1000_configure() then refused -- including 128, which
 *     was the program's own default
 *
 * So the preamble-code range and the option-dependence of the preamble
 * length are the two things here worth a test, and the run is done twice,
 * once with the option and once without (see tests/check-validate.sh).
 *
 * Nothing in this file touches a chip, a driver or a port: the API is a
 * value mapping, and this links against dw1000_validate.c alone.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <dw1000/dw1000.h>
#include <dw1000/dw1000_validate.h>

static int failures;

#define FAIL(fmt, ...)							\
    do {								\
	printf("  %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);	\
	failures++;							\
    } while (0)

/* Accepted, and mapped to the expected encoded value. */
#define OK(fn, in, want)						\
    do {								\
	uint8_t     v   = 0xA5;						\
	const char *msg = (const char *)1;				\
	if (! fn((in), &v, &msg)) {					\
	    FAIL(#fn "(%d) refused: %s", (in), msg ? msg : "(no message)"); \
	} else {							\
	    if (v != (want))						\
		FAIL(#fn "(%d) gave %u, want %u", (in), v, (unsigned)(want)); \
	    if (msg != NULL)						\
		FAIL(#fn "(%d) passed but set a message: %s", (in), msg);\
	}								\
    } while (0)

/* Refused, with a message, and without disturbing the out value. */
#define NO(fn, in)							\
    do {								\
	uint8_t     v   = 0xA5;						\
	const char *msg = NULL;						\
	if (fn((in), &v, &msg)) {					\
	    FAIL(#fn "(%d) accepted, should not be", (in));		\
	} else {							\
	    if (msg == NULL)						\
		FAIL(#fn "(%d) refused with no message", (in));		\
	    if (v != 0xA5)						\
		FAIL(#fn "(%d) refused but wrote %u", (in), v);		\
	}								\
    } while (0)

/* Refused, and the message mentions `needle` -- so that the reason given
 * is the specific one, not the generic list. */
#define NO_BECAUSE(fn, in, needle)					\
    do {								\
	const char *msg = NULL;						\
	if (fn((in), NULL, &msg)) {					\
	    FAIL(#fn "(%d) accepted, should not be", (in));		\
	} else if ((msg == NULL) || (strstr(msg, (needle)) == NULL)) {	\
	    FAIL(#fn "(%d) message is \"%s\", wanted it to mention \"%s\"", \
		 (in), msg ? msg : "(none)", (needle));			\
	}								\
    } while (0)

static void
test_channel(void)
{
    OK(dw1000_validate_channel, 1, 1);
    OK(dw1000_validate_channel, 2, 2);
    OK(dw1000_validate_channel, 3, 3);
    OK(dw1000_validate_channel, 4, 4);
    OK(dw1000_validate_channel, 5, 5);
    OK(dw1000_validate_channel, 7, 7);

    /* There is no channel 6, and the message should say that rather than
     * list the others. */
    NO_BECAUSE(dw1000_validate_channel, 6, "channel 6");

    NO(dw1000_validate_channel, 0);
    NO(dw1000_validate_channel, 8);
    NO(dw1000_validate_channel, -1);
}

static void
test_bitrate(void)
{
    OK(dw1000_validate_bitrate,  110, DW1000_BITRATE_110KBPS);
    OK(dw1000_validate_bitrate,  850, DW1000_BITRATE_850KBPS);
    OK(dw1000_validate_bitrate, 6800, DW1000_BITRATE_6800KBPS);

    NO(dw1000_validate_bitrate,    0);
    NO(dw1000_validate_bitrate,  100);
    NO(dw1000_validate_bitrate, 6801);
}

static void
test_prf(void)
{
    OK(dw1000_validate_prf, 16, DW1000_PRF_16MHZ);
    OK(dw1000_validate_prf, 64, DW1000_PRF_64MHZ);

    /* 4 MHz is in the API and not in the receiver; the message should say
     * which of the two it is. */
    NO_BECAUSE(dw1000_validate_prf, 4, "receiver");

    NO(dw1000_validate_prf,  0);
    NO(dw1000_validate_prf, 32);
}

static void
test_pcode(void)
{
    /* All of 1..24, 13..16 and 21..24 included. This is the regression. */
    for (int c = 1; c <= 24; c++)
	OK(dw1000_validate_pcode, c, c);

    NO(dw1000_validate_pcode,  0);
    NO(dw1000_validate_pcode, 25);
    NO(dw1000_validate_pcode, -1);
}

static void
test_plen(void)
{
    /* Always available, in every build. */
    OK(dw1000_validate_plen,   64, DW1000_PLEN_64);
    OK(dw1000_validate_plen, 1024, DW1000_PLEN_1024);
    OK(dw1000_validate_plen, 4096, DW1000_PLEN_4096);

    /* The five proprietary ones, which must follow the build's option --
     * accepted with it, and refused because of it without, naming it so a
     * caller knows what to change. */
#if DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH
    OK(dw1000_validate_plen,  128, DW1000_PLEN_128);
    OK(dw1000_validate_plen,  256, DW1000_PLEN_256);
    OK(dw1000_validate_plen,  512, DW1000_PLEN_512);
    OK(dw1000_validate_plen, 1536, DW1000_PLEN_1536);
    OK(dw1000_validate_plen, 2048, DW1000_PLEN_2048);
#else
    NO_BECAUSE(dw1000_validate_plen,  128,
	       "DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH");
    NO_BECAUSE(dw1000_validate_plen,  256,
	       "DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH");
    NO_BECAUSE(dw1000_validate_plen,  512,
	       "DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH");
    NO_BECAUSE(dw1000_validate_plen, 1536,
	       "DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH");
    NO_BECAUSE(dw1000_validate_plen, 2048,
	       "DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH");
#endif

    NO(dw1000_validate_plen,    0);
    NO(dw1000_validate_plen,   32);
    NO(dw1000_validate_plen,  127);
    NO(dw1000_validate_plen, 8192);
}

static void
test_pac(void)
{
    OK(dw1000_validate_pac,  8, DW1000_PAC8);
    OK(dw1000_validate_pac, 16, DW1000_PAC16);
    OK(dw1000_validate_pac, 32, DW1000_PAC32);
    OK(dw1000_validate_pac, 64, DW1000_PAC64);

    NO(dw1000_validate_pac,   0);
    NO(dw1000_validate_pac,   4);
    NO(dw1000_validate_pac, 128);
}

/* The antenna delay: a distance in, ticks out, and the numbers matter --
 * 154.6 m is the figure rpi-redskin, probe and the sniffer all use, and
 * the three used to compute it three different ways (integer decimetres,
 * float metres, and a third in spank). They must all still land on the
 * same tick count. */
static void
test_antenna_delay(void)
{
    uint16_t    v   = 0;
    const char *msg = (const char *)1;

    /* One metre is 213.139 ticks; one tick is 4.6918 mm. */
    if (! dw1000_validate_antenna_delay(1.0, &v, &msg))
	FAIL("1 m refused: %s", msg ? msg : "(no message)");
    else if (v != 213)
	FAIL("1 m gave %u ticks, want 213", v);

    /* The house figure, and the value both trees run today: 32951 for the
     * round trip, 16475 once halved. */
    if (! dw1000_validate_antenna_delay(154.6, &v, NULL))
	FAIL("154.6 m refused");
    else if (v != 32951)
	FAIL("154.6 m gave %u ticks, want 32951", v);
    else if ((v / 2) != 16475)
	FAIL("154.6 m halved gives %u, want 16475", v / 2);

    /* Integer metres would have been 154, and this is what that costs --
     * the reason the conversion is floating point. */
    if (! dw1000_validate_antenna_delay(154.0, &v, NULL))
	FAIL("154 m refused");
    else if (v != 32823)
	FAIL("154 m gave %u ticks, want 32823", v);

    /* The round trip: DW1000_CLOCK_TO_METER() must undo it. */
    {
	double back = DW1000_CLOCK_TO_METER(DW1000_METER_TO_CLOCK(154.6));
	if ((back < 154.5999) || (back > 154.6001))
	    FAIL("round trip through the clock gave %f, want 154.6", back);
    }

    /* Zero is a legitimate delay, not an error. */
    v = 0xFF;
    if (! dw1000_validate_antenna_delay(0.0, &v, NULL))
	FAIL("0 m refused");
    else if (v != 0)
	FAIL("0 m gave %u ticks, want 0", v);

    /* Negative, and past what sixteen bits hold (about 307 m). */
    msg = NULL;
    if (dw1000_validate_antenna_delay(-1.0, NULL, &msg))
	FAIL("a negative delay was accepted");
    else if ((msg == NULL) || (strstr(msg, "negative") == NULL))
	FAIL("negative delay message is \"%s\"", msg ? msg : "(none)");

    msg = NULL;
    if (dw1000_validate_antenna_delay(1000.0, NULL, &msg))
	FAIL("1000 m was accepted; it does not fit 16 bits");
    else if (msg == NULL)
	FAIL("an over-long delay was refused with no message");

    /* Just inside and just outside the register. */
    if (! dw1000_validate_antenna_delay(307.0, NULL, NULL))
	FAIL("307 m refused; it fits in 16 bits");
    if (dw1000_validate_antenna_delay(308.0, NULL, NULL))
	FAIL("308 m accepted; it does not fit in 16 bits");
}

/* Both out parameters are documented as optional, and a caller that wants
 * only the verdict passes neither. Nothing here may dereference NULL. */
static void
test_null_out_params(void)
{
    if (! dw1000_validate_channel(5, NULL, NULL))
	FAIL("channel 5 refused with no out parameters");
    if (dw1000_validate_channel(6, NULL, NULL))
	FAIL("channel 6 accepted with no out parameters");
    if (! dw1000_validate_pcode(24, NULL, NULL))
	FAIL("pcode 24 refused with no out parameters");
}

/* dw1000.h guards DW1000_SPEED_OF_LIGHT_MPS so a caller can define its
 * own, and the conversions have to follow it. check-validate.sh builds
 * this a third time with the constant halved: light at half speed takes
 * twice as long to cover a metre, so a metre must come out twice as many
 * ticks. Nothing else in this file holds under that value -- every other
 * number here is against the real one -- so that run does this alone.
 */
static void
test_speed_of_light_override(void)
{
    uint16_t v = 0;

    if (! dw1000_validate_antenna_delay(1.0, &v, NULL))
	FAIL("1 m refused under an overridden speed of light");
    else if (v != 426)        /* 213.139 x 2 = 426.27 */
	FAIL("1 m gave %u ticks at half the speed of light, want 426", v);
}

int
main(void)
{
#if defined(VALUES_SPEED_OF_LIGHT_OVERRIDDEN)
    test_speed_of_light_override();

    printf("validate: speed of light overridden, %d failure(s)\n", failures);
#else
    test_channel();
    test_bitrate();
    test_prf();
    test_pcode();
    test_plen();
    test_pac();
    test_antenna_delay();
    test_null_out_params();

    printf("validate: proprietary preamble lengths %s, %d failure(s)\n",
#if DW1000_WITH_PROPRIETARY_PREAMBLE_LENGTH
	   "on",
#else
	   "off",
#endif
	   failures);
#endif

    return failures == 0 ? 0 : 1;
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * End:
 */
