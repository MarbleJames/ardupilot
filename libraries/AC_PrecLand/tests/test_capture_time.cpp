#include <AP_gtest.h>
#include <AC_PrecLand/AC_PrecLand_Capture.h>

// Already inside the capture radius: no wait, descend at the landing speed.
TEST(PrecLandCapture, InsideRadiusDescendsAtLandSpeed)
{
    float rate_down_ms = -1.0f;
    EXPECT_TRUE(AC_PrecLand_Capture::descent_rate_down_ms(
                    0.2f, Vector2f{0.01f, 0.0f}, Vector2f{1.0f, 0.0f},
                    0.5f, 1.0f, 2.5f, 5.0f, rate_down_ms));
    EXPECT_FLOAT_EQ(rate_down_ms, 0.5f);
}

// Relative velocity has the same sign as the position error: the pin is getting further away.
TEST(PrecLandCapture, MovingAwayHolds)
{
    float rate_down_ms = 1.0f;
    EXPECT_FALSE(AC_PrecLand_Capture::descent_rate_down_ms(
                     0.2f, Vector2f{1.0f, 0.0f}, Vector2f{1.0f, 0.0f},
                     0.5f, 1.0f, 2.5f, 5.0f, rate_down_ms));
    EXPECT_FLOAT_EQ(rate_down_ms, 0.0f);
}

// Stopped 1 m out. The sqrt controller creeps in; this must not be a zero-time or a >5 s hold.
TEST(PrecLandCapture, StationaryCapturesWithBraking)
{
    const Vector2f err_m{1.0f, 0.0f};
    const Vector2f err_dot_ms{0.0f, 0.0f};
    float capture_s = -1.0f;
    EXPECT_TRUE(AC_PrecLand_Capture::time_to_capture_s(err_m, err_dot_ms, 1.0f, 2.5f, 5.0f, capture_s));
    EXPECT_GT(capture_s, 0.2f);
    EXPECT_LT(capture_s, AC_PrecLand_Capture::MAX_CAPTURE_TIME_S);

    float rate_down_ms = -1.0f;
    EXPECT_TRUE(AC_PrecLand_Capture::descent_rate_down_ms(
                    0.3f, err_m, err_dot_ms, 0.5f, 1.0f, 2.5f, 5.0f, rate_down_ms));
    EXPECT_GE(rate_down_ms, AC_PrecLand_Capture::MIN_DESCENT_MS);
    EXPECT_LE(rate_down_ms, 0.5f);
}

// 8 m/s toward a pin 2 m away. Stopping distance is about 14 m, so a coast
// (0.25 s) is the wrong answer. Capture takes more than a second, or does not
// happen inside the 5 s limit.
TEST(PrecLandCapture, OverspeedIsNotACoast)
{
    const Vector2f err_m{2.0f, 0.0f};
    const Vector2f err_dot_ms{-8.0f, 0.0f};
    const float coast_s = 2.0f / 8.0f;
    float capture_s = 0.0f;
    const bool captured = AC_PrecLand_Capture::time_to_capture_s(err_m, err_dot_ms, 1.0f, 2.5f, 10.0f, capture_s);
    if (captured) {
        EXPECT_GT(capture_s, 1.0f);
        EXPECT_GT(capture_s, coast_s);
    } else {
        EXPECT_FLOAT_EQ(capture_s, 0.0f);
    }
}

int hal;

AP_GTEST_MAIN()
