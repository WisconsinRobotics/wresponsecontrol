#include "PIDAlgorithm.hpp"
#include <gtest/gtest.h>

using Algorithm::PID::PIDAlgorithm;
using Algorithm::PID::PIDControlLoopParameters;
using Algorithm::PID::PIDParameters;

TEST(PIDAlgorithm, ConstructAndVerifyParameters) {
    PIDParameters params{1, 2, 3};
    PIDAlgorithm uut{params};
    EXPECT_EQ(uut.getParams(), params);
}

TEST(PIDAlgorithm, TestProportionalResponse) {
    {
        PIDParameters params{1, 0, 0};
        PIDAlgorithm uut{params};
        auto response{uut.executeNextControlLoopCycle({1, 0})};
        EXPECT_EQ(response, 1);
    }

    {
        PIDParameters params{2, 0, 0};
        PIDAlgorithm uut{params};
        auto response{uut.executeNextControlLoopCycle({5, 3})};
        EXPECT_EQ(response, 2 * (5 - 3)); // 4
    }
}

TEST(PIDAlgorithm, TestIntegralResponse) {
    {
        PIDParameters params{0, 1, 0};
        PIDAlgorithm uut{params};
        EXPECT_EQ(uut.executeNextControlLoopCycle({1, 0}), 1);
        EXPECT_EQ(uut.executeNextControlLoopCycle({1, 0}), 2);
    }

    {
        PIDParameters params{0, 3, 0};
        PIDAlgorithm uut{params};
        EXPECT_EQ(uut.executeNextControlLoopCycle({1, 0}), 3);
        EXPECT_EQ(uut.executeNextControlLoopCycle({8, 0}), 27);
    }
}

TEST(PIDAlgorithm, TestDerivativeResponse) {
    {
        PIDParameters params{0, 0, 1};
        PIDAlgorithm uut{params};
        EXPECT_EQ(uut.executeNextControlLoopCycle({1, 0}), 1);
        EXPECT_EQ(uut.executeNextControlLoopCycle({1, 0}), 0);
    }

    {
        PIDParameters params{0, 0, 5};
        PIDAlgorithm uut{params};
        EXPECT_EQ(uut.executeNextControlLoopCycle({3, 0}), 15);
        EXPECT_EQ(uut.executeNextControlLoopCycle({1, 0}), -10);
    }
}

TEST(PIDAlgorithm, TestMaxIntegralValue) {
    PIDParameters params{0, 3, 0};
    params.ICap = 20;
}