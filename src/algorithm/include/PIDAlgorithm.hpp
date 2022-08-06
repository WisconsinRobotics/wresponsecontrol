#ifndef SRC_ALGORITHM_PIDALGORITHM
#define SRC_ALGORITHM_PIDALGORITHM

#include <optional>

namespace Algorithm::PID {

struct PIDParameters {
    double P;
    double I;
    double D;

    std::optional<double> ICap;

    auto operator==(const PIDParameters &rhs) const -> bool {
        return P == rhs.P && I == rhs.I && D == rhs.D;
    };
};

struct PIDControlLoopParameters {
    double target;
    double currentValue;
};

class PIDAlgorithm {
public:
    explicit PIDAlgorithm(const PIDParameters &params);
    [[nodiscard]] auto getParams() const -> PIDParameters;
    auto executeNextControlLoopCycle(const PIDControlLoopParameters &params)
        -> double;

    void reset();

private:
    PIDParameters algorithmParams;
    double accruedErr;
    double prevErr;
};

} // namespace Algorithm::PID

#endif