#ifndef SRC_ALGORITHM_PIDALGORITHM
#define SRC_ALGORITHM_PIDALGORITHM

#include <optional>
#include <tuple>

namespace Algorithm::PID {

struct PIDParameters {
    double P{0};
    double I{0};
    double D{0};

    std::optional<double> ICap{};
    std::optional<double> max{};
    std::optional<double> min{};

    auto operator==(const PIDParameters &rhs) const -> bool {
        return std::tie(P, I, D, ICap, max, min) ==
               std::tie(rhs.P, rhs.I, rhs.D, rhs.ICap, rhs.max, rhs.min);
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