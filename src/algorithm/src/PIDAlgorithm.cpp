#include "PIDAlgorithm.hpp"
#include <cmath>

namespace Algorithm::PID {

PIDAlgorithm::PIDAlgorithm(const PIDParameters &params)
    : algorithmParams{params},
      accruedErr{0},
      prevErr{0} {}

auto PIDAlgorithm::getParams() const -> PIDParameters {
    return algorithmParams;
}

auto PIDAlgorithm::executeNextControlLoopCycle(
    const PIDControlLoopParameters &params) -> double {
    auto err{params.target - params.currentValue};
    accruedErr += err;

    // Cap I component
    if (algorithmParams.ICap && std::abs(accruedErr) > *algorithmParams.ICap) {
        if (*algorithmParams.ICap == 0)
            accruedErr = 0;
        else {
            accruedErr =
                accruedErr / std::abs(accruedErr) * *algorithmParams.ICap;
        }
    }

    auto output{algorithmParams.P * err          //
                + algorithmParams.I * accruedErr //
                + algorithmParams.D * (err - prevErr)};
    prevErr = err;

    // Cap output
    if (algorithmParams.max)
        output = std::min(output, *algorithmParams.max);
    if (algorithmParams.min)
        output = std::max(output, *algorithmParams.min);

    return output;
}

void PIDAlgorithm::reset() {
    accruedErr = 0;
    prevErr = 0;
}

} // namespace Algorithm::PID