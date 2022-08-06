#include "PIDAlgorithm.hpp"

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
    auto output{algorithmParams.P * err          //
                + algorithmParams.I * accruedErr //
                + algorithmParams.D * (err - prevErr)};
    prevErr = err;
    return output;
}

void PIDAlgorithm::reset() {}

} // namespace Algorithm::PID