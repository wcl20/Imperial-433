/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <stdexcept>

namespace arp {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  // TODO: implement...
  double delta_t = (timestampMicroseconds - lastTimestampMicroseconds_) * 1e-6;

  double output = parameters_.k_p * e + parameters_.k_i * integratedError_ + parameters_.k_d * e_dot;

  if (output < minOutput_) {
    output = minOutput_;
  } else if (output > maxOutput_) {
    output = maxOutput_;
  } else {
    if (delta_t <= 0.1) {
      integratedError_ += e * delta_t;
    }
  }

  lastTimestampMicroseconds_ = timestampMicroseconds;

  return output;
}

// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
