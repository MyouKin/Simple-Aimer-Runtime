#ifndef LASER_AIMER_SOLVER_HPP
#define LASER_AIMER_SOLVER_HPP

#include "../../include/pipeline/Solver.hpp"
#include "config.hpp"
#include "control/Controller.hpp"
#include "types.hpp"

namespace laser_aimer {

class LaserAimerSolver final : public aim::Solver<LaserAimerState> {
public:
  explicit LaserAimerSolver(const LaserAimerConfig & cfg);

  aim::Command solve(const aim::FinalTargetState & target,
                     const LaserAimerState & system_state) override;

private:
  LaserAimerConfig cfg_;
  Controller controller_;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_SOLVER_HPP
