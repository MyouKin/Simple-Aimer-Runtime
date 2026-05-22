#ifndef LASER_AIMER_SYSTEM_HPP
#define LASER_AIMER_SYSTEM_HPP

#include "../../include/pipeline/System.hpp"
#include "config.hpp"
#include "types.hpp"

namespace laser_aimer {

class LaserAimerSystem final : public aim::System<LaserAimerInput, LaserAimerState> {
public:
  explicit LaserAimerSystem(const LaserAimerConfig & cfg);

  void update(const LaserAimerInput & input) override;
  void updateSelfState(const aim::SelfState & self) override;
  const LaserAimerState & getState() const override { return state_; }

private:
  LaserAimerConfig cfg_;
  LaserAimerState state_;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_SYSTEM_HPP
