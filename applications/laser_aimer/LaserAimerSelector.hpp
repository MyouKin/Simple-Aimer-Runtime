#ifndef LASER_AIMER_SELECTOR_HPP
#define LASER_AIMER_SELECTOR_HPP

#include "../../include/pipeline/Selector.hpp"
#include "types.hpp"

namespace laser_aimer {

class LaserAimerSelector final : public aim::Selector<LaserAimerState> {
public:
  aim::FinalTargetState select(const LaserAimerState & system_state) override;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_SELECTOR_HPP
