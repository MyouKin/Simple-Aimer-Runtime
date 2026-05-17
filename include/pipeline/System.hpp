#ifndef AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP
#define AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP

namespace aim {

template <typename InputType, typename SystemStateType>
class System {
public:
    virtual ~System() = default;

    // 消化输入数据，更新系统状态
    virtual void update(const InputType& input) = 0;

    // 获取当前系统的全量状态
    virtual const SystemStateType& getState() const = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP
