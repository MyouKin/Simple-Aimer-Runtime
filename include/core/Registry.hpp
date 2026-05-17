#ifndef AIM_FRAMEWORK_CORE_REGISTRY_HPP
#define AIM_FRAMEWORK_CORE_REGISTRY_HPP

#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

namespace aim {

class ParameterRegistry {
public:
    struct FloatParam {
        std::string name;
        double* value;
        double min_val;
        double max_val;
    };

    struct IntParam {
        std::string name;
        int* value;
        int min_val;
        int max_val;
    };

    static ParameterRegistry& getInstance() {
        static ParameterRegistry instance;
        return instance;
    }

    void registerFloat(const std::string& name, double* val, double min_val = 0.0, double max_val = 1.0) {
        float_params_.push_back({name, val, min_val, max_val});
    }

    void registerInt(const std::string& name, int* val, int min_val = 0, int max_val = 100) {
        int_params_.push_back({name, val, min_val, max_val});
    }

    const std::vector<FloatParam>& getFloatParams() const { return float_params_; }
    const std::vector<IntParam>& getIntParams() const { return int_params_; }

private:
    ParameterRegistry() = default;
    ~ParameterRegistry() = default;

    std::vector<FloatParam> float_params_;
    std::vector<IntParam> int_params_;
};

} // namespace aim

#endif // AIM_FRAMEWORK_CORE_REGISTRY_HPP
