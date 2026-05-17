#ifndef AIM_FRAMEWORK_PIPELINE_DATA_PROVIDER_HPP
#define AIM_FRAMEWORK_PIPELINE_DATA_PROVIDER_HPP

namespace aim {

template <typename InputType>
class DataProvider {
public:
    virtual ~DataProvider() = default;

    // 从数据源获取数据，返回是否成功获取到新数据
    virtual bool fetch(InputType& out_data) = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_DATA_PROVIDER_HPP
