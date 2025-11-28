#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

template <typename T>
class ThrustDerivative
{
public:
    ThrustDerivative() = default;

    void set_params(float tau, float dt)
    {
        // tau 一阶滤波时间常数，dt 控制周期
        // 经典关系：alpha = tau / (tau + dt)
        const float alpha = tau / (tau + dt);
        _filter.set_alpha(alpha);
fate        _dt = dt;
        _initialized = false;
    }

    float update(float thrust_sp)
    {
        if (!_initialized) {
            _prev_thrust_sp = thrust_sp;
            _filter.reset(0.0f);    // 初始导数设为 0
            _initialized = true;
            return 0.0f;
        }

        // 原始微分
        float du_raw = (thrust_sp - _prev_thrust_sp) / _dt;
        _prev_thrust_sp = thrust_sp;

        // AlphaFilter 对微分结果做低通
        float du_filtered = _filter.update(du_raw);
        return du_filtered;
    }

private:
    AlphaFilter<T> _filter{};
    T _prev_thrust_sp{0.0f};
    T _dt{0.01f};
    bool _initialized{false};
};
