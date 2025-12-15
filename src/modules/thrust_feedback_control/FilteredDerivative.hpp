#pragma once

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <px4_platform_common/defines.h>   // isfinite

template <typename T>
class ThrustDerivative
{
public:
    ThrustDerivative() = default;

    // 单次调用：输入 thrust_sp 和 dt（可变），内部完成微分+滤波
    // tau: 一阶滤波时间常数（秒）
    float update(T thrust_sp, float dt, float tau)
    {
        // 参数保护
        if (!PX4_ISFINITE(dt) || dt <= 1e-6f) {
            // dt 不合法：输出保持不变（你也可以选择 return 0.0f）
            return _initialized ? _last_output : 0.0f;
        }

        if (!_initialized) {
            _prev_thrust_sp = thrust_sp;
            _filter.reset(T(0));
            _initialized = true;
            _last_output = 0.0f;
            return 0.0f;
        }

        // 原始微分（dt 可变）
        const float du_raw = float(thrust_sp - _prev_thrust_sp) / dt;
        _prev_thrust_sp = thrust_sp;

        // tau <= 0：不滤波
        if (!PX4_ISFINITE(tau) || tau <= 0.0f) {
            _last_output = du_raw;
            return du_raw;
        }

        // AlphaFilter.hpp 的“时间抽象”一致：alpha = dt/(tau+dt)
        const float alpha = dt / (tau + dt);
        _filter.setAlpha(alpha);

        const float du_filtered = float(_filter.update(T(du_raw)));
        _last_output = du_filtered;
        return du_filtered;
    }

    void reset(T thrust_sp = T(0))
    {
        _prev_thrust_sp = thrust_sp;
        _filter.reset(T(0));
        _initialized = false;
        _last_output = 0.0f;
    }

private:
    AlphaFilter<T> _filter{};
    T _prev_thrust_sp{T(0)};
    bool _initialized{false};
    float _last_output{0.0f};
};
