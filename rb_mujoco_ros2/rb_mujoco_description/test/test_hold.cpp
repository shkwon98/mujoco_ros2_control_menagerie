/** @brief Check RB arm settling and step response in ROS's MuJoCo engine. */
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

#include <mujoco/mujoco.h>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        return 2;
    }
    char error[1024]{};
    const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model)
    {
        std::cerr << error << '\n';
        return 2;
    }
    const std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()),
                                                                 mj_deleteData);
    // Held target observed when the wrist kept oscillating without new trajectories.
    const std::array target{2.24361959, 0.461867698, -2.05560975,
                            1.0901419,  -0.477071,   -3.09182985};
    for (const double velocity : {-20.0, 0.0, 20.0})
    {
        mj_resetDataKeyframe(model.get(), data.get(), 0);
        std::ranges::copy(target, data->qpos);
        std::ranges::copy(target, data->ctrl);
        data->qpos[5] = -2.5;
        data->qvel[5] = velocity;
        double low = std::numeric_limits<double>::infinity();
        double high = -low;
        while (data->time < 6.0)
        {
            mj_step(model.get(), data.get());
            if (data->time >= 3.0)
            {
                low = std::min(low, data->qpos[5]);
                high = std::max(high, data->qpos[5]);
            }
        }
        std::cout << "MuJoCo " << mj_versionString() << " initial_velocity=" << velocity
                  << " wrist_oscillation_rad=" << high - low << '\n';
        if (!std::isfinite(high - low) || high - low > 1e-3 ||
            std::abs(data->qpos[5] - target[5]) > 0.01)
        {
            return 1;
        }
    }

    const std::array posture{0.2, -0.3, 0.4, -0.2, 0.3, -0.1};
    for (int joint = 0; joint < 6; ++joint)
    {
        for (const double direction : {-1.0, 1.0})
        {
            mj_resetDataKeyframe(model.get(), data.get(), 0);
            std::ranges::copy(posture, data->qpos);
            std::ranges::copy(posture, data->ctrl);
            while (data->time < 3.0)
            {
                mj_step(model.get(), data.get());
            }
            data->ctrl[joint] = posture[joint] + direction * 0.1;
            double peak = -std::numeric_limits<double>::infinity();
            while (data->time < 7.0)
            {
                mj_step(model.get(), data.get());
                if (!std::isfinite(data->qpos[joint]))
                {
                    return 1;
                }
                peak = std::max(peak, direction * data->qpos[joint]);
            }
            // Separate dynamic overshoot from steady gravity error with a mounted hand.
            const double overshoot = std::max(0.0, peak - direction * data->qpos[joint]);
            std::cout << "joint=" << joint << " direction=" << direction
                      << " overshoot_rad=" << overshoot
                      << " final_error_rad=" << data->qpos[joint] - data->ctrl[joint] << '\n';
            if (overshoot > 1e-3 || std::abs(data->qpos[joint] - data->ctrl[joint]) > 2e-3)
            {
                return 1;
            }
        }
    }
    // Larger coordinated moves expose servo saturation missed by single-joint steps.
    const std::array displacement{0.6, -0.6, 0.6, -0.6, 0.6, -0.6};
    mj_resetDataKeyframe(model.get(), data.get(), 0);
    std::ranges::copy(posture, data->qpos);
    for (int joint = 0; joint < 6; ++joint)
    {
        data->ctrl[joint] = posture[joint] + displacement[joint];
    }
    std::array<double, 6> overshoot{};
    while (data->time < 3.0)
    {
        mj_step(model.get(), data.get());
        for (int joint = 0; joint < 6; ++joint)
        {
            if (!std::isfinite(data->qpos[joint]))
            {
                return 1;
            }
            const double direction = std::copysign(1.0, displacement[joint]);
            overshoot[joint] =
                std::max(overshoot[joint], direction * (data->qpos[joint] - data->ctrl[joint]));
        }
    }
    for (int joint = 0; joint < 6; ++joint)
    {
        std::cout << "coordinated_joint=" << joint << " overshoot_rad=" << overshoot[joint]
                  << " final_error_rad=" << data->qpos[joint] - data->ctrl[joint] << '\n';
        if (overshoot[joint] > 0.01 || std::abs(data->qpos[joint] - data->ctrl[joint]) > 0.01)
        {
            return 1;
        }
    }
}
