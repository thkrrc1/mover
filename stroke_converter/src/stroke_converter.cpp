#include <map>
#include "stroke_converter.h"

void seed_converter::Mover::makeTables() {}

void seed_converter::Mover::Angle2Stroke(std::vector<int16_t> &_strokes, const std::vector<double> &_angles) {
    static const float scale = 100.0;

    for(size_t idx = 0;idx < _strokes.size();++idx){
        _strokes[idx] = 0x7FFF;
    }
}

//無限回転ホイールの回転角度を求める(-180 ~ 180度)
int16_t seed_converter::Mover::calcWheelAngDeg(int idx, int16_t raw_ang) {
    static constexpr int16_t max_ang = 13602; //最大値
    static constexpr int16_t min_ang = -13602; //最大値
    static constexpr int16_t max_ang_mod = max_ang % 360;
    static constexpr int16_t min_ang_mod = min_ang % 360;

    prev_wheel[idx] = cur_wheel[idx];
    cur_wheel[idx] = raw_ang;
    if (cur_wheel[idx] < -13000 && prev_wheel[idx] > 13000) {
        //順方向にオーバーフローした場合
        zero_ofst[idx] = (zero_ofst[idx] + max_ang_mod - min_ang_mod) % 360;
    } else if (prev_wheel[idx] < -13000 && cur_wheel[idx] > 13000) {
        //逆方向にオーバーフローした場合
        zero_ofst[idx] = (zero_ofst[idx] + min_ang_mod - max_ang_mod) % 360;
    }

    auto cur_ang = (cur_wheel[idx] + zero_ofst[idx])%360;
    if(cur_ang < 0){cur_ang += 360;}
    return (cur_ang - 180.);
}

void seed_converter::Mover::Stroke2Angle(std::vector<double> &_angles, const std::vector<int16_t> &_strokes) {
    static const float deg2Rad = M_PI / 180.0;
    static const float scale_inv = 0.01;

    int16_t ang_fl = calcWheelAngDeg(0,_strokes[idx_wheel_front_left]);
    int16_t ang_fr = calcWheelAngDeg(1,_strokes[idx_wheel_front_right]);
    int16_t ang_rl = calcWheelAngDeg(2,_strokes[idx_wheel_rear_left]);
    int16_t ang_rr = calcWheelAngDeg(3,_strokes[idx_wheel_rear_right]);

    _angles[idx_wheel_front_left]  = deg2Rad * ang_fl;
    _angles[idx_wheel_front_right] = deg2Rad * ang_fr;
    _angles[idx_wheel_rear_left]   = deg2Rad * ang_rl;
    _angles[idx_wheel_rear_right]  = deg2Rad * ang_rr;
}

void seed_converter::Mover::setJointNames(const std::vector<std::string> &names) {
    std::map<std::string, int> jnameIndex;
    jnameIndex.clear();
    for (size_t idx = 0; idx < names.size(); ++idx) {
        jnameIndex.emplace(names[idx], idx);
    }

    idx_wheel_front_left = jnameIndex["wheel_front_left"];
    idx_wheel_front_right = jnameIndex["wheel_front_right"];
    idx_wheel_rear_left = jnameIndex["wheel_rear_left"];
    idx_wheel_rear_right = jnameIndex["wheel_rear_right"];
}

void seed_converter::Mover::calcActuatorVel(std::vector<int16_t> &actuator_vel, const std::vector<double> &joint_vel) {
    for (size_t idx = 0; idx < actuator_vel.size(); ++idx) {
        if (std::isnan(joint_vel[idx])) {
            actuator_vel[idx] = 0x7FFF;
        } else {
            actuator_vel[idx] = static_cast<int16_t>(joint_vel[idx] * 180.0 / M_PI);
        }
    }
}

void seed_converter::Mover::calcJointVel(std::vector<double> &joint_vel,const std::vector<int16_t> &actuator_vel){
    for (size_t idx = 0; idx < joint_vel.size(); ++idx) {
        if (actuator_vel[idx] == 0x7FFF) {
            joint_vel[idx] = std::numeric_limits<double>::quiet_NaN();
        } else {
            joint_vel[idx] = static_cast<double>(actuator_vel[idx] * M_PI / 180.0);
        }
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(seed_converter::Mover, seed_converter::StrokeConverter)
