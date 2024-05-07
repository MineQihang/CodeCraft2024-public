#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"

class Good {
   public:
    // 货物的id
    int id;
    // 货物的位置
    Point p{};
    // 货物的价值
    int val{-1};
    // 货物消失的帧数
    int fade_time{-1};
    // 货物的状态
    GoodStatus status{GoodStatus::EXIST};
    // 哪个机器人携带了该货物，只有在status为CARRIED时有效
    // OR 哪个机器人要去拿该货物，只有在status为SELECTED时有效
    int robot_id{-1};

    Good() { id = -1; }
    Good(Point p, int val, int fade_time) : p(p), val(val), fade_time(fade_time) {
        static int good_id = 0;
        id = good_id++;
    }

    void set_status(GoodStatus status) { this->status = status; }
    void set_selected(int robot_id) {
        set_status(GoodStatus::SELECTED);
        this->robot_id = robot_id;
    }
    void set_carried(int robot_id) {
        set_status(GoodStatus::CARRIED);
        this->robot_id = robot_id;
    }
    void set_exist() { set_status(GoodStatus::EXIST); }
    void set_fade() { set_status(GoodStatus::FADE); }
    void set_done() { set_status(GoodStatus::DONE); }
    bool check_status(GoodStatus status) { return this->status == status; }
    bool is_exist() { return check_status(GoodStatus::EXIST); }
    bool is_selected() { return check_status(GoodStatus::SELECTED); }
    bool is_carried() { return check_status(GoodStatus::CARRIED); }
    bool is_fade() { return check_status(GoodStatus::FADE); }
    bool is_done() { return check_status(GoodStatus::DONE); }
};