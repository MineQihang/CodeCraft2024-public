#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"

enum GoodStatus {
    EXIST = 0,     // 存在：当前货物存在
    SELECTED = 1,  // 选中：当前货物已经被某个机器人选中作为目标
    CARRIED = 2,   // 捡起：当前货物已经被捡起
    FADE = 3,      // 消失：当前货物已经消失
    DONE = 4,      // 完成：当前货物已经被放到泊位
    PENDING_ = 5    // 待定：当前货物正在被答题
};

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
    GoodStatus status{GoodStatus::EXIST};  // 请用mp.goods_map判断是否存在货物
    // 哪个机器人携带了该货物，只有在status为CARRIED时有效
    // OR 哪个机器人要去拿该货物，只有在status为SELECTED时有效
    int robot_id{-1};

    Good() { id = -1; }
    Good(Point p, int val, int fade_time) : p(p), val(val), fade_time(fade_time) {
        static int good_id = 0;
        id = good_id++;
    }
    // Good(const Good&) = delete;
    // Good& operator=(const Good&) = delete;

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
    void set_pending() { set_status(GoodStatus::PENDING_); }
    bool check_status(GoodStatus status) { return this->status == status; }
    bool is_exist() { return check_status(GoodStatus::EXIST); }
    bool is_selected() { return check_status(GoodStatus::SELECTED); }
    bool is_carried() { return check_status(GoodStatus::CARRIED); }
    bool is_fade() { return check_status(GoodStatus::FADE); }
    bool is_done() { return check_status(GoodStatus::DONE); }
    bool is_pending() { return check_status(GoodStatus::PENDING_); }
};