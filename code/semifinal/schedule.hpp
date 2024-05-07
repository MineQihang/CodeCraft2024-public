#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"

class Schedule {
   public:
    // 操作序列
    std::deque<Direction> ops;
    // 操作历史
    std::deque<Direction> history;
    // 起点/终点的berth_id
    int berth_id;
    // 目标good_id
    int good_id{-1};
    // JJH
    Direction recover_history{STAY};

    Schedule() : ops() {}

    // 从操作序列中取出一个操作
    Direction get_op(bool need_pop = true) {
        Direction op = ops.front();
        if (!need_pop) {
            return op;
        }
        ops.pop_front();
        history.push_front(op);
        recover_history = op;
        if (history.size() > HISTORY_SIZE) {
            history.pop_back();
        }
        return op;
    }
    // 恢复一个操作
    void recover_op() {
        // if (history.empty()) {
        //     // TODO: 随机返回一个方向
        //     return random_dir();
        // }
        // Direction op = history.front();
        // history.pop_front();
        // ops.push_front(op);
        // return reverse_dir(op);
        ops.push_front(recover_history);
        recover_history = STAY;
    }
    // 添加一个操作
    void put_op(Direction op) { ops.push_back(op); }
    // 删除一个操作
    void pop_op() { ops.pop_back(); }
    // 是否有效
    bool is_valid() { return !ops.empty(); }
    // 翻转操作序列
    Schedule reverse() {
        std::stack<Direction> ops_new;
        while (!ops.empty()) {
            ops_new.push(reverse_dir(ops.front()));
            ops.pop_front();
        }
        Schedule schedule_new;
        while (!ops_new.empty()) {
            schedule_new.put_op(ops_new.top());
            ops_new.pop();
        }
        return schedule_new;
    }
    // 两个操作序列相加
    Schedule operator+(const Schedule& other) {
        Schedule schedule_new;
        for (auto op : ops) {
            schedule_new.put_op(op);
        }
        for (auto op : other.ops) {
            schedule_new.put_op(op);
        }
        return schedule_new;
    }
    // 设置目标good_id
    void set_target_good(int good_id) { this->good_id = good_id; }
    // 清空
    void clear() {
        ops.clear();
        history.clear();
    }
    // 路径长度
    int get_steps() { return ops.size(); }
};

class BoatSchedule {
   public:
    // 操作序列
    std::deque<BoatOp> ops;
    // 起点/终点的berth_id
    int berth_id{-1};
    // 目标delivery_id
    int delivery_id{-1};

    BoatSchedule() : ops() {}

    // 获取操作
    BoatOp get_op(bool need_pop = true) {
        BoatOp op = ops.front();
        if (!need_pop) {
            return op;
        }
        ops.pop_front();
        return op;
    }
    // 后面添加操作
    void push_back(BoatOp op) { ops.push_back(op); }
    // 前面添加操作
    void push_front(BoatOp op) { ops.push_front(op); }
    //
    inline bool check_invalid() { return ops.empty(); }
    //
    inline void clear() { ops.clear(); }
};