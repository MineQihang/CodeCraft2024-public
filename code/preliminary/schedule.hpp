#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"

class Schedule {
   public:
    std::deque<Direction> ops, history;
    Direction recover_history = STAY;
    Schedule() : ops() {}
    Direction get_op(bool need_pop = true) {
        Direction op = ops.front();
        if (!need_pop) {
            return op;
        }
        ops.pop_front();
        recover_history = op;
        return op;
    }
    void recover_op() {
        ops.push_front(recover_history);
        recover_history = STAY;
        // return reverse_dir(op);
    }
    // void put_op(Direction op) {
    //     ops.push_back(op);
    // }
    bool is_valid() { return !ops.empty(); }
    // Schedule reverse() {
    //     std::stack<Direction> ops_new;
    //     while (!ops.empty()) {
    //         if (ops.front() == UP) {
    //             ops_new.push(DOWN);
    //         } else if (ops.front() == DOWN) {
    //             ops_new.push(UP);
    //         } else if (ops.front() == LEFT) {
    //             ops_new.push(RIGHT);
    //         } else if (ops.front() == RIGHT) {
    //             ops_new.push(LEFT);
    //         } else {
    //             ops_new.push(STAY);
    //         }
    //         ops.pop_front();
    //     }
    //     Schedule schedule_new;
    //     while (!ops_new.empty()) {
    //         schedule_new.put_op(ops_new.top());
    //         ops_new.pop();
    //     }
    //     return schedule_new;
    // }
    // Schedule operator+(const Schedule& other) {
    //     Schedule schedule_new;
    //     for (auto op : ops) {
    //         schedule_new.put_op(op);
    //     }
    //     for (auto op : other.ops) {
    //         schedule_new.put_op(op);
    //     }
    //     return schedule_new;
    // }
};