#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"

int pulled_goods_num, pulled_goods_val;

class Robot {
   public:
    // 机器人的id
    int id{-1};
    // 机器人的位置
    Point p;
    // 当前机器人是否有携带货物
    int good_num{0};
    // 机器人的容量
    int capacity{1};
    // 机器人携带的货物id
    std::stack<int> good_ids{};
    // 机器人要去的货物id
    int goto_good_id{-1};
    // 机器人要去的泊位id
    int berth_id{-1};
    // 机器人额外的状态：正在去泊位/货物/正在抉择
    RobotExtraStatus extra_status{RobotExtraStatus::WAITING};  // TODO: GOTOBERTH
    // 机器人的路径
    Schedule schedule{};
    // JJH
    Point next_p[2];
    bool schedule_need_pop;

    Robot() {}
    Robot(int id, Point p, int has_good, int capacity) : id(id), p(p), good_num(has_good), capacity(capacity) {}

    void set_id(int id) { this->id = id; }
    void update(int has_good, Point p) {
        this->p = p;
        this->good_num = has_good;
    }

    void do_op(Map& mp, int frame_id, std::map<int, Direction>& move_commands) {
        move_commands[id] = STAY;
        Direction next_command = STAY;
        next_p[0] = p;
        next_p[1] = p;
        schedule_need_pop = false;
        if (is_disabled()) {
            return;
        }
        // 判断碰撞
        // if (is_recover()) {
        //     std::cerr << "[recover] frame_id: " << frame_id
        //               << "robot_id: " << id << std::endl;
        //     schedule.clear();
        //     mp.update_robot(id, schedule, p);
        //     // TOOD: 碰撞恢复
        //     // throw std::runtime_error("recover not implemented");
        //     return;
        // }
        // 初始去berth
        // if (!schedule.is_valid() && is_goto_berth() && !has_good) {
        //     has_good = true;
        //     good_id = -1;
        //     set_waiting();
        // }
        // [ExtraStatus] && [has_goood] && [check_good/berth] && [matched]
        // == 移动前动作
        // 可以捡起来
        if (is_goto_good() && good_num < capacity && mp.check_good(p) && goto_good_id == mp.get_good(p).id) {
            get(mp);
        }
        // 可以放下
        else if ((is_goto_berth() || is_waiting()) && good_num && mp.check_berth(p) && berth_id == mp.get_berth(p).id) {
            if (goto_good_id == -1) {  // 初始去berth
                good_num = false;
                set_waiting();
            } else if (mp.berths[berth_id].is_disabled()) {
                set_waiting();
            } else {
                pull(mp);
            }
        }
        // if (frame_id == 87 && id == 2)
        //     std::cerr << id << " " << extra_status << " " << has_good << " " << mp.check_berth(p) << " " << berth_id << "
        //     "
        //               << (mp.check_berth(p) ? mp.get_berth(p).id : -10) << std::endl;
        // == 移动
        // [没有路径]
        // 避免碰撞
        // bool flag_no_schedule = false;
        // 去good
        if (!schedule.is_valid() && is_waiting() && (good_num < capacity)) {
            // std::cerr << frame_id << " robot_id: " << id << std::endl;
            auto start = get_time_point();
            schedule = mp.search_good(p);
            double duration = get_duration(start);
            if (global_debug && duration > log_time_threshold)
                std::cerr << frame_id << ": " << id << " search_good time: " << duration << "ms" << std::endl;
            if (schedule.is_valid()) {
                auto& good = mp.get_good(schedule.good_id);
                good.set_selected(id);
                mp.remove_good(p);
                set_goto_good(mp, good.id);
                // mp.update_robot(id, schedule, p);
                berth_id = mp.search_berth(good.p, id, false);
                mp.get_berth(berth_id).add_robot(id);
            } else {
                if (global_debug && frame_id > 1000 && frame_id % 10 == 0)
                    std::cerr << frame_id << ": no good schedule " << id << std::endl;
                // flag_no_schedule = true;
            }
        } else if (!schedule.is_valid() && is_goto_good() && mp.goods_map[p.x][p.y] == -1) {
            // 物品消失了！
            if (global_debug)
                std::cerr << frame_id << ": good disappear" << std::endl;
            set_waiting();
        }
        // 去berth
        else if (!schedule.is_valid() && (is_waiting() || is_goto_berth()) && good_num >= capacity) {
            if (is_waiting()) {
                berth_id = mp.search_berth(p, id, false);
                if (global_debug && berth_id == -1) {
                    std::cerr << "no berth" << std::endl;
                }
            }
            if (berth_id != -1) {
                if (mp.berths[berth_id].is_disabled()) {
                    berth_id = mp.search_berth(p, id, false);
                }
                auto cur = p;
                for (int step = 0; step < 2; step++) {
                    for (int i = 0; i < 4; i++) {
                        auto nxt = cur + Direction(i);
                        if (mp.check_land_invalid(nxt))
                            continue;
                        if (mp.berths_distance[berth_id][nxt.x][nxt.y] < mp.berths_distance[berth_id][cur.x][cur.y]) {
                            cur = nxt;
                            if (step == 0)
                                move_commands[id] = Direction(i);
                            else
                                next_command = Direction(i);
                            break;
                        }
                    }
                }
                set_goto_berth(mp, berth_id);
            }
        }
        // mp.update_robot(id, schedule, p);
        // [有路径]：规划好后执行可能会发生碰撞？TODO
        if (schedule.is_valid()) {
            Direction op = schedule.get_op();
            move_commands[id] = op;
            if (schedule.is_valid()) {
                next_command = schedule.get_op(false);
            }
        }
        next_p[0] = p + move_commands[id];
        next_p[1] = next_p[0] + next_command;
        // == 移动后动作
        // // 可以捡起来
        // if (is_goto_good() && !has_good && mp.check_good(p) &&
        //     good_id == mp.get_good(p).id) {
        //     get(mp, frame_id);
        //     schedule.clear();
        // }
        // // 可以放下
        // else if (is_goto_berth() && has_good && mp.check_berth(p) &&
        //          berth_id == mp.get_berth(p).id) {
        //     pull(mp, frame_id);
        // }
    }

    // 判断当前状态
    bool is_waiting() { return extra_status == RobotExtraStatus::WAITING; }
    bool is_goto_good() { return extra_status == RobotExtraStatus::GOTOGOOD; }
    bool is_goto_berth() { return extra_status == RobotExtraStatus::GOTOBERTH; }
    bool is_disabled() { return extra_status == RobotExtraStatus::DISABLED; }
    // 设置当前状态
    bool set_waiting() {
        extra_status = RobotExtraStatus::WAITING;
        return true;
    }
    bool set_goto_good(Map& mp, int good_id) {
        if (berth_id != -1) {
            mp.get_berth(berth_id).remove_robot(id);
        }
        this->goto_good_id = good_id;
        extra_status = RobotExtraStatus::GOTOGOOD;
        return true;
    }
    bool set_goto_berth(Map& mp, int berth_id) {
        this->berth_id = berth_id;
        if (berth_id == -1) {
            std::cerr << "set_goto_berth: " << id << std::endl;
        }
        mp.get_berth(berth_id).add_robot(id);
        extra_status = RobotExtraStatus::GOTOBERTH;
        return true;
    }
    bool set_disabled() {
        extra_status = RobotExtraStatus::DISABLED;
        return true;
    }
    // 机器人的操作
    void get(Map& mp) {
        printf("get %d\n", id);
        auto& good = mp.get_good(p);
        good.set_carried(id);
        mp.remove_good(p);
        good_ids.push(good.id);
        good_num ++;
        set_waiting();
        schedule.clear();
    }
    void pull(Map& mp) {
        if (!good_num) {
            std::cerr << "[pull invalid] id: " << id << ", frame_id: " << frame_id << std::endl;
            throw std::runtime_error("pull invalid");
        }
        printf("pull %d\n", id);
        while (!good_ids.empty()) {
            auto good_id = good_ids.top();
            auto& good = mp.get_good(good_id);
            good.set_done();
            mp.berths[berth_id].add_good(good.val);
            good_ids.pop();
#ifdef DEBUG
            pulled_goods_num++;
            pulled_goods_val += good.val;
#endif
        }
        goto_good_id = -1;
        good_num = 0;
        set_waiting();
        schedule.clear();
    }
    void move(Direction op) {
        if (op == STAY) {
            return;
        }
        printf("move %d %d\n", id, op);
        p = p + op;
    }
};