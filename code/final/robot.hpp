#pragma once
#pragma GCC optimize("O2")
#include "llm.hpp"
#include "robot_base.hpp"
#include "utils.hpp"

class Robot : public BaseRobot {
   public:
    // 机器人的private_id
    int private_id{-1};
    // 机器人要去的货物id
    int goto_good_id{-1};
    // 机器人要去的泊位id
    int goto_berth_id{-1};
    // 机器人要去的patch id
    int goto_patch_id{-1};
    // 机器人额外的状态：正在去泊位/货物/正在抉择
    RobotExtraStatus extra_status{RobotExtraStatus::WAITING};
    // 题目
    std::string problem;
    // 机器人的路径
    Schedule schedule{};
    // 后两个位置，避让使用
    Point next_p[2];
    // 避让使用
    bool schedule_need_pop;
    // LLM
    LLM llm{LLM_DEBUG};
    bool is_posted{false};
    int pending_frame_id{-1};
    bool ans_result{false};  // 回答是否正确：0: no, 1: yes
    int full_time = 0;       // 满载货物的时间

    int nxt_good_num{0};
    int good_signal{-1};
    int good_distance{0x3f3f3f3f};

    Robot() {}
    Robot(int id, Point p, int good_num, int capacity, int private_id)
        : BaseRobot(id, p, good_num, capacity, true), private_id(private_id) {}
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    void set_question(std::string problem) { this->problem = problem; }

    void do_op(Map& mp, std::map<int, Direction>& move_commands) {
        if (p != next_p[0] && !is_waiting()) {
            mp.col_robots.insert(private_id);
            if (is_goto_good() || is_goto_patch())
                schedule.recover_op();
            else if (is_goto_berth() && schedule.is_valid())
                schedule.recover_op();
        }
        move_commands[private_id] = STAY;
        // Direction next_command = STAY;
        next_p[0] = p;
        next_p[1] = p;
        schedule_need_pop = false;
        if (dulu) {
            if (p == dulu_points[private_id]) {
                if (private_id < 6)
                    return;
                else
                    set_waiting();
            } else if (!schedule.is_valid()) {
                schedule = mp.get_schedule_by_a_star(p, dulu_points[private_id]);
            }
        }
        if (good_num >= capacity) {
            full_time++;
        } else {
            full_time = 0;
        }
        if (is_disabled()) {
            return;
        }
        if (is_pending()) {
            if (pending_frame_id == -2) {
                std::cerr << "get error" << std::endl;
                if (stay_time > ROBOT_MAX_WAIT_TIME)
                    set_waiting();
                else
                    set_goto_good(mp, goto_good_id);
            } else if (frame_id != pending_frame_id) {
                if (ans_result) {
                    // std::cerr << id << " get_done " << p.x << " " << p.y << mp.check_good(p) << std::endl;
                    get_done(mp, goto_good_id, true);
                }
                if (LLM_DEBUG && !ans_result) {
                    std::cerr << frame_id << ": " << public_id << ": ans_result: false" << std::endl;
                    std::cerr << problem << std::endl;
                }
                llm_answer_flag.push_back(ans_result);
                set_waiting();
            } else {
                if (!is_posted) {
                    // std::cerr << id << " ask_question: " << problem << std::endl;
                    llm.ask_question(problem, true, LLM_TEMPERATURE);
                    is_posted = true;
                }
                if (llm.is_result_ready()) {
                    double duration = 0.0;
                    auto res = llm.get_result_option(duration);
                    // std::cerr << frame_id << " " << public_id << " get_result: " << res << std::endl;
                    if (res == -1) {
                        llm.ask_question(problem, false, LLM_TEMPERATURE);
                    } else {
                        llm_answer_time.push_back(duration);
                        answer(res);
                    }
                }
            }
            return;
        }

        // [ExtraStatus] && [has_goood] && [check_good/berth] && [matched]
        // == 移动前动作
        // 可以捡起来
        if (is_goto_good() && nxt_good_num < capacity && mp.check_good(p) && goto_good_id == mp.get_good(p).id) {
            get(mp);
            schedule.clear();
        }
        // 可以放下
        else if ((is_goto_berth() || is_waiting()) && nxt_good_num && mp.check_berth(p) &&
                 goto_berth_id == mp.get_berth(p).id) {
            if (goto_good_id == -1) {  // 初始去berth
                nxt_good_num = 0;
                set_waiting();
            } else if (mp.berths[goto_berth_id].is_disabled()) {
                set_waiting();
            } else {
                if (mp.get_berth(p).is_ours_boat || full_time > 800)
                    pull(mp);
                else
                    set_waiting();
            }
        }
        if (is_pending()) {
            return;
        }
        // if (frame_id == 87 && private_id == 2)
        //     std::cerr << private_id << " " << extra_status << " " << has_good << " " << mp.check_berth(p) << " " <<
        //     berth_id << "
        //     "
        //               << (mp.check_berth(p) ? mp.get_berth(p).id : -10) << std::endl;
        // == 移动
        // [没有路径]
        // 避免碰撞
        // 去patch
        if (!schedule.is_valid() && is_waiting() && (nxt_good_num < capacity)) {
            schedule = mp.search_patch(p);
            if (schedule.is_valid()) {
                set_goto_patch(schedule.patch_id);
                mp.map_patches[goto_patch_id].add_goto_robot();
            } else {
                // waiting
            }
        }
        // 物品消失了
        else if (!schedule.is_valid() && is_goto_good() && mp.goods_map[p.x][p.y] == -1) {
            // if (global_debug)
            //     std::cerr << frame_id << ": good disappear" << std::endl;
            set_waiting();
        }
        // 去berth
        else if (!schedule.is_valid() && (is_waiting() || is_goto_berth()) && nxt_good_num >= capacity) {
            // if (is_waiting())
            goto_berth_id = mp.search_berth(p, private_id, check_good_expensive(mp));
            if (goto_berth_id != -1) {
                if (mp.berths[goto_berth_id].is_disabled()) {
                    goto_berth_id = mp.search_berth(p, private_id, check_good_expensive(mp));
                }
                set_goto_berth(mp, goto_berth_id);
            }
        }

        // 去patch的过程中看有没有good
        if ((is_goto_patch() || is_waiting()) && (nxt_good_num < capacity)) {
            if (good_signal != -1) {
                auto& good = mp.get_good(good_signal);
                if (good.is_exist()) {
                    auto good_schedule = mp.get_schedule_by_a_star(p, good.p);
                    good_schedule.good_id = good_signal;
                    if (good_schedule.is_valid()) {
                        schedule = good_schedule;
                        auto& good = mp.get_good(schedule.good_id);
                        // if (private_id == 0)
                        //     std::cerr << frame_id << " " << good.p.x << " " << good.p.y << std::endl;
                        good.set_selected(private_id);
                        // mp.remove_good(p);
                        set_goto_good(mp, good.id);
                        mp.map_patches.at(goto_patch_id).remove_goto_robot();
                        // mp.update_robot(private_id, schedule, p);
                        goto_berth_id = mp.search_berth(good.p, private_id, check_good_expensive(mp));
                    } else {
                        // if (global_debug && frame_id > 1000 && frame_id % 10 == 0 && bfs_cnt > 1)
                        //     std::cerr << frame_id << ": no good schedule " << private_id << std::endl;
                    }
                }
                good_signal = -1;
                good_distance = 0x3f3f3f3f;
            } else if (mp.get_patch(p).high_goods_num > 0) {
                auto good_schedule = mp.search_good(p);
                if (good_schedule.is_valid()) {
                    schedule = good_schedule;
                    auto& good = mp.get_good(schedule.good_id);
                    // if (private_id == 0)
                    //     std::cerr << frame_id << " " << good.p.x << " " << good.p.y << std::endl;
                    good.set_selected(private_id);
                    // mp.remove_good(p);
                    set_goto_good(mp, good.id);
                    mp.map_patches[goto_patch_id].remove_goto_robot();
                    // mp.update_robot(private_id, schedule, p);
                    goto_berth_id = mp.search_berth(good.p, private_id, check_good_expensive(mp));
                } else {
                    // if (global_debug && frame_id > 1000 && frame_id % 10 == 0 && bfs_cnt > 1)
                    //     std::cerr << frame_id << ": no good schedule " << private_id << std::endl;
                }
            }
            if (!schedule.is_valid() && !is_waiting()) {
                set_waiting();
                mp.map_patches[goto_patch_id].remove_goto_robot();
            }
        }
        if (is_goto_berth() && schedule.is_valid() && nxt_good_num >= 1) {
            int goto_berth_id_tmp = mp.search_berth(p, private_id, check_good_expensive(mp));
            if (goto_berth_id_tmp != goto_berth_id) {
                goto_berth_id = goto_berth_id_tmp;
                schedule.clear();
            }
        }
        // mp.update_robot(private_id, schedule, p);
        // 不去有人的货物
        if (is_goto_good() && mp.check_good_col(mp.get_good(goto_good_id).p)) {
            set_waiting();
            schedule.clear();
        }
        if (schedule.is_valid()) {
            if (is_goto_good() && goto_good_id != -1 &&
                (!mp.check_good(mp.get_good(goto_good_id).p) || mp.get_good(goto_good_id).is_pending())) {
                // if (global_debug)
                //     std::cerr << frame_id << ": good disappear [tiqian]" << std::endl;
                set_waiting();
                schedule.clear();
                return;
            }
            if (realtime_change_goto_good && is_goto_good() && good_signal != -1 && mp.check_good(mp.get_good(goto_good_id).p) && mp.check_good(mp.get_good(good_signal).p)) {
                auto& good_old = mp.get_good(goto_good_id);
                good_old.set_exist();
                if (mp.search_max_distance(p, good_old.p) > good_distance) {
                    auto& good = mp.get_good(good_signal);
                    if (good.is_exist()) {
                        auto good_schedule = mp.get_schedule_by_a_star(p, good.p);
                        good_schedule.good_id = good_signal;
                        if (good_schedule.is_valid()) {
                            schedule = good_schedule;
                            good.set_selected(private_id);
                            set_goto_good(mp, good.id);
                            goto_berth_id = mp.search_berth(good.p, private_id, check_good_expensive(mp));
                        } else {
                        }
                    }
                    good_signal = -1;
                    good_distance = 0x3f3f3f3f;
                }
            }
            Direction op = schedule.get_op();
            // if (op == STAY && global_debug) {
            //     std::cerr << frame_id << ": STAY " << private_id << std::endl;
            // }
            move_commands[private_id] = op;
        }
        next_p[0] = p + move_commands[private_id];
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
    bool is_goto_patch() { return extra_status == RobotExtraStatus::GOTOPATCH; }
    bool is_disabled() { return extra_status == RobotExtraStatus::DISABLED; }
    bool is_pending() { return extra_status == RobotExtraStatus::PENDING; }
    // 设置当前状态
    bool set_waiting() {
        extra_status = RobotExtraStatus::WAITING;
        schedule.clear();
        return true;
    }
    bool set_goto_good(Map& mp, int good_id) {
        this->goto_good_id = good_id;
        extra_status = RobotExtraStatus::GOTOGOOD;
        return true;
    }
    bool set_goto_berth(Map& mp, int berth_id) {
        this->goto_berth_id = berth_id;
        if (berth_id == -1) {
            std::cerr << "set_goto_berth: " << private_id << std::endl;
        }
        extra_status = RobotExtraStatus::GOTOBERTH;
        return true;
    }
    bool set_goto_patch(int patch_id) {
        this->goto_patch_id = patch_id;
        if (patch_id == -1) {
            std::cerr << "set_goto_patch: " << private_id << std::endl;
        }
        extra_status = RobotExtraStatus::GOTOPATCH;
        return true;
    }
    bool set_disabled() {
        extra_status = RobotExtraStatus::DISABLED;
        return true;
    }
    bool set_pending(int fid = -1) {
        extra_status = RobotExtraStatus::PENDING;
        if (fid == -1) {
            pending_frame_id = frame_id;
        } else {
            pending_frame_id = fid;
        }
        return true;
    }
    // 机器人的操作
    void get(Map& mp) {
        // std::cerr << frame_id << ": get: " << public_id << std::endl;
        printf("get %d\n", private_id);
        auto& good = mp.get_good(p);
        if (good.val < GOOD_VAL_SPLIT) {
            get_done(mp, good.id);
        } else {
            set_pending(-2);
        }
    }
    void get_done(Map& mp, int good_id, bool is_prv = false) {
        auto& good = mp.get_good(good_id);
        good.set_carried(private_id);
        // good_ids.push(good.id);
        // good_num++;
        if (!is_prv)
            nxt_good_num++;
        set_waiting();
        schedule.clear();
    }
    void pull(Map& mp) {
        if (!good_num) {
            std::cerr << "[pull invalid] private_id: " << private_id << ", frame_id: " << frame_id << std::endl;
            throw std::runtime_error("pull invalid");
        }
        printf("pull %d\n", private_id);

        std::stack<int> temp;
        while (!good_ids.empty()) {
            auto good_id = good_ids.top();
            good_ids.pop();
            temp.push(good_id);
            // std::cerr << "pull: " << good_id << std::endl;
            auto& good = mp.get_good(good_id);
            good.set_done();
#ifdef DEBUG
            pulled_goods_num++;
            pulled_goods_val += good.val;
#endif
        }
        while (!temp.empty()) {
            good_ids.push(temp.top());
            temp.pop();
        }
        goto_good_id = -1;
        // good_num = 0;
        nxt_good_num = 0;
        set_waiting();
        schedule.clear();
    }
    void move(Direction op) {
        if (op == STAY) {
            return;
        }
        printf("move %d %d\n", private_id, op);
        p = p + op;
    }
    void answer(int option) {
        printf("ans %d %d\n", private_id, option);
        is_posted = false;
    }
    void check_ans_result(int good_num, Map& mp) {
        if (!is_pending())
            return;
        // std::cerr << id << " check_ans_result: " << good_num << " " << this->good_num << std::endl;
        auto good_val = mp.get_good(goto_good_id).val;
        if (good_num > this->good_num && good_val > GOOD_VAL_SPLIT) {
            ans_result = true;
            // std::cerr << std::endl << id << " ans_result: true" << std::endl;
        } else {
            ans_result = false;
        }
    }
    bool check_good_expensive(Map& mp) {
        if (!good_ids.empty()) {
            auto good_id = good_ids.top();
            auto& good = mp.get_good(good_id);
            if (good.val > GOOD_VAL_SPLIT) {
                return true;
            } else {
                auto tp = good_id;
                good_ids.pop();
                auto res = check_good_expensive(mp);
                good_ids.push(tp);
                return res;
            }
        }
        return false;
    }
    int get_goods_val(Map& mp) {
        if (!good_ids.empty()) {
            auto good_id = good_ids.top();
            auto& good = mp.get_good(good_id);
            good_ids.pop();
            auto res = get_goods_val(mp) + good.val;
            good_ids.push(good_id);
            return res;
        }
        return 0;
    }
};