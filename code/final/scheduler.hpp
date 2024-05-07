#pragma once
#pragma GCC optimize("O2")
#include "boat.hpp"
#include "map.hpp"
#include "robot.hpp"
#include "schedule.hpp"
#include "utils.hpp"

class Scheduler {
   public:
    static Schedule do_schedule(Map& mp, Point from, Point to) {
        Schedule schedule;
        schedule.put_op(UP);
        schedule.put_op(STAY);
        return schedule;
    }

    // 轮船避让算法
    static void do_boats_commands(Map& mp, std::vector<Boat*> boats, std::set<int>& do_schedule_boats) {
        std::map<int, BoatOp> boats_op;
        int random_option;
        bool debug = global_debug && 0;
        int debug_begin_frame = 0;
        int debug_end_frame = 0;
        for (int boat_id : do_schedule_boats) {
            auto& boat = *boats[boat_id];
            boats_op[boat_id] = boat.schedule.get_op();
            if (mp.col_boats.count(boat_id)) {
                random_option = random_int(0, 99);
                if (debug)
                    std::cerr << frame_id << ": boat " << boat_id << " col " << random_option << " " << boats_op[boat_id]
                              << std::endl;
                // 原地停3帧
                if (random_option < 50) {
                    boat.schedule.recover_op();
                    boat.schedule.ops.push_front(STAY_);
                    boat.schedule.ops.push_front(STAY_);
                    boats_op[boat_id] = STAY_;
                }
                // 重新规划路径
                else {
                    ppd now = {boat.p, boat.dir};
                    if (boat.check_goto_berth()) {
                        boat.schedule = mp.get_boat_schedule_a_star(
                            now, {mp.berths[boat.obj_berth_id].p, mp.berth_direction[boat.obj_berth_id]}, boat.public_id,
                            true);
                        if (boat.schedule.check_invalid()) {
                            boats_op[boat_id] = STAY_;
                        } else {
                            boats_op[boat_id] = boat.schedule.get_op();
                        }
                    } else if (boat.check_goto_delivery()) {
                        boat.schedule = mp.get_boat_schedule_a_star(now, {mp.delivery_points[boat.obj_delivery_id], STAY},
                                                                    boat.public_id, true);
                        if (boat.schedule.check_invalid()) {
                            boats_op[boat_id] = STAY_;
                        } else {
                            boats_op[boat_id] = boat.schedule.get_op();
                        }
                    } else
                        std::cerr << frame_id << ": boat status error" << std::endl;
                    if (debug && frame_id >= debug_begin_frame && frame_id <= debug_end_frame)
                        std::cerr << frame_id << ": boat " << boat_id << " " << boats_op[boat_id] << std::endl;
                }
            }
        }

        for (int boat_id : do_schedule_boats) {
            if (debug && frame_id >= debug_begin_frame && frame_id <= debug_end_frame)
                std::cerr << frame_id << ":"
                          << " " << boat_id << ":" << boats_op[boat_id] << " " << std::endl;
            Boat& boat = *boats[boat_id];
            boat.nxt_ppd = next_ppd({boat.p, boat.dir}, boats_op[boat_id]);
            if (boats_op[boat_id] == BoatOp::STAY_)
                continue;
            if (boats_op[boat_id] == BoatOp::ANTICLOCKWISE)
                boat.do_anticlockwise_rotate();
            else if (boats_op[boat_id] == BoatOp::CLOCKWISE)
                boat.do_clockwise_rotate();
            else if (boats_op[boat_id] == BoatOp::FORWARD)
                boat.do_ship();
            else
                throw "error";
        }
    }

    // 机器人避让算法
    static void do_commands(std::map<int, Direction>& move_commands, Map& mp, std::vector<Robot*> robots) {
        // if (frame_id == 6265)
        // {
        // }
        // bool debug = 0;
        // int debug_start_frame = 3850;
        // int debug_end_frame = 3900;
        // int debug_ids[3] = {1, 3, 8};

        for (int i = 0; i < (int)robots.size(); i++) {
            auto& robot_i = *robots[i];
            int random_option{0};
            if (mp.col_robots.count(i)) {
                // 随机选择避让策略
                random_option = random_int(0, 99);
                // if (frame_id >= 3850 && frame_id <= 3900)
                // std::cerr << frame_id << ": robot " << i << " col " << random_option << " " << move_commands[i]
                //           << std::endl;
            }
            // 有碰撞且去货物
            if (mp.col_robots.count(i) && (robot_i.is_goto_good() || robot_i.is_goto_patch())) {
                // TODO：优化
                // 搜索避让
                if (random_option < 30) {
                    robot_i.schedule.recover_op();
                    Point cur = robot_i.p;
                    size_t pos = 0;
                    while (pos < robot_col_search_range * 2) {
                        if (pos >= robot_i.schedule.ops.size())
                            break;
                        cur = cur + robot_i.schedule.ops[pos];
                        pos++;
                        if (pos > robot_col_search_range &&
                            (mp.robots_map.count({cur.x, cur.y}) == 0 || mp.land_main[cur.x][cur.y]))
                            break;
                    }
                    if (pos >= robot_col_search_range * 2) {
                        if (global_debug)
                            std::cerr << frame_id << ": robot " << i << " col_search " << move_commands[i] << std::endl;
                        move_commands[i] = STAY;
                    } else {
                        Schedule local_schedule = mp.get_schedule_by_bfs(robot_i.p, cur);
                        if (!local_schedule.is_valid()) {
                            move_commands[i] = STAY;
                        } else {
                            while (pos--) {
                                robot_i.schedule.ops.pop_front();
                            }
                            while (local_schedule.ops.size()) {
                                robot_i.schedule.ops.push_front(local_schedule.ops.back());
                                local_schedule.ops.pop_back();
                            }
                            move_commands[i] = robot_i.schedule.get_op();
                        }
                    }
                }
                // 随机避让
                else if (random_option < 40) {
                    robot_i.schedule.recover_op();
                    move_commands[i] = mp.get_avoid_dir(robot_i.p, move_commands[i]);
                    robot_i.schedule.recover_history = move_commands[i];
                    robot_i.schedule.ops.push_front(reverse_dir(move_commands[i]));
                }
                // 原地不动
                else if (random_option < 70) {
                    robot_i.schedule.recover_op();
                    move_commands[i] = STAY;
                }
                // 继续原来的命令
                else {
                    // Point nxt = robot_i.p + move_commands[i];
                    // if (mp.robots_num[robot_i.p.x][robot_i.p.y] > 1 && mp.land_main[robot_i.p.x][robot_i.p.y] &&
                    //     !mp.land_main[nxt.x][nxt.y]) {
                    //     robot_i.schedule.recover_op();
                    //     move_commands[i] = STAY;
                    // }
                }
                // std::cerr << frame_id << ": robot " << i << " col_go_good " << move_commands[i] << std::endl;
            }
            // 去泊位，且无规划
            if (robot_i.is_goto_berth() && move_commands[i] == STAY) {
                move_commands[i] = mp.get_robot_next_dir(robot_i.p, robot_i.goto_berth_id);
            }
            // 去泊位且有碰撞
            if (mp.col_robots.count(i) && robot_i.is_goto_berth()) {
                // 搜索避让
                if (random_option < 30) {
                    robot_i.schedule.clear();
                    Point cur = robot_i.p;
                    size_t cnt = 0;
                    while (cnt < robot_col_search_range * 2) {
                        cur = cur + mp.get_robot_next_dir(cur, robot_i.goto_berth_id);
                        cnt++;
                        if (cnt > robot_col_search_range &&
                            (mp.robots_map.count({cur.x, cur.y}) == 0 || mp.land_main[cur.x][cur.y]))
                            break;
                        if (mp.check_berth(cur) && robot_i.goto_berth_id == mp.get_berth(cur).id)
                            break;
                    }
                    if (cnt >= robot_col_search_range * 2) {
                        if (global_debug)
                            std::cerr << frame_id << ": robot " << i << " col_search2 " << move_commands[i] << std::endl;
                        move_commands[i] = STAY;
                    } else {
                        robot_i.schedule = mp.get_schedule_by_bfs(robot_i.p, cur);
                        if (robot_i.schedule.is_valid()) {
                            move_commands[i] = robot_i.schedule.get_op();
                        } else {
                            move_commands[i] = STAY;
                        }
                    }
                }
                // 重选方向,不去已有机器人的地方
                else if (random_option < 35) {
                    robot_i.schedule.clear();
                    move_commands[i] = mp.get_robot_next_dir(robot_i.p, robot_i.goto_berth_id, true);
                    if (move_commands[i] == STAY) {
                        if (global_debug)
                            std::cerr << frame_id << ": no dir " << i << std::endl;
                    }
                }
                // 随机避让
                else if (random_option < 40) {
                    robot_i.schedule.clear();
                    move_commands[i] = mp.get_avoid_dir(robot_i.p, move_commands[i]);
                }
                // 原地不动
                else if (random_option < 70) {
                    robot_i.schedule.clear();
                    move_commands[i] = STAY;
                }
                // 执行原来的命令
            }
        }
        // if (frame_id == 53)
        //     std::cerr << "robot " << 0 << " " << move_commands[0] << std::endl;
        // 执行命令
        for (const auto& pair : move_commands) {
            int robot_id = pair.first;
            Direction dir = pair.second;
            robots[robot_id]->next_p[0] = robots[robot_id]->p + dir;
            if (dir == STAY)
                continue;
            if (dir < 0 || dir > 3) {
                std::cerr << "frame " << frame_id << " move " << robot_id << " " << int(dir) << std::endl;
                continue;
            }
            printf("move %d %d\n", robot_id, int(dir));
        }
    }
};