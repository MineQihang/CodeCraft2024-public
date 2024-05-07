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

    // 避让算法
    static void do_boats_commands(Map& mp, std::vector<Boat>& boats, std::set<int>& do_schedule_boats) {
        std::map<int, int> boat_to_col_zone;
        std::map<int, int> col_zone_not_avoid;
        std::map<int, BoatOp> boats_op;
        bool flag = 1;
        bool debug = 0;
        int debug_begin_frame = 10298;
        int debug_end_frame = 10300;
        int col_cnt = 0;
        while (flag) {
            flag = 0;
            col_cnt++;
            if (col_cnt > COL_CNT_MAX) {
                std::cerr << "frame" << frame_id << ":boat_col_err!!!\t";
                break;
            }
            boats_op.clear();
            std::vector<ppd> changeable_pos;
            for (auto& boat : boats)
                changeable_pos.push_back({boat.p, boat.dir});
            for (int boat_id : do_schedule_boats) {
                Boat& boat = boats[boat_id];
                boats_op[boat_id] = boat.schedule.get_op(false);
                ppd now_pos = {boat.p, boat.dir};
                if (now_pos == boat.col_recover_ppd)
                    boat.col_recover_ppd = {{-1, -1}, STAY};
                if (boat.col_recover_ppd != default_ppd) {
                    auto recover_schedule = mp.get_boat_schedule(now_pos, boat.col_recover_ppd);
                    boats_op[boat_id] = recover_schedule.ops.front();
                    // while (!recover_schedule.ops.empty()) {
                    //     boat.schedule.ops.push_front(recover_schedule.ops.back());
                    //     recover_schedule.ops.pop_back();
                    // }
                    // boats_op[boat_id] = boat.schedule.get_op(false);
                    // flag = 1;
                }
                if (debug && frame_id >= debug_begin_frame && frame_id <= debug_end_frame)
                    std::cerr << frame_id << ": " << col_cnt << " " << boat_id << ":" << boats_op[boat_id] << std::endl;
                ppd new_ppd = next_ppd(now_pos, boats_op[boat_id]);
                // 碰撞检测
                for (int i = 0; i < (int)changeable_pos.size(); i++) {
                    if (i == boat_id)
                        continue;
                    if (mp.check_boat_collision(new_ppd, changeable_pos[i])) {
                        if (debug && frame_id >= debug_begin_frame && frame_id <= debug_end_frame && boat_id == 2 && i == 0)
                            std::cerr << frame_id << ": ??" << boat_id << " " << i << " " << boat_to_col_zone.count(boat_id)
                                      << std::endl;
                        if (boat_to_col_zone.count(boat_id) == 0 && boat_to_col_zone.count(i) == 0) {
                            int col_zone_id;
                            if (boat_col_zone_reverse == 0)
                                col_zone_id = std::min(boat_id, i);
                            else
                                col_zone_id = std::max(boat_id, i);
                            boat_to_col_zone[boat_id] = col_zone_id;
                            boat_to_col_zone[i] = col_zone_id;
                            col_zone_not_avoid[col_zone_id] = col_zone_id;
                        } else if (boat_to_col_zone.count(boat_id) == 0) {
                            boat_to_col_zone[boat_id] = boat_to_col_zone[i];
                        } else if (boat_to_col_zone.count(i) == 0) {
                            boat_to_col_zone[i] = boat_to_col_zone[boat_id];
                        } else if (boat_to_col_zone[boat_id] != boat_to_col_zone[i]) {
                            int col_zone_id, col_zone_id_max;
                            if (boat_col_zone_reverse == 0) {
                                col_zone_id = std::min(boat_to_col_zone[boat_id], boat_to_col_zone[i]);
                                col_zone_id_max = std::max(boat_to_col_zone[boat_id], boat_to_col_zone[i]);
                            } else {
                                col_zone_id = std::max(boat_to_col_zone[boat_id], boat_to_col_zone[i]);
                                col_zone_id_max = std::min(boat_to_col_zone[boat_id], boat_to_col_zone[i]);
                            }
                            boat_to_col_zone[boat_id] = col_zone_id;
                            boat_to_col_zone[i] = col_zone_id;
                            col_zone_not_avoid.erase(col_zone_id_max);
                        }
                    }
                    if (debug && frame_id >= debug_begin_frame && frame_id <= debug_end_frame &&
                        boat_to_col_zone.count(2) > 0)
                        std::cerr << frame_id << ": " << boat_id << " " << i << " " << boat_to_col_zone[boat_id]
                                  << std::endl;
                }
                // 无碰撞
                if (boat_to_col_zone.count(boat_id) == 0) {
                }
                // 碰撞避免
                else {
                    if (col_zone_not_avoid[boat_to_col_zone[boat_id]] == boat_id) {
                        boats_op[boat_id] = STAY_;
                    } else {
                        std::vector<ppd> col_boats;
                        for (int i = 0; i < (int)changeable_pos.size(); i++) {
                            if (i == boat_id)
                                continue;
                            if (boat_to_col_zone.count(i) && boat_to_col_zone[boat_id] == boat_to_col_zone[i]) {
                                col_boats.push_back(changeable_pos[i]);
                            }
                        }
                        boats_op[boat_id] = mp.get_col_op(now_pos, col_boats);
                        if (boats_op[boat_id] == BoatOp::STAY_) {
                            flag = 1;
                            col_zone_not_avoid[boat_to_col_zone[boat_id]] = boat_id;
                            boat_col_zone_reverse = !boat_col_zone_reverse;
                        }
                        boats_op[col_zone_not_avoid[boat_to_col_zone[boat_id]]] = STAY_;
                    }
                }
                // 位置更新
                changeable_pos[boat_id] = next_ppd(now_pos, boats_op[boat_id]);
            }
        }

        for (int boat_id : do_schedule_boats) {
            if (debug && frame_id >= debug_begin_frame && frame_id <= debug_end_frame)
                std::cerr << frame_id << ":"
                          << " " << boat_id << ":" << boats_op[boat_id] << " " << boat_to_col_zone.count(boat_id)
                          << std::endl;
            if (boats_op[boat_id] == BoatOp::STAY_)
                continue;
            Boat& boat = boats[boat_id];
            if (boat_to_col_zone.count(boat_id) == 0) {
                if (boat.col_recover_ppd == default_ppd)
                    boat.schedule.ops.pop_front();
            } else if (boat.col_recover_ppd == default_ppd) {
                boat.col_recover_ppd = {boat.p, boat.dir};
            }
            ppd new_ppd = next_ppd({boat.p, boat.dir}, boats_op[boat_id]);
            boat.p = new_ppd.first;
            boat.dir = new_ppd.second;
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

    // 防撞算法, 不排除理论碰撞可能
    static void do_commands(std::map<int, Direction>& move_commands, Map& mp, std::vector<Robot>& robots) {
        // if (frame_id == 6265)
        // {
        // }
        bool debug = 0;
        int debug_start_frame = 6265;
        int debug_end_frame = 6265;
        int debug_ids[3] = {1, 3, 8};

        bool has_col = 1;
        bool recovered[(int)robots.size()]{};
        memset(recovered, 0, sizeof(recovered));
        int col_cnt = 0;
        int avoiding[robots.size()]{};
        memset(avoiding, 0, sizeof(avoiding));

        std::function<void(int)> avoid_col_4stay = [&](int k) {
            if (robots[k].is_goto_berth()) {
                robots[k].next_p[1] = robots[k].next_p[0];
                robots[k].next_p[0] = robots[k].p;
                move_commands[k] = STAY;
                avoiding[k]++;
            } else {
                if (robots[k].schedule_need_pop) {
                    robots[k].schedule.ops.pop_front();
                    robots[k].schedule_need_pop = 0;
                }
                if (!recovered[k])
                    robots[k].schedule.recover_op();
                // if(debug&&frame_id==4022&&k==0)std::cerr<<"0:"<<robots[k].schedule.ops.front()<<std::endl;
                robots[k].next_p[1] = robots[k].next_p[0];
                robots[k].next_p[0] = robots[k].p;
                move_commands[k] = STAY;
                avoiding[k]++;
                recovered[k] = true;
                // robots[k].schedule_need_pop = 1;
            }
        };
        std::function<void(int)> avoid_col_4go_berth = [&](int k) {
            move_commands[k] = mp.get_avoid_dir(robots[k].p, move_commands[k]);
            // if (frame_id == 272 && robots[k].p.x == 56 && robots[k].p.y ==
            // 121)
            //     std::std::cerr << move_commands[k];
            robots[k].next_p[0] = robots[k].p + move_commands[k];
            for (int i = 0; i < 4; i++) {
                auto next_p = robots[k].next_p[0] + Direction(i);
                if (mp.check_invalid(next_p))
                    continue;
                if (mp.berths_distance[robots[k].berth_id][next_p.x][next_p.y] <
                    mp.berths_distance[robots[k].berth_id][robots[k].next_p[0].x][robots[k].next_p[0].y]) {
                    robots[k].next_p[1] = next_p;
                    // move_commands[k] = Direction(i);
                    break;
                }
            }
            avoiding[k]++;
        };
        std::function<void(int)> avoid_col_4go_good = [&](int k) {
            if (robots[k].schedule_need_pop)
                robots[k].schedule.ops.pop_front();
            if (!recovered[k])
                robots[k].schedule.ops.push_front(move_commands[k]);
            move_commands[k] = mp.get_avoid_dir(robots[k].p, move_commands[k]);
            robots[k].schedule.recover_history = move_commands[k];
            robots[k].schedule.ops.push_front(reverse_dir(move_commands[k]));
            robots[k].next_p[0] = robots[k].p + move_commands[k];
            robots[k].next_p[1] = robots[k].p;
            robots[k].schedule_need_pop = 1;
            recovered[k] = true;
            avoiding[k]++;
        };
        int col_zone[(int)robots.size()]{-1};  // robot所属碰撞域
        memset(col_zone, -1, sizeof(col_zone));
        int col_zone_not_avoid[(int)robots.size()]{-1};  // 碰撞域中距目标最近的机器人不会进行避让
                                                         // // 不好算距离，直接用最大id
        memset(col_zone_not_avoid, -1, sizeof(col_zone_not_avoid));
        int col_zone_not_avoid_dis[(int)robots.size()]{0x3f3f3f3f};
        memset(col_zone_not_avoid_dis, 0x3f, sizeof(col_zone_not_avoid_dis));
        bool max_or_min = 1;
        std::function<void(int, int)> set_col_zone = [&](int u, int v) {
            if (col_zone[u] >= 0 && col_zone[v] < 0)
                col_zone[v] = col_zone[u];
            else if (col_zone[u] < 0 && col_zone[v] >= 0)
                col_zone[u] = col_zone[v];
            else if (col_zone[u] < 0 && col_zone[v] < 0)
                col_zone[u] = col_zone[v] = std::max(u, v);
            int the_col_zone = col_zone[u];
            if (max_or_min)
                col_zone_not_avoid[the_col_zone] = std::max(std::max(u, v), col_zone_not_avoid[the_col_zone]);
            else
                col_zone_not_avoid[the_col_zone] = std::min(std::min(u, v), col_zone_not_avoid[the_col_zone]);
        };
        while (has_col) {
            has_col = 0;
            col_cnt++;
            if (col_cnt > COL_CNT_MAX) {
                std::cerr << "frame" << frame_id << ":col_err!!!\t";
                break;
            }
            if (col_cnt == COL_CNT_MAX / 2) {
                max_or_min = 0;
            }
            if (debug && frame_id >= debug_start_frame && frame_id <= debug_end_frame) {
                for (int i : debug_ids) {
                    std::cerr << "frame:" << frame_id << "  col_cnt:" << col_cnt << " robot_id:" << i << " "
                              << move_commands[i] << std::endl;
                }
            }
            for (int i = 0; i < (int)robots.size(); i++)
                for (int j = i + 1; j < (int)robots.size(); j++) {
                    bool can_stay = 1;
                    bool has_swap = 0;
                    if (avoiding[i] > avoiding[j]) {
                        std::swap(i, j);
                        has_swap = 1;
                    }
                    // 原地不动
                    if (robots[i].next_p[0] == robots[j].next_p[0] && !mp.check_land_main(robots[i].next_p[0])) {
                        set_col_zone(i, j);
                        int the_col_zone = col_zone[i];
                        bool is_not_avoid{};
                        if (col_zone_not_avoid[the_col_zone] == i) {
                            std::swap(i, j);
                            has_swap = !has_swap;
                            is_not_avoid = 1;
                        }
                        if (col_zone_not_avoid[the_col_zone] == j) {
                            is_not_avoid = 1;
                        }
                        if (robots[i].p != robots[j].next_p[1] && move_commands[i] != STAY && move_commands[j] != STAY) {
                            avoid_col_4stay(i);
                        } else if (robots[j].p != robots[i].next_p[1] && move_commands[i] != STAY &&
                                   move_commands[j] != STAY && !is_not_avoid) {
                            avoid_col_4stay(j);
                        } else {
                            can_stay = 0;
                        }
                        has_col = 1;
                    }
                    // if (robots[i].next_p[0] == robots[j].next_p[0] &&
                    // has_col)
                    //     can_stay = 0;

                    // 原地不动不行
                    if (can_stay == 0 ||
                        (robots[i].p == robots[j].next_p[0] && robots[i].next_p[0] == robots[j].p &&
                         (!mp.check_land_main(robots[i].p) || !mp.check_land_main(robots[j].p)))) {  // 有微小优化空间
                        set_col_zone(i, j);
                        int the_col_zone = col_zone[i];
                        bool is_not_avoid{};
                        if (col_zone_not_avoid[the_col_zone] == i) {
                            std::swap(i, j);
                            has_swap = !has_swap;
                            is_not_avoid = 1;
                        }
                        if (col_zone_not_avoid[the_col_zone] == j) {
                            is_not_avoid = 1;
                        }
                        if (is_not_avoid)
                            avoiding[j]++;
                        if (((robots[i].is_goto_berth())) /* && !robots[i].is_recover()*/) {
                            avoid_col_4go_berth(i);
                        } else {
                            if ((robots[j].is_goto_berth()) && avoiding[i] == avoiding[j] &&
                                !is_not_avoid /* && !robots[j].is_recover()*/)
                                avoid_col_4go_berth(j);
                            // 撞到的话无法正常恢复
                            else if ((robots[i].is_goto_good()) /* && !robots[i].is_recover()*/) {
                                avoid_col_4go_good(i);
                            } else if ((robots[j].is_goto_good()) && !is_not_avoid /*&& !robots[j].is_recover()*/) {
                                avoid_col_4go_good(j);
                            }
                            // 多个休息/碰撞后还没做，打算做计算通道距离的避让
                            else {
                                // std::cerr << "frame" << frame_id <<
                                // ":err107\t";
                            }
                        }
                        has_col = 1;
                    }
                    if (has_swap)
                        std::swap(i, j);
                }
        }
        for (const auto& pair : move_commands) {
            int robot_id = pair.first;
            Direction dir = pair.second;
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