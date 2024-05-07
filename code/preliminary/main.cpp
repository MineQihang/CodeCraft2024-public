#pragma GCC optimize("O2")
#include "berth.hpp"
#include "boat.hpp"
#include "map.hpp"
#include "robot.hpp"
#include "utils.hpp"

using namespace std;

vector<Robot> robots(ROBOT_NUM);
vector<Boat> boats(BOAT_NUM);
vector<Point> init_robots(ROBOT_NUM);
Map mp(MAP_SIZE);
int current_money, new_good_num, boat_capacity, frame_id, skip_frames, goods_cnt, all_val;
bool debug_flag;

// 防撞算法, 仍有理论碰撞可能
void do_commands(std::map<int, Direction>& move_commands) {
    // if (frame_id == 256)
    // {
    //     cerr << "ok72" << endl;
    //     return;
    // }
    bool debug = 0;
    int debug_start_frame = 1932;
    int debug_end_frame = 1936;
    int debug_ids[4] = {0, 3, 6, 8};

    bool has_col = 1;
    bool recovered[ROBOT_NUM]{};
    memset(recovered, 0, sizeof(recovered));
    int col_cnt = 0;
    int avoiding[10]{};
    memset(avoiding, 0, sizeof(avoiding));

    std::function<void(int)> avoid_col_4stay = [&](int k) {
        if (robots[k].going == 'b' || robots[k].going == 'f' || robots[k].going == 'r') {
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
            // if(debug&&frame_id==4022&&k==0)cerr<<"0:"<<robots[k].schedule.ops.front()<<endl;
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
        // if (frame_id == 272 && robots[k].p.x == 56 && robots[k].p.y == 121)
        //     std::cerr << move_commands[k];
        robots[k].next_p[0] = robots[k].p + move_commands[k];
        robots[k].next_p[1] =
            robots[k].next_p[0] +
            mp.berths_path[0][mp.point_to_berth_idx(robots[k].target)][robots[k].next_p[0].x][robots[k].next_p[0].y];
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
    int col_zone[ROBOT_NUM]{-1};  // robot所属碰撞域
    memset(col_zone, -1, sizeof(col_zone));
    int col_zone_not_avoid[ROBOT_NUM]{-1};  // 碰撞域中距目标最近的机器人不会进行避让 // 不好算距离，直接用最大id
    memset(col_zone_not_avoid, -1, sizeof(col_zone_not_avoid));
    int col_zone_not_avoid_dis[ROBOT_NUM]{0x3f3f3f3f};
    memset(col_zone_not_avoid_dis, 0x3f, sizeof(col_zone_not_avoid_dis));
    bool max_or_min = 1;
    std::function<void(int, int)> set_col_zone = [&](int u, int v) {
        if (col_zone[u] >= 0 && col_zone[v] < 0)
            col_zone[v] = col_zone[u];
        else if (col_zone[u] < 0 && col_zone[v] >= 0)
            col_zone[u] = col_zone[v];
        else if (col_zone[u] < 0 && col_zone[v] < 0)
            col_zone[u] = col_zone[v] = max(u, v);
        int the_col_zone = col_zone[u];
        if (max_or_min)
            col_zone_not_avoid[the_col_zone] = max(max(u, v), col_zone_not_avoid[the_col_zone]);
        else
            col_zone_not_avoid[the_col_zone] = min(min(u, v), col_zone_not_avoid[the_col_zone]);
        // if (robots[u].target_dis < col_zone_not_avoid_dis[the_col_zone])
        // {
        //     col_zone_not_avoid[the_col_zone] = u;
        //     col_zone_not_avoid_dis[the_col_zone] = robots[u].target_dis;
        // }
        // if (robots[v].target_dis < col_zone_not_avoid_dis[the_col_zone])
        // {
        //     col_zone_not_avoid[the_col_zone] = v;
        //     col_zone_not_avoid_dis[the_col_zone] = robots[v].target_dis;
        // }
    };
    while (has_col) {
        has_col = 0;
        col_cnt++;
        if (col_cnt > 20) {
            cerr << "frame" << frame_id << ":col_err!!!\t";
            break;
        }
        if (col_cnt == 10) {
            max_or_min = 0;
        }
        if (debug && frame_id >= debug_start_frame && frame_id <= debug_end_frame) {
            for (int i : debug_ids) {
                cerr << "frame:" << frame_id << "  col_cnt:" << col_cnt << " robot_id:" << i << " " << move_commands[i]
                     << endl;
            }
        }
        for (int i = 0; i < ROBOT_NUM; i++)
            for (int j = i + 1; j < ROBOT_NUM; j++) {
                bool can_stay = 1;
                bool has_swap = 0;
                if (avoiding[i] > avoiding[j]) {
                    swap(i, j);
                    has_swap = 1;
                }
                // 原地不动
                if (robots[i].next_p[0] == robots[j].next_p[0] /* && !has_col*/) {
                    set_col_zone(i, j);
                    int the_col_zone = col_zone[i];
                    bool is_not_avoid{};
                    if (col_zone_not_avoid[the_col_zone] == i) {
                        swap(i, j);
                        has_swap = !has_swap;
                        is_not_avoid = 1;
                    }
                    if (col_zone_not_avoid[the_col_zone] == j) {
                        is_not_avoid = 1;
                    }
                    if (robots[i].p != robots[j].next_p[1] && move_commands[i] != STAY && move_commands[j] != STAY) {
                        avoid_col_4stay(i);
                    } else if (robots[j].p != robots[i].next_p[1] && move_commands[i] != STAY && move_commands[j] != STAY &&
                               !is_not_avoid) {
                        avoid_col_4stay(j);
                    } else {
                        can_stay = 0;
                    }
                    has_col = 1;
                }
                // if (robots[i].next_p[0] == robots[j].next_p[0] && has_col)
                //     can_stay = 0;

                // 原地不动不行
                if (can_stay == 0 || robots[i].p == robots[j].next_p[0] && robots[i].next_p[0] == robots[j].p) {
                    set_col_zone(i, j);
                    int the_col_zone = col_zone[i];
                    bool is_not_avoid{};
                    if (col_zone_not_avoid[the_col_zone] == i) {
                        swap(i, j);
                        has_swap = !has_swap;
                        is_not_avoid = 1;
                    }
                    if (col_zone_not_avoid[the_col_zone] == j) {
                        is_not_avoid = 1;
                    }
                    if (is_not_avoid)
                        avoiding[j]++;
                    if ((robots[i].has_target && robots[i].going == 'b' || robots[i].going == 'f' ||
                         robots[i].going == 'r') &&
                        !robots[i].is_recover()) {
                        avoid_col_4go_berth(i);
                    } else {
                        if ((robots[j].has_target && robots[j].going == 'b' || robots[j].going == 'f' ||
                             robots[j].going == 'r') &&
                            avoiding[i] == avoiding[j] && !is_not_avoid && !robots[j].is_recover())
                            avoid_col_4go_berth(j);
                        // 撞到的话无法正常恢复
                        else if (robots[i].has_target && robots[i].going == 'g' && !robots[i].is_recover()) {
                            avoid_col_4go_good(i);
                        } else if (robots[j].has_target && robots[j].going == 'g' && !is_not_avoid &&
                                   !robots[j].is_recover()) {
                            avoid_col_4go_good(j);
                        }
                        // 多个休息/碰撞后还没做，打算做计算通道距离的避让
                        else {
                            // cerr << "frame" << frame_id << ":err107\t";
                        }
                    }
                    has_col = 1;
                }
                if (has_swap)
                    swap(i, j);
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

void init_manual_boat_plan() {
    for (int i = 0; i < BOAT_NUM; i++) {
        int load_time[2], loop_time[2]{}, now_time = 15000;
        memset(loop_time, 0, sizeof(loop_time));
        Berth* berth[2] = {&mp.berths[boat_plan_init[map_name][i]], &mp.berths[boat_plan_init[map_name][i + 5]]};
        // cerr << "ok86 ";
        load_time[0] = (boats[i].capacity + 5) / berth[0]->loading_speed;
        load_time[1] = (boats[i].capacity + 5) / berth[1]->loading_speed;
        loop_time[0] = berth[0]->new_transport_time * 2 + load_time[0];
        loop_time[1] = berth[1]->new_transport_time * 2 + load_time[1];
        int flag = 0, last_leave = 1, same_berth = 0;
        if (berth[flag]->id == berth[!flag]->id)
            same_berth = 1;
        // cerr << "ok87 ";
        while (1) {
            // cerr << "ok88 " << loop_time[flag];
            // abort();
            int in_berth_ship_ = (!same_berth && boat_in_berth_ship[map_name][i]) ? 1 : 0;
            if (in_berth_ship_) {
                if (!flag)
                    now_time -= berth[0]->new_transport_time + load_time[0] + time_in_berth_move;
                else
                    now_time -= berth[1]->new_transport_time + load_time[1];
            } else
                now_time -= loop_time[flag];
            if (now_time < 1)
                break;
            int new_start = now_time;
            if (berth[flag]->does_trans) {
                if (in_berth_ship_ && !flag) {
                    boat_plan[i].emplace(new_start, "ship " + to_string(i) + " " + to_string(berth[0]->id) + "\n");
                    new_start += time_in_berth_move + load_time[0];
                } else {
                    boat_plan[i].emplace(new_start, "ship " + to_string(i) + " " + to_string(min_trans_id) + "\n");
                    boat_plan[i].emplace(new_start + min_trans_time,
                                         "ship " + to_string(i) + " " + to_string(berth[flag]->id) + "\n");
                    new_start += berth[flag]->new_transport_time + load_time[flag];
                }
                if (in_berth_ship_ && flag)
                    boat_plan[i].emplace(new_start, "ship " + to_string(i) + " " + to_string(berth[0]->id) + "\n");
                else {
                    boat_plan[i].emplace(new_start, "ship " + to_string(i) + " " + to_string(min_trans_id) + "\n");
                    boat_plan[i].emplace(new_start + time_in_berth_move, "go " + to_string(i) + "\n");
                }
            } else {
                boat_plan[i].emplace(new_start, "ship " + to_string(i) + " " + to_string(berth[flag]->id) + "\n");
                if (in_berth_ship_ && !flag)
                    new_start += time_in_berth_move + load_time[0];
                else
                    new_start += berth[flag]->new_transport_time + load_time[flag];
                if (in_berth_ship_ && flag)
                    boat_plan[i].emplace(new_start, "ship " + to_string(i) + " " + to_string(berth[0]->id) + "\n");
                else
                    boat_plan[i].emplace(new_start, "go " + to_string(i) + "\n");
            }
            if (flag && last_leave && !same_berth) {
                disable_plan[map_name].push_back({new_start - 1, berth[1]->id});
                mp.berths[berth[1]->id].disable_time = new_start - 1;
                last_leave = 0;
            }
            flag = !flag;
        }
    }
}

void init() {
    cerr << endl;
    mp.init(init_robots);
    vector<Berth> berths_init(BERTH_NUM);
    for (int i = 0, berth_id; i < BERTH_NUM; i++) {
        scanf("%d", &berth_id);
        berths_init[berth_id].init(berth_id);
    }
    mp.init_berths(berths_init);
    for (int i = 0; i < ROBOT_NUM; i++)
        robots[i].set_id(i);
    scanf("%d", &boat_capacity);
    mp.special_init(boat_capacity);
    // average_trans_time: 1000
    cerr << "boat_capacity: " << boat_capacity << endl;
    for (int i = 0; i < BOAT_NUM; i++) {
        boats[i].set_id(i);
        boats[i].init_capacity(boat_capacity);
    }
    mp.init_boat_plan(init_robots);
    // cerr << "ok85 ";
    if (manual_plan && !get_data)
        init_manual_boat_plan();
    if (get_data)
        for (int i = 0; i < BOAT_NUM; i++) {
            boat_plan[i].emplace(1, "ship " + to_string(i) + " " + to_string(i) + "\n");
            boat_plan[i].emplace(1 + mp.berths[i].transport_time + boat_capacity, "go " + to_string(i) + "\n");
            boat_plan[i].emplace(1 + mp.berths[i].transport_time * 2 + boat_capacity,
                                 "ship " + to_string(i) + " " + to_string(i + 5) + "\n");
            boat_plan[i].emplace(12000, "go " + to_string(i) + "\n");
        }
    get_ok();
    put_ok();
}

void interaction_input() {
    bool debug = 0;
    scanf("%d%d", &frame_id, &current_money);
    scanf("%d", &new_good_num);
    for (int i = 0; i < new_good_num; i++) {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        goods_cnt++;
        all_val += val;
        // min_val = min(val, min_val);
        mp.goods.add(x, y, val, frame_id + FADE_TIME);
    }
    if (good_select)
        mp.process_goods(frame_id);
    // mp.init_robots_map();
    for (int i = 0; i < ROBOT_NUM; i++) {
        robots[i].update();
        mp.update_robot(robots[i].p, i);
    }
    if (!mp.robots_available[ROBOT_NUM]) {
        for (int i = 0; i < ROBOT_NUM; i++) {
            for (int j = 0; j < BERTH_NUM; j++) {
                if (mp.berths_distance[j * 16 + 4 + 1][robots[i].p.x][robots[i].p.y] < max_int_) {
                    mp.robots_available[i] = 1;
                    break;
                }
            }
            // std::cerr << i << " " << mp.nearest_berth_distance[robots[i].p.x][robots[i].p.y] << std::endl;
        }
        // std::cerr << robots_available[4] << std::endl;
        mp.robots_available[ROBOT_NUM] = true;
    }
    // 更新自己算的分
    for (int i = 0; i < BOAT_NUM; i++) {
        boats[i].update();
        if (boats[i].time_score_pair.first == frame_id) {
            if (debug)
                cerr << "time:" << frame_id << " get score:" << boats[i].time_score_pair.second << "  by boat:" << i << endl;
            my_score += boats[i].time_score_pair.second;
            boats[i].time_score_pair.first = -1;
            boats[i].time_score_pair.second = 0;
        }
    }
    if (manual_disable)
        for (auto e : disable_plan.at(map_name)) {
            if (e.first == frame_id)
                mp.berth_disabled[e.second] = true;
        }
    get_ok();
    // cerr << "frame_id: " << frame_id << endl;
}

void final_log() {
    if (skip_frames)
        std::cerr << std::endl;
    if (skip_frames)
        std::cerr << "skipped frames number: " << skip_frames << std::endl;
    int val_sum_sum = 0;
    if (max_trans_time - min_trans_time > 500)
        cerr << "max_trans_time-min_trans_time>500 " << max_trans_time << " " << min_trans_time << endl;
    cerr << "pulled_goods_num: " << pulled_goods_num << "  pulled_goods_val: " << pulled_goods_val
         << "  average_pulled_goods_val: " << double(pulled_goods_val) / pulled_goods_num << endl;
    cerr << "goods_num: " << goods_cnt << "  goods_value: " << all_val
         << "  average_goods_value: " /*<< std::fixed << std::setprecision(1) */ << double(all_val) / goods_cnt
         << "  my_score: " << my_score << endl;
    double ave_pred_num_per100_diff = 0;
    for (int i = 0; i < BERTH_NUM; i++) {
        val_sum_sum += mp.berths[i].val_sum;
        std::cerr << "berth:" << i << ", num_per_100:"
                  << double(mp.berths[i].num_sum) / 150
                  //<< ",\tpred_num_per100:" << mp.berths[i].pred_num_per100
                  //<< ",\tefficiency:" << mp.berths[i].efficiency << ", val_sum: " << mp.berths[i].val_sum
                  << ", goods_cnt: " << mp.berths[i].goods_cnt << ",\ttransport_time: " << mp.berths[i].transport_time
                  << std::endl;
        ave_pred_num_per100_diff += abs(double(mp.berths[i].num_sum) / 150 - mp.berths[i].pred_num_per100);
    }
    std::cerr << "berth_val_sum: "
              << val_sum_sum
              //<< ", ave_pred_num_per100_diff:" << ave_pred_num_per100_diff / BERTH_NUM
              << std::endl;
}

void interaction_output() {
    try {
        skip_frames += frame_skip_detection(frame_id);
        if (my_score != current_money && !get_data) {
            if (!debug_flag) {
                debug_flag = 1;
                cerr << "my_score!=current_money at frame " << frame_id << endl;
            }
            put_ok();
            return;
        }
        // if (frame_id > 300)
        // {
        //     put_ok();
        //     return;
        // }
        if (skip_frames && skip_frame_detect) {
            abort();
            exit(-1);
            put_ok();
            return;
        }
        if (frame_id == FRAME_NUM) {
            final_log();
            put_ok();
            return;
        }
        std::map<int, Direction> move_commands;
        // std::cerr << "frame_id: " << frame_id << std::endl;
        vector<int> v;
        for (int i = 0; i < 10; i++)
            v.push_back(i);
        // shuffle(v.begin(), v.end(), gen);
        for (int i : v) {
            robots[i].do_op(mp, move_commands, frame_id);
        }
        do_commands(move_commands);
        for (int i = 0; i < BOAT_NUM; i++) {
            boats[i].do_op(mp, frame_id);
        }
        put_ok();
        for (auto& berth : mp.berths) {
            berth.update_efficiency(frame_id);
        }
    } catch (const exception& e) {
        std::cerr << "Caught an exception: " << e.what() << endl;
    }
}

int main() {
    // std::ofstream file("C:/Users/Yipeng/Contest/CodeCraft2024/SDK/logs/info.log");
    // std::streambuf* oldCerrStreamBuf = std::cerr.rdbuf();
    // std::cerr.rdbuf(file.rdbuf());
    init();
    // std::cerr << robots[0].scale_search << std::endl;
    for (int frame = 1; frame <= FRAME_NUM; frame++) {
        interaction_input();
        interaction_output();
    }
    // Restore old cerr stream buffer
    // std::cerr.rdbuf(oldCerrStreamBuf);
    return 0;
}
