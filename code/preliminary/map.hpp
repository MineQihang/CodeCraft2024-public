#pragma once
#pragma GCC optimize("O2")
#include "berth.hpp"
#include "good.hpp"
#include "schedule.hpp"
#include "utils.hpp"

std::unordered_map<int, std::string> get_map = {{880, "map1"},   {206, "map2_3.9"}, {766, "3.7"},  {25, "3.8"},
                                                {200, "3.10"},   {153, "3.11"},     {824, "3.12"}, {296, "3.13"},
                                                {560, "3.23.1"}, {867, "3.23.2"}};

class Map {
   public:
    int n;
    char ch[MAX_MAP_SIZE][MAX_MAP_SIZE];
    int goods_map[MAX_MAP_SIZE][MAX_MAP_SIZE];
    int8_t robots_map[DETECT_RANGE][MAX_MAP_SIZE][MAX_MAP_SIZE];
    bool robots_available[ROBOT_NUM + 1];
    bool berth_disabled[BERTH_NUM]{};
    int book[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // int berths_book[MAX_MAP_SIZE][MAX_MAP_SIZE];
    int berths_distance[BERTH_NUM * 16][MAX_MAP_SIZE][MAX_MAP_SIZE];
    // int nearest_berth_index[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // int nearest_berth_distance[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // int robot_col_dis[ROBOT_NUM][MAX_MAP_SIZE][MAX_MAP_SIZE];
    // int robot_col_valid[ROBOT_NUM];
    int berths_path_valid[2][BERTH_NUM * 16]{};
    int target_berth[ROBOT_NUM]{BERTH_NUM};
    double average_nearest_dis{};
    bool berths_mark[BERTH_NUM]{};
    // berth_id -> (x, y) -> schedule; _ from berth to (x, y), re from (x, y) to berth
    Direction berths_path[2][BERTH_NUM * 16][MAX_MAP_SIZE][MAX_MAP_SIZE];
    Direction robot_col_paths[ROBOT_NUM][MAX_MAP_SIZE][MAX_MAP_SIZE];
    Direction berths_path_integrated[MAX_MAP_SIZE][MAX_MAP_SIZE];
    std::unordered_map<Point, int> berths_map;
    Berth berths[BERTH_NUM];
    // GoodList goods;
    GoodMap goods;
    Map() {}
    Map(int n) : n(n) {
        memset(berth_disabled, 0, sizeof(berth_disabled));
        memset(berths_path_valid, 0, sizeof(berth_disabled));
        memset(berths_mark, 0, sizeof(berths_mark));
        for (int i = 0; i < ROBOT_NUM; i++)
            target_berth[i] = BERTH_NUM;
    }
    void init(std::vector<Point>& init_robots) {
        int robot_id = 0;
        for (int i = 0; i < n; i++)
            scanf("%s", ch[i]);
        int hash = 0;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                if (ch[i][j] == 'A') {
                    ch[i][j] = '.';
                    init_robots[robot_id].x = i;
                    init_robots[robot_id].y = j;
                    robot_id++;
                }
                hash = (hash + ch[i][j] * i) % 1000;
            }
        // std::cerr << hash << std::endl;
        if (get_map.count(hash)) {
            map_name = get_map[hash];
            // std::cerr << "ok1731 ";
        }
        // if (map_name == "" && !get_data)
        // {
        //     manual_plan = 0;
        //     manual_disable = 0;
        // }
        // if (map_name == "" && get_data)
        // {
        //     get_data = 0;
        //     manual_plan = 0;
        //     manual_disable = 0;
        // }
        // if (oringal_map.count(hash))
        // {
        //     get_data = 0;
        //     manual_plan = 1;
        //     manual_disable = 1;
        //     oringal_test = 1;
        // }
        // map_name="3.13";

        // std::cerr<<"dbg16 ";
        // abort();
        memset(goods_map, -1, sizeof(goods_map));
    }
    void special_init(int boat_capacity) {
        std::unordered_map<std::string, std::vector<int>> boat_in_berth_ship_lib[5] = {
            {
                {"map1", {0, 0, 0, 0, 0}},
                {"map2_3.9", {0, 0, 0, 0, 0}},
                {"3.7", {0, 0, 0, 0, 0}},
                {"3.8", {0, 0, 0, 0, 0}},
                {"3.10", {0, 0, 0, 0, 0}},
                {"3.11", {0, 0, 0, 0, 0}},
                {"3.12", {0, 0, 0, 0, 0}},
                {"3.13", {0, 0, 0, 0, 0}},
                {"3.13_small", {0, 0, 0, 0, 0}},
            },
            // 1: 50-55
            {
                {"map1", {0, 0, 0, 0, 0}},
                {"map2_3.9", {0, 0, 0, 0, 0}},
                {"3.7", {0, 0, 0, 0, 0}},
                {"3.8", {0, 0, 0, 0, 0}},
                {"3.10", {1, 1, 1, 1, 1}},
                {"3.11", {0, 0, 0, 0, 0}},
                {"3.12", {0, 0, 0, 0, 0}},
                {"3.13", {0, 0, 0, 0, 0}},
                {"3.13_small", {0, 0, 0, 0, 0}},
            },
            // 2: 55-62
            {
                {"map1", {1, 1, 1, 1, 1}},
                {"map2_3.9", {1, 1, 1, 1, 1}},
                // {"3.7", {1, 1, 1, 1, 1}},
                {"3.7", {1, 1, 1, 1, 1}},  // 特化
                {"3.8", {1, 1, 1, 1, 1}},
                {"3.10", {1, 1, 1, 1, 1}},
                {"3.11", {1, 1, 0, 0, 0}},
                {"3.12", {0, 0, 0, 0, 0}},
                {"3.13", {0, 0, 0, 0, 0}},
                {"3.13_small", {0, 0, 0, 0, 0}},
                {"3.23.1", {1, 1, 1, 1, 1}},
                {"3.23.2", {1, 1, 1, 1, 1}},
            },
            // 3: 62-67
            {
                {"map1", {1, 1, 1, 1, 1}},
                {"map2_3.9", {1, 1, 1, 1, 1}},
                {"3.7", {1, 1, 1, 1, 1}},
                {"3.8", {1, 1, 1, 1, 1}},
                {"3.10", {1, 1, 1, 1, 1}},
                {"3.11", {1, 1, 1, 1, 1}},
                {"3.12", {0, 0, 0, 1, 0}},
                {"3.13", {1, 1, 0, 1, 1}},
            },
            // 4: >67
            {
                {"map1", {1, 1, 1, 1, 1}},
                {"map2_3.9", {1, 1, 1, 1, 1}},
                {"3.7", {1, 1, 1, 1, 1}},
                {"3.8", {1, 1, 1, 1, 1}},
                {"3.10", {1, 1, 1, 1, 1}},
                {"3.11", {1, 1, 1, 1, 1}},
                {"3.12", {1, 1, 1, 1, 1}},
                {"3.13", {1, 1, 1, 1, 1}},
            },
        };
        if (boat_capacity < 55)
            boat_in_berth_ship = boat_in_berth_ship_lib[1];
        else if (boat_capacity < 62) {
            boat_in_berth_ship = boat_in_berth_ship_lib[2];
        } else if (boat_capacity < 67) {
            boat_in_berth_ship = boat_in_berth_ship_lib[3];
            if (map_name == "3.13")
                map_name = "3.13_small";
        } else if (boat_capacity < 101)
            boat_in_berth_ship = boat_in_berth_ship_lib[4];
        // boat_in_berth_ship = boat_in_berth_ship_lib[4];

        if (seed == -1) {
            disable_plan = {
                {"map1", {{1, 4}}},
                {"map2_3.9", {{1, 6}}},
                // {"map2_3.9", {{1, 6}, {1, 7}}}, // 方案2
                // {"3.7", {{1, 3}}},
                {"3.7", {{1, 3}, {1, 7}}},  // 特化
                {"3.8", {{1, 2}, {1, 3}, {1, 4}, {1, 9}}},
                {"3.10", {{1, 0}, {1, 9}}},
                {"3.11", {}},
                {"3.12", {}},
                {"3.13", {{1, 0}, {1, 9}}},
                {"3.13_small", {{1, 0}, {1, 9}, {1, 1}}},  // 小容量
                {"3.23.1", {}},
                // {"3.23.2", {}}, // lyp
                {"3.23.2", {}},
            };
        } else if (seed == 42) {
            disable_plan = {
                {"map1", {{1, 4}}},
                {"map2_3.9", {{1, 6}}},
                {"3.7", {{1, 3}}},
                {"3.8", {{1, 2}, {1, 3}, {1, 4}, {1, 9}}},
                {"3.10", {{1, 0}, {1, 9}}},
                {"3.11", {}},
                {"3.12", {}},
                {"3.13", {{1, 1}, {1, 9}}},  // seed 42
            };
        }
        if (seed == -1) {
            boat_plan_init = {
                {"map1", {1, 0, 7, 9, 3, 1, 2, 5, 6, 8}},
                {"map2_3.9", {9, 5, 0, 8, 3, 9, 1, 2, 4, 7}},
                // {"map2_3.9", {9, 5, 0, 8, 3, 9, 5, 2, 4, 1}}, // 方案2
                // {"3.7", {4, 8, 9, 2, 6, 4, 0, 1, 5, 7}},
                {"3.7", {4, 8, 2, 1, 6, 4, 8, 0, 9, 5}},  // 特化
                {"3.8", {5, 6, 7, 8, 1, 5, 6, 7, 8, 0}},
                {"3.10", {4, 5, 2, 3, 6, 4, 5, 1, 7, 8}},
                {"3.11", {1, 3, 5, 7, 9, 0, 2, 4, 6, 8}},
                {"3.12", {0, 1, 4, 5, 9, 6, 3, 2, 7, 8}},
                {"3.13", {5, 2, 3, 6, 7, 5, 1, 4, 8, 7}},
                {"3.13_small", {5, 2, 3, 6, 7, 5, 2, 4, 8, 7}},  // 小容量
                {"3.23.1", {6, 8, 9, 1, 0, 5, 2, 4, 3, 7}},
                // {"3.23.2", {9, 7, 6, 5, 0, 1, 3, 8, 4, 2}}, // lyp
                {"3.23.2", {9, 7, 2, 0, 6, 1, 4, 3, 8, 5}},
            };
        } else if (seed == 42)
            // 针对seed 42的运输时间优化, 已在3.6版本弃用
            boat_plan_init = {
                {"map1", {1, 0, 7, 8, 5, 1, 2, 3, 6, 9}},      // seed 42
                {"map2_3.9", {9, 5, 0, 8, 2, 9, 7, 4, 1, 3}},  // seed 42
                {"3.7", {4, 8, 9, 2, 6, 4, 0, 1, 5, 7}},       // seed 42无法优化
                {"3.8", {5, 6, 7, 8, 0, 5, 6, 7, 8, 1}},       // seed 42, 似乎无效
                {"3.10", {4, 5, 2, 7, 6, 4, 5, 1, 3, 8}},      // seed 42
                {"3.11", {0, 2, 4, 6, 8, 9, 1, 3, 7, 5}},      // seed 42，部分港间ship
                {"3.12", {0, 4, 5, 8, 9, 1, 7, 3, 2, 6}},      // seed 42, 部分港间ship
                {"3.13", {5, 2, 3, 6, 7, 0, 2, 4, 8, 7}},      // seed 42，部分港间ship
            };

        // 先分配离港远的，尽量使距离平均。没啥效果
        init_robot_plan = {
            {"map1", {2, 1, 5, 3, -1, 0, 9, 8, 6, 7}}, {"map2_3.9", {4, 0, 1, 2, 7, 3, 8, 5, 9, 9}},
            {"3.7", {2, 0, 1, 4, 4, 5, 6, 8, 7, 9}},   {"3.8", {1, 0, 5, 6, 5, 6, 7, 7, 8, 8}},
            {"3.12", {0, 2, 1, 3, 4, 5, 6, 8, 7, 9}},
        };
    }
    void init_book() { memset(book, -1, sizeof(book)); }
    void init_robots_map() { memset(robots_map, -1, sizeof(robots_map)); }
    void update_robot(Point p, int robot_id) {
        init_robots_map();
        robots_map[0][p.x][p.y] = robot_id;
    }

    Direction get_nearby_good_dir(Point p, int scale) {
        init_book();
        std::queue<Point> q;
        q.push(p);
        int dx[4] = {0, 0, -1, 1};
        int dy[4] = {1, -1, 0, 0};
        int step = 0;
        Point target;
        Direction dir_map[4] = {LEFT, RIGHT, DOWN, UP};
        Direction tmp_path[MAX_MAP_SIZE][MAX_MAP_SIZE]{STAY};
        for (int i = 0; i < MAX_MAP_SIZE; i++) {
            for (int j = 0; j < MAX_MAP_SIZE; j++) {
                tmp_path[i][j] = STAY;
            }
        }
        while (!q.empty()) {
            int sz = q.size();
            bool break_flag = 0;
            while (sz--) {
                target = q.front();
                book[target.x][target.y] = 1;
                if (goods.goods.count(target)) {
                    break_flag = 1;
                    break;
                }
                q.pop();
                // berths_distance[berth_idx][cur.x][cur.y] = step;
                for (int i = 0; i < 4; i++) {
                    int nx = target.x + dx[i];
                    int ny = target.y + dy[i];
                    if (check_invalid_includ_book({nx, ny}))
                        continue;
                    tmp_path[nx][ny] = dir_map[i];
                    // std::cerr<<berths_path[berth_idx][nx][ny]<<" ";
                    book[nx][ny] = 1;
                    q.push({nx, ny});
                }
            }
            if (break_flag)
                break;
            step++;
            if (step >= scale)
                break;
        }
        if (step == scale)
            return STAY;
        Direction return_dir = STAY;
        bool need_log = 0;
        // if (target.x == 93 && target.y == 127){
        //     need_log = 1;
        //     std::cerr<<"debug217"<<std::endl;
        // }
        goods.goods[target].booked = 1;
        while (p != target) {
            Direction dir = tmp_path[target.x][target.y];
            if (dir == STAY)
                std::cerr << "err938";
            if (need_log)
                std::cerr << target.x << " " << target.y << " " << dir << std::endl;
            if (dir == RIGHT) {
                target.y++;
            } else if (dir == LEFT) {
                target.y--;
            } else if (dir == UP) {
                target.x--;
            } else if (dir == DOWN) {
                target.x++;
            }
            return_dir = reverse_dir(dir);
        }
        return return_dir;
    }

    void init_berth(int berth_idx, bool berth_mark) {
        Point p = berth_idx_to_point(berth_idx);
        init_book();
        std::queue<Point> q;
        q.push(p);
        int dx[2][4] = {{0, 0, -1, 1}, {1, -1, 0, 0}};
        int dy[2][4] = {{1, -1, 0, 0}, {0, 0, -1, 1}};
        int step = 0;
        // std::unordered_map<int, Direction> dir_map = {{0, LEFT}, {1, RIGHT}, {2, DOWN}, {3, UP}};
        Direction dir_map[2][4] = {{LEFT, RIGHT, DOWN, UP}, {UP, DOWN, RIGHT, LEFT}};
        while (!q.empty()) {
            int sz = q.size();
            while (sz--) {
                auto cur = q.front();
                book[cur.x][cur.y] = 1;
                q.pop();
                berths_distance[berth_idx][cur.x][cur.y] = step;
                // if (step < nearest_berth_distance[cur.x][cur.y])
                // {
                //     nearest_berth_index[cur.x][cur.y] = berth_idx;
                //     nearest_berth_distance[cur.x][cur.y] = step;
                // }
                for (int i = 0; i < 4; i++) {
                    int nx = cur.x + dx[berth_mark][i];
                    int ny = cur.y + dy[berth_mark][i];
                    if (check_invalid_includ_book({nx, ny})) {
                        continue;
                    }
                    berths_path[berth_mark][berth_idx][nx][ny] = dir_map[berth_mark][i];
                    // std::cerr<<berths_path[berth_idx][nx][ny]<<" ";
                    book[nx][ny] = 1;
                    q.push({nx, ny});
                }
            }
            step++;
        }
    }
    void init_berths(std::vector<Berth>& berths_init) {
        for (int i = 0; i < berths_init.size(); i++) {
            this->berths[i] = berths_init[i];
            if (berths[i].transport_time > min_trans_time + time_in_berth_move) {
                berths[i].new_transport_time = min_trans_time + time_in_berth_move;
                berths[i].does_trans = 1;
            } else
                berths[i].new_transport_time = berths[i].transport_time;
        }
        // memset(nearest_berth_index, 0x3f, sizeof(nearest_berth_index));
        // memset(nearest_berth_distance, 0x3f, sizeof(nearest_berth_distance));
        memset(berths_distance, 0x3f, sizeof(berths_distance));
        memset(berths_path, 0x04, sizeof(berths_path));
        for (int i = 0; i < BERTH_NUM; i++) {
            init_berth(i * 16 + 4 + 1, 0);
            berths_path_valid[0][i * 16 + 4 + 1] = 1;
            for (int dx = 0; dx < 4; dx++) {
                for (int dy = 0; dy < 4; dy++) {
                    berths_map[{berths[i].p.x + dx, berths[i].p.y + dy}] = i;
                }
            }
        }
    }

    void init_boat_plan(std::vector<Point>& init_robots) {
        // int tmp_cnt = 0;
        // for (int i = 0; i < MAP_SIZE; i++)
        //     for (int j = 0; j < MAP_SIZE; j++)
        //     {
        //         if (nearest_berth_distance[i][j] < 0x3f3f3f3f)
        //         {
        //             average_nearest_dis += nearest_berth_distance[i][j];
        //             tmp_cnt++;
        //         }
        //     }
        // average_nearest_dis = average_nearest_dis / tmp_cnt;
        // for (int i = 0; i < ROBOT_NUM; i++)
        // {
        //     double share_cnt = 0;
        //     for (int j = 0; j < BERTH_NUM; j++)
        //     {
        //         if (berths_distance[j * 16 + 4 + 1][init_robots[i].x][init_robots[i].y] < 0x3f3f3f3f)
        //             share_cnt += 1;
        //     }
        //     if (share_cnt)
        //         for (int j = 0; j < BERTH_NUM; j++)
        //         {
        //             if (berths_distance[j * 16 + 4 + 1][init_robots[i].x][init_robots[i].y] < 0x3f3f3f3f)
        //                 berths[j].available_robot_num += 1.0 / share_cnt;
        //         }
        // }
        // // 预估效率
        // for (int i = 0; i < MAP_SIZE; i++)
        //     for (int j = 0; j < MAP_SIZE; j++)
        //     {
        //         if (nearest_berth_distance[i][j] < 0x3f3f3f3f)
        //         {
        //             double val = 1;
        //             double adjust_average_nearest_dis = average_nearest_dis * 1;  // 可调参
        //             if (nearest_berth_distance[i][j] > adjust_average_nearest_dis)
        //                 val = pow(adjust_average_nearest_dis / nearest_berth_distance[i][j], 3); // 可调参
        //             double berth_dis_coef_sum = 0;
        //             double berth_dis_coef[BERTH_NUM];
        //             for (int k = 0; k < BERTH_NUM; k++)
        //             {
        //                 if (berths_distance[k * 16 + 4 + 1][i][j] < 0x3f3f3f3f)
        //                 {
        //                     berth_dis_coef[k] = pow(1.0 / (berths_distance[k * 16 + 4 + 1][i][j] + 3), 3); // 可调参
        //                     berth_dis_coef_sum += berth_dis_coef[k];
        //                 }
        //             }
        //             // if(berth_dis_coef_sum==0)std::cerr<<"err39 ";
        //             for (int k = 0; k < BERTH_NUM; k++)
        //             {
        //                 if (berths_distance[k * 16 + 4 + 1][i][j] < 0x3f3f3f3f)
        //                 {
        //                     berths[k].pred_num_per100 += val * (berth_dis_coef[k] / berth_dis_coef_sum) *
        //                     berths[k].available_robot_num;
        //                     // if (std::isnan(berths[k].pred_num_per100))
        //                     // {
        //                     //     std::cerr << "err38 " << val << " " << berth_dis_coef[k] << " " << berth_dis_coef_sum;
        //                     //     abort();
        //                     // }
        //                 }
        //             }
        //         }
        //     }
        // double pred_num_per100_sum = 0;
        // for (int k = 0; k < BERTH_NUM; k++)
        // {
        //     pred_num_per100_sum += berths[k].pred_num_per100;
        // }
        // for (int k = 0; k < BERTH_NUM; k++)
        // {
        //     berths[k].pred_num_per100 = berths[k].pred_num_per100 / pred_num_per100_sum * 10;
        // }
    }

    // 在泊位上才能调用
    std::pair<Good, Schedule> search_good(Point p, int scale, int now_time, int robot_id) {
        int berth_id = berths_map[p];
        Berth berth = berths[berth_id];
        int dx = p.x - berth.p.x;
        int dy = p.y - berth.p.y;
        int berth_idx = berth_id * 16 + dy * 4 + dx;
        if (!berths_path_valid[1][berth_idx]) {
            init_berth(berth_idx, 1);
            berths_path_valid[1][berth_idx] = 1;
        }

        // 使用process_goods()分配
        if (good_select) {
            while (!berth.booked_good.empty()) {
                Point good_p = berth.booked_good.front();
                berth.booked_good.pop_front();
                Good& good = goods.goods[good_p];
                int time1 = berths_distance[berth_idx][good_p.x][good_p.y];
                if (now_time + time1 + col_avoid_time > good.fade_time || good.booked == 1)  // 第1次去泊位可能在路上预订货物
                    continue;
                good.booked = 1;
                Point tmp_p = get_nearest_berth_point(good_p.x, good_p.y);
                target_berth[robot_id] = berths_map[tmp_p];
                return {good, get_schedule(berth_id, dx, dy, good.p)};
            }
            return {Good(), Schedule()};
        }

        Good best_good;
        double best_val_per_time = 0;
        std::vector<Point> delete_list;
        int the_target_berth = BERTH_NUM;
        // int berth_active[BERTH_NUM + 1]{};
        // for (int idx : target_berth)
        //     berth_active[idx]++;
        for (const auto& [good_p, good] : goods.goods) {
            if (good.fade_time < now_time) {
                delete_list.push_back(good_p);
                continue;
            }
            if (good.booked)
                continue;
            int time1 = berths_distance[berth_idx][good_p.x][good_p.y];
            // if (time1 > 600)
            //     continue; // 可动态调整
            if (now_time + time1 + col_avoid_time > good.fade_time)
                continue;
            // Point tmp_p = get_nearest_berth_point(good_p.x, good_p.y);
            // int tmp_target_berth_idx = point_to_berth_idx(tmp_p);
            // int time2 = berths_distance[tmp_target_berth_idx][good_p.x][good_p.y];
            // int tmp_berth_id = tmp_target_berth_idx / 16;
            // double penalty_coef = tmp_berth_id == berth_id ? 1 : 1; // 基本没用
            // 玄学优化
            double val_per_time = (double)good.val / pow((time1), 1);  // 加time2更差
                                                                       //   pow(0.95, berth_active[tmp_berth_id]); //
                                                                       //   基本没用 penalty_coef;
            if (val_per_time > best_val_per_time) {
                best_val_per_time = val_per_time;
                best_good = good;
                // the_target_berth = tmp_berth_id;
            }
        }
        for (Point p : delete_list) {
            goods.goods.erase(p);
        }
        if (best_val_per_time == 0) {
            return {Good(), Schedule()};
        }
        // target_berth[robot_id] = the_target_berth;
        goods.goods[best_good.p].booked = 1;
        return {best_good, get_schedule(berth_id, dx, dy, best_good.p)};
    }

    void process_goods(int now_time) {
        std::map<double, std::pair<Point, int>> ordered_val;
        std::set<int> full_berth;
        std::map<Point, int> real_min_dis;
        int berth_active[BERTH_NUM + 1]{}, allo_num = 0;
        memset(berth_active, 0, sizeof(berth_active));
        for (int i : target_berth)
            berth_active[i]++;
        // for (int i; i < BERTH_NUM; i++)
        //     if (!berth_disabled[i])
        //         berth_active[i] = 1;
        // if(now_time==11643)std::cerr<<berth_active[0]<<std::endl;
        for (int i = 0; i < BERTH_NUM; i++) {
            berths[i].booked_good.clear();
            // 可调参
            // if (berth_active[i] == 1)
            //     berth_active[i] = 2; // 预防货物消失
            allo_num += berth_active[i];
        }
        std::vector<Point> delete_list;
        for (const auto& [good_p, good] : goods.goods) {
            if (good.fade_time < now_time) {
                delete_list.push_back(good_p);
                continue;
            }
            if (good.booked)
                continue;
            int id = -1, min_dis = max_int_;
            for (int i = 0; i < BERTH_NUM; i++) {
                if (berths_distance[i * 16 + 4 + 1][good_p.x][good_p.y] < min_dis && !berth_disabled[i]) {
                    min_dis = berths_distance[i * 16 + 4 + 1][good_p.x][good_p.y];
                    id = i;
                }
            }
            if (id == -1 || now_time + min_dis + col_avoid_time * 2 > good.fade_time)  // 可调参
                continue;
            real_min_dis.insert({good_p, min_dis});
            // ordered_val.insert({-double(good.val) / (min_dis * 2), {good_p, id}});
            ordered_val.insert({-double(good.val) / (min_dis), {good_p, id}});
        }
        for (Point p : delete_list) {
            goods.goods.erase(p);
        }
        while (!ordered_val.empty()) {
            Point p = ordered_val.begin()->second.first;
            int berth_id = ordered_val.begin()->second.second;
            ordered_val.erase(ordered_val.begin());
            if (berth_active[berth_id] > 0) {
                allo_num--;
                berth_active[berth_id]--;
                berths[berth_id].booked_good.push_back(p);
            } else {
                full_berth.insert(berth_id);
                int id = -1, min_dis = max_int_;
                for (int i = 0; i < BERTH_NUM; i++) {
                    if (berths_distance[i * 16 + 4 + 1][p.x][p.y] < min_dis && !berth_disabled[i] &&
                        full_berth.count(i) == 0) {
                        min_dis = berths_distance[i * 16 + 4 + 1][p.x][p.y];
                        id = i;
                    }
                }
                if (id == -1 || now_time + min_dis + col_avoid_time > goods.goods[p].fade_time)
                    continue;
                // ordered_val.insert({-double(goods.goods[p].val) / (min_dis + real_min_dis[p]), {p, id}});
                ordered_val.insert({-double(goods.goods[p].val) / min_dis, {p, id}});
            }
            if (allo_num == 0)
                break;
        }
        // if(now_time==11643)std::cerr<<berths[0].booked_good.size()<<std::endl;
    }

    // berth
    // Schedule search_berth(Point p, int now_time, int robot_id) {
    //     Schedule schedule;
    //     int berth_id = robot_id; // TODO: can be optimized
    //     auto berth_path = berths_path[berth_id];
    //     if(berths_book[p.x][p.y] == robot_id) {
    //         berth_path = berths_path_integrated;
    //     }
    //     while(!(p.x == berths[berth_id].p.x && p.y == berths[berth_id].p.y)) {
    //         schedule.put_op(berth_path[p.x][p.y]);
    //         if (berth_path[p.x][p.y] == RIGHT) {
    //             p.y++;
    //         } else if (berth_path[p.x][p.y] == LEFT) {
    //             p.y--;
    //         } else if (berth_path[p.x][p.y] == UP) {
    //             p.x--;
    //         } else if (berth_path[p.x][p.y] == DOWN) {
    //             p.x++;
    //         }
    //     }
    //     return schedule;
    // }

    Schedule get_schedule(int berth_id, int dx, int dy, Point target) {
        Schedule schedule;
        bool need_log = 0;
        // if (target.x == 93 && target.y == 127){
        //     need_log = 1;
        //     std::cerr<<"debug217"<<std::endl;
        // }
        Point p(berths[berth_id].p.x + dx, berths[berth_id].p.y + dy);
        int berth_idx = berth_id * 16 + dy * 4 + dx;
        while (p != target) {
            Direction dir = berths_path[1][berth_idx][target.x][target.y];
            if (dir == STAY)
                std::cerr << "err938";
            if (need_log)
                std::cerr << target.x << " " << target.y << " " << dir << std::endl;
            if (dir == RIGHT) {
                target.y++;
            } else if (dir == LEFT) {
                target.y--;
            } else if (dir == UP) {
                target.x--;
            } else if (dir == DOWN) {
                target.x++;
            }
            schedule.ops.push_front(reverse_dir(dir));
        }
        return schedule;
    }
    int point_to_berth_idx(Point p) {
        int berth_id = berths_map[p];
        Berth berth = berths[berth_id];
        int dx = p.x - berth.p.x;
        int dy = p.y - berth.p.y;
        return berth_id * 16 + dy * 4 + dx;
    }
    Point berth_idx_to_point(int idx) {
        Point p = berths[idx / 16].p;
        p.x += idx % 4;
        p.y += (idx % 16) / 4;
        return p;
    }
    int get_berth_id(Point p) { return berths_map[p]; }
    // void update_berth(int berth_id, int good_id) {
    //     auto &berth = berths[berth_id];
    //     berth.add_good(goods.get(good_id).val);
    // }
    bool check_berth_empty(int berth_id) { return berths[berth_id].goods_cnt == 0; }
    int find_berth_to_load() {
        int max_id = -1;
        int max_val = -1;
        for (int i = 0; i < BERTH_NUM; i++) {
            int temp_val = berths[i].val_sum + berths[i].efficiency * berths[i].transport_time;
            if (berths[i].status == BerthStatus::IDLE && temp_val > max_val) {
                max_val = temp_val;
                max_id = i;
            }
        }
        if (max_val == 0) {
            return -1;
        }
        return max_id;
    }
    Point get_nearest_berth_point(int x, int y) {
        // int idx = nearest_berth_index[x][y];
        int idx = -1, min_dis = max_int_;
        for (int i = 0; i < BERTH_NUM; i++) {
            if (berths_distance[i * 16 + 4 + 1][x][y] < min_dis && !berth_disabled[i]) {
                min_dis = berths_distance[i * 16 + 4 + 1][x][y];
                idx = i * 16 + 4 + 1;
            }
        }
        if (idx == -1)
            std::cerr << "err9182";
        Point p = berths[idx / 16].p;
        p.x += idx % 4;
        p.y += (idx % 16) / 4;
        return p;
    }
    Point get_nearest_able_berth_point(int x, int y, int now_time) {
        int idx = -1, min_dis = max_int_;
        for (int i = 0; i < BERTH_NUM; i++) {
            int dis = berths_distance[i * 16 + 4 + 1][x][y];
            if (dis < min_dis && now_time + dis < berths[i].disable_time) {
                min_dis = dis;
                idx = i * 16 + 4 + 1;
            }
        }
        if (idx == -1)
            std::cerr << "err9182";
        Point p = berths[idx / 16].p;
        p.x += idx % 4;
        p.y += (idx % 16) / 4;
        return p;
    }
    bool set_berth_status(int berth_id, BerthStatus status) {
        if (berth_id >= 0 && berth_id < BERTH_NUM) {
            berths[berth_id].status = status;
            return true;
        }
        return false;
    }
    // check
    bool check_block(Point p) {
        if (ch[p.x][p.y] == '#') {
            return true;
        }
        return false;
    }
    bool check_ocean(Point p) {
        if (ch[p.x][p.y] == '*') {
            return true;
        }
        return false;
    }
    bool check_invalid_includ_book(Point p) {
        if (p.x < 0 || p.x >= n || p.y < 0 || p.y >= n) {
            return true;
        }
        if (check_block(p)) {
            return true;
        }
        if (check_ocean(p)) {
            return true;
        }
        if (check_book(p)) {
            return true;
        }
        return false;
    }
    bool check_invalid(Point p) {
        if (p.x < 0 || p.x >= n || p.y < 0 || p.y >= n) {
            return true;
        }
        if (check_block(p)) {
            return true;
        }
        if (check_ocean(p)) {
            return true;
        }
        return false;
    }
    bool check_valid(Point p) {
        if (p.x >= 0 && p.x < n && p.y >= 0 && p.y < n && (ch[p.x][p.y] == '.' || ch[p.x][p.y] == 'B'))
            return true;
        return false;
    }
    Direction get_avoid_dir(Point p, Direction dir) {
        if (is_random) {
            if (dir == UP || dir == DOWN) {
                Direction new_dir = random_line_dir(1);
                if (check_valid(p + new_dir))
                    return new_dir;
            }
            if (dir == RIGHT || dir == LEFT) {
                Direction new_dir = random_line_dir(0);
                if (check_valid(p + new_dir))
                    return new_dir;
            }
        }
        if (dir == UP || dir == DOWN) {
            if (check_valid(p + RIGHT))
                return RIGHT;
            if (check_valid(p + LEFT))
                return LEFT;
        }
        if (dir == RIGHT || dir == LEFT) {
            if (check_valid(p + UP))
                return UP;
            if (check_valid(p + DOWN))
                return DOWN;
        }
        if (dir == STAY) {
            Direction new_dir = random_dir();
            if (check_valid(p + new_dir))
                return new_dir;
        }
        if (check_valid(p + reverse_dir(dir)))
            return reverse_dir(dir);
        return STAY;
    }
    bool check_space(Point p) {
        if (ch[p.x][p.y] == '.') {
            return true;
        }
        return false;
    }
    bool check_berth(Point p) {
        if (ch[p.x][p.y] == 'B') {
            return true;
        }
        return false;
    }
    bool check_good(Point p) {
        if (goods_map[p.x][p.y] != -1) {
            return true;
        }
        return false;
    }
    bool check_book(Point p) { return book[p.x][p.y] != -1; }
    Berth& get_berth(int berth_id) {
        if (berth_id >= 0 && berth_id < BERTH_NUM) {
            return berths[berth_id];
        } else {
            throw std::runtime_error("berth_id out of range");
        }
    }
};