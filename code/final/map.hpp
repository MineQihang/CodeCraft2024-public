#pragma once
#pragma GCC optimize("O2")
#include "map_base.hpp"
#include "schedule.hpp"
#include "utils.hpp"

class Map : public BaseMap {
   public:
    std::unordered_map<Point, int> robots_map;  // 该位置机器人id
    std::unordered_map<Point, int> robots_num;  // 该位置的机器人数量
    std::vector<Point> robots_pos;              // 未满机器人的位置
    std::unordered_map<Point, int> boats_map;   // 该位置轮船id，优先为己方轮船，忽略主航道
    // 机器人的路径
    std::unordered_map<int, int> robots_path[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // 每个泊位到每个点的陆地距离
    std::vector<std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>> berths_distance;
    int nearest_berth_distance[MAX_MAP_SIZE][MAX_MAP_SIZE];  // 最近泊位的距离
    // 泊位部分点的单源最短路径
    std::unordered_map<Point, std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>> berths_distance_more;
    std::unordered_map<Point, std::array<std::array<Direction, MAX_MAP_SIZE>, MAX_MAP_SIZE>> berths_path_more;
    // 部分购买点的单源最短路径
    std::unordered_map<int, std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>> robot_purchase_distance;
    // 机器人匹配的商品id
    std::unordered_map<int, int> match_good_id;
    // 查找最短路径的方向
    std::stack<std::pair<Direction, int>> book_dirs[MAX_MAP_SIZE][MAX_MAP_SIZE];

    // 船舶路径
    // std::unordered_map<ppd, std::unordered_map<ppd, int, ppd_hash>, ppd_hash> boat_berths_dis;
    // std::unordered_map<ppd, std::unordered_map<ppd, BoatOp, ppd_hash>, ppd_hash> boat_berths_ops;
    std::unordered_map<int, std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>>
        berths_ocean_distance;  // 泊位只考虑点的单源最短路径
    std::vector<std::unordered_map<ppd, int, ppd_hash>> boat_purchase_dis;
    std::vector<std::unordered_map<ppd, BoatOp, ppd_hash>> boat_purchase_ops;
    // 交货点的单源最短路径
    // std::unordered_map<ppd, std::unordered_map<ppd, int, ppd_hash>, ppd_hash> boat_delivery_dis;
    // std::unordered_map<ppd, std::unordered_map<ppd, BoatOp, ppd_hash>, ppd_hash> boat_delivery_ops;
    std::unordered_map<int, std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>> delivery_distance;
    // 泊位的方向
    std::vector<Direction> berth_direction;
    // 泊位到达交货点的方向, 用于交货点单源最短路
    // std::map<pii, Direction> berth_to_delivery_dir;

    std::map<int, int> effictive_berth_num;       // 有效泊位
    std::set<int> working_berths;                 // 船在去和在装载的泊位，用于禁用泊位
    std::unordered_map<int, int> working_boats;   // 一段时间内有装货的船
    std::unordered_map<int, int> working_robots;  // 一段时间内有装货的机器人
    double share_efficiency = 0;                  // 禁用泊位后的效率共享
    double share_num_efficiency = 0;              // 禁用泊位后的效率共享
    std::set<int> need_dept;                      // 进行dept的泊位
    std::set<int> occupied_berths;                // 被长期占用的泊位
    std::vector<int> nearest_boat;                // 距泊位最近船
    std::vector<std::array<std::array<int, MAP_SIZE>, MAP_SIZE>> random_points_distance;  // 随机点的bfs距离
    int our_loading_boats_num = 0;                                                        // 我方装货船数量
    double our_robots_efficiency = 3.0 / 1000;             // 我方机器人效率，不要设为0，小心除0
    double our_robots_num_efficiency = 3.0 * 1500 / 1000;  // 我方机器人数量效率
    int our_robots_goods_val_1000 = 0;                     // 我方机器人1000帧产生的价值
    int our_robots_goods_num_1000 = 0;                     // 我方机器人1000帧产生的数量
    int our_robots_goods_val = 0;                          // 我方机器人持有货物的价值
    int our_robots_goods_num = 0;                          // 我方机器人持有货物的数量

    // 船舶区域划分
    std::map<int, std::vector<int>> zone_to_boat_purchase_points;  // key: 有效区域id, value: 船只购买点id
    std::map<int, int> boat_purchase_point_to_zone;                // key: 船只购买点id, value: 区域id
    std::map<int, std::vector<int>> ocean_zone_to_berths;
    std::map<int, int> berth_to_ocean_zone;
    std::map<int, std::vector<int>> zone_to_delivery_points;
    std::map<int, int> delivery_point_to_zone;
    std::map<int, int> ocean_zone_boat_num;  // 每个区域的船只数量
    std::map<int, int> ocean_zone_boat_num_now;

    // 机器人区域划分
    std::map<int, std::set<int>> zone_to_robot_purchase_points;  // 区域id是区域最小机器人购买点id
    std::map<int, int> robot_purchase_point_to_zone;
    std::map<int, std::vector<int>> land_zone_to_berths;
    std::map<int, int> berth_to_land_zone;
    std::map<int, int> land_zone_space;  // 每个有效(有购买点和泊位)区域空地数量
    double average_nearest_dis{};
    int land_zone[MAX_MAP_SIZE][MAX_MAP_SIZE];  // 区域id
    std::map<int, int> land_zone_robots_num;    // 每个区域的机器人数量
    std::map<int, int> land_zone_robots_num_now;

    int strict_zone[MAP_SIZE][MAP_SIZE];  // 防堵路区域id

    std::map<int, std::queue<int>> robot_purchase_ids;  // 机器人购买顺序
    std::map<int, std::queue<int>> boat_purchase_ids;   // 轮船购买顺序

    std::unordered_set<int> col_robots;  // 碰撞的机器人
    std::unordered_set<int> col_boats;   // 碰撞的货物

    // Map() {}
    Map(int n) : BaseMap(n) {
        // 初始化各种参数
        init_book();
        init_robots_path();
    }
    Map(const Map&) = delete;
    Map& operator=(const Map&) = delete;

    void init_robots_path() {
        // memset(robots_path, -1, sizeof(robots_path));
    }
    void init_book_dirs() {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                while (!book_dirs[i][j].empty())
                    book_dirs[i][j].pop();
            }
        }
    }

    // 初始的时候进行预处理
    void preprocess() {
        preprocess_();
        // 手动尽快购买
        if (manual_quickly_buy && manual_quickly_buy_lib.count(map_name) > 0) {
            quickly_buy = manual_quickly_buy_lib[map_name];
        }
        if (manual_stay_until && stay_until_plan.count(map_name) > 0) {
            stay_until = stay_until_plan[map_name];
        }
        robot_price = ROBOT_PRICE[robot_type];

        memset(nearest_berth_distance, 0x3f, sizeof(nearest_berth_distance));
        memset(land_zone, -1, sizeof(land_zone));
        memset(strict_zone, -1, sizeof(strict_zone));

        // 初始化berth
        init_berths();
        // std::ofstream outputFile("output.csv");
        // if (!outputFile.is_open()) {
        //     std::cerr << "无法打开文件" << std::endl;
        // }
        // for (const auto& row : berths_ocean_distance.at(0)) {
        //     for (size_t i = 0; i < row.size(); ++i) {
        //         outputFile << row[i];
        //         if (i != row.size() - 1) {
        //             outputFile << ",";
        //         }
        //     }
        //     outputFile << "\n";
        // }
        // outputFile.close();
        if (global_debug)
            timer.print_duration("to init berths");

        // 初始化各区域机器人购买点到其他点
        for (int i = 0; i < (int)robot_purchase_points.size(); i++) {
            Point p = robot_purchase_points[i];
            // 只遍历没去过的区域
            if (land_zone[p.x][p.y] != -1)
                continue;
            std::queue<Point> q;
            init_book();
            q.push(p);
            book[p.x][p.y] = 1;
            int step = 0;
            while (!q.empty()) {
                int sz = q.size();
                while (sz--) {
                    Point cur = q.front();
                    robot_purchase_distance[i][cur.x][cur.y] = step;
                    q.pop();
                    land_zone[cur.x][cur.y] = i;
                    if (check_robot_purchase_point(cur)) {
                        int cur_id = robot_purchase_points_map[cur];
                        zone_to_robot_purchase_points[i].insert(cur_id);
                        robot_purchase_point_to_zone[cur_id] = i;

                    } else if (check_berth(cur)) {
                        int cur_id = berths_map[cur];
                        land_zone_to_berths[i].push_back(cur_id);
                        berth_to_land_zone[cur_id] = i;
                    }
                    for (int i = 0; i < 4; i++) {
                        Point nxt = cur + Direction(i);
                        if (check_land_invalid(nxt) || check_book(nxt))
                            continue;
                        book[nxt.x][nxt.y] = 1;
                        q.push(nxt);
                    }
                }
                step++;
            }
        }

        // 初始化随机点的bfs
        random_points_distance.resize(15);
        for (auto& dis : random_points_distance) {
            for (auto& dis2 : dis) {
                for (auto& dis3 : dis2) {
                    dis3 = -1;
                }
            }
        }
        for (auto& random_point_distance : random_points_distance) {
            Point p = {random_int(0, MAP_SIZE - 1), random_int(0, MAP_SIZE - 1)};
            while (check_land_invalid(p) || land_zone[p.x][p.y] == -1) {
                p = {random_int(0, MAP_SIZE - 1), random_int(0, MAP_SIZE - 1)};
            }
            init_book();
            std::queue<Point> q;
            q.push({p.x, p.y});
            book[p.x][p.y] = 1;
            int step = 0;
            while (!q.empty()) {
                int sz = q.size();
                while (sz--) {
                    auto cur = q.front();
                    random_point_distance[cur.x][cur.y] = step;
                    q.pop();
                    for (int i = 0; i < 4; i++) {
                        auto nxt = cur + Direction(i);
                        if (check_land_invalid(nxt) || check_book(nxt))
                            continue;
                        book[nxt.x][nxt.y] = 1;
                        q.push(nxt);
                    }
                }
                step++;
            }
        }

        // timer.print_duration("to init robot_purchase_points", global_debug);
        // 初始化船从交货点到其他地方的bfs距离
        for (int i = 0; i < (int)delivery_points.size(); i++) {
            ocean_bfs(delivery_points[i], delivery_distance[i]);
        }
        // std::cerr << delivery_distance[5][95][213] << " " << delivery_distance[5][95][214] << std::endl;
        // std::ofstream outputFile("output.csv");
        // if (!outputFile.is_open()) {
        //     std::cerr << "无法打开文件" << std::endl;
        // }
        // for (const auto& row : delivery_distance[5]) {
        //     for (size_t i = 0; i < row.size(); ++i) {
        //         outputFile << row[i];
        //         if (i != row.size() - 1) {
        //             outputFile << ",";
        //         }
        //     }
        //     outputFile << "\n";
        // }
        // outputFile.close();
        // timer.print_duration("to init delivery_points", global_debug);

        // 初始化船从购买点到其他地方的距离和路径
        boat_purchase_dis.resize(boat_purchase_points.size());
        boat_purchase_ops.resize(boat_purchase_points.size());
        for (int i = 0; i < (int)boat_purchase_points.size(); i++) {
            ppd from{boat_purchase_points[i], Direction::RIGHT};
            get_boat_distance(boat_purchase_dis[i], boat_purchase_ops[i], from, false);
        }
        timer.print_duration("to init boat_purchase_points", global_debug);

        // 找最近的交货点
        for (int i = 0; i < (int)berths.size(); i++) {
            for (int j = 0; j < (int)delivery_points.size(); j++) {
                // 不联通的交货点不考虑
                if (berth_to_ocean_zone.count(i) == 0 || delivery_point_to_zone.count(j) == 0 ||
                    berth_to_ocean_zone[i] != delivery_point_to_zone[j])
                    continue;
                int dis_tmp = berths_ocean_distance[i][delivery_points[j].x][delivery_points[j].y];
                if (dis_tmp < berths[i].nearest_delivery_dis) {
                    berths[i].nearest_delivery_id = j;
                    berths[i].nearest_delivery_dis = dis_tmp;
                }
            }
        }

        // 初始化加权帧数
        for (int i = 1; i <= FRAME_NUM; i++) {
            weighted_frames[i] = weighted_frames[i - 1] + 1.0 / (FRAME_NUM * 2 - i + 150);
        }
        // 删除没有泊位或者没有购买点的陆地区域
        std::vector<int> delete_land_zone_list;
        for (auto& i : zone_to_robot_purchase_points) {
            if (land_zone_to_berths[i.first].size() == 0) {
                delete_land_zone_list.push_back(i.first);
            } else if (zone_to_robot_purchase_points[i.first].size() == 0) {
                delete_land_zone_list.push_back(i.first);
            }
        }
        for (auto& i : delete_land_zone_list) {
            for (auto& j : zone_to_robot_purchase_points[i]) {
                robot_purchase_point_to_zone.erase(j);
            }
            for (int& berth_id : land_zone_to_berths[i]) {
                berth_to_land_zone.erase(berth_id);
                berths[berth_id].set_disabled();
            }
            zone_to_robot_purchase_points.erase(i);
            land_zone_to_berths.erase(i);
        }
        // 效率预估与机器人购买
        init_effitiency_predict();
        // std::cerr << "init_effitiency_predict done" << std::endl;
        // 删除没有交货点或者泊位价值不足的海洋区域
        std::vector<int> delete_ocean_zone_list;
        for (auto& i : zone_to_boat_purchase_points) {
            if (zone_to_delivery_points[i.first].size() == 0) {
                delete_ocean_zone_list.push_back(i.first);
            } else {
                // double val_pred = 0;
                // int robots_num = 0;
                // for (int& berth_id : ocean_zone_to_berths[i.first]) {
                //     val_pred += berths[berth_id].pred_num_per100 * 150 * mean_good_val_preset;
                //     robots_num += berths[berth_id].available_robot_num;
                // }
                // if (val_pred < BOAT_PRICE[0] + robots_num * robot_price) {
                //     delete_ocean_zone_list.push_back(i.first);
                // }
            }
        }
        for (auto& i : delete_ocean_zone_list) {
            for (auto& j : zone_to_boat_purchase_points[i]) {
                boat_purchase_point_to_zone.erase(j);
            }
            for (int& berth_id : ocean_zone_to_berths[i]) {
                berth_to_ocean_zone.erase(berth_id);
                berths[berth_id].set_disabled();
            }
            zone_to_boat_purchase_points.erase(i);
            ocean_zone_to_berths.erase(i);
        }
        // 船舶购买决策
        for (auto& i : zone_to_boat_purchase_points) {
            ocean_zone_boat_num[i.first] = init_boat_num;  // 待分区
            ocean_zone_boat_num_now[i.first] = 0;
        }
        // std::cerr << "zone_to_boat_purchase_points.size(): " << zone_to_boat_purchase_points.size() << std::endl;
        // if (zone_to_boat_purchase_points.size() != 1)
        //     abort();
        if (manual_boat_num && zone_to_boat_purchase_points.size() == 1 && manual_boat_num_plan.count(map_name) > 0) {
            ocean_zone_boat_num.begin()->second = manual_boat_num_plan[map_name];
        }
        // 船舶数据统计
        // int delivery_dis_sum_one = 0;
        // for (auto& i : ocean_zone_to_berths) {
        //     int delivery_dis_sum = 0;
        //     for (int& berth_id : i.second) {
        //         if (!berths[berth_id].is_disabled()) {
        //             effictive_berth_num[i.first]++;
        //             delivery_dis_sum += berths[berth_id].nearest_delivery_dis;
        //         }
        //     }
        //     average_berth_dis[i.first] = delivery_dis_sum / effictive_berth_num[i.first];
        //     delivery_dis_sum_one += delivery_dis_sum;
        //     // double boats_ability = 1000.0 / average_berth_dis[i.first];
        //     std::cerr << i.first << " average_berth_dis: " << average_berth_dis[i.first] << std::endl;
        // }
        // int effictive_berth_sum = 0;
        // for (auto& i : effictive_berth_num) {
        //     effictive_berth_sum += i.second;
        // }
        // average_berth_dis_one = delivery_dis_sum_one / effictive_berth_sum;
        // 删除没有有效泊位的陆地区域
        // delete_land_zone_list.clear();
        // for (auto& i : zone_to_robot_purchase_points) {
        //     bool flag = false;
        //     for (int& berth_id : land_zone_to_berths[i.first]) {
        //         if (berth_to_ocean_zone.count(berth_id) > 0) {
        //             flag = true;
        //             break;
        //         }
        //     }
        //     if (!flag)
        //         delete_land_zone_list.push_back(i.first);
        // }
        // for (int& i : delete_land_zone_list) {
        //     std::cerr << "delete_land_zone_list: " << i << std::endl;
        //     for (int j : zone_to_robot_purchase_points[i]) {
        //         robot_purchase_point_to_zone.erase(j);
        //     }
        //     for (int& berth_id : land_zone_to_berths[i]) {
        //         berth_to_land_zone.erase(berth_id);
        //         berths[berth_id].set_disabled();
        //     }
        //     zone_to_robot_purchase_points.erase(i);
        //     land_zone_to_berths.erase(i);
        //     land_zone_robots_num.erase(i);
        // }
        // 机器人购买手动设置
        if (manual_robot_num && map_name != "") {
            if (land_zone_robots_num.size() == 0)
                std::cerr << "land_zone_robots_num.size() == 0" << std::endl;
            else if (land_zone_robots_num.size() == 1 && robot_max_num_lib.count(map_name) > 0)
                land_zone_robots_num.begin()->second = robot_init_num;
            else if (land_zone_robot_num_lib.count(map_name) > 0) {
                land_zone_robots_num = land_zone_robot_num_lib[map_name];
            }
        }
        // 手动轮船购买顺序
        if (manual_buy_boat_order && buy_boat_order_plan.count(map_name) > 0) {
            int cnt = 50;
            while (cnt > 0) {
                for (auto i : buy_boat_order_plan[map_name]) {
                    boat_purchase_ids[0].push(i);  // 待分区
                }
                cnt -= buy_boat_order_plan[map_name].size();
            }
        }
        if (dulu)
            init_boat_num = 1;
        // std::cerr << ": robot_max_num" << robot_max_num << std::endl;
        // timer.print_duration("to init land_zone", global_debug);
    }

    void init_berths() {
        // 初始化距离
        berths_distance.resize(berths.size());
        for (auto& dis : berths_distance) {
            for (auto& dis2 : dis) {
                for (auto& dis3 : dis2) {
                    dis3 = -1;
                }
            }
        }
        nearest_boat.resize(berths.size());
        // 初始化方向
        berth_direction.resize(berths.size());
        // 初始化每个泊位路径长度+标记
        for (int i = 0; i < (int)berths.size(); i++) {
            init_berth(i);
        }
        // 是否dept
        for (int i = 0; i < (int)berths.size(); i++) {
            if (berths[i].boat_ppd.second != berth_direction[i]) {
                need_dept.insert(i);
                // std::cerr << "need_dept: " << i << std::endl;
            }
        }
    }

    // 初始化单个泊位
    void init_berth(int berth_id) {
        auto& berth = berths[berth_id];
        // 标记泊位所在区域
        std::queue<Point> q;
        init_book();
        q.push({berth.p.x, berth.p.y});
        book[berth.p.x][berth.p.y] = 1;
        Point left_top = {n, n}, right_bottom = {0, 0};
        while (!q.empty()) {
            auto cur = q.front();
            q.pop();
            if (check_berth(cur)) {
                berths_map[cur] = berth_id;
                left_top.x = std::min(left_top.x, cur.x);
                left_top.y = std::min(left_top.y, cur.y);
                right_bottom.x = std::max(right_bottom.x, cur.x);
                right_bottom.y = std::max(right_bottom.y, cur.y);
            } else if (check_near_berth_area(cur)) {
                berth_near_area[cur] = berth_id;
            } else {
                continue;
            }
            for (int i = 0; i < 4; i++) {
                auto nxt = cur + Direction(i);
                if (check_ocean_invalid(nxt) || check_book(nxt))
                    continue;
                book[nxt.x][nxt.y] = 1;
                q.push(nxt);
            }
        }
        // std::cerr << "end" << std::endl;

        // 标记泊位的方向
        if (right_bottom.y - left_top.y + 1 > right_bottom.x - left_top.x + 1) {
            // 左上角
            if (berth.p == left_top) {
                berth_direction[berth_id] = Direction::RIGHT;
                // 右下角
            } else {
                berth_direction[berth_id] = Direction::LEFT;
            }
        } else {
            // 右上角
            if (berth.p == Point(left_top.x, right_bottom.y)) {
                berth_direction[berth_id] = Direction::DOWN;
                // 左下角
            } else {
                berth_direction[berth_id] = Direction::UP;
            }
        }
        // std::cerr << "end2" << std::endl;

        // 初始化机器人距离，注意这里从泊位的核心点开始
        auto& berth_distance = berths_distance[berth_id];
        init_book();
        q.push({berth.p.x, berth.p.y});
        book[berth.p.x][berth.p.y] = 1;
        int step = 0;
        while (!q.empty()) {
            int sz = q.size();
            // std::cerr << step << " " << sz << std::endl;
            while (sz--) {
                // std::cerr <<sz << std::endl;
                auto cur = q.front();
                // std::cerr << "here2" << std::endl;
                berth_distance[cur.x][cur.y] = step;
                // std::cerr << "here3" << std::endl;
                q.pop();
                if (step < nearest_berth_distance[cur.x][cur.y]) {
                    nearest_berth_distance[cur.x][cur.y] = step;
                }
                for (int i = 0; i < 4; i++) {
                    auto nxt = cur + Direction(i);
                    if (check_land_invalid(nxt) || check_book(nxt))
                        continue;
                    book[nxt.x][nxt.y] = 1;
                    q.push(nxt);
                }
            }
            step++;
        }

        // 船在泊位的方向，dept后，可泛化
        Direction dir = STAY;
        for (int i = 0; i < 4; i++) {
            if (check_boat_one_invalid({berth.p, Direction(i)}))
                continue;
            if (check_boat_all_main({berth.p, Direction(i)})) {
                dir = Direction(i);
                break;
            }
        }
        if (dir == STAY) {
            std::cerr << "init berth " << berth_id << " error!" << std::endl;
            return;
        }
        auto from = ppd(berth.p, dir);
        berth.boat_ppd = from;
        if (check_boat_one_invalid(from))
            return;

        // 海上bfs
        ocean_bfs(berth.p, berths_ocean_distance[berth_id]);
    }

    // 船的单源最短路(Dijkstra)
    ppd get_boat_distance(std::unordered_map<ppd, int, ppd_hash>& dis,     // 距离
                          std::unordered_map<ppd, BoatOp, ppd_hash>& ops,  // 操作
                          ppd from,                                        // from
                          bool is_berth_init = false,                      // 是否对全部的点进行搜索
                          ppd to = {{-1, -1}, STAY},                       // to
                          int berth_id = -1) {
        // std::priority_queue<pip, std::vector<pip>, ComparePip> pq;
        std::set<pip, std::less<pip>> st;
        // 区域划分：未考虑A能去B，B不能去A的情况
        bool new_zone = 0;
        int zone_id = -1;
        if (is_boat_purchase_point(from.first)) {
            if (boat_purchase_point_to_zone.count(get_boat_purchase_point_id(from.first)) == 0) {
                new_zone = true;
                zone_id = get_boat_purchase_point_id(from.first);  // 区域id是船只购买点id
            } else
                return ppd{{-2, -2}, STAY};
        }
        st.emplace(0, from);
        while (!st.empty()) {
            // auto [step, node] = pq.top();  // node is ppd
            // pq.pop();
            auto [step, node] = *st.begin();
            st.erase(st.begin());
            if (!is_berth_init) {
                if (to.second == STAY) {
                    for (int i = 0; i < 4; i++) {
                        if (dis.count({to.first, Direction(i)})) {
                            return {to.first, Direction(i)};
                        }
                    }
                } else {
                    if (dis.count(to)) {
                        return to;
                    }
                }
            } else {
            }
            if (new_zone) {
                if (is_boat_purchase_point(node.first) && node.second == RIGHT) {
                    int id = boat_purchase_points_map[node.first];
                    if (boat_purchase_point_to_zone.count(id) == 0) {
                        boat_purchase_point_to_zone[id] = zone_id;
                        zone_to_boat_purchase_points[zone_id].push_back(id);
                    }
                } else if (is_delivery_point(node.first)) {
                    int id = delivery_points_map[node.first];
                    if (delivery_point_to_zone.count(id) == 0) {
                        delivery_point_to_zone[id] = zone_id;
                        zone_to_delivery_points[zone_id].push_back(id);
                    }
                } else if (berths_map.count(node.first)) {
                    int id = berths_map[node.first];
                    if (berth_to_ocean_zone.count(id) == 0) {
                        berth_to_ocean_zone[id] = zone_id;
                        ocean_zone_to_berths[zone_id].push_back(id);
                    }
                }
            }
            for (int i = 0; i < 3; i++) {
                auto op = BoatOp(i);
                auto nxt = next_ppd(node, op);
                if (check_boat_op_invalid(nxt))
                    continue;
                int nxt_step = step + 1;
                if (check_boat_one_main(nxt))
                    nxt_step++;
                if (!dis.count(nxt) || dis[nxt] > nxt_step) {
                    // if (dis.count(nxt))
                    //     st.erase(st.find({dis[nxt], nxt}));
                    dis[nxt] = nxt_step;
                    ops[nxt] = op;
                    st.emplace(nxt_step, nxt);
                }
            }
        }
        return to;
    }

    void init_effitiency_predict() {
        if (manual_init_efficiency)
            for (int i = 0; i < int(berths.size()); i++) {
                berths[i].weighted_efficiency = init_effitiency[i];
                berths[i].weighted_num_efficiency = init_num_effitiency[i];
            }
        int tmp_cnt = 0;
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++) {
                if (!is_berth({i, j}) && !is_robot_purchase_point({i, j}) && land_zone[i][j] != -1) {
                    if (zone_to_boat_purchase_points.count(land_zone[i][j]) != 0) {
                        land_zone_space[land_zone[i][j]]++;
                        if (global_debug && nearest_berth_distance[i][j] >= 0x3f3f3f3f)
                            std::cerr << "err375 " << i << " " << j << std::endl;
                        average_nearest_dis += nearest_berth_distance[i][j];
                        tmp_cnt++;
                    }
                }
            }
        average_nearest_dis = average_nearest_dis / tmp_cnt;
        // 机器人如何购买
        int all_space = 0, now_robot_num = 0;
        for (auto& i : land_zone_space) {
            all_space += i.second;
        }
        for (auto it = land_zone_space.begin(); it != land_zone_space.end(); it++) {
            // std::cerr << "land_zone_space: " << it->first << " " << it->second << std::endl;
            double ratio = double(it->second) / all_space;
            land_zone_robots_num[it->first] = round(robot_init_num * ratio);
            now_robot_num += land_zone_robots_num[it->first];
            if (land_zone_robots_num[it->first] == 0 && ratio > 3000.0 / 170000.0 && now_robot_num < 1.5 * robot_max_num) {
                land_zone_robots_num[it->first] = 1;
                now_robot_num++;
            }
        }
        // robot_max_num = now_robot_num;
        // 预估效率
        for (auto& i : land_zone_to_berths) {
            for (int& berth_id : i.second) {
                berths[berth_id].available_robot_num = double(land_zone_robots_num[i.first]) / i.second.size();
            }
        }
        // for (int i = 0; i < MAP_SIZE; i++)
        //     for (int j = 0; j < MAP_SIZE; j++) {
        //         if (land_zone_space.count(land_zone[i][j]) != 0) {
        //             int zone_id = land_zone[i][j];
        //             double val = 1;
        //             double adjust_average_nearest_dis = average_nearest_dis * 1;  // 可调参
        //             if (nearest_berth_distance[i][j] > adjust_average_nearest_dis)
        //                 val = pow(adjust_average_nearest_dis / nearest_berth_distance[i][j], 3);  // 可调参
        //             double berth_dis_coef_sum = 0;
        //             std::map<int, double> berth_dis_coef;
        //             for (int& berth_id : land_zone_to_berths[zone_id]) {
        //                 berth_dis_coef[berth_id] = pow(1.0 / (berths_distance[berth_id][i][j] + 3), 3);  // 可调参
        //                 if (global_debug && berths_distance[berth_id][i][j] == -1)
        //                     std::cerr << "err376 " << i << " " << j << " " << berth_id << std::endl;
        //                 berth_dis_coef_sum += berth_dis_coef[berth_id];
        //             }
        //             for (int& berth_id : land_zone_to_berths[zone_id]) {
        //                 berths[berth_id].pred_num_per100 +=
        //                     val * (berth_dis_coef[berth_id] / berth_dis_coef_sum) * berths[berth_id].available_robot_num;
        //                 if (global_debug && std::isnan(berths[berth_id].pred_num_per100))
        //                     std::cerr << "err388 " << val << " " << berth_dis_coef[berth_id] << " " << berth_dis_coef_sum;
        //             }
        //         }
        //     }
        // timer.print_duration("init_effitiency_predict high complexity done");
        // std::cerr << "init_effitiency_predict high complexity done: " << timer.get_duration(init_start)<<"ms" <<
        // std::endl; double pred_num_per100_sum = 0; for (int k = 0; k < berth_num; k++) {
        //     pred_num_per100_sum += berths[k].pred_num_per100;
        // }
        // double num_per100_coef = 10.5;  // 应为pulled_goods_num / 150
        // for (int k = 0; k < berth_num; k++) {
        //     berths[k].pred_num_per100 = berths[k].pred_num_per100 / pred_num_per100_sum * num_per100_coef;
        // }
    }

    // ===========================================更新处理=======================================================

    // 每帧的更新处理
    void update_patch() {
        for (auto& [patch_id, patch] : map_patches) {
            if (patch.high_goods_num < 0) {
                patch.high_goods_num = 0;
                std::cerr << "patch.high_goods_num < 0" << std::endl;
            }
            patch.clear();
        }
    }

    void update() {
        col_robots.clear();
        col_boats.clear();
        // 删除不活跃的船舶
        if (!working_boats.empty()) {
            std::vector<int> delete_boats;
            for (auto& i : working_boats) {
                if (i.second - frame_id > 3000) {
                    delete_boats.push_back(i.first);
                }
            }
            for (int i : delete_boats) {
                working_boats.erase(i);
            }
        }
        // 删除不活跃的机器人
        if (!working_robots.empty()) {
            std::vector<int> delete_robots;
            for (auto& i : working_robots) {
                if (i.second - frame_id > 800) {
                    delete_robots.push_back(i.first);
                }
            }
            for (int i : delete_robots) {
                working_robots.erase(i);
            }
        }
        // 计算泊位效率
        for (auto& berth : berths) {
            berth.update_efficiency(share_efficiency, share_num_efficiency, working_berths.size());
        }
    }

    // ============================================机器人=======================================================

    // 找patch
    Schedule search_patch(Point p) {
        // timer.start();
        // if (patch_bfs_cnt >= patch_bfs_cnt_max)
        //     return Schedule();
        // patch_bfs_cnt++;
        auto& cur_patch = get_patch(p);
        std::vector<int> dis(map_patches.size(), -1);
        std::vector<int> prv(map_patches.size(), -1);
        // 从图中一个节点往外搜索，找到最大的potential
        double best_val{0};
        int best_patch_id{-1};
        std::queue<int> q;
        q.push(cur_patch.patch_id);
        dis[cur_patch.patch_id] = 0;
        while (!q.empty()) {
            auto cur = q.front();
            q.pop();
            if (map_patches.count(cur) == 0) {
                throw std::runtime_error("map_patches.count(cur) == 0");
            }
            auto potential = map_patches[cur].get_potential() / (dis[cur] + 1);
            if (potential > best_val) {
                best_val = potential;
                best_patch_id = cur;
            }
            for (auto nxt : patches_edge[cur]) {
                if (dis[nxt] == -1) {
                    dis[nxt] = dis[cur] + 1;
                    prv[nxt] = cur;
                    q.push(nxt);
                }
            }
        }
        if (best_patch_id == -1) {
            return Schedule();
        }
        if (best_patch_id == cur_patch.patch_id) {
            Schedule schedule;
            int temp_n = random_int(1, 5);
            for(int i = 0; i < temp_n; i++) {
                schedule.ops.push_back(STAY);
            }
            schedule.patch_id = best_patch_id;
            return schedule;
        }
        // 回溯找到下一个要去的patch
        int temp = best_patch_id;
        while (prv[best_patch_id] != cur_patch.patch_id) {
            best_patch_id = prv[best_patch_id];
        }
        auto schedule = get_schedule_by_a_star(p, map_patches[best_patch_id].center);
        schedule.patch_id = temp;
        // timer.print_duration_with_reset("search_patch", global_debug, LOG_TIME_THRESHOLD * 0.4);
        return schedule;
    }

    // 从某个点开始查找货物 (KEY FUNCTION)
    Schedule search_good(Point p) {
        timer.start();
        // if (patch_bfs_cnt >= patch_bfs_cnt_max)
        //     return Schedule();
        // patch_bfs_cnt++;
        double max_val = 0;
        int best_good_id = -1;
        std::queue<Point> q;
        init_book();
        q.push(p);
        book[p.x][p.y] = STAY;
        int step = 0;
        // bfs
        while (!q.empty()) {
            int sz = q.size();
            while (sz--) {
                auto cur = q.front();
                q.pop();
                if (check_good(cur)) {
                    auto& good = get_good(goods_map[cur.x][cur.y]);
                    if (good.is_exist() && good.val > GOOD_VAL_SPLIT && !check_good_col(cur)) {
                        if (frame_id + step + SAFE_TIME <= good.fade_time) {
                            auto val = double(good.val) / (step + dis_plus);
                            if (val > max_val) {
                                max_val = val;
                                best_good_id = good.id;
                            }
                        }
                    }
                }
                for (int i = 0; i <= 3; i++) {
                    auto nxt = cur + Direction(i);
                    if (check_land_invalid(nxt) || check_book(nxt))
                        continue;
                    book[nxt.x][nxt.y] = Direction(i);
                    q.push(nxt);
                }
            }
            step++;
            // 小范围搜索
            if (step >= search_range)
                break;
        }
        if (best_good_id == -1) {
            return Schedule();
        }
        // 回溯
        Schedule schedule;
        schedule.set_target_good(best_good_id);
        auto cur = goods[best_good_id].p;
        while (!(cur == p)) {
            auto op = Direction(book[cur.x][cur.y]);
            cur += reverse_dir(op);
            schedule.ops.push_front(op);
        }
        timer.print_duration_with_reset("search_good", global_debug, LOG_TIME_THRESHOLD * 0.4);
        return schedule;
    }

    // 找最近的泊位
    int search_berth(Point p, int robot_id, bool is_expensive_good = false) {
        int best_berth_id = -1;
        double min_dis = max_int_;
        for (int i = 0; i < (int)berths.size(); i++) {
            if (berths[i].is_disabled())
                continue;
            auto& berth_distance = berths_distance[i];
            if (berth_distance[p.x][p.y] == -1)
                continue;
            auto& berth = get_berth(i);
            auto tmp_dis = 0;
            if (is_expensive_good) {
                if (berth.is_ours_boat)
                    tmp_dis = -700;  // 数值的影响不大
                else if (boats_going_berths.count(berth.id) > 0)
                    tmp_dis = -250;
            } else {
                if (berth.is_ours_boat)
                    tmp_dis = -0;
            }
            tmp_dis += (berth_distance[p.x][p.y] /** berths[i].nearest_delivery_dis*/);
            if (tmp_dis < min_dis) {
                min_dis = tmp_dis;
                best_berth_id = i;
            }
        }
        if (best_berth_id == -1)
            std::cerr << frame_id << ": search_berth error " << p.x << " " << p.y << std::endl;
        return best_berth_id;
    }

    // 通过已有最短路获取路线
    Schedule get_schedule(Point from, Point to) {
        Schedule schedule;
        auto cur = to;
        while (!(cur == from)) {
            auto op = berths_path_more[from][cur.x][cur.y];
            cur += Direction(op);
            schedule.ops.push_front(reverse_dir(op));
        }
        return schedule;
    }

    // 使用A*搜索从from点到to点的最短路径
    Schedule get_schedule_by_a_star(Point from, Point to) {
        init_book();
        std::priority_queue<std::pair<int, std::pair<int, Point>>, std::vector<std::pair<int, std::pair<int, Point>>>,
                            CompareInt>
            pq;
        pq.push({0, {0, from}});
        book[from.x][from.y] = Direction::STAY;
        int step = 0;
        while (!pq.empty()) {
            auto [_, step_point] = pq.top();
            pq.pop();
            auto [step, cur] = step_point;
            step++;
            if (check_book(to)) {
                break;
            }
            for (int i = 0; i < 4; i++) {
                auto nxt = cur + Direction(i);
                if (check_land_invalid(nxt) || check_book(nxt))
                    continue;
                book[nxt.x][nxt.y] = Direction(i);
                pq.push({step + l1_dis(nxt, to), {step, nxt}});
            }
        }
        if (!check_book(to)) {
            std::cerr << frame_id << ": get_schedule_by_a_star error " << from.x << " " << from.y << " " << to.x << " "
                      << to.y << std::endl;
            return Schedule();
        }
        // 回溯
        Schedule schedule;
        auto cur = to;
        while (!(cur == from)) {
            auto op = Direction(book[cur.x][cur.y]);
            schedule.ops.push_front(op);
            cur += reverse_dir(op);
        }
        return schedule;
    }

    // 使用BFS搜索从from点到to点的最短路径，避让时使用，会考虑其他机器人的位置
    Schedule get_schedule_by_bfs(Point from, Point to) {
        init_book();
        std::queue<Point> q;
        q.push(from);
        book[from.x][from.y] = Direction::STAY;
        int step = 0;
        while (!q.empty()) {
            int sz = q.size();
            while (sz--) {
                auto cur = q.front();
                q.pop();
                for (int i = 0; i < 4; i++) {
                    auto nxt = cur + Direction(i);
                    if (check_land_invalid(nxt) || check_book(nxt) ||
                        (land_main[nxt.x][nxt.y] == 0 && (robots_map.count(nxt) || check_main_col(nxt))))
                        continue;
                    book[nxt.x][nxt.y] = Direction(i);
                    q.push(nxt);
                }
            }
            step++;
            if (check_book(to)) {
                break;
            }
            if (step >= robot_col_search_range * 4) {
                break;
            }
        }
        if (!check_book(to)) {
            return Schedule();
        }
        // 回溯
        Schedule schedule;
        auto cur = to;
        while (!(cur == from)) {
            auto op = Direction(book[cur.x][cur.y]);
            schedule.ops.push_front(op);
            cur += reverse_dir(op);
        }
        return schedule;
    }

    // 获取去泊位的下一个方向
    Direction get_robot_next_dir(Point cur, int goto_berth_id, bool consider_other_robots = false) {
        int min_dis = consider_other_robots ? max_int_ : berths_distance[goto_berth_id][cur.x][cur.y];
        Direction best_dir = STAY;
        for (int j = 0; j < 4; j++) {
            auto nxt = cur + Direction(j);
            if (check_land_invalid(nxt))
                continue;
            if (consider_other_robots && robots_map.count({nxt.x, nxt.y}) && land_main[nxt.x][nxt.y] == 0)
                continue;
            if (berths_distance[goto_berth_id][nxt.x][nxt.y] < min_dis) {
                if (!consider_other_robots)
                    return Direction(j);
                min_dis = berths_distance[goto_berth_id][nxt.x][nxt.y];
                best_dir = Direction(j);
            }
        }
        return best_dir;
    }
    // 机器人第一次到达泊位某点时调用
    void bfs_berth_distance(Point p) {
        std::queue<Point> q;
        init_book();
        q.push({p.x, p.y});
        book[p.x][p.y] = 1;
        int step = 0;
        while (!q.empty()) {
            int sz = q.size();
            while (sz--) {
                auto cur = q.front();
                berths_distance_more[p][cur.x][cur.y] = step;
                q.pop();
                for (int i = 0; i <= 3; i++) {  // 换方向
                    auto nxt = cur + Direction(i);
                    if (check_book(nxt) || check_land_invalid(nxt))
                        continue;
                    berths_path_more[p][nxt.x][nxt.y] = reverse_dir(Direction(i));
                    book[nxt.x][nxt.y] = 1;
                    q.push(nxt);
                }
            }
            step++;
        }
    }

    // 陆地上任意两点的距离预估
    int search_max_distance(Point p1, Point p2) {
        int max_dis = -1;
        int sz = berths.size();
        for (int i = 0; i < sz; i += sz / 5) {
            auto& berth_distance = berths_distance[i];
            if (berth_distance[p1.x][p1.y] == -1 || berth_distance[p2.x][p2.y] == -1)
                continue;
            max_dis = std::max(max_dis, abs(berth_distance[p1.x][p1.y] - berth_distance[p2.x][p2.y]));
        }
        for (auto& random_point_distance : random_points_distance) {
            if (random_point_distance[p1.x][p1.y] == -1 || random_point_distance[p2.x][p2.y] == -1)
                continue;
            max_dis = std::max(max_dis, abs(random_point_distance[p1.x][p1.y] - random_point_distance[p2.x][p2.y]));
        }
        if (max_dis == -1)
            return max_int_;
        return std::max(max_dis, l1_dis(p1, p2));
    }

    // 获取机器人避让方向
    Direction get_avoid_dir(Point p, Direction dir) {
        if (dir == UP || dir == DOWN || dir == STAY) {
            Direction new_dir = random_line_dir(1);
            if (check_land_valid(p + new_dir))
                return new_dir;
        }
        if (dir == RIGHT || dir == LEFT || dir == STAY) {
            Direction new_dir = random_line_dir(0);
            if (check_land_valid(p + new_dir))
                return new_dir;
        }
        if (dir == UP || dir == DOWN || dir == STAY) {
            if (check_land_valid(p + RIGHT))
                return RIGHT;
            if (check_land_valid(p + LEFT))
                return LEFT;
        }
        if (dir == RIGHT || dir == LEFT || dir == STAY) {
            if (check_land_valid(p + UP))
                return UP;
            if (check_land_valid(p + DOWN))
                return DOWN;
        }
        if (check_land_valid(p + reverse_dir(dir)))
            return reverse_dir(dir);
        return STAY;
    }

    // =================================================机器人结束，轮船开始=================================================

    // 海上bfs
    void ocean_bfs(Point from, std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>& dis) {
        std::queue<Point> q1;
        std::queue<Point> q2;  // 偶数步，主航道
        std::queue<Point> q3;  // 奇数主航道
        init_book();
        // 初始化dis
        for (int i = 0; i < MAX_MAP_SIZE; i++)
            for (int j = 0; j < MAX_MAP_SIZE; j++)
                dis[i][j] = max_int_;
        q1.push(from);
        book[from.x][from.y] = 1;
        int step = 0;
        while (!q1.empty() || !q2.empty() || !q3.empty()) {
            int sz1 = q1.size(), sz2 = q2.size(), sz3 = q3.size();
            while (true) {
                Point cur;
                if (sz1) {
                    cur = q1.front();
                    q1.pop();
                    sz1--;
                } else if ((step & 1) == 0) {
                    if (sz2 == 0)
                        break;
                    cur = q2.front();
                    q2.pop();
                    sz2--;
                } else {
                    if (sz3 == 0)
                        break;
                    cur = q3.front();
                    q3.pop();
                    sz3--;
                }
                dis[cur.x][cur.y] = step;
                for (int i = 0; i < 4; i++) {
                    auto nxt = cur + Direction(i);
                    if (check_ocean_invalid(nxt) || check_book(nxt))
                        continue;
                    book[nxt.x][nxt.y] = 1;
                    if (ocean_main[nxt.x][nxt.y]) {
                        if ((step & 1) == 0)
                            q2.push(nxt);
                        else
                            q3.push(nxt);
                    } else
                        q1.push(nxt);
                }
            }
            step++;
        }
    }

    // A*算法
    BoatSchedule get_boat_schedule_a_star(ppd from,
                                          ppd to,
                                          int public_id = -1,
                                          bool consider_other_boats = false,
                                          int truncate = a_star_trunc_steps,
                                          int quick_berth_id = -1) {
        // std::cerr << "get_boat_schedule_a_star" << std::endl;
        // return BoatSchedule();
        std::set<std::pair<int, pip>, std::less<std::pair<int, pip>>> st;
        std::unordered_map<ppd, int, ppd_hash> dis;
        std::unordered_map<ppd, BoatOp, ppd_hash> ops;
        std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>* estimated_dis;
        if (check_delivery_point(to.first)) {
            estimated_dis = &delivery_distance[delivery_points_map[to.first]];
        } else {
            estimated_dis = &berths_ocean_distance[berths_map[to.first]];
        }
        bool is_full = true;
        st.insert({l1_dis(from.first, to.first), {0, from}});
        while (!st.empty()) {
            auto [_, t] = *st.begin();
            auto [step, node] = t;
            st.erase(st.begin());
            if (step > truncate) {
                is_full = false;
                to = node;
                break;
            }
            // 核心点到达靠泊区
            // if (quick_berth_id != -1 && check_near_berth_area(node.first) &&
            //     berth_near_area.at(node.first) == quick_berth_id) {
            //     to = node;
            //     break;
            // }
            if (to.second != STAY && node == to) {
                break;
            } else if (to.second == STAY && node.first == to.first) {
                to = node;
                break;
            }
            for (int i = 0; i < 3; i++) {
                auto op = BoatOp(i);
                auto nxt = next_ppd(node, op);
                if (check_boat_op_invalid(nxt))
                    continue;
                if (consider_other_boats) {
                    std::vector<Point> points = get_boat_points(nxt);
                    bool flag = false;
                    for (auto& point : points) {
                        if (boats_map.count(point) && boats_map.at(point) != public_id) {
                            flag = true;
                            break;
                        }
                    }
                    if (flag)
                        continue;
                }
                int nxt_step = step + 1;
                if (check_boat_one_main(nxt))
                    nxt_step++;
                if (!dis.count(nxt) || dis[nxt] > nxt_step) {
                    dis[nxt] = nxt_step;
                    ops[nxt] = op;
                    auto val = get_min_estimated_dis(nxt, *estimated_dis);
                    // st.insert({(*estimated_dis)[nxt.first.x][nxt.first.y] + nxt_step, {nxt_step, nxt}});
                    st.insert({val + nxt_step, {nxt_step, nxt}});
                }
            }
        }
        if (!dis.count(to)) {
            return BoatSchedule();
        }
        BoatSchedule schedule;
        auto cur = to;
        while (!(cur == from)) {
            auto op = ops[cur];
            schedule.push_front(op);
            cur = next_ppd_rev(cur, op);
        }
        if (!is_full) {
            for (int i = 0; i < 2; i++) {
                schedule.ops.pop_back();
            }
        }
        return schedule;
    }

    // A*使用
    int get_min_estimated_dis(ppd p, std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>& estimated_dis) {
        auto [node, dir] = p;
        int min_dis = estimated_dis[node.x][node.y];
        for (int i = 0; i < BOAT_WIDTH; i++) {
            for (int j = 0; j < BOAT_LONG - 1; j++) {
                node += dir;
                min_dis = std::min(min_dis, estimated_dis[node.x][node.y]);
            }
            dir = clockwise_dir(dir);
            node += dir;
            min_dis = std::min(min_dis, estimated_dis[node.x][node.y]);
            dir = clockwise_dir(dir);
        }
        return min_dis;
    }

    // dijkstra算法，找靠泊区
    BoatSchedule get_boat_schedule(ppd from, int quick_berth_id = -1) {
        std::priority_queue<pip, std::vector<pip>, ComparePip> pq;
        std::unordered_map<ppd, int, ppd_hash> dis;
        std::unordered_map<ppd, BoatOp, ppd_hash> ops;
        ppd to;
        pq.emplace(0, from);
        while (!pq.empty()) {
            auto [step, node] = pq.top();
            pq.pop();
            // 核心点到达靠泊区
            if (quick_berth_id != -1 && check_near_berth_area(node.first) &&
                berth_near_area.at(node.first) == quick_berth_id) {
                to = node;
                break;
            }
            for (int i = 0; i < 3; i++) {
                auto op = BoatOp(i);
                auto nxt = next_ppd(node, op);
                if (check_boat_op_invalid(nxt))
                    continue;
                int nxt_step = step + 1;
                if (check_boat_one_main(nxt))
                    nxt_step++;
                if (!dis.count(nxt) || dis[nxt] > nxt_step) {
                    dis[nxt] = nxt_step;
                    ops[nxt] = op;
                    pq.emplace(nxt_step, nxt);
                }
            }
        }
        if (!dis.count(to)) {
            return BoatSchedule();
        }
        BoatSchedule schedule;
        auto cur = to;
        while (!(cur == from)) {
            auto op = ops[cur];
            schedule.push_front(op);
            cur = next_ppd_rev(cur, op);
        }
        return schedule;
    }

    // 不与col_boats的任意船冲突的操作
    BoatOp get_col_op(ppd from, std::vector<ppd> col_boats) {
        std::unordered_map<ppd, int, ppd_hash> dis;
        std::unordered_map<ppd, BoatOp, ppd_hash> ops;
        std::priority_queue<pip, std::vector<pip>, ComparePip> pq;

        pq.emplace(0, from);
        while (!pq.empty()) {
            auto [step, cur] = pq.top();
            pq.pop();
            for (int i = 0; i < 3; i++) {
                auto op = BoatOp(i);
                auto nxt = next_ppd(cur, op);
                if (check_boat_op_invalid(nxt))
                    continue;
                bool col = false;
                for (auto& col_boat : col_boats) {
                    if (check_boat_collision(nxt, col_boat)) {
                        col = true;
                        break;
                    }
                }
                if (col)
                    continue;
                return op;
            }
        }
        return BoatOp::STAY_;
    }

    // ==============================================tools================================================

    // 检测一个货物旁是否有2个及以上机器人
    bool check_good_col(Point p) {
        int cnt = 0;
        for (int i = 0; i < 4; i++) {
            auto nxt = p + Direction(i);
            if (robots_map.count({nxt.x, nxt.y}))
                cnt++;
        }
        return cnt >= 2;
    }

    // 从主干道出来型的碰撞
    bool check_main_col(Point p) {
        for (int i = 0; i < 4; i++) {
            auto nxt = p + Direction(i);
            if (check_land_main(nxt) && robots_num.count(nxt) && robots_num[nxt] >= 2)
                return true;
        }
        return false;
    }

    Berth& get_land_nearest_berth(Point p) {
        if (check_berth(p)) {
            return get_berth(p);
        }
        std::queue<Point> q;
        q.push(p);
        init_book();
        book[p.x][p.y] = 1;
        while (!q.empty()) {
            auto cur = q.front();
            q.pop();
            if (check_berth(cur)) {
                return get_berth(cur);
            }
            for (int i = 0; i < 4; i++) {
                auto nxt = cur + Direction(i);
                if (check_land_invalid(nxt) || check_book(nxt))
                    continue;
                book[nxt.x][nxt.y] = 1;
                q.push(nxt);
            }
        }
        throw std::runtime_error("no berth (land)");
    }
    Berth& get_ocean_nearest_berth(Point p) {
        if (check_berth(p)) {
            return get_berth(p);
        }
        std::queue<Point> q;
        q.push(p);
        init_book();
        book[p.x][p.y] = 1;
        while (!q.empty()) {
            auto cur = q.front();
            q.pop();
            if (check_berth(cur)) {
                return get_berth(cur);
            }
            for (int i = 0; i < 4; i++) {
                auto nxt = cur + Direction(i);
                if (check_ocean_invalid(nxt) || check_book(nxt))
                    continue;
                book[nxt.x][nxt.y] = 1;
                q.push(nxt);
            }
        }
        throw std::runtime_error("no berth (ocean)");
    }

    bool check_book_dir(Point p) { return !book_dirs[p.x][p.y].empty(); }
};