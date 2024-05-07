#pragma once
#pragma GCC optimize("O2")
#include "berth.hpp"
#include "good.hpp"
#include "schedule.hpp"
#include "utils.hpp"

class Map {
   public:
    // 地图大小
    int n;
    // 地图
    char ch[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // 当前帧数
    int berth_num;
    // 泊位
    std::vector<Berth> berths;
    // 点 -> 泊位id, 靠泊区
    std::unordered_map<Point, int> berths_map;
    std::unordered_map<Point, int> berth_near_area;
    // 货物
    std::vector<Good> goods;
    std::vector<int> now_goods_id;
    // 机器人购买点
    std::vector<Point> robot_purchase_points;
    std::unordered_map<Point, int> robot_purchase_points_map;
    // 船只购买点
    std::vector<Point> boat_purchase_points;
    std::unordered_map<Point, int> boat_purchase_points_map;
    // 交付点
    std::vector<Point> delivery_points;
    std::unordered_map<Point, int> delivery_points_map;

    // 地图的book数组：-1表示没有访问过，>=0表示direction
    int book[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // 货物的位置：-1表示没有货物，>=0表示货物的id
    int goods_map[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // 机器人的位置：-1表示没有机器人，>=0表示机器人的id
    int robots_map[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // 机器人的路径
    std::unordered_map<int, int> robots_path[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // 每个泊位到每个点的距离
    std::vector<std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>> berths_distance;
    int nearest_berth_distance[MAX_MAP_SIZE][MAX_MAP_SIZE];  // 最近泊位的距离
    // 泊位部分点的单源最短路径
    std::unordered_map<Point, std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>> berths_distance_more;
    std::unordered_map<Point, std::array<std::array<Direction, MAX_MAP_SIZE>, MAX_MAP_SIZE>> berths_path_more;
    // 部分购买点的单源最短路径
    std::unordered_map<int, std::array<std::array<int, MAX_MAP_SIZE>, MAX_MAP_SIZE>> robot_purchase_distance;
    // 泊位与商品的匹配
    std::vector<std::queue<int>> match_good_id;
    // 查找最短路径的方向
    std::stack<std::pair<Direction, int>> book_dirs[MAX_MAP_SIZE][MAX_MAP_SIZE];

    // 船舶路径
    std::unordered_map<ppd, std::unordered_map<ppd, int, ppd_hash>, ppd_hash> boat_berths_dis;
    std::unordered_map<ppd, std::unordered_map<ppd, BoatOp, ppd_hash>, ppd_hash> boat_berths_ops;
    std::vector<std::unordered_map<ppd, int, ppd_hash>> boat_purchase_dis;
    std::vector<std::unordered_map<ppd, BoatOp, ppd_hash>> boat_purchase_ops;
    // 交货点的单源最短路径
    std::unordered_map<ppd, std::unordered_map<ppd, int, ppd_hash>, ppd_hash> boat_delivery_dis;
    std::unordered_map<ppd, std::unordered_map<ppd, BoatOp, ppd_hash>, ppd_hash> boat_delivery_ops;
    // 泊位的方向
    std::vector<Direction> berth_direction;
    Direction default_dir_my = Direction::RIGHT;
    // 泊位到达交货点的方向, 用于交货点单源最短路
    std::map<pii, Direction> berth_to_delivery_dir;

    std::map<int, int> effictive_berth_num;  // 有效泊位
    std::set<int> working_berths;            // 船在去和在装载的泊位
    double share_efficiency = 0;             // 禁用泊位后的效率共享
    double share_num_efficiency = 0;         // 禁用泊位后的效率共享
    std::set<int> need_dept;                 // 进行dept的泊位

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
    int land_zone[MAX_MAP_SIZE][MAX_MAP_SIZE];
    std::map<int, int> land_zone_robots_num;  // 每个区域的机器人数量
    std::map<int, int> land_zone_robots_num_now;
    std::map<int, std::queue<int>> robot_purchase_ids;  // 机器人购买顺序

    // Map() {}
    Map(int n) : n(n) {
        // 初始化各种参数
        init_book();
        init_goods_map();
        init_robots_map();
        init_robots_path();
    }
    void init_book() { memset(book, -1, sizeof(book)); }
    void init_goods_map() { memset(goods_map, -1, sizeof(goods_map)); }
    void init_robots_map() { memset(robots_map, -1, sizeof(robots_map)); }
    void init_robots_path() {
        // memset(robots_path, -1, sizeof(robots_path));
    }
    void init_match_good_id() {
        for (int i = 0; i < (int)berths.size(); i++) {
            while (!match_good_id[i].empty())
                match_good_id[i].pop();
        }
    }
    void init_book_dirs() {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                while (!book_dirs[i][j].empty())
                    book_dirs[i][j].pop();
            }
        }
    }
    void input() {
        // 输入地图
        for (int i = 0; i < n; i++)
            scanf("%s", ch[i]);
        // 输入泊位
        scanf("%d", &berth_num);
        berths.resize(berth_num);
        for (int i = 0, berth_id; i < berth_num; i++) {
            scanf("%d", &berth_id);
            auto& berth = berths[berth_id];
            scanf("%d%d%d", &berth.p.x, &berth.p.y, &berth.loading_speed);
            berth.id = berth_id;
        }
    }

    // 初始的时候进行预处理
    void preprocess() {
        // 地图特殊点初始化+判断地图
        int hash = 0;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (check_robot_purchase_point({i, j})) {
                    robot_purchase_points.push_back({i, j});
                    robot_purchase_points_map[{i, j}] = robot_purchase_points.size() - 1;
                }
                if (check_boat_purchase_point({i, j})) {
                    boat_purchase_points.push_back({i, j});
                    boat_purchase_points_map[{i, j}] = boat_purchase_points.size() - 1;
                }
                if (check_delivery_point({i, j})) {
                    delivery_points.push_back({i, j});
                    delivery_points_map[{i, j}] = delivery_points.size() - 1;
                }
                hash = (hash + ch[i][j] * i) % 10000;
            }
        }
        // 地图特化
        // std::cerr << "hash: " << hash << std::endl;
        if (hash2map.find(hash) != hash2map.end()) {
            map_name = hash2map[hash];
        }
        // 手动设置递归搜索深度
        if (manual_recursive_depth && recursive_depth_max_plan.count(map_name) > 0) {
            recursive_depth_max = recursive_depth_max_plan[map_name];
        }
        // 手动禁用泊位
        if (manual_disable && disable_plan.count(map_name) > 0) {
            for (auto& i : disable_plan[map_name]) {
                berths[i].set_disabled();
            }
        }
        // 手动尽快购买
        if (manual_quickly_buy && manual_quickly_buy_lib.count(map_name) > 0) {
            quickly_buy = manual_quickly_buy_lib[map_name];
        }
        // 手动设置是否使用初始效率
        if (manual_using_init_efficiency && using_init_efficiency_plan.count(map_name) > 0) {
            using_init_efficiency = using_init_efficiency_plan[map_name];
        }
        if (manual_stay_until && stay_until_plan.count(map_name) > 0) {
            stay_until = stay_until_plan[map_name];
        }
        if(manual_move_at_beginning && move_at_beginning_plan.count(map_name) > 0) {
            move_at_beginning = move_at_beginning_plan[map_name];
            // std::cerr << "move_at_beginning: " << move_at_beginning << std::endl;
        }
        if (robot_type == 0)
            robot_price = ROBOT_PRICE_LOW;
        else if (robot_type == 1)
            robot_price = ROBOT_PRICE_HIGH;

        memset(nearest_berth_distance, 0x3f, sizeof(nearest_berth_distance));
        memset(land_zone, -1, sizeof(land_zone));

        // 初始化berth
        init_berths();

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
        // 初始化船从购买点到其他地方的距离
        boat_purchase_dis.resize(boat_purchase_points.size());
        boat_purchase_ops.resize(boat_purchase_points.size());
        for (int i = 0; i < (int)boat_purchase_points.size(); i++) {
            ppd from{boat_purchase_points[i], Direction::RIGHT};
            get_boat_distance(boat_purchase_dis[i], boat_purchase_ops[i], from, false);
            // // 加入STAY
            // for (size_t j = 0; j < berths.size(); j++) {
            //     int min_dis = -1;
            //     for (size_t k = 0; k < 4; k++) {
            //         ppd to{berths[j].p, Direction(k)};
            //         if (boat_purchase_dis[i].count(to)) {
            //             if (min_dis == -1 || boat_purchase_dis[i][to] < min_dis) {
            //                 min_dis = boat_purchase_dis[i][to];
            //             }
            //         }
            //     }
            //     if (min_dis != -1) {
            //         boat_purchase_dis[i][ppd(berths[j].p, Direction::STAY)] = min_dis;
            //     }
            // }
        }
        // 初始化船从交货点到其他地方的距离
        for (int i = 0; i < (int)delivery_points.size(); i++) {
            for (size_t u = 0; u < berths.size(); u++) {
                if (berth_to_delivery_dir.count({u, i}) == 0)
                    continue;
                Direction dir = berth_to_delivery_dir[{u, i}];
                ppd from{delivery_points[i], dir};
                if (boat_delivery_dis.count(from))
                    continue;
                get_boat_distance(boat_delivery_dis[from], boat_delivery_ops[from], from, false);
                // 加入STAY
                for (size_t j = 0; j < berths.size(); j++) {
                    int min_dis = -1;
                    for (size_t k = 0; k < 4; k++) {
                        ppd to{berths[j].p, Direction(k)};
                        if (boat_delivery_dis[from].count(to)) {
                            if (min_dis == -1 || boat_delivery_dis[from][to] < min_dis) {
                                min_dis = boat_delivery_dis[from][to];
                            }
                        }
                    }
                    if (min_dis != -1) {
                        boat_delivery_dis[from][ppd(berths[j].p, Direction::STAY)] = min_dis;
                    }
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
        // 删除没有交货点或者泊位价值不足的海洋区域
        std::vector<int> delete_ocean_zone_list;
        for (auto& i : zone_to_boat_purchase_points) {
            if (zone_to_delivery_points[i.first].size() == 0) {
                delete_ocean_zone_list.push_back(i.first);
            } else {
                double val_pred = 0;
                int robots_num = 0;
                for (int& berth_id : ocean_zone_to_berths[i.first]) {
                    val_pred += berths[berth_id].pred_num_per100 * 150 * mean_good_val_preset;
                    robots_num += berths[berth_id].available_robot_num;
                }
                if (val_pred < BOAT_PRICE + robots_num * robot_price) {
                    delete_ocean_zone_list.push_back(i.first);
                }
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
            ocean_zone_boat_num[i.first] = 1;
            ocean_zone_boat_num_now[i.first] = 0;
        }
        if (manual_boat_num && zone_to_boat_purchase_points.size() == 1 && manual_boat_num_plan.count(map_name) > 0) {
            ocean_zone_boat_num.begin()->second = manual_boat_num_plan[map_name];
        }
        // 船舶数据统计
        int delivery_dis_sum_one = 0;
        for (auto& i : ocean_zone_to_berths) {
            int delivery_dis_sum = 0;
            for (int& berth_id : i.second) {
                if (!berths[berth_id].is_disabled()) {
                    effictive_berth_num[i.first]++;
                    delivery_dis_sum += berths[berth_id].nearest_delivery_dis;
                }
            }
            average_berth_dis[i.first] = delivery_dis_sum / effictive_berth_num[i.first];
            delivery_dis_sum_one += delivery_dis_sum;
            // double boats_ability = 1000.0 / average_berth_dis[i.first];
            std::cerr << i.first << " average_berth_dis: " << average_berth_dis[i.first] << std::endl;
        }
        int effictive_berth_sum = 0;
        for (auto& i : effictive_berth_num) {
            effictive_berth_sum += i.second;
        }
        average_berth_dis_one = delivery_dis_sum_one / effictive_berth_sum;
        // 自动禁用泊位
        if (auto_disable) {
            for (int i = 0; i < (int)berths.size(); i++) {
                if (berths[i].nearest_delivery_dis > average_berth_dis_one * 1.5) {
                    berths[i].set_disabled();
                }
            }
        }
        // 删除没有有效泊位的陆地区域
        delete_land_zone_list.clear();
        for (auto& i : zone_to_robot_purchase_points) {
            bool flag = false;
            for (int& berth_id : land_zone_to_berths[i.first]) {
                if (berth_to_ocean_zone.count(berth_id) > 0) {
                    flag = true;
                    break;
                }
            }
            if (!flag)
                delete_land_zone_list.push_back(i.first);
        }
        for (int& i : delete_land_zone_list) {
            for (int j : zone_to_robot_purchase_points[i]) {
                robot_purchase_point_to_zone.erase(j);
            }
            for (int& berth_id : land_zone_to_berths[i]) {
                berth_to_land_zone.erase(berth_id);
                berths[berth_id].set_disabled();
            }
            zone_to_robot_purchase_points.erase(i);
            land_zone_to_berths.erase(i);
            land_zone_robots_num.erase(i);
        }
        // 机器人购买手动设置
        if (manual_robot_num && map_name != "") {
            if (land_zone_robots_num.size() == 0)
                std::cerr << "land_zone_robots_num.size() == 0" << std::endl;
            else if (land_zone_robots_num.size() == 1 && robot_max_num_lib.count(map_name) > 0)
                land_zone_robots_num.begin()->second = robot_max_num_lib[map_name];
            else if (land_zone_robot_num_lib.count(map_name) > 0) {
                land_zone_robots_num = land_zone_robot_num_lib[map_name];
            }
        }
        if (test_map && map_name == "map1") {
            land_zone_robots_num.begin()->second = 1;
        }
        if (test_map && map_name == "map2") {
            land_zone_robots_num.begin()->second = 2;
        }
        if (test_map && map_name == "map3") {
            land_zone_robots_num.begin()->second = 3;
        }
        // 重新统计机器人购买数量
        robot_max_num = 0;
        for (auto& i : land_zone_robots_num) {
            robot_max_num += i.second;
            land_zone_robots_num_now[i.first] = 0;
            int now_num = -5;
            while (now_num < i.second) {
                for (auto& j : zone_to_robot_purchase_points[i.first]) {
                    robot_purchase_ids[i.first].push(j);
                    now_num++;
                    if (now_num >= i.second)
                        break;
                }
            }
        }
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
        // 初始化方向
        berth_direction.resize(berths.size());
        // 初始化match_good_id
        match_good_id.resize(berths.size());
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
        /*
        if(dir == UP) {
            if(check_land_valid(berth.p + LEFT)) dir = RIGHT;
            else dir = LEFT;
        } else if(dir == DOWN) {
            if(check_land_valid(berth.p + RIGHT)) dir = LEFT;
            else dir = RIGHT;
        }
        */
        if (dir == STAY) {
            std::cerr << "init berth " << berth_id << " error!" << std::endl;
            return;
        }
        auto from = ppd(berth.p, dir);
        berth.boat_ppd = from;
        if (check_boat_one_invalid(from))
            return;

        // 船在泊位的路径
        auto& dis = boat_berths_dis[from];
        auto& ops = boat_berths_ops[from];
        // 核心调用：
        get_boat_distance(dis, ops, from, true, {{-1, -1}, STAY}, berth_id);
        // 加入去其他泊位STAY
        for (size_t i = 0; i < berths.size(); i++) {
            if (int(i) == berth_id)
                continue;
            int min_dis = -1;
            for (size_t j = 0; j < 4; j++) {
                ppd to{berths[i].p, Direction(j)};
                if (dis.count(to)) {
                    if (min_dis == -1 || dis[to] < min_dis) {
                        min_dis = dis[to];
                    }
                }
            }
            if (min_dis != -1) {
                dis[ppd(berths[i].p, Direction::STAY)] = min_dis;
            }
        }
        // 获取最近交货点
        int dis_tmp = max_int_;
        for (int j = 0; j < (int)delivery_points.size(); j++) {
            if (berth_to_delivery_dir.count({berth_id, j}) == 0)
                continue;
            ppd delivery_point{delivery_points[j], berth_to_delivery_dir[{berth_id, j}]};
            if (dis.count(delivery_point)) {
                if (dis[delivery_point] < dis_tmp) {
                    dis_tmp = dis[delivery_point];
                    berth.nearest_delivery_id = j;
                    berth.nearest_delivery_dis = dis_tmp;
                }
            }
        }
        // std::cerr << "== " << berth_id << " " << from.first.x << " " <<
        // from.first.y << " " << from.second << std::endl;
    }

    // 船的单源最短路(Dijkstra)
    ppd get_boat_distance(std::unordered_map<ppd, int, ppd_hash>& dis,     // 距离
                          std::unordered_map<ppd, BoatOp, ppd_hash>& ops,  // 操作
                          ppd from,                                        // from
                          bool is_berth_init = false,                      // 是否对全部的点进行搜索
                          ppd to = {{-1, -1}, STAY},                       // to
                          int berth_id = -1) {
        std::priority_queue<pip, std::vector<pip>, ComparePip> pq;
        int now_num = 0, search_num = 2;  // 为每个泊位找两个交货点
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
        pq.emplace(0, from);
        while (!pq.empty()) {
            auto [step, node] = pq.top();  // node is ppd
            pq.pop();
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
                if (check_delivery_point(node.first) &&
                    berth_to_delivery_dir.count({berth_id, delivery_points_map[node.first]}) == 0 && now_num < search_num) {
                    berth_to_delivery_dir[{berth_id, delivery_points_map[node.first]}] = node.second;
                    now_num++;
                }
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
                    dis[nxt] = nxt_step;
                    ops[nxt] = op;
                    pq.emplace(nxt_step, nxt);
                }
            }
        }
        return to;
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

    void init_effitiency_predict() {
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
            double ratio = double(it->second) / all_space;
            land_zone_robots_num[it->first] = round(robot_max_num * ratio);
            now_robot_num += land_zone_robots_num[it->first];
            if (land_zone_robots_num[it->first] == 0 && ratio > 3000.0 / 170000.0 && now_robot_num < 1.5 * robot_max_num) {
                land_zone_robots_num[it->first] = 1;
                now_robot_num++;
            }
        }
        robot_max_num = now_robot_num;
        // 预估效率
        for (auto& i : land_zone_to_berths) {
            for (int& berth_id : i.second) {
                berths[berth_id].available_robot_num = double(land_zone_robots_num[i.first]) / i.second.size();
            }
        }
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++) {
                if (land_zone_space.count(land_zone[i][j]) != 0) {
                    int zone_id = land_zone[i][j];
                    double val = 1;
                    double adjust_average_nearest_dis = average_nearest_dis * 1;  // 可调参
                    if (nearest_berth_distance[i][j] > adjust_average_nearest_dis)
                        val = pow(adjust_average_nearest_dis / nearest_berth_distance[i][j], 3);  // 可调参
                    double berth_dis_coef_sum = 0;
                    std::map<int, double> berth_dis_coef;
                    for (int& berth_id : land_zone_to_berths[zone_id]) {
                        berth_dis_coef[berth_id] = pow(1.0 / (berths_distance[berth_id][i][j] + 3), 3);  // 可调参
                        if (global_debug && berths_distance[berth_id][i][j] == -1)
                            std::cerr << "err376 " << i << " " << j << " " << berth_id << std::endl;
                        berth_dis_coef_sum += berth_dis_coef[berth_id];
                    }
                    for (int& berth_id : land_zone_to_berths[zone_id]) {
                        berths[berth_id].pred_num_per100 +=
                            val * (berth_dis_coef[berth_id] / berth_dis_coef_sum) * berths[berth_id].available_robot_num;
                        if (global_debug && std::isnan(berths[berth_id].pred_num_per100))
                            std::cerr << "err388 " << val << " " << berth_dis_coef[berth_id] << " " << berth_dis_coef_sum;
                    }
                }
            }
        double pred_num_per100_sum = 0;
        for (int k = 0; k < berth_num; k++) {
            pred_num_per100_sum += berths[k].pred_num_per100;
        }
        double num_per100_coef = 10.5;  // 应为pulled_goods_num / 150
        for (int k = 0; k < berth_num; k++) {
            berths[k].pred_num_per100 = berths[k].pred_num_per100 / pred_num_per100_sum * num_per100_coef;
        }
    }

    // ============================初始化结束，输入处理===============================

    // == 货物 ==
    // 添加货物
    void add_good(Point p, int val, int fade_time) {
        goods.push_back(Good(p, val, fade_time));
        goods_map[p.x][p.y] = goods.back().id;
    }
    // 移除货物
    void remove_good(Point p) { goods_map[p.x][p.y] = -1; }

    // == 机器人 ==
    void update_robot(int robot_id, Schedule& schedule, Point p) {
        // if(robot_id == 9 && frame_id > 2400) {
        //     std::cerr << "update_robot " << robot_id << " " <<
        //     schedule.ops.size() << std::endl;
        // }
        int step = frame_id;
        robots_path[p.x][p.y][step] = robot_id;
        // if(!schedule.is_valid()) robots_path[p.x][p.y].insert(step + 1);
        for (auto& op : schedule.ops) {
            p = p + Direction(op);
            robots_path[p.x][p.y][++step] = robot_id;
        }
    }

    // ===========================================输出处理=======================================================

    // 每帧的更新处理
    void update() {
        // 禁用泊位
        if (allow_disable) {
            for (auto& berth : berths) {
                // 重启被禁用的泊位
                if (boats_going_berths.count(berth.id) && berth.is_disabled()) {
                    if (global_debug)
                        std::cerr << frame_id << ": enable berth " << berth.id << std::endl;
                    berth.set_idle();
                    working_berths.insert(berth.id);
                    share_efficiency -= berth.weighted_efficiency * efficiency_share_rate;  // 粗略的
                    share_num_efficiency -= berth.weighted_num_efficiency * efficiency_share_rate;
                    continue;
                }
                if (berth.is_disabled())
                    continue;
                if (frame_id > disable_start_time && !boats_going_berths.count(berth.id)) {
                    int delivery_id = berth.nearest_delivery_id;
                    // 船不可达
                    if (delivery_id == -1) {
                        berth.set_disabled();
                        continue;
                    }
                    int ocean_zone_id = berth_to_ocean_zone[berth.id];
                    int land_zone_id = berth_to_land_zone[berth.id];
                    std::map<int, int> ocean_zone_to_active_berth;
                    std::map<int, int> land_zone_to_active_berth;
                    for (int i : working_berths) {
                        ocean_zone_to_active_berth[berth_to_ocean_zone[i]]++;
                        land_zone_to_active_berth[berth_to_land_zone[i]]++;
                    }
                    // 不够好的禁用策略
                    if (FRAME_NUM - frame_id < berth.mean_boat_time * mean_boat_time_rate + berth.nearest_delivery_dis &&
                        ocean_zone_to_active_berth[ocean_zone_id] > 1 && land_zone_to_active_berth[land_zone_id] > 1) {
                        berth.set_disabled();
                        std::cerr << frame_id << ": disable berth " << berth.id << std::endl;
                        if (working_berths.count(berth.id))
                            working_berths.erase(berth.id);
                        share_efficiency += berth.weighted_efficiency * efficiency_share_rate;
                        share_num_efficiency += berth.weighted_num_efficiency * efficiency_share_rate;
                    }
                }
            }
        }
        // 计算泊位效率
        for (auto& berth : berths) {
            berth.update_efficiency(share_efficiency, share_num_efficiency, working_berths.size());
            // robots_sum += berth.get_robot_num();
        }
        int robots_sum = 0;
        for (auto& i : match_good_id)
            robots_sum += i.size();
        // if (global_debug && frame_id % 100 == 0)
        //     std::cerr << frame_id << ": robots_sum: " << robots_sum << std::endl;
        // 计算货物衰减，同时统计货物
        now_goods_id.clear();
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (goods_map[i][j] != -1) {
                    auto& good = get_good(goods_map[i][j]);
                    // 如果已经消失，就移除
                    if (good.fade_time <= frame_id) {
                        remove_good({i, j});
                    } else {
                        now_goods_id.push_back(goods_map[i][j]);
                    }
                }
            }
        }
        // 泊位与货物的匹配
        auto start = get_time_point();
        match_berth_to_good();
        double duration = get_duration(start);
        if (global_debug && duration > log_time_threshold * 0.8) {
            std::cerr << "match_berth_to_good time: " << duration << "ms" << std::endl;
        }
        // 最后的时候进行更深的搜索
        if (FRAME_NUM - frame_id < average_berth_dis_one * rate_recursive_depth) {
            recursive_depth_max = std::max(recursive_depth_max_end, recursive_depth_max);
        }
    }

    // KEY FUNCTION: 匹配泊位与货物  TODO: 考虑可不可以改为匈牙利算法
    void match_berth_to_good() {
        int now_good_num = now_goods_id.size();
        init_match_good_id();
        std::priority_queue<std::tuple<double, int, int>> q;
        int cnt = 0, berth_goods_num[berth_num]{};
        for (int i = 0; i < (int)berths.size(); i++) {
            if (berths[i].is_disabled())
                continue;
            berth_goods_num[i] = berths[i].get_robot_num();  // +1 应该不用
            cnt += berth_goods_num[i];
            int good_num = 0;
            for (int j = 0; j < now_good_num; j++) {
                auto& good = get_good(now_goods_id[j]);
                if (good.is_selected() || good.is_carried())
                    continue;
                auto dis = berths_distance[i][good.p.x][good.p.y];
                if (dis == -1)
                    continue;
                if (frame_id + dis + SAFE_TIME > good.fade_time)
                    continue;
                // 价值计算方法：(good.val - (double) (good.fade_time - (frame_id + dis + SAFE_TIME)) / 50) / (double) dis;
                // 使用联合距离：auto com_dis = search_com_distance(berths[i].p, good.p);
                // int dis2 = max_int_;
                // for (int k = 0; k < (int)berths.size(); k++) {
                //     if (berths[k].is_disabled())
                //         continue;
                //     dis2 = std::min(dis2, berths_distance[k][good.p.x][good.p.y]);
                // }
                auto val = (good.val) / (dis + dis_plus);
                good_num++;
                q.push({val, i, j});
            }
            if (good_num < berth_goods_num[i]) {
                cnt -= berth_goods_num[i] - good_num;
            }
        }
        std::vector<int> vis(now_good_num, 0);
        while (!q.empty()) {
            auto [val, i, j] = q.top();
            (void)val;
            q.pop();
            if (vis[j] || match_good_id[i].size() >= size_t(berth_goods_num[i]))
                continue;  // +1
            // TOFIX:
            // 如果真的被分配出去，实际上可能产生的时间是多少。如何去快速估计真正的时间？
            // auto schedule = get_schedule_to_good(berths[i].p, now_goods_id[j]);
            // auto& good = get_good(now_goods_id[j]);
            // if (!schedule.is_valid() || schedule.get_steps() + frame_id + SAFE_TIME > good.fade_time)
            //     continue;
            vis[j] = 1;
            match_good_id[i].push(now_goods_id[j]);
            cnt--;
            if (cnt == 0)
                break;
        }
    }

    // ============================================机器人=======================================================

    // 从某个点开始查找货物 (KEY FUNCTION)
    Schedule search_good(Point p) {
        // 如果在泊位上，就直接用分配好的货物
        if (check_berth(p)) {
            auto& berth = get_berth(p);
            while (!match_good_id[berth.id].empty()) {
                auto good_id = match_good_id[berth.id].front();
                match_good_id[berth.id].pop();
                auto& good = get_good(good_id);
                if (good.is_selected() || good.is_carried())
                    continue;
                Schedule schedule;
                if (berths_distance_more.count(p))
                    schedule = get_schedule(p, good.p);
                else {
                    // std::cerr <<frame_id<< ": bfs_berth_distance" << std::endl;
                    bfs_berth_distance(p);
                    // std::cerr <<frame_id<< ": bfs_berth_distance ok " <<good.p.x<<" "<<good.p.y << std::endl;
                    schedule = get_schedule(p, good.p);
                    // if(schedule.is_valid())std::cerr <<frame_id<< ": get_schedule ok" << std::endl;
                }
                if (schedule.is_valid() && schedule.get_steps() + frame_id + SAFE_TIME <= good.fade_time) {
                    schedule.set_target_good(good.id);
                    return schedule;
                }
                // break;
            }
            // 找不到应该是被禁用了，调用下面的部分
            if (!berths[berths_map[p]].is_disabled())
                return Schedule();
            // else if (global_debug)
            //     std::cerr << frame_id << ": err: search_good " << p.x << " " << p.y << std::endl;
        }
        auto start = get_time_point();
        // if(frame_id>1000)std::cerr << "warning: search_good" << std::endl;
        // 不在泊位上就需要直接BFS搜索, 如初始
        std::priority_queue<std::pair<double, int>> q;
        for (auto& good_id : now_goods_id) {
            auto& good = get_good(good_id);
            if (good.fade_time <= frame_id)
                continue;
            if (good.is_selected() || good.is_carried())
                continue;
            auto dis = search_max_distance(p, good.p);
            if (dis == -1)
                continue;
            if (frame_id + dis + SAFE_TIME > good.fade_time)
                continue;
            if (map_name != "mapf2") {
                dis = search_com_distance(p, good.p);
            }
            auto val = double(good.val) / (dis + dis_plus);
            q.push({val, good.id});
        }
        if (q.empty()) {
            return Schedule();
        }
        while (!q.empty()) {
            auto max_id = q.top().second;
            q.pop();
            auto& good = get_good(max_id);
            auto schedule = get_schedule_by_bfs(p, good.p);
            if (schedule.is_valid() && schedule.get_steps() + frame_id + SAFE_TIME / 2 <= good.fade_time) {
                schedule.set_target_good(good.id);
                return schedule;
            }
        }
        double duration = get_duration(start);
        if (global_debug && duration > 3) {
            std::cerr << frame_id << ": special search_good time: " << duration << "ms" << std::endl;
        }
        return Schedule();
    }

    // 找最近的泊位
    int search_berth(Point p, int robot_id, bool is_first = false) {
        std::priority_queue<std::pair<double, int>> q;
        for (int i = 0; i < (int)berths.size(); i++) {
            if (berths[i].is_disabled())
                continue;
            auto& berth_distance = berths_distance[i];
            if (berth_distance[p.x][p.y] == -1)
                continue;
            auto val = -(berth_distance[p.x][p.y] /** berths[i].nearest_delivery_dis*/);
            q.push({val, i});
        }
        while (!q.empty()) {
            auto [val, i] = q.top();
            (void)val;
            q.pop();
            // auto& berth = berths[i];
            // if (berth.is_disabled())
            //     continue;
            return i;
        }
        std::cerr << frame_id << ": search_berth error " << p.x << " " << p.y << std::endl;
        return -1;
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

    // 使用BFS搜索从from点到to点的最短路径，仅在初始时使用
    Schedule get_schedule_by_bfs(Point from, Point to) {
        // 先判断是否不可达
        if (search_max_distance(from, to) == -1) {
            return Schedule();
        }
        // 不在berth
        init_book();
        std::queue<Point> q;
        q.push(from);
        book[from.x][from.y] = Direction::STAY;
        int step = frame_id + 1;
        while (!q.empty()) {
            int sz = q.size();
            while (sz--) {
                auto cur = q.front();
                q.pop();
                for (int i = 0; i < 4; i++) {
                    auto nxt = cur + Direction(i);
                    if (check_land_invalid(nxt) || check_book(nxt))
                        continue;
                    book[nxt.x][nxt.y] = Direction(i);
                    q.push(nxt);
                }
            }
            step++;
            if (check_book(to)) {
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
            auto op = reverse_dir(Direction(book[cur.x][cur.y]));
            schedule.put_op(op);
            cur += Direction(op);
        }
        // std::cerr << "?? " << schedule.ops.size() << std::endl;
        schedule = schedule.reverse();
        return schedule;
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
            // std::cerr << step << " " << sz << std::endl;
            while (sz--) {
                // std::cerr <<sz << std::endl;
                auto cur = q.front();
                // std::cerr << "here2" << std::endl;
                berths_distance_more[p][cur.x][cur.y] = step;
                // std::cerr << "here3" << std::endl;
                q.pop();
                // std::cerr << "here1" << std::endl;
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

    int search_max_distance(Point p1, Point p2) {
        int max_dis = -1;
        for (int i = 0; i < (int)berths.size(); i++) {
            auto& berth_distance = berths_distance[i];
            if (berth_distance[p1.x][p1.y] == -1 || berth_distance[p2.x][p2.y] == -1)
                continue;
            max_dis = std::max(max_dis, abs(berth_distance[p1.x][p1.y] - berth_distance[p2.x][p2.y]));
        }
        return max_dis;
    }
    int search_com_distance(Point p1, Point p2) {
        int com_dis = 0x3f3f3f3f;
        for (int i = 0; i < (int)berths.size(); i++) {
            auto& berth_distance = berths_distance[i];
            if (berth_distance[p1.x][p1.y] == -1 || berth_distance[p2.x][p2.y] == -1)
                continue;
            com_dis = std::min(
                com_dis, abs(berth_distance[p1.x][p1.y] - berth_distance[p2.x][p2.y]) + abs(berth_distance[p2.x][p2.y]));
        }
        return com_dis;
    }

    // 获取机器人避让方向
    Direction get_avoid_dir(Point p, Direction dir) {
        if (is_random) {
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

    // KEY FUNCTION: 船只到达某个点
    BoatSchedule get_boat_schedule(ppd from, ppd to) {
        std::unordered_map<ppd, int, ppd_hash>* dis;
        std::unordered_map<ppd, BoatOp, ppd_hash>* ops;
        bool flag = false;
        // 开始点在购买点
        if (check_boat_purchase_point(from.first) && from.second == RIGHT) {
            // std::cerr << "get_boat_schedule: from is purchase point" <<
            // std::endl;
            auto purchase_id = get_boat_purchase_point_id(from.first);
            dis = &boat_purchase_dis[purchase_id];
            ops = &boat_purchase_ops[purchase_id];
            if (to.second == STAY) {
                int min_dis = -1;
                Direction dir = STAY;
                for (int i = 0; i < 4; i++) {
                    if ((*dis).count({to.first, Direction(i)})) {
                        if (min_dis == -1 || (*dis)[{to.first, Direction(i)}] < min_dis) {
                            min_dis = (*dis)[{to.first, Direction(i)}];
                            dir = Direction(i);
                        }
                    }
                }
                to.second = dir;
            }
        }
        // 开始点在交货点
        else if (check_delivery_point(from.first)) {
            // std::cerr << "get_boat_schedule: from is delivery point" <<
            // std::endl;
            // auto delivery_id = get_delivery_point_id(from.first);
            dis = &boat_delivery_dis[from];
            ops = &boat_delivery_ops[from];
            if (to.second == STAY) {
                int min_dis = -1;
                Direction dir = STAY;
                for (int i = 0; i < 4; i++) {
                    if ((*dis).count({to.first, Direction(i)})) {
                        if (min_dis == -1 || (*dis)[{to.first, Direction(i)}] < min_dis) {
                            min_dis = (*dis)[{to.first, Direction(i)}];
                            dir = Direction(i);
                        }
                    }
                }
                to.second = dir;
            }
        }
        // 开始在泊位
        else if (boat_berths_dis.count(from)) {
            // if(frame_id==261)std::cerr << "get_boat_schedule: from is berth" << std::endl;
            dis = &boat_berths_dis[from];
            ops = &boat_berths_ops[from];
            if (to.second == STAY) {
                int min_dis = -1;
                Direction dir = STAY;
                for (int i = 0; i < 4; i++) {
                    if ((*dis).count({to.first, Direction(i)})) {
                        if (min_dis == -1 || (*dis)[{to.first, Direction(i)}] < min_dis) {
                            min_dis = (*dis)[{to.first, Direction(i)}];
                            dir = Direction(i);
                        }
                    }
                }
                to.second = dir;
            }
        }
        // 找不到再调用一次
        else {
            // std::cerr << "get_boat_schedule: from is not found in init" << std::endl;
            // std::cerr << frame_id << " " << from.first.x << " " << from.first.y << " " << from.second << std::endl;
            flag = true;
            dis = new std::unordered_map<ppd, int, ppd_hash>();
            ops = new std::unordered_map<ppd, BoatOp, ppd_hash>();
            to = get_boat_distance(*dis, *ops, from, false, to);
            if (!(*dis).count(to)) {
                delete dis;
                delete ops;
                return BoatSchedule();
            }
        }
        if (to.second == STAY) {
            std::cerr << frame_id << " get_boat_schedule: to is STAY" << std::endl;
            return BoatSchedule();
        }
        // 回溯
        BoatSchedule schedule;
        auto cur = to;
        while (!(cur == from)) {
            auto op = (*ops)[cur];
            schedule.push_front(op);
            cur = next_ppd_rev(cur, op);
        }
        // if(frame_id==261)std::cerr << "get_boat_schedule ok" << std::endl;
        if (flag) {
            delete dis;
            delete ops;
        }
        return schedule;
    }

    // ==============================================tools================================================

    // == 泊位 ==
    Berth& get_berth(int berth_id) {
        if (berth_id >= 0 && berth_id < (int)berths.size()) {
            return berths[berth_id];
        } else {
            std::cerr << "berth_id out of range" << std::endl;
            return berths[0];
        }
    }
    Berth& get_berth(Point p) {
        if (berths_map.find(p) == berths_map.end()) {
            throw std::runtime_error("p is not a berth");
        }
        int berth_id = berths_map[p];
        return get_berth(berth_id);
    }

    int get_boat_purchase_point_id(Point p) {
        if (boat_purchase_points_map.find(p) != boat_purchase_points_map.end()) {
            return boat_purchase_points_map[p];
        }
        return -1;
    }
    int get_delivery_point_id(Point p) {
        if (delivery_points_map.find(p) != delivery_points_map.end()) {
            return delivery_points_map[p];
        }
        return -1;
    }

    // 获取货物
    Good& get_good(int good_id) {
        if (good_id >= 0 && good_id < (int)goods.size()) {
            return goods[good_id];
        } else {
            throw std::runtime_error("good_id out of range");
        }
    }
    Good& get_good(Point p) {
        int good_id = goods_map[p.x][p.y];
        return get_good(good_id);
    }

    // 船舶
    bool check_boat_collision(ppd node1, ppd node2) {
        std::vector<Point> points1, points2;
        auto& [p1, dir1] = node1;
        auto& [p2, dir2] = node2;
        points1.push_back(p1);
        points2.push_back(p2);
        for (int i = 0; i < 2; i++) {
            p1 += dir1;
            p2 += dir2;
            points1.push_back(p1);
            points2.push_back(p2);
        }
        dir1 = clockwise_dir(dir1);
        p1 += dir1;
        points1.push_back(p1);
        dir1 = clockwise_dir(dir1);
        dir2 = clockwise_dir(dir2);
        p2 += dir2;
        points2.push_back(p2);
        dir2 = clockwise_dir(dir2);
        for (int i = 0; i < 2; i++) {
            p1 += dir1;
            p2 += dir2;
            points1.push_back(p1);
            points2.push_back(p2);
        }
        for (auto& point1 : points1) {
            for (auto& point2 : points2) {
                if (point1 == point2 && !check_ocean_main(point1)) {
                    return true;
                }
            }
        }
        return false;
    }
    // // 允许node2运动
    // bool check_boat_collision_large_scale(ppd node1, ppd node2) {
    //     std::vector<Point> points1, points2;
    //     auto& [p1, dir1] = node1;
    //     auto& [p2, dir2] = node2;
    //     points1.push_back(p1);
    //     points2.push_back(p2);
    //     for (int i = 0; i < 2; i++) {
    //         p1 += dir1;
    //         points1.push_back(p1);
    //     }
    //     dir1 = clockwise_dir(dir1);
    //     p1 += dir1;
    //     points1.push_back(p1);
    //     dir1 = clockwise_dir(dir1);
    //     std::function<void()> clockwise_step = [&]() {
    //         dir2 = clockwise_dir(dir2);
    //         p2 += dir2;
    //         points2.push_back(p2);
    //     };
    //     std::function<void()> anticlockwise_step = [&]() {
    //         dir2 = anticlockwise_dir(dir2);
    //         p2 += dir2;
    //         points2.push_back(p2);
    //     };
    //     p2 += dir2;
    //     points2.push_back(p2);
    //     anticlockwise_step();
    //     clockwise_step();
    //     clockwise_step();
    //     anticlockwise_step();
    //     clockwise_step();
    //     clockwise_step();
    //     anticlockwise_step();
    //     clockwise_step();
    //     anticlockwise_step();
    //     for (int i = 0; i < 2; i++) {
    //         p1 += dir1;
    //         points1.push_back(p1);
    //     }
    //     for (auto& point1 : points1) {
    //         for (auto& point2 : points2) {
    //             if (point1 == point2 && !check_ocean_main(point1)) {
    //                 return true;
    //             }
    //         }
    //     }
    //     return false;
    // }
    bool check_boat_op_invalid(ppd node) {
        auto& [p, dir] = node;
        for (int i = 0; i < BOAT_WIDTH; i++) {
            p += dir;
        }
        if (check_ocean_invalid(p))
            return true;
        for (int i = 0; i < BOAT_WIDTH; i++) {
            for (int j = 0; j < BOAT_LONG - BOAT_WIDTH - 1; j++) {
                p += dir;
                if (check_ocean_invalid(p))
                    return true;
            }
            dir = clockwise_dir(dir);
            p += dir;
            if (check_ocean_invalid(p))
                return true;
            dir = clockwise_dir(dir);
        }
        return false;
    }
    bool check_boat(ppd node, std::function<bool(Point)>& check_func) {
        auto& [p, dir] = node;
        if (check_func(p))
            return true;
        for (int i = 0; i < BOAT_WIDTH; i++) {
            for (int j = 0; j < BOAT_LONG - 1; j++) {
                p += dir;
                if (check_func(p))
                    return true;
            }
            dir = clockwise_dir(dir);
            p += dir;
            if (check_func(p))
                return true;
            dir = clockwise_dir(dir);
        }
        return false;
    }
    bool check_boat_one_main(ppd node) {
        std::function<bool(Point)> func = [this](Point p) { return this->check_ocean_main(p); };
        return check_boat(node, func);
    }
    bool check_boat_all_main(ppd node) {
        std::function<bool(Point)> func = [this](Point p) { return !this->check_ocean_main(p); };
        return !check_boat(node, func);
    }
    bool check_boat_one_invalid(ppd node) {
        std::function<bool(Point)> func = [this](Point p) { return this->check_ocean_invalid(p); };
        return check_boat(node, func);
    }

    // is
    // 空地
    inline bool is_space(Point p) { return ch[p.x][p.y] == '.'; }
    // 陆地主干道
    inline bool is_land_main(Point p) { return ch[p.x][p.y] == '>'; }
    // 海洋
    inline bool is_ocean(Point p) { return ch[p.x][p.y] == '*'; }
    // 海洋主航道
    inline bool is_ocean_main(Point p) { return ch[p.x][p.y] == '~'; }
    // 障碍
    inline bool is_block(Point p) { return ch[p.x][p.y] == '#'; }
    // 机器人购买地块
    inline bool is_robot_purchase_point(Point p) { return ch[p.x][p.y] == 'R'; }
    // 船舶购买地块
    inline bool is_boat_purchase_point(Point p) { return ch[p.x][p.y] == 'S'; }
    // 泊位
    inline bool is_berth(Point p) { return ch[p.x][p.y] == 'B'; }
    // 靠泊区
    inline bool is_near_berth_area(Point p) { return ch[p.x][p.y] == 'K'; }
    // 海陆立体交通地块
    inline bool is_sea_land_traffic_point(Point p) { return ch[p.x][p.y] == 'C'; }
    // 海陆立体交通地块 + 主干道/主航道
    inline bool is_sea_land_traffic_main(Point p) { return ch[p.x][p.y] == 'c'; }
    // 交货点
    inline bool is_delivery_point(Point p) { return ch[p.x][p.y] == 'T'; }

    // check
    // 是否超出了地图范围
    inline bool check_invalid(Point p) { return p.x < 0 || p.x >= n || p.y < 0 || p.y >= n; }
    // 是否在陆地上
    inline bool check_land_valid(Point p) {
        if (check_invalid(p))
            return false;
        return is_space(p) || is_land_main(p) || is_robot_purchase_point(p) || is_berth(p) || is_sea_land_traffic_point(p) ||
               is_sea_land_traffic_main(p);
    }
    inline bool check_land_invalid(Point p) { return !check_land_valid(p); }
    // 是否在海洋上
    inline bool check_ocean_valid(Point p) {
        if (check_invalid(p))
            return false;
        return is_ocean(p) || is_ocean_main(p) || is_boat_purchase_point(p) || is_berth(p) || is_near_berth_area(p) ||
               is_sea_land_traffic_point(p) || is_sea_land_traffic_main(p) || is_delivery_point(p);
    }
    inline bool check_ocean_invalid(Point p) { return !check_ocean_valid(p); }
    // 是否在主干道上
    inline bool check_land_main(Point p) {
        return is_land_main(p) || is_robot_purchase_point(p) || is_berth(p) || is_sea_land_traffic_main(p);
    }
    // 是否在主航道上
    inline bool check_ocean_main(Point p) {
        return is_ocean_main(p) || is_boat_purchase_point(p) || is_berth(p) || is_near_berth_area(p) ||
               is_sea_land_traffic_main(p) || is_delivery_point(p);
    }
    // 机器人购买点
    inline bool check_robot_purchase_point(Point p) { return is_robot_purchase_point(p); }
    // 船舶购买点
    inline bool check_boat_purchase_point(Point p) { return is_boat_purchase_point(p); }
    // 交货点
    inline bool check_delivery_point(Point p) { return is_delivery_point(p); }
    // 泊位
    inline bool check_berth(Point p) { return is_berth(p); }
    // 泊位停靠区域
    inline bool check_near_berth_area(Point p) { return is_near_berth_area(p); }

    inline bool check_good(Point p) { return goods_map[p.x][p.y] != -1; }
    bool check_book(Point p) { return book[p.x][p.y] != -1; }
    bool check_book_dir(Point p) { return !book_dirs[p.x][p.y].empty(); }
};