#pragma once
#pragma GCC optimize("O2")
#include "berth.hpp"
#include "boat_pos.hpp"
#include "good.hpp"
#include "utils.hpp"

class MapPatch {
   public:
    int patch_id{0};
    int size{0};
    int max_good_num{0};
    Point center{};
    int robot_num{0};
    int now_our_robot_num{0};
    int goto_our_robot_num{0};
    int high_goods_num{0};
    int zone_id{-1};

    int get_robot_num() { return robot_num; }
    int get_our_robot_num() { return now_our_robot_num + goto_our_robot_num; }
    void clear() {
        robot_num = 0;
        now_our_robot_num = 0;
    }
    void add_goto_robot() { goto_our_robot_num++; }
    void remove_goto_robot() { goto_our_robot_num--; }
    double get_potential() { return 1.0 * max_good_num / (robot_num + goto_our_robot_num * 5 + now_our_robot_num * 4 + 0.1) * (high_goods_num + 1); }
};

class BaseMap {
   public:
    // 地图大小
    int n;
    // 地图
    char ch[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // 类型判断
    bool land_valid[MAX_MAP_SIZE][MAX_MAP_SIZE];
    bool ocean_valid[MAX_MAP_SIZE][MAX_MAP_SIZE];
    bool land_main[MAX_MAP_SIZE][MAX_MAP_SIZE];
    bool ocean_main[MAX_MAP_SIZE][MAX_MAP_SIZE];
    bool boat_op_invalid[MAX_MAP_SIZE][MAX_MAP_SIZE][4];
    bool boat_one_main[MAX_MAP_SIZE][MAX_MAP_SIZE][4];
    // 地图的book数组：-1表示没有访问过，>=0可以表示direction
    int book[MAX_MAP_SIZE][MAX_MAP_SIZE];
    // 泊位
    std::vector<Berth> berths;
    std::unordered_map<Point, int> berths_map;       // 泊位点 -> 泊位id
    std::unordered_map<Point, int> berth_near_area;  // 靠泊区点 -> 泊位id
    // 货物
    std::vector<Good> goods;
    std::vector<int> now_goods_id;
    int goods_map[MAX_MAP_SIZE][MAX_MAP_SIZE];  // 货物的位置：-1表示没有货物，>=0表示货物的id
    // 机器人购买点
    std::vector<Point> robot_purchase_points;
    std::unordered_map<Point, int> robot_purchase_points_map;
    // 船只购买点
    std::vector<Point> boat_purchase_points;
    std::unordered_map<Point, int> boat_purchase_points_map;
    // 交付点
    std::vector<Point> delivery_points;
    std::unordered_map<Point, int> delivery_points_map;
    // 缓存货物
    std::unordered_set<int> cache_goods;
    // std::unordered_set<int> precious_goods;  // 珍贵货物
    // 地图划分
    std::unordered_map<Point, int> map_split;
    std::unordered_map<int, MapPatch> map_patches;
    std::unordered_map<int, std::unordered_set<int>> patches_edge;

    BaseMap(int n) : n(n) { init_goods_map(); }
    BaseMap(const BaseMap&) = delete;
    BaseMap& operator=(const BaseMap&) = delete;

    // init
    void init_goods_map() { memset(goods_map, -1, sizeof(goods_map)); }
    void init_book() { memset(book, -1, sizeof(book)); }

    // input
    void input_map_() {
        for (int i = 0; i < n; i++) {
            scanf("%s", ch[i]);
            get_ok();
            // 初始化每一行
            for (int j = 0; j < n; j++) {
                land_valid[i][j] = check_land_valid_(Point(i, j));
                ocean_valid[i][j] = check_ocean_valid_(Point(i, j));
                land_main[i][j] = check_land_main_(Point(i, j));
                ocean_main[i][j] = check_ocean_main_(Point(i, j));
            }
            put_ok();
        }
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < 4; k++) {
                    boat_op_invalid[i][j][k] = check_boat_op_invalid_(ppd{Point(i, j), Direction(k)});
                    boat_one_main[i][j][k] = check_boat_one_main_(ppd{Point(i, j), Direction(k)});
                }
    }
    void input_berth_() {
        int berth_num;
        scanf("%d", &berth_num);
        for (int i = 0; i < berth_num; i++) {
            int berth_id;
            scanf("%d", &berth_id);
            if (i != berth_id)
                throw std::runtime_error("berth_id error");
            int x, y, loading_speed;
            scanf("%d%d%d", &x, &y, &loading_speed);
            berths.emplace_back(berth_id, Point(x, y), loading_speed);
        }
    }
    void input() {
        // 输入地图
        input_map_();
        // 输入泊位
        input_berth_();
    }

    // preprocess
    void preprocess_() {
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
        // 地图针对陆地分块
        split_map_land();
    }

    void split_map_land() {
        init_book();
        int patches_num = 0;
        // 先手动选点
        std::queue<Point> points;
        // 对于小区域
        int zone_id = 0;
        for (auto p : small_area_points) {
            for(int i = 0; i < small_area_size / small_area_patch_size; i ++) {
                for(int j = 0; j < small_area_size / small_area_patch_size; j ++) {
                    Point np(p.x + small_area_patch_size / 2 + i * small_area_patch_size, p.y + small_area_patch_size / 2 + j * small_area_patch_size);
                    if (check_land_valid(np)) {
                        points.push(np);
                        book[np.x][np.y] = patches_num;
                        map_patches[patches_num].patch_id = patches_num;
                        map_patches[patches_num].center = np;
                        map_patches[patches_num].zone_id = zone_id;
                        patches_num++;
                    }
                }
            }
            zone_id ++;
        }
        // 对于大区域
        for (auto p : big_area_points) {
            for(int i = 0; i < big_area_size / big_area_patch_size; i ++) {
                for(int j = 0; j < big_area_size / big_area_patch_size; j ++) {
                    Point np(p.x + big_area_patch_size / 2 + i * big_area_patch_size, p.y + big_area_patch_size / 2 + j * big_area_patch_size);
                    if (check_land_valid(np)) {
                        points.push(np);
                        book[np.x][np.y] = patches_num;
                        map_patches[patches_num].patch_id = patches_num;
                        map_patches[patches_num].center = np;
                        map_patches[patches_num].zone_id = zone_id;
                        patches_num++;
                    }
                }
            }
            zone_id ++;
        }
        std::cerr << "patches_num: " << patches_num << std::endl;
        std::cerr << "zone_id: " << zone_id << std::endl;
        // 然后根据点进行bfs
        while (!points.empty()) {
            auto p = points.front();
            points.pop();
            auto patch_id = book[p.x][p.y];
            for (int k = 0; k < 4; k++) {
                auto np = p + Direction(k);
                if (check_land_valid(np)) {
                    if (!check_book(np)) {
                        points.push(np);
                        book[np.x][np.y] = patch_id;
                    } else {
                        auto np_patch_id = book[np.x][np.y];
                        if (np_patch_id != patch_id) {
                            patches_edge[patch_id].insert(np_patch_id);
                            patches_edge[np_patch_id].insert(patch_id);
                        }
                    }
                }
            }
        }
        // 统计patch信息
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                auto patch_id = book[i][j];
                if (patch_id != -1) {
                    map_patches[patch_id].size += 1;
                    map_patches[patch_id].max_good_num += 1 - check_land_main({i, j});
                    map_split[{i, j}] = patch_id;
                }
            }
        }
        // // 打印地图
        // for (int i = 0; i < n; i++) {
        //     for (int j = 0; j < n; j++) {
        //         std::cerr << book[i][j] << "\t";
        //     }
        //     std::cerr << std::endl;
        // }
        // // 打印边信息
        // for (auto& [patch_id, edges] : patches_edge) {
        //     std::cerr << "patch_id: " << patch_id << " edges: ";
        //     for (auto& edge : edges) {
        //         std::cerr << edge << " ";
        //     }
        //     std::cerr << std::endl;
        // }
        // // 打印patch信息
        // for (auto& [patch_id, patch] : map_patches) {
        //     std::cerr << "patch_id: " << patch_id << " size: " << patch.size << std::endl;
        // }
        if (patches_edge.size() != patches_num) {
            std::cerr << "patches_edge.size() != patches_num" << std::endl;
        }
        if (map_patches.size() != patches_num) {
            std::cerr << "map_patches.size() != patches_num" << std::endl;
        }
        // 取出map_patches key的最大值
        int max_patch_id = 0;
        for (auto& [patch_id, patch] : map_patches) {
            max_patch_id = std::max(max_patch_id, patch_id);
        }
        if (max_patch_id != patches_num - 1) {
            std::cerr << "max_patch_id != patches_num - 1" << std::endl;
        }
    }

    // ============================== is ==============================
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

    // ============================== check base ==============================
    // 是否超出了地图范围
    inline bool check_invalid(Point p) { return p.x < 0 || p.x >= n || p.y < 0 || p.y >= n; }
    // 是否在陆地上
    inline bool check_land_valid_(Point p) {
        return is_space(p) || is_land_main(p) || is_robot_purchase_point(p) || is_berth(p) || is_sea_land_traffic_point(p) ||
               is_sea_land_traffic_main(p);
    }
    inline bool check_land_valid(Point p) {
        if (check_invalid(p))
            return false;
        return land_valid[p.x][p.y];
    }
    inline bool check_land_invalid(Point p) { return !check_land_valid(p); }
    // 是否在海洋上
    inline bool check_ocean_valid_(Point p) {
        return is_ocean(p) || is_ocean_main(p) || is_boat_purchase_point(p) || is_berth(p) || is_near_berth_area(p) ||
               is_sea_land_traffic_point(p) || is_sea_land_traffic_main(p) || is_delivery_point(p);
    }
    inline bool check_ocean_valid(Point p) {
        if (check_invalid(p))
            return false;
        return ocean_valid[p.x][p.y];
    }
    inline bool check_ocean_invalid(Point& p) { return !check_ocean_valid(p); }
    // 是否在主干道上
    inline bool check_land_main_(Point p) {
        return is_land_main(p) || is_robot_purchase_point(p) || is_berth(p) || is_sea_land_traffic_main(p);
    }
    inline bool check_land_main(Point p) {
        if (check_invalid(p))
            return false;
        return land_main[p.x][p.y];
    }
    // 是否在主航道上
    inline bool check_ocean_main_(Point p) {
        return is_ocean_main(p) || is_boat_purchase_point(p) || is_berth(p) || is_near_berth_area(p) ||
               is_sea_land_traffic_main(p) || is_delivery_point(p);
    }
    inline bool check_ocean_main(Point p) {
        if (check_invalid(p))
            return false;
        return ocean_main[p.x][p.y];
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

    // ============================== check 船舶 ==============================
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

    bool check_boat_op_invalid_(ppd node) {
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
    std::vector<Point> get_boat_points(ppd node) {
        std::vector<Point> points;
        auto& [p1, dir1] = node;
        points.push_back(p1);
        for (int i = 0; i < 2; i++) {
            p1 += dir1;
            points.push_back(p1);
        }
        dir1 = clockwise_dir(dir1);
        p1 += dir1;
        points.push_back(p1);
        dir1 = clockwise_dir(dir1);
        for (int i = 0; i < 2; i++) {
            p1 += dir1;
            points.push_back(p1);
        }
        return points;
    }
    bool check_boat_op_invalid(ppd& node) { return boat_op_invalid[node.first.x][node.first.y][node.second]; }
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
    bool check_boat_one_main_(ppd node) {
        std::function<bool(Point)> func = [this](Point p) { return this->check_ocean_main(p); };
        return check_boat(node, func);
    }
    bool check_boat_one_main(ppd& node) { return boat_one_main[node.first.x][node.first.y][node.second]; }
    bool check_boat_all_main(ppd node) {
        std::function<bool(Point)> func = [this](Point p) { return !this->check_ocean_main(p); };
        return !check_boat(node, func);
    }
    bool check_boat_one_invalid(ppd node) {
        std::function<bool(Point)> func = [this](Point p) { return this->check_ocean_invalid(p); };
        return check_boat(node, func);
    }
    // ============================== 货物 ==============================
    // 添加货物
    void add_good(Point p, int val, int fade_time) {
        goods.emplace_back(p, val, fade_time);
        goods_map[p.x][p.y] = goods.back().id;
        if (val > GOOD_VAL_SPLIT && check_patch(p)) {
            auto& patch = get_patch(p);
            patch.high_goods_num++;
        }
        // precious_goods.insert(goods.back().id);
    }
    // 移除货物
    void remove_good_cached(const Point& p) {
        if (!check_good(p)) {
            std::cerr << "remove_good_cached error" << std::endl;
            return;
        }
        auto& good = get_good(p);
        if (good.fade_time - 1 > frame_id) {
            goods_get_val_sum.back() = double(good.val) + goods_get_val_sum.back();
        }
        cache_goods.insert(good.id);
    }
    void clear_cache_goods() {
        for (auto& i : cache_goods) {
            remove_good(i);
        }
        cache_goods.clear();
    }
    void remove_good(const Point& p) { goods_map[p.x][p.y] = -1; }
    void remove_good(const int& good_id) {
        auto& good = get_good(good_id);
        remove_good(good.p);
        if (good.val > GOOD_VAL_SPLIT) {
            auto& patch = get_patch(good.p);
            if (!good.is_fade())
                patch.high_goods_num--;
            // if (precious_goods.count(good.id)) {
            //     std::cerr << frame_id << ": should be removed before" << std::endl;
            //     precious_goods.erase(good.id);
            // }
        }
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
    // 是否有货物
    inline bool check_good(Point p) { return goods_map[p.x][p.y] != -1; }

    // ============================== 泊位 ==============================
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

    bool check_book(Point p) { return book[p.x][p.y] != -1; }

    bool check_patch(Point p) { return map_split.find(p) != map_split.end(); }
    MapPatch& get_patch(Point p) {
        if (map_split.find(p) == map_split.end()) {
            std::cerr << p.x << " " << p.y << std::endl;
            throw std::runtime_error("p is not in patch");
        }
        return map_patches[map_split[p]];
    }
};