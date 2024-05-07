#pragma once
#pragma GCC optimize("O2")
#include <bits/stdc++.h>

// ============================== 赛题参数 ==============================
const int MAP_SIZE = 800;
const int FRAME_NUM = 20000;
const int FADE_TIME = 1000;
const int VALUED_FADE_TIME = FRAME_NUM;
const int MAX_MAP_SIZE = 802;
const int BOAT_PRICE[] = {8000, 20000};
const int ROBOT_PRICE[] = {2000, 3000};
const int ROBOT_CAPACITY[] = {1, 2};
const int BOAT_LONG = 3;
const int BOAT_WIDTH = 2;
const int GOOD_VAL_SPLIT = 1000;

// ============================== 可调参数 ==============================
// 分区+货物
const int PATCH_SIZE = 50;
const int SEARCH_PATCH_STEP = 100;  // 搜索patch的步长
const int SEARCH_GOOD_STEP = 200;   // 搜索good的步长
const int patch_bfs_cnt_max = 1;    // patch搜索次数，正赛记得改大点
const int good_bfs_cnt_max = 2;     // 货物搜索次数，正赛记得改大点
// 购买
const int CANT_BUY_BOAT_TIME = 1500;  // 不能购买船的时间间隔
bool realtime_change_goto_good = false;
// 船舶
const bool manual_recursive_depth = 0;    // 手动控制递归搜索层数
const bool manual_stay_until = 0;         // 手动控制初始船运动时间
const bool manual_boat_num = 1;           // 测试中，手动控制船只数量，目前仅单区域
const bool manual_buy_boat_order = 1;     // 手动轮船购买顺序
const bool manual_init_efficiency = 1;    // 手动控制是否使用初始效率
int init_boat_num = 2;                    // 初始船只数量
const int boat_num_max = 12;              // 最大船只数量
bool boat_real_time_decide = 1;           // 船只实时决策
bool dulu = 0;                            // 堵路
const int occupy_time = 1000;             // 长期占用时间
const double mean_boat_time_rate = 0.16;  // 禁用会缩短mean_boat_time；有可能在来的路上。0-0.5
const double boat_col_time_rate = 1.09;   // 船舶碰撞时间倍率
const double rate_recursive_depth = 6;    // 时间倍率，乘的是平均交货距离
const double efficiency_decay = 0.85;     // 去泊位的距离系数（递归预测）
int stay_until = 0;                       // 初始船运动时间
bool using_init_efficiency = 0;           // 使用初始效率，防止unload 1个货物，开启船开始就动，有bug
std::unordered_map<std::string, int> stay_until_plan{{"map", 175}};
std::unordered_map<std::string, int> manual_boat_num_plan{{"map", 1}};
std::unordered_map<std::string, std::vector<int>> buy_boat_order_plan{{"map1", {3, 1, 2, 0}},
                                                                      {"mapj", {2, 7, 4, 5, 0, 6, 3, 2}}};
// 机器人
const int nearest_robots_num = 3;          // 最近机器人数量，决定是否选择货物
const int HISTORY_SIZE = 50;               // 机器人历史记录大小
const bool manual_robot_num = 1;           // 手动控制机器人数量，目前已无效
const bool manual_quickly_buy = 0;         // 手动控制尽快购买机器人
const int robot_init_num = 4;              // 初始机器人数量
int robot_type = 0;                        // 机器人类型
size_t robot_max_num = 50;                 // 预设机器人最大数量，待计算，统计berth预设8队12
const int search_range = PATCH_SIZE;       // 机器人搜索范围
const int robot_col_search_range = 10;     // 机器人碰撞搜索范围
int quickly_buy = 0;                       // 尽快购买机器人，为2时允许打断装货
const double efficiency_share_rate = 0.5;  // 机器人去其他泊位的效率衰减
const int SAFE_TIME = 5;                   // 碰撞可能产生的额外时间（最大能忍受的时间）,可尝试动态
const double dis_plus = 2;                 // 调大可避免捡便宜的货物（不建议），可做随机因子
const bool search_com_dis = 0;             // 加上去泊位距离
const int all_robots_time_plus = 200;      // 买好机器人后到达稳定状态的时间
std::unordered_map<std::string, int> manual_quickly_buy_lib{{"map", 0}};
// 机器人数量预设，船为1，种子为10。区域数量大于1时可不用设置
std::unordered_map<std::string, size_t> robot_max_num_lib{{"map", 14}};
// debug之类的参数
const bool is_random = 1;    // 避让随机，时间做种子 and 是否开启大模型
int p_id = -1;               // 程序id，可能与判题器给的id不同
const int global_info = 0;   // 打印更多信息
int liuyishou = 0;           // 多买的船只数量，1船=8000分
const bool debug_mode = 1;   // 牺牲时间换取更多信息
const bool zhan_wei_fu = 1;  // 占位符

const int a_star_trunc_steps = 16;  //
const bool test_capacity = 1;
// 基本不用调的参数
const bool using_weighted_efficiency = 1;  // 无可见影响
double mean_good_val_preset = 1000;        // 预设的平均货物价值
const int disable_start_time = 13000;
const int COL_CNT_MAX = 20;
uint32_t rand_uint = 0;

const int max_int_ = 0x3f3f3f3f;
const int const100000 = 100000;

int final_score = 25000;

std::unordered_map<int, std::string> hash2map{{5006, "map1"}, {3470, "mapj"}};

std::unordered_map<std::string, int> using_init_efficiency_plan{{"map", 0}};

// 机器人数量预设，区域分配，区域数量为1时可不用设置
std::unordered_map<std::string, std::map<int, int>> land_zone_robot_num_lib{{"map", {{0, 14}}}};

// 其他全局变量
// 距交货点平均距离 map1: 127, map2: 202, map3: 91
int average_berth_dis_one;
std::map<int, int> average_berth_dis;
int all_robots_time = FRAME_NUM + 1;  // 机器人购买完毕时间
// 地图名称
std::string map_name = "";
// 手动规划的船只计划
std::map<int, std::string> boat_plan[10];
// 最后的分数计算
int delivered_goods_val = 0;
// 加权帧数
double weighted_frames[FRAME_NUM + 1];
// 在去或在的泊位
std::set<int> boats_going_berths;
int robot_price;
int boat_col_zone_reverse;
int patch_bfs_cnt;                  // 货物搜索次数
int good_bfs_cnt;                   // 货物搜索次数
std::deque<int> goods_get_val_sum;  // 1000帧被拿货物的价值的估计

// 机器人最多等待时间
const int ROBOT_MAX_WAIT_TIME = 10;

// ============================== 全局变量 ==============================
int frame_id = 0;          // 当前frame_id
int current_money = 0;     // 当前钱数
int boat_capacity = 0;     // 船的容积
int boat_capacity1 = 0;    // 船的容积
int pulled_goods_num = 0;  // 已经放下的货物数量
int pulled_goods_val = 0;  // 已经放下的货物价值

// ============================== DEBUG参数 ==============================
const int SKIP_FRAME_LOG_LIMIT = 200;         // 跳帧日志限制
const std::string ERR_FILE_PATH = "./logs/";  // ERR输出路径
std::string err_path;
bool SKIP_FRAME_ABORT = 1;       // 跳帧是否直接退出
double LOG_TIME_THRESHOLD = 20;  // 每帧超时判断
bool global_debug = false;       // 是否打印log，在main函数中条件编译
bool LLM_DEBUG = false;          // 开启的话不会使用LLM，而是用本地
const int LLM_TIME_AVG = 1000;
const int LLM_TIME_STD = 500;
const double LLM_TEMPERATURE = 0;
std::vector<double> llm_answer_time;  // LLM回答时间
std::vector<bool> llm_answer_flag;    // LLM回答是否正确

std::vector<double> init_effitiency = {
    3.31452,  4.41962, 2.59165, 4.83918,  4.87632,  3.4297,   0.4547,   0.131,   0.44805, 3.7386,   7.27905,  0.239725,
    0.247175, 0.40005, 0.7487,  1.1147,   0.3423,   0.401325, 0.51795,  0.3493,  7.8309,  4.05452,  4.14168,  3.83155,
    3.28647,  4.20665, 7.89682, 4.44518,  0.30425,  0.357175, 0.385125, 0.1594,  0.81065, 0.626075, 0.788725, 0.293725,
    0.601325, 3.7946,  6.94487, 0.475425, 0.358375, 0.848175, 5.04948,  2.65955, 3.60078, 5.43785,  3.78852,  4.35473};
std::vector<double> init_num_effitiency = {
    0.0276,    0.0319,     0.0259334,  0.0392667,  0.0394,     0.0281,     0.00416666, 0.00233333, 0.00413333, 0.0257334,
    0.0411666, 0.00323333, 0.00243333, 0.00606666, 0.00656667, 0.00693333, 0.0028,     0.0024,     0.0036,     0.00283333,
    0.0468333, 0.0262,     0.0293334,  0.0321333,  0.0304,     0.0308,     0.0461667,  0.0263,     0.00346667, 0.00326667,
    0.003,     0.002,      0.00493334, 0.00546667, 0.00723334, 0.0028,     0.00406666, 0.0261666,  0.0406333,  0.00346666,
    0.0025,    0.00696667, 0.0443,     0.0256,     0.0297,     0.0421,     0.0320667,  0.0350667};

// ============================== 方向 ==============================
enum Direction { RIGHT = 0, LEFT = 1, UP = 2, DOWN = 3, STAY = 4 };

std::ostream& operator<<(std::ostream& os, const Direction& dir) {
    static const std::string dir_map[] = {"right", "left", "up", "down", "stay"};
    os << dir_map[dir];
    return os;
}

Direction reverse_dir(Direction dir) {
    static const Direction reverse_dir_map[] = {LEFT, RIGHT, DOWN, UP, STAY};
    return reverse_dir_map[dir];
}

Direction clockwise_dir(Direction dir) {
    static const Direction clockwise_dir_map[] = {DOWN, UP, RIGHT, LEFT, STAY};
    return clockwise_dir_map[dir];
}

Direction anticlockwise_dir(Direction dir) {
    static const Direction anticlockwise_dir_map[] = {UP, DOWN, LEFT, RIGHT, STAY};
    return anticlockwise_dir_map[dir];
}

void update_rand(int i = 0) {
    uint64_t tmp = rand_uint;
    tmp += i;
    rand_uint = tmp * 48271 % 2147483647;
}

int random_int(int min, int max) {
    update_rand();
    if (is_random) {
        uint64_t tmp = std::chrono::system_clock::now().time_since_epoch().count() + rand_uint;
        unsigned seed = tmp % 2147483647;
        // std::cerr << frame_id << ": seed: " << seed << std::endl;
        std::mt19937 gen(seed);
        std::uniform_int_distribution<int> distribution(min, max);
        int r = distribution(gen);
        // std::cerr << frame_id << ": random: " << r << std::endl;
        return r;
    }
    return round(min + (max - min) * (rand_uint / 2147483646.0));
}

Direction random_line_dir(int dir) {
    // i = 0: 返回竖直方向，i = 1: 返回水平方向
    int r01 = random_int(0, 1);
    return dir == 0 ? (r01 == 0 ? UP : DOWN) : (r01 == 0 ? LEFT : RIGHT);
}

// ============================== 点 ==============================
class Point {
   public:
    int x{-1};
    int y{-1};
    Point() {}
    Point(int x, int y) : x(x), y(y) {}
    bool operator<(const Point& other) const {
        if (x != other.x)
            return x < other.x;
        return y < other.y;
    }
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Point& other) const { return x != other.x || y != other.y; }
    friend Point operator+(const Point& a, const Direction& b) {
        if (b == RIGHT)
            return Point(a.x, a.y + 1);
        if (b == LEFT)
            return Point(a.x, a.y - 1);
        if (b == UP)
            return Point(a.x - 1, a.y);
        if (b == DOWN)
            return Point(a.x + 1, a.y);
        return a;
    }
    friend Point operator+=(Point& a, const Direction& b) {
        a = a + b;
        return a;
    }
};

struct CompareInt {
    bool operator()(const std::pair<int, std::pair<int, Point>>& a, const std::pair<int, std::pair<int, Point>>& b) {
        return a.first > b.first;
    }
};

namespace std {
template <>
struct hash<Point> {
    std::size_t operator()(const Point& p) const { return p.x * MAP_SIZE + p.y; }
};
}  // namespace std

std::vector<Point> dulu_points;

// ============================== 基础函数 ==============================
bool get_ok() {
    char okk[100];
    scanf("%s", okk);
    if (okk[0] == 'O' && okk[1] == 'K')
        return true;
    return false;
}

bool put_ok() {
    printf("OK\n");
    fflush(stdout);
    return true;
}

// 曼哈顿距离
int l1_dis(const Point& a, const Point& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// ============================== DEBUG函数 ==============================
int frame_skip_detection(int fid) {
    static int pre_frame_id = 0;
    static bool output_info = false;
    if (fid != ++pre_frame_id) {
        if (!output_info)
            std::cerr << "skipped_frames: ";
        int sf = fid - pre_frame_id;
        for (int i = pre_frame_id; i < fid; ++i)
            std::cerr << "(" << i << ") ";
        pre_frame_id = fid;
        output_info = true;
        return sf;
    }
    return 0;
}

// ============================== 船操作 ==============================
enum BoatOp { FORWARD = 0, CLOCKWISE = 1, ANTICLOCKWISE = 2, STAY_ = 3 };
std::ostream& operator<<(std::ostream& os, const BoatOp& op) {
    static const std::string op_map[] = {"forward", "clockwise", "anticlockwise", "stay"};
    os << op_map[op];
    return os;
}

// ============================== 计时器 ==============================
class Timer {
   public:
    std::stack<std::chrono::system_clock::time_point> start_time;
    Timer() { start_time.push(get_time_point()); }

    std::chrono::system_clock::time_point get_time_point() { return std::chrono::system_clock::now(); }
    void start() { start_time.push(get_time_point()); }
    void reset() {
        if (start_time.empty()) {
            throw std::runtime_error("reset timer failed");
        }
        start_time.pop();
    }
    double get_duration() {
        if (start_time.empty())
            return 0;
        return get_duration(start_time.top());
    }
    double get_duration(const std::chrono::system_clock::time_point& start) {
        auto now_time = get_time_point();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now_time - start).count();
        return duration / 1000.0;
    }
    void print_duration(const std::string& info = "", bool flag = true, double threshold = 0) {
        if (!flag)
            return;
        auto duration = get_duration();
        if (duration < threshold)
            return;
        duration = std::round(duration * 10) / 10;
        std::cerr << "[TIME] "
                  << "[" << frame_id << "] " << info << ": " << duration << "ms" << std::endl;
    }
    void print_duration_with_reset(const std::string& info = "", bool flag = true, double threshold = 0) {
        print_duration(info, flag, threshold);
        reset();
    }
} timer;

// patch
std::vector<Point> small_area_points{{90, 350},  {125, 125}, {125, 575}, {350, 90},
                                     {350, 610}, {575, 125}, {575, 575}, {610, 350}};
std::vector<Point> big_area_points{{260, 260}};
int small_area_size = 100;
int big_area_size = 280;
int small_area_patch_size = 20;
int big_area_patch_size = 40;