#pragma once
#pragma GCC optimize("O2")
#include <bits/stdc++.h>

// 可调参数
// 船舶
const bool manual_disable = 1;                // 初始手动禁用泊位，优先级高
const bool manual_recursive_depth = 1;        // 手动控制递归搜索层数
const bool manual_stay_until = 1;             // 手动控制初始船运动时间
const bool manual_move_at_beginning = 1;      // 手动控制开局船只是否开始就动
const bool manual_boat_num = 1;               // 测试中，手动控制船只数量，目前仅单区域
const bool manual_using_init_efficiency = 1;  // 手动控制是否使用初始效率。目前不管
const bool allow_disable = 1;                 // 是否禁用泊位，无明显效果
const bool auto_disable = 1;
const double mean_boat_time_rate = 0.16;       // 禁用会缩短mean_boat_time；有可能在来的路上。0-0.5
const int boat_col_time = 15;                 // 船舶碰撞预留时间
int recursive_depth_max = 1;                  // 递归搜索层数
int recursive_depth_max_end = 5;        // 最后启用递归以搜索局部最优，好像没用，可尝试动态，4-5
const double rate_recursive_depth = 6;  // 时间倍率，乘的是平均交货距离
const double efficiency_decay = 0.5;    // 去泊位的距离系数（递归预测），0.6
int stay_until = 175;                   // 初始船运动时间
int move_at_beginning = -1;             // berth_id，开启船开始就动
bool using_init_efficiency = 0;         // 使用初始效率，防止unload 1个货物，开启船开始就动，有bug
const bool using_val = 1;               // 1，预测时使用价值/数量
const double boat_dis_proportion = 1;   // 1
const bool bug_nan = 0;                 // 0，开启nan bug（影响船）
// 开局禁用，map2无稳定优化（可做随机因子）
std::unordered_map<std::string, std::vector<int>> disable_plan{{"map2", {2}}};
// 递归搜索层数，map2无禁用时应为1
std::unordered_map<std::string, int> recursive_depth_max_plan{{"map1", 4}, {"map2", 2}, {"map3", 2}, {"mapf1", 1}, {"mapf2", 1},{"", 1}};
std::unordered_map<std::string, int> stay_until_plan{{"map1", 175}, {"map2", 0}, {"map3", 0}, {"mapf1", 0}, {"mapf2", 0},{"",0}}; // 1:250
std::unordered_map<std::string, int> move_at_beginning_plan{{"map1", -1}, {"map2", -1}, {"map3", -1}, {"mapf1",1}, {"mapf2", -1}, {"", -1}};
std::unordered_map<std::string, int> manual_boat_num_plan{{"map1", 1}, {"map2", 1}, {"map3", 1}, {"mapf1", 2}, {"mapf2", 1}, {"", 1}};
// 机器人
const bool manual_robot_num = 1;           // 手动控制机器人数量
const bool manual_quickly_buy = 1;         // 手动控制尽快购买机器人
int robot_type = 0;                        // 机器人类型
size_t robot_max_num = 14;                 // 预设机器人最大数量，待计算
int quickly_buy = 0;                       // 尽快购买机器人，为2时允许打断装货
const double efficiency_share_rate = 0.5;  // 机器人去其他泊位的效率衰减
const int SAFE_TIME = 8;                   // 碰撞可能产生的额外时间（最大能忍受的时间）,可尝试动态
const double dis_plus = 2;                 // 调大可避免捡便宜的货物（不建议），可做随机因子
const bool is_random = 1;                  // 机器人避让随机
int r01 = 0;                               // 机器人避让随机数
int r03 = 0;                               // 机器人避让随机数
const int all_robots_time_plus = 200;      // 买好机器人后到达稳定状态的时间
std::unordered_map<std::string, int> manual_quickly_buy_lib{{"map1", 0}, {"map2", 0}, {"map3", 1}, {"mapf1", 0}, {"mapf2", 0}};
// 机器人数量预设，船为1，种子为10。区域数量大于1时可不用设置
std::unordered_map<std::string, size_t> robot_max_num_lib{{"map1", 14}, {"map2", 13}, {"map3", 17}, {"mapf1", 14}, {"mapf2", 14}};
// debug之类的参数
const int global_info = 1;  // 打印更多信息
int liuyishou = 0;
const bool skip_frame_abort = 1;
double log_time_threshold = 15;
const char* ERR_FILE_NAME = "C:/Users/Yipeng/Contest/CodeCraft2024/SDK/logs/info.log";
const bool test_map = 0;
const bool test_capacity = 1;
// 基本不用调的参数
const bool deliver_delay1 = 0;             // 交货点停1帧，理论上应该关闭
const bool using_weighted_efficiency = 1;  // 无可见影响
int recursive_del_num_max = 10;
double mean_good_val_preset = 88;  // 预设的平均货物价值
const int disable_start_time = 10000;
const int COL_CNT_MAX = 20;
// 不用调的参数
const int MAP_SIZE = 200;
const int FRAME_NUM = 15000;
const int FADE_TIME = 1000;
const int MAX_MAP_SIZE = 210;
const int HISTORY_SIZE = 50;
const int SKIP_FRAME_LOG_LIMIT = 200;
const int BOAT_PRICE = 8000;
const int ROBOT_PRICE_LOW = 2000;
const int ROBOT_PRICE_HIGH = 5000;
const int BOAT_LONG = 3;
const int BOAT_WIDTH = 2;
const int max_int_ = 0x3f3f3f3f;
const int const100000 = 100000;
bool global_debug = 0;  // 是否打印log，在main函数中条件编译
int final_score = 25000;

std::unordered_map<int, std::string> hash2map{{1525, "map1"},
                                              {4769, "map2"},
                                              {2304, "map3"},
                                              {3244, "mapf1"},
                                              {2506, "mapf2"}};

std::unordered_map<std::string, int> using_init_efficiency_plan{{"map1", 0}, {"map2", 0}, {"map3", 0}};

// 机器人数量预设，区域分配，区域数量为1时可不用设置
std::unordered_map<std::string, std::map<int, int>> land_zone_robot_num_lib{{"map1", {{0, 14}}},
                                                                            {"map2", {{0, 14}}},
                                                                            {"map3", {{0, 14}}}};

// 其他全局变量
// 距交货点平均距离 map1: 127, map2: 202, map3: 91
int average_berth_dis_one;
std::map<int, int> average_berth_dis;
int frame_id = 0, current_money = 0, boat_capacity;
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

enum Direction { RIGHT = 0, LEFT = 1, UP = 2, DOWN = 3, STAY = 4 };
std::ostream& operator<<(std::ostream& os, const Direction& dir) {
    switch (dir) {
        case RIGHT:
            os << "right";
            break;
        case LEFT:
            os << "left";
            break;
        case UP:
            os << "up";
            break;
        case DOWN:
            os << "down";
            break;
        case STAY:
            os << "stay";
            break;
        default:
            os << "unknown";
            break;
    }
    return os;  // 支持链式调用
}

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

namespace std {
template <>
struct hash<Point> {
    std::size_t operator()(const Point& p) const { return p.x * MAP_SIZE + p.y; }
};
}  // namespace std

// Direction random_dir() {
//     int r = r03 % 4;
//     r03 = (r03 + 1) % 4;
//     return Direction(r);
// }

Direction reverse_dir(Direction dir) {
    if (dir == UP)
        return DOWN;
    if (dir == DOWN)
        return UP;
    if (dir == LEFT)
        return RIGHT;
    if (dir == RIGHT)
        return LEFT;
    return STAY;
}

Direction clockwise_dir(Direction dir) {
    if (dir == RIGHT)
        return DOWN;
    if (dir == DOWN)
        return LEFT;
    if (dir == LEFT)
        return UP;
    if (dir == UP)
        return RIGHT;
    return STAY;
}

Direction anticlockwise_dir(Direction dir) {
    if (dir == RIGHT)
        return UP;
    if (dir == UP)
        return LEFT;
    if (dir == LEFT)
        return DOWN;
    if (dir == DOWN)
        return RIGHT;
    return STAY;
}
enum RobotExtraStatus { GOTOBERTH = 0, GOTOGOOD = 1, WAITING = 2, DISABLED = 3 };

enum BoatStatus { MOVE = 0, RECOVER = 1, LOAD = 2 };

enum BoatExtraStatus { WAITING_ = 0, GOTOBERTH_ = 1, GOTODELIVERY = 2 };

enum BerthStatus {
    IDLE = 0,      // 空闲：当前没有船只停靠/前往
    BUSY = 1,      // 忙碌：当前有船只停靠装载
    DISABLED_ = 2  // 禁用：当前泊位不可用
};

enum GoodStatus {
    EXIST = 0,     // 存在：当前货物存在
    SELECTED = 1,  // 选中：当前货物已经被某个机器人选中作为目标
    CARRIED = 2,   // 捡起：当前货物已经被捡起
    FADE = 3,      // 消失：当前货物已经消失
    DONE = 4       // 完成：当前货物已经被放到泊位
};

bool get_ok() {
    char okk[100];
    scanf("%s", okk);
    if (okk[0] == 'O' && okk[1] == 'K') {
        return true;
    }
    return false;
}

bool put_ok() {
    printf("OK\n");
    fflush(stdout);
    return true;
}

int frame_skip_detection(int fid) {
    static int pre_frame_id = 0;
    static bool output_info = false;
    if (fid != ++pre_frame_id) {
        if (!output_info)
            std::cerr << "skipped_frames: ";
        int sf = fid - pre_frame_id;
        for (int i = pre_frame_id; i < fid; ++i)
            std::cerr << i << " ";
        pre_frame_id = fid;
        output_info = true;
        return sf;
    }
    return 0;
}

// JJH
Direction random_line_dir(int i) {
    // int r = distrib01(gen);
    r01++;
    if (r01 > 1)
        r01 = 0;
    int r = r01;
    if (i == 0) {
        if (r == 0)
            return UP;
        if (r == 1)
            return DOWN;
    } else {
        if (r == 0)
            return RIGHT;
        if (r == 1)
            return LEFT;
    }
    return STAY;
}

enum BoatOp { FORWARD = 0, CLOCKWISE = 1, ANTICLOCKWISE = 2, STAY_ = 3 };
std::ostream& operator<<(std::ostream& os, const BoatOp& op) {
    switch (op) {
        case FORWARD:
            os << "forward";
            break;
        case CLOCKWISE:
            os << "clockwise";
            break;
        case ANTICLOCKWISE:
            os << "anticlockwise";
            break;
        case STAY_:
            os << "stay";
            break;
        default:
            os << "unknown";
            break;
    }
    return os;  // 支持链式调用
}

using ppd = std::pair<Point, Direction>;
struct ppd_hash {
    std::size_t operator()(const ppd& p) const {
        std::size_t h1 = std::hash<Point>{}(p.first);
        std::size_t h2 = p.second;
        return h1 * 5 + h2;  // 注意Direction最大值是5（也可以是4）
    }
};

const ppd default_ppd = {{-1, -1}, STAY};

using pip = std::pair<int, ppd>;
using pii = std::pair<int, int>;

// 想要不疯狂旋转需要将比较函数改为std::greater<pip>
struct ComparePip {
    bool operator()(const pip& a, const pip& b) const { return a.first > b.first; }
};

ppd next_ppd(ppd now, BoatOp op) {
    if (op == FORWARD) {
        now.first += now.second;
    } else if (op == CLOCKWISE) {
        now.first = now.first + now.second + now.second;
        now.second = clockwise_dir(now.second);
    } else if (op == ANTICLOCKWISE) {
        now.first = now.first + now.second + clockwise_dir(now.second);
        now.second = anticlockwise_dir(now.second);
    }
    return now;
}

ppd next_ppd_rev(ppd now, BoatOp op) {
    if (op == FORWARD) {
        now.first += reverse_dir(now.second);
    } else if (op == CLOCKWISE) {
        now.second = anticlockwise_dir(now.second);
        now.first = now.first + reverse_dir(now.second) + reverse_dir(now.second);
    } else if (op == ANTICLOCKWISE) {
        now.second = clockwise_dir(now.second);
        now.first = now.first + reverse_dir(now.second) + anticlockwise_dir(now.second);
    }
    return now;
}

std::chrono::system_clock::time_point get_time_point() {
    return std::chrono::system_clock::now();
}

// 返回1位小数的毫秒间隔
double get_duration(std::chrono::system_clock::time_point start) {
    return double((std::chrono::duration_cast<std::chrono::microseconds>(get_time_point() - start).count() + 50) / 100) / 10;
}