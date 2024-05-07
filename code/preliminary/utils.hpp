#pragma once
#pragma GCC optimize("O2")
#include <bits/stdc++.h>

const int MAP_SIZE = 200;
const int ROBOT_NUM = 10;
const int BERTH_NUM = 10;
const int BOAT_NUM = 5;
const int FRAME_NUM = 15000;
const int FADE_TIME = 1000;
const int MAX_MAP_SIZE = 202;
const int HISTORY_SIZE = 50;
const int DETECT_RANGE = 3;
const int COLLISION_MODE = 1;
const int time_in_berth_move = 500;
const int max_int_ = 0x3f3f3f3f;
const int good_select = 1;  // 为泊位分配商品
const int first_search_scale = 8;
const int seed = -1;
const bool is_random = 1;
const bool skip_frame_detect = 1;
bool manual_plan = 1;
bool manual_disable = manual_plan;
const bool select_nearby_at_beginning = 1;
const bool manual_init_robot = 0;  // 没用甚至少了几百分
const bool fixed_berth = 0;
bool get_data = 0;
std::string map_name = "";
int col_avoid_time = 6;  // 0、10都没5好(差距小于1k)
bool oringal_test = 0;
int map_cnt = 0;

std::unordered_map<std::string, std::vector<std::pair<int, int>>> disable_plan;

// 不知道为什么不能在这里初始化
std::unordered_map<std::string, std::vector<int>> boat_plan_init;

std::unordered_map<std::string, std::vector<int>> init_robot_plan;

// 每个boat是否进行港间ship
std::unordered_map<std::string, std::vector<int>> boat_in_berth_ship;

std::map<int, std::string> boat_plan[10];  // 手动规划的船只计划

std::unordered_map<int, std::string> oringal_map = {{880, "map1"}, {206, "map2_3.9"}, {766, "3.7"},  {25, "3.8"},
                                                    {200, "3.10"}, {153, "3.11"},     {824, "3.12"}, {296, "3.13"}};
std::unordered_map<std::string, int> map_cnt_up = {{"map1", 10}, {"map2_3.9", 20}, {"3.7", 30},  {"3.8", 40},
                                                   {"3.10", 50}, {"3.11", 60},     {"3.12", 70}, {"3.13", 80}};

// std::mt19937 gen(42); // 使用常量作为种子
// std::uniform_int_distribution<> distrib01(0, 1);
// std::uniform_int_distribution<> distrib04(0, 4);
int r01 = 0;
int r04 = 0;

enum Direction : uint8_t { RIGHT = 0x0, LEFT = 0x1, UP = 0x2, DOWN = 0x3, STAY = 0x4 };

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
    int x;
    int y;
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
};

namespace std {
template <>
struct hash<Point> {
    std::size_t operator()(const Point& p) const { return p.x * MAP_SIZE + p.y; }
};
}  // namespace std

Direction random_dir() {
    // int r = distrib04(gen);
    r04++;
    if (r04 > 4)
        r04 = 0;
    int r = r04;
    // std::cerr << " r:" << r;
    if (r == 0)
        return RIGHT;
    if (r == 1)
        return LEFT;
    if (r == 2)
        return UP;
    if (r == 3)
        return DOWN;
    return STAY;
}

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

Direction reverse_dir(Direction dir) {
    if (dir == UP)
        return DOWN;
    if (dir == DOWN)
        return UP;
    if (dir == LEFT)
        return RIGHT;
    if (dir == RIGHT)
        return LEFT;
    if (dir == STAY)
        return STAY;
    std::cerr << "reverse_dir error!" << std::endl;
    return STAY;
}

enum RobotStatus { RECOVER = 0, NORMAL = 1 };

enum BoatStatus { MOVE = 0, WORK = 1, WAIT = 2 };

enum BerthStatus { IDLE = 0, BUSY = 1 };

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

int frame_skip_detection(int frame_id) {
    static int pre_frame_id = 0;
    static bool output_info = false;
    if (frame_id != ++pre_frame_id) {
        if (!output_info)
            std::cerr << "skipped_frames: ";
        int skip_frames = frame_id - pre_frame_id;
        for (int i = pre_frame_id; i < frame_id; ++i)
            std::cerr << i << " ";
        pre_frame_id = frame_id;
        output_info = true;
        return skip_frames;
    }
    return 0;
}