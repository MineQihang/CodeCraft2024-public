#pragma GCC optimize("O2")
#include "berth.hpp"
#include "boat.hpp"
#include "map.hpp"
#include "robot.hpp"
#include "scheduler.hpp"
#include "utils.hpp"

using namespace std;

vector<Robot> robots;
vector<Boat> boats;
Map mp(MAP_SIZE);
int change_good_num, last_buy_boat_time = -100000;
// DEBUG
int skip_frames, all_goods_val_sum, all_goods_num;
int good_not_space, good_same_place, robots_money_sum;
bool robots_num_log;
std::chrono::system_clock::time_point frame_start_time;
double time_sum;

void buy_boat();
void buy_robot();
void final_log();

void init() {
    // 输入地图和泊位
    mp.input();
    // 初始化船只容积
    scanf("%d", &boat_capacity);
    get_ok();
    // 预处理
    mp.preprocess();
    if (global_debug)
        cerr << "robot_max_num:" << robot_max_num /* << "\tboat_capacity:" << boat_capacity */ << "\tmap_name:" << map_name
             << endl;
    put_ok();
}

void interaction_input() {
    scanf("%d%d", &frame_id, &current_money);
    frame_start_time = get_time_point();  // 收到数据的时间
    scanf("%d", &change_good_num);
    for (int i = 0; i < change_good_num; i++) {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
#ifdef DEBUG
        if (val > 0) {
            if (mp.check_good({x, y})) {
                good_same_place++;
            }
            all_goods_val_sum += val;
            all_goods_num++;
            if (!mp.check_land_valid({x, y})) {
                good_not_space++;
            }
        }
#endif
        if (val > 0) {
            // 新增的
            mp.add_good({x, y}, val, frame_id + FADE_TIME);
        } else {
            // 消失/拿走的
            mp.remove_good({x, y});
        }
    }
    int robots_num;
    // std::cerr << "here0" << std::endl;
    scanf("%d", &robots_num);
    for (int i = 0; i < robots_num; i++) {
        int id, has_good, x, y;
        scanf("%d%d%d%d", &id, &has_good, &x, &y);
        robots[id].update(has_good, {x, y});
        // mp.update_robot(robots[i].p, robots[i].next_pos(1), i);
    }
    int boats_num;
    scanf("%d", &boats_num);
    for (int i = 0; i < boats_num; i++) {
        int id, goods_num, x, y, dir, status;
        scanf("%d%d%d%d%d%d", &id, &goods_num, &x, &y, &dir, &status);
        boats[id].update(goods_num, {x, y}, (Direction)dir, (BoatStatus)status);
    }
    boats_going_berths.clear();
    for (auto& boat : boats)
        if (boat.check_goto_berth() && boat.obj_berth_id != -1)  // 在去或在某个泊位
            boats_going_berths.insert(boat.obj_berth_id);
    // std::cerr << "here1" << std::endl;
    get_ok();
}

void interaction_output() {
    // if (frame_id > 1029) {
    //     put_ok();
    //     return;
    // }
    // std::cerr << "here2" << std::endl;
    auto start = get_time_point();
    mp.update();
    double duration = get_duration(start);
    if (global_debug && duration > log_time_threshold)
        std::cerr << frame_id << ": mp.update time: " << duration << "ms" << std::endl;
    // 机器人
    std::map<int, Direction> move_commands;
    for (int i = 0; i < (int)robots.size(); i++) {
        // std::cerr << "robot_id: " << i << std::endl;
        robots[i].do_op(mp, frame_id, move_commands);
    }
    start = get_time_point();
    if (robots.size())
        Scheduler::do_commands(move_commands, mp, robots);
    duration = get_duration(start);
    if (global_debug && duration > log_time_threshold)
        std::cerr << frame_id << ": do_commands time: " << duration << "ms" << std::endl;
    // 船舶
    start = get_time_point();
    set<int> do_schedule_boats;
    for (int i = 0; i < (int)boats.size(); i++) {
        if (boats[i].do_op(mp, robots.size())) {
            do_schedule_boats.insert(i);
        }
    }
    if (!(test_map && (map_name == "map1" || map_name == "map2" || map_name == "map3")))
        Scheduler::do_boats_commands(mp, boats, do_schedule_boats);
    duration = get_duration(start);
    if (global_debug && duration > log_time_threshold)
        std::cerr << frame_id << ": boats do_op time: " << duration << "ms" << std::endl;
    // 跳帧检测
    skip_frames += frame_skip_detection(frame_id);
    if (skip_frames && skip_frame_abort) {
        throw std::runtime_error("skip_frames");
    }
    // 打印结果
    if (global_debug && frame_id == FRAME_NUM) {
        // auto start = get_time_point();
        final_log();
        // 3.4ms
        // cerr << "final_log time: " << get_duration(start) << "ms" << endl;
    }
    buy_boat();
    buy_robot();
    put_ok();
}

int main() {
#ifdef DEBUG
    global_debug = true;
#ifdef LOG
    std::ofstream file_err(ERR_FILE_NAME);
    std::streambuf* oldCerrStreamBuf = std::cerr.rdbuf();
    std::cerr.rdbuf(file_err.rdbuf());
#endif  // LOG
#endif  // DEBUG
    try {
        auto start = get_time_point();
        double duration;
        init();
        if (global_debug)
            std::cerr << "init time: " << get_duration(start) << "ms" << std::endl;
        for (int frame = 1; frame <= FRAME_NUM; frame++) {
            // 输入
            interaction_input();
            duration = get_duration(frame_start_time);
            if (global_debug && duration > log_time_threshold)
                std::cerr << frame_id << ": interaction_input time: " << duration << "ms" << std::endl;
            // 输出
            interaction_output();
            frame = frame_id;
            duration = get_duration(frame_start_time);
            time_sum += duration;
            if (global_debug && duration > log_time_threshold)
                std::cerr << frame_id << ": time: " << duration << "ms" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
#ifdef DEBUG
#ifdef LOG
    std::cerr.rdbuf(oldCerrStreamBuf);
    file_err.close();
#endif  // LOG
#endif  // DEBUG
    return 0;
}

void buy_boat() {
    if (frame_id - last_buy_boat_time < 2000)
        return;
    for (auto& i : mp.ocean_zone_boat_num) {
        int zone_val_sum = 0;
        for (int& berth_id : mp.ocean_zone_to_berths[i.first]) {
            zone_val_sum += mp.berths[berth_id].val_sum;
        }
        if ((i.second > mp.ocean_zone_boat_num_now[i.first] ||
             (frame_id < 8000 && zone_val_sum > 20000 && !manual_boat_num)) &&
            current_money >= BOAT_PRICE) {
            current_money -= BOAT_PRICE;
            final_score -= BOAT_PRICE;
            last_buy_boat_time = frame_id;
            mp.ocean_zone_boat_num_now[i.first]++;
            Point purchase_point = mp.boat_purchase_points[i.first];
            boats.push_back(Boat(boats.size(), boat_capacity, purchase_point, frame_id));
            printf("lboat %d %d\n", purchase_point.x, purchase_point.y);
            if (mp.ocean_zone_boat_num_now[i.first] > 1) {
                recursive_depth_max_end = 1;
                if (global_debug)
                    cerr << frame_id << ": buy boat " << mp.ocean_zone_boat_num_now[i.first] << endl;
            }
            break;  // 1帧最多买1个船
        }
    }
}

void buy_robot() {
    while ((robots.size() < robot_max_num && current_money >= robot_price) ||
           (frame_id == 15000 && liuyishou && map_name == "map3")) {
        map<double, int> robot_purchase_ratio;
        for (auto& i : mp.land_zone_robots_num) {
            robot_purchase_ratio.insert({mp.land_zone_robots_num_now[i.first] / double(i.second), i.first});
        }
        int zone_id = robot_purchase_ratio.begin()->second;
        mp.land_zone_robots_num_now[zone_id]++;
        Point robot_purchase_point = mp.robot_purchase_points[mp.robot_purchase_ids[zone_id].front()];
        mp.robot_purchase_ids[zone_id].pop();
        current_money -= robot_price;
        final_score -= robot_price;
        robots_money_sum += robot_price;
        if (robot_type == 0) {
            robots.push_back(Robot(robots.size(), robot_purchase_point, 0, 1));
            printf("lbot %d %d 0\n", robot_purchase_point.x, robot_purchase_point.y);
        } else if (robot_type == 1) {
            robots.push_back(Robot(robots.size(), robot_purchase_point, 0, 2));
            printf("lbot %d %d 1\n", robot_purchase_point.x, robot_purchase_point.y);
        }
        if (frame_id == 15000) {
            liuyishou -= 2000;
        }
    }
    if (!robots_num_log && robots.size() == robot_max_num) {
        robots_num_log = true;
        all_robots_time = frame_id;
        if (global_debug)
            std::cerr << frame_id << ": all robots" << std::endl;
    }
}

void final_log() {
    // std::cerr << "final_log: " << std::endl;
    if (skip_frames)
        std::cerr << std::endl;
    if (skip_frames)
        std::cerr << "skipped frames number: " << skip_frames << std::endl;
    int val_sum_sum = 0, on_boats = 0;
    for (int i = 0; i < (int)mp.berths.size(); i++) {
        val_sum_sum += mp.berths[i].val_sum;
        std::cerr << "berth" << i << ",\tgoods_cnt: " << mp.berths[i].get_goods_num()
                  << ",\tval_sum: " << mp.berths[i].val_sum << ",\tefficiency: "
                  << mp.berths[i].efficiency
                  //   << ",\tpred_num_per100: " << mp.berths[i].pred_num_per100
                  << ",\tnum_per_100: " << mp.berths[i].good_num_all / 150.0 << std::endl;
    }
    for (int i = 0; i < (int)boats.size(); i++) {
        on_boats += boats[i].good_val;
    }
    // robots_ability - left_all + 25000 - 8000*boat = score
    std::cerr << "berth_val_sum: " << val_sum_sum << "\ton_boats: " << on_boats << "\tleft_all: " << val_sum_sum + on_boats
              << std::endl;
    cerr << "pulled_goods_num: " << pulled_goods_num << "\tpulled_goods_val: " << pulled_goods_val
         << "\trobots_ability: " << pulled_goods_val - robots_money_sum << "\tdelivered_goods_val: "
         << delivered_goods_val
         //  << "\taverage_pulled_goods_val: "
         //  << double(pulled_goods_val) / pulled_goods_num
         << endl;
    std::cerr << "all_goods_num: " << all_goods_num << "\tall_goods_val_sum: " << all_goods_val_sum << std::endl;
    if (good_not_space)
        std::cerr << "! good_not_space: " << good_not_space << std::endl;
    if (good_same_place)
        std::cerr << "! good_same_place: " << good_same_place << std::endl;
    std::cerr << "final_money: " << final_score << std::setprecision(3) << "\ttime: " << double(time_sum) / 1000 << "s"
              << std::endl;
}
