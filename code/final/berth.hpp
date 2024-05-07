#pragma once
#pragma GCC optimize("O2")
#include "boat_pos.hpp"
#include "utils.hpp"

enum BerthStatus {
    IDLE = 0,      // 空闲：当前没有船只停靠/前往
    BUSY = 1,      // 忙碌：当前有船只停靠装载
    DISABLED_ = 2  // 禁用：当前泊位不可用
};

class Berth {
   public:
    // 泊位的id
    int id;
    // 泊位起始点（左上角）
    Point p;
    // 上货速度：每一帧上货的数量
    int loading_speed;
    // 当前所有的货物
    std::queue<int> good_vals;
    double good_num_weighted{0};
    // 当前所有货物的价值总和
    int val_sum{0};
    // 截至目前在本泊位的所有货物的价值总和
    int val_sum_all{0};
    // 截至目前在本泊位的所有货物的价值总和的加权和
    double val_sum_all_weighted{0};
    // 截至目前在本泊位的所有货物的数量总和
    int good_num_all{0};
    // 泊位的状态
    BerthStatus status{BerthStatus::IDLE};
    // 以该泊位为目的地的船只的id
    std::set<int> obj_boat_ids;
    // 以该泊位为目的地的机器人id
    std::set<int> obj_robot_ids;
    // 泊位产生货物的价值效率
    double efficiency{0};
    double num_efficiency{0};
    // 泊位产生货物的加权价值效率(可分两段计算)
    double weighted_efficiency{0};
    double weighted_num_efficiency{0};
    double available_robot_num{};  // 用于开始时预估效率
    double pred_num_per100{};      // 用于开始时预估效率
    // 最邻近交货点
    int nearest_delivery_id{-1};
    int nearest_delivery_dis{max_int_};
    ppd boat_ppd;  // dept后船的位置
    // int first_boat_time{0};  // 第1次有船停靠的时间
    // int boat_cnt{0};         // 有几次停靠
    // double mean_boat_time{15000};
    // 当前在泊位上的船的public_id
    int current_boat_id{-1};
    bool is_ours_boat{false};
    std::set<int> near_boat_ids;        // 在靠泊区的所有船
    int nearest_our_going_boat_id{-1};  // 最近的在去的我方非装载船
    int nearest_our_boat_id{-1};        // 最近的我方非满船

    Berth() {}
    Berth(int id, Point p, int loading_speed) : id(id), p(p), loading_speed(loading_speed) {}
    // Berth(const Berth&) = delete;
    // Berth& operator=(const Berth&) = delete;

    // 添加货物
    void add_good(int val) {
        val_sum += val;
        val_sum_all += val;
        val_sum_all_weighted += val;
        good_num_weighted += 1;
        good_num_all++;
        good_vals.push(val);
    }
    // 移除货物
    int remove_good() {
        if (good_vals.empty())
            return -1;
        auto val = good_vals.front();
        val_sum -= val;
        good_vals.pop();
        return val;
    }
    // 泊位状态
    void set_status(BerthStatus status) { this->status = status; }
    void set_idle() { status = BerthStatus::IDLE; }
    void set_busy() { status = BerthStatus::BUSY; }
    void set_disabled() { status = BerthStatus::DISABLED_; }
    bool is_empty() { return good_vals.empty(); }
    // TODO: 在多个船到达同一个地方时会出问题
    bool is_idle() { return status == BerthStatus::IDLE; }
    bool is_busy() { return status == BerthStatus::BUSY; }
    bool is_disabled() { return status == BerthStatus::DISABLED_; }
    bool is_selected() { return !obj_boat_ids.empty(); }
    // 泊位与船只的关系
    void add_our_boat(int boat_id) { obj_boat_ids.insert(boat_id); }
    void remove_our_boat(int boat_id) {
        if (obj_boat_ids.count(boat_id))
            obj_boat_ids.erase(boat_id);
    }
    // 泊位与机器人的关系，没用
    void add_robot(int robot_id) { obj_robot_ids.insert(robot_id); }
    void remove_robot(int robot_id) { obj_robot_ids.erase(robot_id); }
    auto get_robot_num() { return obj_robot_ids.size(); }
    // 进行一次装载
    void load_once(int& good_num, int& good_val, const int& capacity) {
        for (int i = 0; i < loading_speed; i++) {
            if (good_vals.empty() || good_num >= capacity)
                break;
            auto val = remove_good();
            if (val == -1) {
                throw std::runtime_error("remove_good failed");
            }
            good_val += val;
            good_num++;
        }
    }
    // 返回现在有多少货物
    int get_goods_num() { return good_vals.size(); }
    // 更新泊位的效率
    void update_efficiency(double share_efficiency, double share_num_efficiency, int working_berths_num) {
        efficiency = (double)val_sum_all / (double)frame_id;
        num_efficiency = (double)good_num_all / (double)frame_id;
        if (frame_id % 1000 == 0) {
            // if (frame_id == 1000) {
            weighted_efficiency =
                weighted_efficiency * 0.2 + val_sum_all_weighted / 1000.0 * 0.2 + init_effitiency[id] * 0.6;
            weighted_num_efficiency =
                weighted_num_efficiency * 0.2 + good_num_weighted / 1000.0 * 0.2 + init_num_effitiency[id] * 0.6;
            // } else {
            //     weighted_efficiency = weighted_efficiency * 0.6 + val_sum_all_weighted / 1000.0 * 0.4;
            //     weighted_num_efficiency = weighted_num_efficiency * 0.6 + good_num_weighted / 1000.0 * 0.4;
            // }
            val_sum_all_weighted = 0;
            good_num_weighted = 0;
        }
        if (!manual_init_efficiency && frame_id < 1000) {
            weighted_efficiency = efficiency;
            weighted_num_efficiency = num_efficiency;
        }
        // if (id == 0 && frame_id % 1000 == 0) {
        //     std::cerr << "efficiency: " << efficiency << "\tweighted_efficiency: " << weighted_efficiency << std::endl;
        //     std::cerr << "num_efficiency: " << num_efficiency << "\tweighted_num_efficiency: " << weighted_num_efficiency
        //               << std::endl;
        // }
        if (using_init_efficiency && frame_id < 400) {
            weighted_efficiency += pred_num_per100 / 50 * (mean_good_val_preset / 100);
            weighted_num_efficiency += pred_num_per100 / 50;
        }
        if (is_disabled()) {
            weighted_efficiency = 0;
            weighted_num_efficiency = 0;
        }
    }
    // void up_boat_data() {
    //     if (frame_id > all_robots_time + all_robots_time_plus) {
    //         boat_cnt++;
    //         if (first_boat_time == 0)
    //             first_boat_time = frame_id;
    //         else
    //             mean_boat_time = (frame_id - first_boat_time) / boat_cnt;
    //     }
    // }
};