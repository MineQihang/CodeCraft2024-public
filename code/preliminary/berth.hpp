#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"

int min_trans_time = 0x3f3f3f3f, min_trans_id, max_trans_time, trans_time_sum;

class Berth {
   public:
    Point p;
    Point p1;  // (1,1)
    int id;
    int transport_time;
    int new_transport_time;  // 允许中转的运输时间
    bool does_trans{};
    int loading_speed;
    int disable_time{max_int_};
    double available_robot_num{};
    std::queue<int> good_vals;
    std::deque<Point> booked_good;
    int val_sum{0}, goods_cnt{0}, val_sum_all{0}, num_sum{};
    // BerthStatus status{BerthStatus::EMPTY};
    BerthStatus status{BerthStatus::IDLE};
    double efficiency{0};
    double pred_num_per100{0};
    Berth() {}
    Berth(int x, int y, int transport_time, int loading_speed)
        : p(x, y), transport_time(transport_time), loading_speed(loading_speed) {}
    void init(int berth_id) {
        scanf("%d%d%d%d", &p.x, &p.y, &transport_time, &loading_speed);
        p1.x = p.x + 1;
        p1.y = p.y + 1;
        min_trans_time = std::min(min_trans_time, transport_time);
        if (transport_time == min_trans_time)
            min_trans_id = berth_id;
        max_trans_time = std::max(max_trans_time, transport_time);
        // trans_time_sum += transport_time;
        id = berth_id;
    }
    void add_good(int val) {
        good_vals.push(val);
        val_sum += val;
        goods_cnt++;
        num_sum++;
        val_sum_all += val;
    }
    void remove_good(int val) {}
    void set_status(BerthStatus status) { this->status = status; }
    void load_once_(int& goods_val, int& good_num, int& capacity) {
        int i = 0;
        for (; i < loading_speed; i++) {
            if (good_vals.empty() || good_num >= capacity)
                break;
            int tmp_val = good_vals.front();
            val_sum -= tmp_val;
            goods_cnt--;
            good_num++;
            goods_val += tmp_val;
            good_vals.pop();
        }
    }
    void update_efficiency(int frame_id) { efficiency = double(val_sum_all) / frame_id; }
};