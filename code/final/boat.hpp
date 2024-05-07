#pragma once
#pragma GCC optimize("O2")
#include "boat_base.hpp"

class Boat : public BaseBoat {
   public:
    // 船只的private_id
    int private_id{-1};
    // 船只要去的泊位的id
    int obj_berth_id{-1};
    // 船只要去的交货点id
    int obj_delivery_id{-1};
    // 船只额外的状态
    BoatExtraStatus extra_status{BoatExtraStatus::GOTOBERTH_};
    // 船只的规划
    BoatSchedule schedule;
    // 上次离开交货点或购买点的时间
    int last_time{1};
    ppd nxt_ppd = {{-1, -1}, STAY};  // 下一个位置
    int debug_times{};
    bool has_quick_berth_schedule{};

    Boat() {}
    Boat(BaseBoat* base, int private_id)
        : BaseBoat(base->public_id, base->good_num, base->p, base->dir, base->status, base->capacity),
          private_id(private_id),
          last_time(frame_id) {}
    Boat(const Boat&) = delete;
    Boat& operator=(const Boat&) = delete;

    // 每帧的操作，返回值代表是否需要执行schedule
    int do_op(Map& mp, size_t robots_num, int our_boats_num, int all_boats_num, std::vector<int>& boat_goods_num) {
        if (frame_id <= stay_until)
            return 0;
        if (check_goto_delivery() && schedule.check_invalid() && check_recover())
            if (mp.check_delivery_point(p))
                unload(mp);
        // 如果在恢复，不应该执行任何指令
        if (check_recover())
            return 0;
        if (check_load())
            mp.get_berth(obj_berth_id).remove_our_boat(private_id);
        if (nxt_ppd.second != STAY && nxt_ppd != ppd{p, dir}) {
            mp.col_boats.insert(private_id);
            schedule.recover_op();
            nxt_ppd = {p, dir};
        }
        // 逐帧调度
        if (boat_real_time_decide)
            get_plan(mp, robots_num, boat_goods_num);
        // 在去泊位
        if (check_goto_berth() && check_move()) {
            // 到达无船泊位
            if (obj_berth_id != -1 && mp.berth_near_area.count(p) && mp.berth_near_area[p] == obj_berth_id &&
                mp.get_berth(obj_berth_id).current_boat_id == -1) {
                do_berth();
                schedule.clear();
                has_quick_berth_schedule = 0;
                return 0;
            }
            // 到达泊位
            else if (obj_berth_id != -1 && mp.get_berth(obj_berth_id).p == p && mp.berth_direction[obj_berth_id] == dir) {
                // std::cerr << frame_id << ": boat " << private_id << " arrive at berth " << obj_berth_id << std::endl;
                if (mp.occupied_berths.count(obj_berth_id))
                    get_plan(mp, robots_num, boat_goods_num);
                else {
                    do_berth();
                    schedule.clear();
                    return 0;
                }
            }
            // 无路径规划时
            else if (schedule.check_invalid()) {
                if (mp.check_boat_purchase_point(p) && dir == RIGHT) {  // 购买点
                    get_plan(mp, robots_num, boat_goods_num);
                    // std::cerr << frame_id << ": boat " << private_id << " go from purchase point" << std::endl;
                }
                if (obj_berth_id != -1) {
                    schedule =
                        mp.get_boat_schedule_a_star({p, dir}, {mp.berths[obj_berth_id].p, mp.berth_direction[obj_berth_id]});
                    // if (frame_id >= 800 && frame_id <= 1000)
                    //     std::cerr << frame_id << ": boat " << private_id << " go to berth " << obj_berth_id << " "
                    //               << schedule.get_op(false) << std::endl;
                }
            }
            // 快速berth
            if (!has_quick_berth_schedule && obj_berth_id != -1 && mp.get_berth(obj_berth_id).current_boat_id == -1 &&
                l1_dis(p, mp.get_berth(obj_berth_id).p) <= 13) {
                timer.start();
                schedule = mp.get_boat_schedule({p, dir}, obj_berth_id);
                if (schedule.check_invalid())
                    std::cerr << "err2482" << std::endl;
                has_quick_berth_schedule = 1;
                timer.print_duration_with_reset("quick_berth", global_debug, LOG_TIME_THRESHOLD * 0.2);
            }
        }
        // 去交货点
        if (check_goto_delivery() && check_move() && schedule.check_invalid()) {
            if (mp.check_delivery_point(p)) {
                last_time = frame_id;
                get_plan(mp, robots_num, boat_goods_num);
                if (check_goto_berth()) {
                    schedule =
                        mp.get_boat_schedule_a_star({p, dir}, {mp.berths[obj_berth_id].p, mp.berth_direction[obj_berth_id]});
                }
            } else {
                schedule = mp.get_boat_schedule_a_star({p, dir}, {mp.delivery_points[obj_delivery_id], STAY});
            }
        }
        // 如果是在运动的
        if (check_move() && !schedule.check_invalid()) {
            // 返回1,表示会执行Schedule中的操作
            return 1;
        }
        // 如果装满了或没货了，去其他地方
        if (check_load()) {
            auto& berth = mp.get_berth(obj_berth_id);
            // if ((quickly_buy == 2 || (quickly_buy == 1 && (check_full() || berth.is_empty()))) &&
            //     robots_num < robot_max_num && current_money + good_val >= robot_price) {
            //     do_dept();
            //     set_goto_delivery();
            //     mp.berths[obj_berth_id].set_idle();
            //     obj_delivery_id = berth.nearest_delivery_id;
            //     return 0;
            // }
            // 优解搜索
            // ||berth.is_empty()
            if (check_full() || our_boats_num == 1) {
                int tag = get_plan(mp, robots_num, boat_goods_num);
                if (tag == 1)
                    return 0;
            }
        }
        // 如果是在装货
        if (check_load()) {
            auto& berth = mp.get_berth(obj_berth_id);
            // fram_id+1出发，nearest_delivery_dis-1产生价值，最后1帧不产生价值
            if (frame_id + berth.nearest_delivery_dis * 1.2 >= FRAME_NUM) {
                do_dept();
                set_goto_delivery();
                obj_delivery_id = berth.nearest_delivery_id;
                if (global_debug)
                    std::cerr << frame_id << ": boat " << private_id << " no time no load" << std::endl;
                return 0;
            }
            // berth.load_once(good_num, good_val, capacity);
        }
        return 0;
    }

    // 获取下一个目的地, 设置状态，返回值代表本函数是否输出了指令
    int get_plan(Map& mp, size_t robots_num, std::vector<int>& boat_goods_num) {
        int my_return = 0;
        double max_time_val = -1;
        int next_plan = -1;
        ppd from = {p, dir};
        // 去交货点(必须有货)
        if (good_num > 0) {
            for (size_t i = 0; i < mp.delivery_points.size(); i++) {
                // TODO:不可达判断
                int dis_tmp = mp.delivery_distance[i][p.x][p.y] * boat_col_time_rate + 1;
                int adjusted_good_val = good_val * (1.0 * good_num / capacity);  // 可调参
                if (robots_num < robot_max_num)
                    adjusted_good_val = good_val;
                double val_tmp = get_val_re(mp, i + const100000, good_num, good_num, adjusted_good_val, adjusted_good_val,
                                            frame_id + dis_tmp, 1, 0);
                if (val_tmp > max_time_val) {
                    max_time_val = val_tmp;
                    next_plan = i + const100000;
                }
            }
        }
        // 去泊位
        for (size_t i = 0; i < mp.berths.size(); i++) {
            if (good_num >= capacity)
                continue;
            // TODO:不可达判断
            auto& target_berth = mp.get_berth(i);
            // if(target_berth.is_busy())
            //     continue;
            if (!boat_real_time_decide && target_berth.is_selected() && target_berth.obj_boat_ids.count(private_id) == 0)
                continue;
            if (boat_real_time_decide && target_berth.nearest_our_going_boat_id != -1 &&
                target_berth.nearest_our_going_boat_id != private_id)
                continue;
            if (boat_real_time_decide && check_goto_berth() && obj_berth_id == i &&
                target_berth.nearest_our_going_boat_id != -1 && target_berth.nearest_our_going_boat_id != private_id &&
                !(check_load() && obj_berth_id == i))
                continue;
            if (boat_real_time_decide && check_load() && target_berth.nearest_our_boat_id != private_id)
                continue;
            // if (target_berth.is_selected() && obj_berth_id != i && !check_goto_delivery())
            //     continue;
            // if (mp.occupied_berths.count(i) && !(check_load() && obj_berth_id == i))
            //     continue;
            double dis_tmp = mp.berths_ocean_distance[i][p.x][p.y] * boat_col_time_rate + 1;
            double waiting_time = 0;
            if (target_berth.current_boat_id != -1 && target_berth.current_boat_id != public_id) {
                if (!target_berth.near_boat_ids.empty() && *target_berth.near_boat_ids.begin() < public_id)
                    continue;  // 抢不过
                if (target_berth.weighted_num_efficiency == 0)
                    continue;
                waiting_time =
                    double(capacity - boat_goods_num[target_berth.current_boat_id]) / target_berth.weighted_num_efficiency;
                if (waiting_time < dis_tmp)
                    waiting_time = capacity / target_berth.weighted_num_efficiency * 0.35;
            }
            double get_rate = 1;
            if (mp.nearest_boat[i] != -1 && mp.nearest_boat[i] != public_id) {
                get_rate = 0;
            }
            double num_pred = capacity;  // 不考虑路上产生的货物
            if (num_pred == 0)
                continue;
            int share_num = mp.our_loading_boats_num + 1;
            double others_trans_decay = 0.85, our_trans_decay = 0.85;
            // double others_trans_decay = 1, our_trans_decay = 1;
            double existing_num =
                get_rate * target_berth.good_vals.size() + our_trans_decay * mp.our_robots_goods_num / share_num;
            // double existing_num = get_rate * target_berth.good_vals.size();
            double existing_val = get_rate * target_berth.val_sum + our_trans_decay * mp.our_robots_goods_val / share_num;
            // double existing_val = get_rate * target_berth.val_sum;
            double val_pred = 0;
            // 考虑自家机器人
            double val_efficiency_pred =
                target_berth.weighted_efficiency * others_trans_decay + mp.our_robots_efficiency / share_num;
            // double val_efficiency_pred = target_berth.weighted_efficiency * others_trans_decay + 1.5;
            double num_efficiency_pred =
                target_berth.weighted_num_efficiency * others_trans_decay + mp.our_robots_num_efficiency / share_num;
            // double num_efficiency_pred = target_berth.weighted_num_efficiency * others_trans_decay + 0.001;
            // 不考虑路上产生的货物
            val_pred += existing_val + val_efficiency_pred * ((capacity - existing_num) / num_efficiency_pred);
            double new_num_pred = std::min(double(capacity - good_num), num_pred);
            int new_val_pred = round(val_pred * new_num_pred / num_pred);
            auto load_time = existing_num / target_berth.loading_speed + (new_num_pred - existing_num) / num_efficiency_pred;
            dis_tmp += waiting_time + load_time;
            // dis_tmp += new_num_pred / target_berth.loading_speed + 1;  // 1帧berth，1帧dept
            // if (check_load() && obj_berth_id == i)
            //     dis_tmp--;
            double val_tmp = get_val_re(mp, i, good_num + new_num_pred, good_num + new_num_pred, good_val + new_val_pred,
                                        good_val + new_val_pred, frame_id + dis_tmp, 1, load_time);
            if (val_tmp > max_time_val) {
                max_time_val = val_tmp;
                next_plan = i;
            }
        }
        if (next_plan == -1) {
            // if (frame_id > 1000 && global_debug)
            //     std::cerr << frame_id << " err211" << std::endl;
            return 0;
        }
        if (frame_id > 1000 && max_time_val <= 0 && global_debug && debug_times++ < 2) {
            std::cerr << frame_id << ": boat " << private_id << " no time" << std::endl;
        }
        // 开始的时候
        if (max_time_val == 0)
            return 0;
        if (next_plan >= const100000) {
            if (next_plan - const100000 != obj_delivery_id)
                schedule.clear();
            obj_delivery_id = next_plan - const100000;
            // TODO: 最后的时候
            set_goto_delivery();
            mp.berths[obj_berth_id].remove_our_boat(private_id);
        } else {
            if (next_plan != obj_berth_id)
                schedule.clear();
            set_goto_berth();
        }
        // std::cerr << frame_id << ": boat " << private_id << " go to " << next_plan << std::endl;
        // 从泊位出发
        if (check_load() && (!check_goto_berth() || obj_berth_id != next_plan)) {
            auto& berth = mp.get_berth(obj_berth_id);
            berth.remove_our_boat(private_id);
            if (mp.need_dept.count(obj_berth_id)) {
                do_dept();
            } else {
                if (check_goto_delivery())
                    schedule = mp.get_boat_schedule_a_star(from, {mp.delivery_points[obj_delivery_id], STAY});
                else
                    schedule = mp.get_boat_schedule_a_star(from, {mp.berths[next_plan].p, mp.berth_direction[next_plan]});
                auto op = schedule.get_op();
                if (op == BoatOp::ANTICLOCKWISE)
                    do_anticlockwise_rotate();
                else if (op == BoatOp::CLOCKWISE)
                    do_clockwise_rotate();
                else if (op == BoatOp::FORWARD)
                    do_ship();
            }
            my_return = 1;
        }
        // std::cerr << frame_id << ": boat " << private_id << " go to " << next_plan << std::endl;
        if (check_goto_berth()) {
            if (obj_berth_id != -1)
                mp.berths[obj_berth_id].remove_our_boat(private_id);
            obj_berth_id = next_plan;
            mp.berths[obj_berth_id].add_our_boat(private_id);
        }
        return my_return;
    }

    // 1层递归搜索，now表示到达泊位后或到达交货点前。计算单位时间价值
    double get_val_re(Map& mp,
                      int uni_id,
                      double num_now,
                      double num_all,
                      int val_now,
                      int val_all,
                      int now_time,
                      int depth,
                      int loading_time) {
        double max_time_val = -1;
        // std::function<void(int)> log = [&](int l) {
        //     if (frame_id > 6620 && frame_id < 6630 && max_time_val > 0.092)
        //         std::cerr << l << " " << depth << " " << uni_id << " " << max_time_val << " " << num_all << " " <<
        //         now_time
        //                   << std::endl;
        // };
        std::function<double()> time_detection = [&]() {
            if (now_time - loading_time * 0.7 > FRAME_NUM) {
                if (num_now == num_all) {
                    // 可以考虑减少泊位停留时间
                    if (uni_id >= const100000 && val_now > 0)
                        return -((now_time + 10.0 - FRAME_NUM) / (FRAME_NUM * 2));  // 垂死挣扎，-1到0
                    else
                        return -((now_time + 10.0) / (FRAME_NUM * 2));  // 去泊位优先级低于去交货点
                }
                val_all -= val_now;
                num_all -= num_now;
                return double(val_all) / (FRAME_NUM - last_time);
            }
            return -10.0;
        };
        double ret = time_detection();
        if (ret != -10.0)
            return ret;
        if (uni_id < const100000) {
            auto& berth = mp.get_berth(uni_id);
            int delivery_id = berth.nearest_delivery_id;
            now_time += berth.nearest_delivery_dis * boat_col_time_rate + 1;
            double ret = time_detection();
            if (ret != -10.0)
                return ret;
            // now_time -= berth.nearest_delivery_dis * boat_col_time_rate + 1;
        }
        // if (frame_id > 11240 && frame_id < 11250 && double(num_all) / (now_time - last_time) > 0.87)
        //     std::cerr << depth << " " << uni_id << " " << num_all << " " << now_time << " "
        //               << double(num_all) / (now_time - last_time) << std::endl;
        // max_time_val = double(val_all) / (now_time - last_time);
        max_time_val = double(val_all) / (now_time - frame_id + 1);
        // log(1);
        return max_time_val;
    }

    // 对货物的处理
    // void load(int cnt, int val) {
    //     good_num += cnt;
    //     good_val += val;
    // }
    void unload(Map& mp) {
        if (global_debug && (global_info || frame_id > FRAME_NUM - 300)) {
            int num_now = 0;
            for (Berth& berth : mp.berths) {
                num_now += berth.good_vals.size();
            }
            std::cerr << frame_id << ": boat " << private_id << " unload " << good_num << " " << good_val << " " << num_now
                      << std::endl;
        }
        final_score += good_val;
        delivered_goods_val += good_val;
        good_num = 0;
        good_val = 0;
    }
    // 船只的操作
    // 尝试将对应船位置重置到主航道上，会导致船进入恢复状态。
    void do_dept() {
        printf("dept %d\n", private_id);
        nxt_ppd.second = STAY;
    }
    // 尝试将对应船靠泊到泊位上，会导致船进入恢复状态。
    void do_berth() {
        printf("berth %d\n", private_id);
        nxt_ppd.second = STAY;
    }
    // 旋转，0为顺时针旋转。1为逆时针旋转。
    void do_rotate(int d) { printf("rot %d %d\n", private_id, d); }
    void do_clockwise_rotate() { do_rotate(0); }
    void do_anticlockwise_rotate() { do_rotate(1); }
    // 向正方向前进1格
    void do_ship() { printf("ship %d\n", private_id); }
    // check
    inline bool check_extra_status(BoatExtraStatus status) { return this->extra_status == status; }
    inline bool check_waiting() { return check_extra_status(BoatExtraStatus::WAITING_); }
    inline bool check_goto_berth() { return check_extra_status(BoatExtraStatus::GOTOBERTH_); }
    inline bool check_goto_delivery() { return check_extra_status(BoatExtraStatus::GOTODELIVERY); }
    inline void set_goto_berth() { extra_status = BoatExtraStatus::GOTOBERTH_; }
    inline void set_goto_delivery() { extra_status = BoatExtraStatus::GOTODELIVERY; }
    inline void set_waiting() { extra_status = BoatExtraStatus::WAITING_; }
};