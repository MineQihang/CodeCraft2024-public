#pragma once
#pragma GCC optimize("O2")

int my_score;

class Boat {
   public:
    int id, obj_berth_id;
    BoatStatus status;
    int good_num{0}, capacity, goods_val{};
    std::pair<int, int> time_score_pair{-1, 0};
    int log_flag{1};
    Boat() {}
    Boat(int id) : id(id) {}
    void set_id(int id) { this->id = id; }
    void init_capacity(int capacity) { this->capacity = capacity; }
    void update() {
        int temp;
        scanf("%d%d", &temp, &obj_berth_id);
        this->status = BoatStatus(temp);
    }
    void do_op(Map& mp, int frame_id) {
        bool debug = 0;
        if (manual_plan) {
            if (status != BoatStatus::WORK)
                log_flag = 1;
            if (boat_plan[id].count(frame_id)) {
                std::string cmd = boat_plan[id][frame_id];
                if (cmd[0] == 'g') {
                    auto& berth = mp.get_berth(obj_berth_id);
                    time_score_pair.first = frame_id + berth.transport_time;
                    time_score_pair.second = goods_val;
                    good_num = 0;
                    goods_val = 0;
                }
                std::cout << cmd;
                // std::cerr << cmd << " at " << frame_id << std::endl;
            } else if (obj_berth_id != -1 && status == BoatStatus::WORK)  // 可能有等待
            {
                auto& berth = mp.get_berth(obj_berth_id);
                berth.load_once_(goods_val, good_num, capacity);
            }
            if (obj_berth_id != -1 && good_num >= capacity && log_flag && status == BoatStatus::WORK) {
                std::cerr << "at berth " << obj_berth_id << " at time" << frame_id
                          << " had left:" << mp.berths[obj_berth_id].goods_cnt << " \n";
                log_flag = 0;
            }
            return;  // 状态什么的就不用管了
        }
        if (status == BoatStatus::WORK) {
            if (obj_berth_id == -1) {
                goods_val = 0;
                int berth_id = mp.find_berth_to_load();
                if (berth_id != -1) {
                    mp.set_berth_status(berth_id, BerthStatus::BUSY);
                    printf("ship %d %d\n", id, berth_id);
                }
            } else {
                auto& berth = mp.get_berth(obj_berth_id);
                std::function<void(void)> go = [&]() {
                    if (debug)
                        std::cerr << "time:" << frame_id << "  boat:" << id << " go from " << obj_berth_id << std::endl;
                    printf("go %d\n", id);
                    mp.set_berth_status(obj_berth_id, BerthStatus::IDLE);
                    time_score_pair.first = frame_id + berth.transport_time;
                    time_score_pair.second = goods_val;
                    good_num = 0;
                };
                if (good_num >= capacity || berth.transport_time + frame_id >= FRAME_NUM - 1) {
                    if (manual_plan && good_num >= capacity) {
                        std::cerr << "err239";
                        abort();
                    }
                    go();
                } else if (!manual_plan && mp.check_berth_empty(obj_berth_id) &&
                           FRAME_NUM - frame_id > 4 * berth.transport_time + 20) {
                    int margin_num = 10;
                    // if (capacity - good_num < margin_num || FRAME_NUM - frame_id < 4 * berth.transport_time + 20 + 500 +
                    // margin_num) // 100帧单港期望收入为1
                    go();
                    // else
                    // {
                    //     int taget_berth_id = -10;
                    //     int max_goods_num = 0;
                    //     for (int i = 0; i < BERTH_NUM; i++)
                    //     {
                    //         auto &target_berth = mp.get_berth(i);
                    //         if (target_berth.status == BerthStatus::EMPTY)
                    //         {
                    //             int tmp_num = target_berth.goods_cnt + target_berth.efficiency * time_in_berth_move;
                    //             if (tmp_num > max_goods_num)
                    //             {
                    //                 max_goods_num = tmp_num;
                    //                 taget_berth_id = i;
                    //             }
                    //         }
                    //     }
                    //     if (max_goods_num <= margin_num)
                    //         go();
                    //     else
                    //     {
                    //         mp.set_berth_status(obj_berth_id, BerthStatus::EMPTY);
                    //         mp.set_berth_status(taget_berth_id, BerthStatus::BUSY);
                    //         printf("ship %d %d\n", id, taget_berth_id);
                    //     }
                    // }
                } else
                    berth.load_once_(goods_val, good_num, capacity);
                if (debug)
                    std::cerr << "time:" << frame_id << "  boat:" << id << " get goods " << good_num << "\t";
            }
        } else if (status == BoatStatus::MOVE) {
            // DO NOTHING
        } else if (status == BoatStatus::WAIT) {
            // DO NOTHING
        }
    }
};