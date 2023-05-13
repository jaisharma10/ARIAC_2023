#include <iostream>
#include <vector>
#include <cmath>


class TrackBin {
public:
   
   struct Slot {
        std::string status;
        double x;
        double y;
    };

    std::vector<std::vector<Slot>> slot_status;
    int left_full_counter = 0;
    int right_full_counter = 0;

    TrackBin() {
        // Define the bin and slot information
        std::vector<std::pair<double, double>> bin_centers = {
            {-1.9, 3.375},
            {-1.9, 2.625},
            {-2.65, 2.625},
            {-2.65, 3.375},
            {-1.9, -3.375},
            {-1.9, -2.625},
            {-2.65, -2.625},
            {-2.65, -3.375}
        };
        
        std::vector<std::pair<double, double>> slot_offsets = {
            {-0.18, -0.18},
            {-0.18, 0.0},
            {-0.18, 0.18},
            {0.0, -0.18},
            {0.0, 0.0},
            {0.0, 0.18},
            {0.18, -0.18},
            {0.18, 0.0},
            {0.18, 0.18},            
        };

        // Initialize the array to store slot status and coordinates
        slot_status.resize(8);
        for (int i = 0; i < 8; i++) {
            slot_status[i].resize(9);
            for (int j = 0; j < 9; j++) {
                double slot_x = bin_centers[i].first + slot_offsets[j].first;
                double slot_y = bin_centers[i].second + slot_offsets[j].second;
                slot_status[i][j] = {"empty", slot_x, slot_y};
            }
        }
    }

    void empty_left_slot_status(){
        for (int i = 4; i < 8; i++) {
            for (int j = 0; j < 9; j++) {
                slot_status[i][j].status = "empty";
            }
        }
    }

    void empty_right_slot_status(){
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 9; j++) {
                slot_status[i][j].status = "empty";
            }
        }
    }

    void update_left_slot_status(std::vector<std::pair<double, double>> part_locs) 
    {
        empty_left_slot_status();
        std::vector<std::pair<int,int>> index_list;
        
        for (auto part_loc : part_locs) {
            for (int i = 4; i < 8; i++) {
                for (int j = 0; j < 9; j++) {
                    double slot_x = slot_status[i][j].x;
                    double slot_y = slot_status[i][j].y;
                    double dist = std::sqrt(std::pow(part_loc.first - slot_x, 2) + std::pow(part_loc.second - slot_y, 2));
                    if (dist < 0.05) {  // set the threshold distance here
                        index_list.push_back({i,j});
                        goto cnt;
                    }
                }
            }
            cnt:;
        }
        left_full_counter = 0;
        for (auto index : index_list) {
            slot_status[index.first][index.second].status = "occupied";
            left_full_counter += 1;
        }

    }

    bool bin_full_check(){
        if (left_full_counter >= 18){
            std::cout << " Left Full Counter is 18 " << std::endl;
            return true;
        }
        else if (right_full_counter >= 18){
            std::cout << " Right Full Counter is 18 " << std::endl;
            return true;
        }
        else{
            std::cout << " Both Counters have empty spotss " << std::endl;
            std::cout << " Left Full Counter is " << left_full_counter <<  std::endl;
            std::cout << " Right Full Counter is " << right_full_counter << std::endl;

            return false;
        }

    }

    void update_right_slot_status(std::vector<std::pair<double, double>> part_locs) 
    {
        empty_right_slot_status();
        std::vector<std::pair<int,int>> index_list;
        
        for (auto part_loc : part_locs) {
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 9; j++) {
                    double slot_x = slot_status[i][j].x;
                    double slot_y = slot_status[i][j].y;
                    double dist = std::sqrt(std::pow(part_loc.first - slot_x, 2) + std::pow(part_loc.second - slot_y, 2));
                    if (dist < 0.05) {  // set the threshold distance here
                        index_list.push_back({i,j});
                        goto cnt;
                    }
                }
            }
            cnt:;
        }
        right_full_counter = 0;
        for (auto index : index_list) {
            slot_status[index.first][index.second].status = "occupied";
            right_full_counter += 1;
        }
    }

    std::pair<double, double> get_place_pose(int bin_num) {
        std::pair<double, double>  place_pose;
        for (int j = 0; j < 9; j++) {
            if(slot_status[bin_num - 1][j].status == "empty"){
                place_pose.first = slot_status[bin_num - 1][j].x;
                place_pose.second = slot_status[bin_num - 1][j].y;
                break;
            }
        }
        return place_pose;
    }

    void print_slot_status() {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 9; j++) {
                std::cout << "Bin " << i+1 << " Slot " << j+1 << ": {" << slot_status[i][j].status << ", " << slot_status[i][j].x << ", " << slot_status[i][j].y << "}" << std::endl;
            }
        }
    }

};