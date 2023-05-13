#include <iostream>
#include <vector>
#include <cmath>


class TrackBin {
public:
   
   struct Slot {
        double x;
        double y;
    };

    TrackBin() {

        std::vector<std::pair<double, double>> bin_centers = {
            // {-1.9, 3.375},      // bin 2
            // {-1.9, 2.625},      // bin 1
            // {-1.9, -3.375},     // bin 5
            // {-1.9, -2.625},     // bin 6
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
            }
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