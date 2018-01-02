#ifndef PLANNER_H
#define PLANNER_H

#include <vector>

using namespace std;

class Planner {
    const size_t N = 50; //number of points to generate 
    const double ref_v = 49 * 0.447; //ref speed in m/s
    const double max_a = 8; //max acceleration, less than 10m/s^2
    const double safe_gap = 30; //gap to keep when following a vehicle to avoid collision
    //when changing lane, minimun gap in front of and behind us
    const double lane_change_gap = 15;
    //when changing lane, if the vehicle behind is faster than us, use a bigger gap
    const double lane_change_gap_big = 25;
    int lane_changing; //set when lane changing is happending
    int target_lane; //the target lane during a lane changing
    //data structure used when evaluating a lane before lane changing
    struct lane_stats_ {
        double front_dist;
        double back_dist;
        double v_front;
        double v_back;
        double cost;
    };


    void calculate_cost(double car_speed, int lane, vector<lane_stats_> &lane_stats);
    int get_target_lane(double car_speed, int lane, double car_s, const vector<vector<double>> &sensor_fusion);
public:

    Planner();
    ~Planner();

    vector<vector<double>> get_path(const vector<double> &state, const vector<double> &previous_path_x,
                                    const vector<double> &previous_path_y, const vector<vector<double>> &sensor_fusion,
                                    const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                    const vector<double> &map_waypoints_s); 
};
#endif
