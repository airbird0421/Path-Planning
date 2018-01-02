#include <iostream>
#include "planner.h"
#include "spline.h"
#include "helper.h"

/*
 * constructor
 */
Planner::Planner()
{
    lane_changing = 0;
}

/*
 * destructor
 */

Planner::~Planner()
{}

/*
 * function to calcualte lanes' cost before doing a lane change
 */

void Planner::calculate_cost(double car_speed, int lane, vector<lane_stats_> &lane_stats)
{
    for (ssize_t i = 0; i < lane_stats.size(); i++) {
        // only evaluate current lane and adjacent lanes
        if (i > lane + 1 || i < lane - 1)
            continue;
                
        double gap_front = lane_stats[i].front_dist;
        double gap_back = lane_stats[i].back_dist;
        double v_front = lane_stats[i].v_front;
        double v_back = lane_stats[i].v_back;
        double v_cost, gap_cost;

        // if no enough gap for a lane change, set a high cost
        if (i != lane && (gap_front < lane_change_gap || gap_back < lane_change_gap ||
            (v_back > car_speed && gap_back < lane_change_gap_big))) {
            lane_stats[i].cost = 10;
            continue;
        }

        lane_stats[i].cost = 0;

        // speed cost
        if (v_front <= 0) {
            v_cost = 1;
        } else if (v_front >= ref_v) {
            v_cost = 0;
        } else {
            v_cost = 1 - v_front / ref_v;
        }

        // distance cost
        if (gap_front <= 10) {
            gap_cost = 1;
        } else if (gap_front >= 160) {
            gap_cost = 0;
        } else {
            gap_cost = 1 - (gap_front - 10) / 150;
        }
        lane_stats[i].cost = v_cost + gap_cost;

        // reduce the cost for current lane to avoid oscillation, i.e., when
        // two lanes have similar cost, vehicle may change lane back and forth
        if (i == lane) {
            lane_stats[i].cost -= 0.1;
        }
    }
 
}

/*
 * Get the target lane when a lane change is necessary
 */

int Planner::get_target_lane(double car_speed, int lane, double car_s, const vector<vector<double>> &sensor_fusion)
{
    double front_dist, v_f, back_dist;
    vector<lane_stats_> lane_stats(3);
 
   for (int i = 0; i < 3; i++) {
        lane_stats[i].front_dist = 9999;
        lane_stats[i].v_front = ref_v;
        lane_stats[i].back_dist = 9999;
    }

    // go through sensor fusion data and get the data associated 
    // with each lane
    for (size_t i = 0; i < sensor_fusion.size(); i++) {
        double s = sensor_fusion[i][5];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double v = sqrt(vx * vx + vy * vy);
        double d = sensor_fusion[i][6];
        int cur_lane = d / 4;

        // only care about adjacent lanes
        if (cur_lane > lane + 1 || cur_lane < lane - 1) continue;

        if (s < car_s) {
            if (car_s - s < lane_stats[cur_lane].back_dist) {
                lane_stats[cur_lane].v_back = v;
                lane_stats[cur_lane].back_dist = car_s -s;
            }
        } else {
            if (s - car_s < lane_stats[cur_lane].front_dist) {
                lane_stats[cur_lane].v_front = v;
                lane_stats[cur_lane].front_dist = s - car_s;
            }
        }
    }
   
    // calculate the cost for each lane 
    calculate_cost(car_speed, lane, lane_stats);

    // select a lane with minimum cost
    double min_cost = 10;
    int target;
    for (int i = lane - 1; i <= lane + 1; i++) {
        if (i < 0 || i > 2) continue;
        if (lane_stats[i].cost < min_cost) {
            min_cost = lane_stats[i].cost;
            target = i;
        }
    }

    return target;
}

/*
 * Function called to return the path points in two vectors, x-value vector and y-value vector
 */

vector<vector<double>> Planner::get_path(const vector<double> &state, const vector<double> &previous_path_x,
                                         const vector<double> &previous_path_y, const vector<vector<double>> &sensor_fusion,
                                         const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                         const vector<double> &map_waypoints_s) 
{
    double car_x = state[0];
    double car_y = state[1];
    double car_s = state[2];
    double car_d = state[3];
    double car_yaw = deg2rad(state[4]);
    double car_speed = state[5] * .447; // speed in m/s
    int lane = car_d / 4;
    double end_speed = ref_v;
    double car_v; // vehicle speed at the beginning of a spline

    // see if a lane changing is completed. When a lane changing is going on,
    // no more lane-change logic will run.
    if (lane_changing) {
        if (car_d > target_lane * 4 + 1 && car_d < target_lane * 4 + 3) {
            lane_changing = 0;
        }
    }
    // define spline and take points from spline
    // always take the first two points from previous path, plus two points
    // in the target lane
    vector<double> sp_x, sp_y;
    tk::spline s;
    size_t pre_size = previous_path_x.size();
    
    // use current position, and an estimated previous position when no enough
    // points from previous cycle 
    if (pre_size < 2) {
        sp_x.push_back(car_x - cos(car_yaw));
        sp_x.push_back(car_x);
        sp_y.push_back(car_y - sin(car_yaw));
        sp_y.push_back(car_y);
        car_v = car_speed;
    } else if (pre_size < 10){
        // use the last two points from previous cycle
        sp_x.push_back(previous_path_x[pre_size - 2]);
        sp_x.push_back(previous_path_x[pre_size - 1]);
        sp_y.push_back(previous_path_y[pre_size - 2]);
        sp_y.push_back(previous_path_y[pre_size - 1]);
        car_v = sqrt(pow(sp_x[1] - sp_x[0], 2) + pow(sp_y[1] - sp_y[0], 2)) / 0.02;
    } else {
        // use the 9th and 10th points from previous cycle
        sp_x.push_back(previous_path_x[8]);
        sp_x.push_back(previous_path_x[9]);
        sp_y.push_back(previous_path_y[8]);
        sp_y.push_back(previous_path_y[9]);
        car_v = sqrt(pow(sp_x[1] - sp_x[0], 2) + pow(sp_y[1] - sp_y[0], 2)) / 0.02;
        pre_size = 10;
    }

    // always use a fixed acceleration when calculating the distance between two
    // adjacent points. Calcuate this acceleration to be used.
    double car_a = (end_speed - car_v) / (0.02 * (N - pre_size));
    // acceleration can't exceed allowed maximum
    if (car_a > max_a) {
        car_a = max_a;
        end_speed = car_v + car_a * 0.02 * (N - pre_size);
    }

    // see if it will be too close to the vehicle ahead of us. If so
    // adjust the acceleration
    double end_s = car_s + (end_speed + car_v) * (N - pre_size) * 0.02 / 2;

    // find the vehicle in the same lane just in front of us
    double min_dist = safe_gap + 1;
    double change_lane = false;
    
    for (size_t i = 0; i < sensor_fusion.size(); i++) {
        double s = sensor_fusion[i][5];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double v = sqrt(vx * vx + vy * vy);
        double d = sensor_fusion[i][6];

        if (d > lane * 4 && d < lane * 4 + 4 && s > car_s) {
            double dist = v * (N - pre_size) * 0.02 + s - end_s;
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
    }

    if (min_dist < safe_gap) {
        // if we are following too close, adjust the acceleration. This calculation
        // is based on:
        // 1. s = (v1 + v2) * t / 2
        // 2. v2 = v1 + a * t
        end_speed = end_speed - (safe_gap - min_dist) / ((N - pre_size) * 0.01);
        if (end_speed < 0) end_speed = 0;
        car_a = (end_speed - car_v) / ((N - pre_size) * 0.02);
        if (car_a < -max_a )
            car_a = - max_a;
        if (end_speed < 45 && !lane_changing) {
            // prepare to change lane, only when a lane change is already done
            change_lane = true;
        }
    } 
                
    if (change_lane) {
        target_lane = get_target_lane(car_speed, lane, car_s, sensor_fusion);
        if (target_lane != lane) {
            lane_changing = 1;
        }
    }

    // make sure the other two points for spline is in the target lane
    if (lane_changing)
        lane = target_lane;
    auto xy = getXY(car_s + 30, lane * 4 + 2, map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
    sp_x.push_back(xy[0]);
    sp_y.push_back(xy[1]);
    xy = getXY(car_s + 60, lane * 4 + 2, map_waypoints_s,
               map_waypoints_x, map_waypoints_y);
    sp_x.push_back(xy[0]);
    sp_y.push_back(xy[1]);

    // transform to car's coordinate system
    vector<double> sp_xc, sp_yc;
    for (size_t i = 0; i < sp_x.size(); i++) {
        double xc = (sp_x[i] - car_x) * cos(car_yaw) + (sp_y[i] - car_y) * sin(car_yaw);
        double yc = -(sp_x[i] - car_x) * sin(car_yaw) + (sp_y[i] - car_y) * cos(car_yaw);
        sp_xc.push_back(xc);
        sp_yc.push_back(yc);
    } 
    // fit spline
    s.set_points(sp_xc, sp_yc);
    // sample the spline
    double v_sx = car_v;
    double a_sx = car_a;
    vector<double> next_x_vals, next_y_vals;
    for (size_t i = 0; i < pre_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    for (size_t i = 1; i <= N - pre_size; i++) {
        double sx = sp_xc[1] + v_sx * 0.02 * i + a_sx * 0.02 * i * i * 0.01;
        double sy = s(sx);
        // transform back to map coordinate
        double mx = car_x + sx * cos(car_yaw) - sy * sin(car_yaw);
        double my = car_y + sx * sin(car_yaw) + sy * cos(car_yaw);
        next_x_vals.push_back(mx);
        next_y_vals.push_back(my);
    }
    return {next_x_vals, next_y_vals};
}     
