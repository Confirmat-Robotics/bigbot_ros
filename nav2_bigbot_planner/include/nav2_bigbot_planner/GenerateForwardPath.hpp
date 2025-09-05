#ifndef GENERATE_FORWARD_PATH_HPP
#define GENERATE_FORWARD_PATH_HPP

#include <iostream>
#include <cmath>
#include <array>
#include <vector>
#include <set>
#include <algorithm> // For std::min_element
#include <memory>  // For std::shared_ptr
#include <Eigen/Dense>
#include "Rotate.hpp"
#include "Translate.hpp"

namespace nav2_bigbot_planner {

class GenerateForwardPath {
public:
    GenerateForwardPath(
        const Eigen::Vector2d& P1,
        const Eigen::Vector2d& V1,
        const Eigen::Vector2d& P2,
        const Eigen::Vector2d& V2,
        double rmin,
        bool strictly_forward);

    std::pair<Eigen::Vector2d, Eigen::Vector2d> getCircles(const Eigen::Vector2d& P, const Eigen::Vector2d& V, double radius);
    std::vector<std::array<Eigen::Vector2d, 2>> getRaaklijnen(const Eigen::Vector2d& Cstart, const Eigen::Vector2d& Cend, double r);

    double anglePointOnCircle(const Eigen::Vector2d& C, const Eigen::Vector2d& P);
    
    // Calculates the arc angle between two points on a circle
    double anglearc(const Eigen::Vector2d& C, const Eigen::Vector2d& startP, const Eigen::Vector2d& endP, bool goingLeft = true);
    
    // Calculates the arc angle between two angles
    double anglearc_angles_in(double start_angle, double end_angle, bool goingLeft = true);
    // Checks if a circle center is left of the line from point F to point S
    bool leftleave(const Eigen::Vector2d& F, const Eigen::Vector2d& S, const Eigen::Vector2d& C);

    // Determines which tangent lines can be used without changing direction
    std::set<int> whichleftleave(const std::vector<std::array<Eigen::Vector2d, 2>>& tangents, const Eigen::Vector2d& centerpoint, bool inverse = false);

    // Checks if two circles are close enough
    bool circles_close_enough(const Eigen::Vector2d& C1, const Eigen::Vector2d& C2, double r);
    
    // Method to get paths between two circles
    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> GetPathTwoCircles(
        const Eigen::Vector2d& CS, 
        const Eigen::Vector2d& CE, 
        bool startleft, 
        bool endleft, 
        double rmin);

    // Calculate the positions of the two potential third circle centers between two circles
    std::tuple<std::vector<Eigen::Vector2d>, double, Eigen::Vector2d, Eigen::Vector2d>
    ThirdCircle(const Eigen::Vector2d& C1, const Eigen::Vector2d& C2, double r);
    
    // Method to generate paths using three circles
    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> GetPathThreeCircles(
        const Eigen::Vector2d& CS, 
        const Eigen::Vector2d& CE, 
        bool start_and_endleft, 
        double rmin);

    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> GetAllPaths();

    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> GetAllThreeCirclePaths();
    
    std::pair<std::vector<std::shared_ptr<void>>, double> GetShortestPath();
    
private:
    Eigen::Vector2d P1_;
    Eigen::Vector2d V1_;
    Eigen::Vector2d P2_;
    Eigen::Vector2d V2_;
    double rmin_;
    bool strictly_forward_;
};

} // namespace nav2_bigbot_planner

#endif // GENERATE_FORWARD_PATH_HPP

