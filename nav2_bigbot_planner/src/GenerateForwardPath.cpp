/*********************************************************************
 *
 * Software License Agreement 
 *
 *  Copyright (c) 2025 Edward Hage
 *  All rights reserved.
 *
 *********************************************************************/

#include "nav2_bigbot_planner/GenerateForwardPath.hpp"
#include <cmath>
#include <algorithm> 
#include <memory>  // For std::shared_ptr
#include <Eigen/Dense>

namespace nav2_bigbot_planner {

GenerateForwardPath::GenerateForwardPath(
        const Eigen::Vector2d& P1,
        const Eigen::Vector2d& V1,
        const Eigen::Vector2d& P2,
        const Eigen::Vector2d& V2,
        double rmin,
        bool strictly_forward)
        : P1_(P1), V1_(V1), P2_(P2), V2_(V2), rmin_(rmin), strictly_forward_(strictly_forward) {}

std::pair<Eigen::Vector2d, Eigen::Vector2d> GenerateForwardPath::getCircles(const Eigen::Vector2d& P, const Eigen::Vector2d& V, double radius) {
    Eigen::Vector2d v = V.normalized();
    Eigen::Vector2d left_circle = P + Eigen::Vector2d(-v.y(), v.x()) * radius;
    Eigen::Vector2d right_circle = P + Eigen::Vector2d(v.y(), -v.x()) * radius;
    return {left_circle, right_circle};
}

std::vector<std::array<Eigen::Vector2d, 2>> GenerateForwardPath::getRaaklijnen(const Eigen::Vector2d& Cstart, const Eigen::Vector2d& Cend, double r) {
    Eigen::Vector2d delta = Cend - Cstart;
    Eigen::Vector2d v = delta.normalized();
    Eigen::Vector2d left_vec(-v.y(), v.x());
    Eigen::Vector2d cl1 = left_vec * r;
    Eigen::Vector2d cr1 = Eigen::Vector2d(v.y(), -v.x()) * r;

    std::array<Eigen::Vector2d, 2> raaklijn1 = {Cstart + cl1, Cend + cl1};
    std::array<Eigen::Vector2d, 2> raaklijn2 = {Cstart + cr1, Cend + cr1};

    double ll = delta.norm() / 2.0;
    double a = r * r / ll;
    double g = (a < r) ? std::sqrt(r * r - a * a) : 0.0;

    std::array<Eigen::Vector2d, 2> raaklijn3 = {Cstart + v * a + left_vec * g, Cend - v * a - left_vec * g};
    std::array<Eigen::Vector2d, 2> raaklijn4 = {Cstart + v * a - left_vec * g, Cend - v * a + left_vec * g};

    return {raaklijn1, raaklijn2, raaklijn3, raaklijn4};
}

double GenerateForwardPath::anglePointOnCircle(const Eigen::Vector2d& C, const Eigen::Vector2d& P) {
    Eigen::Vector2d F = P - C;
    double angle = std::atan2(F.y(), F.x());
    return std::fmod(angle + 2 * M_PI, 2 * M_PI);
}
    

double GenerateForwardPath::anglearc(const Eigen::Vector2d& C, const Eigen::Vector2d& startP, const Eigen::Vector2d& endP, bool goingLeft) {
    double ang1 = anglePointOnCircle(C, startP);
    double ang2 = anglePointOnCircle(C, endP);
    return anglearc_angles_in(ang1, ang2, goingLeft);
}

// Calculates the arc angle between two angles
double GenerateForwardPath::anglearc_angles_in(double start_angle, double end_angle, bool goingLeft) {
    double delta = std::fmod(end_angle - start_angle + 2 * M_PI, 2 * M_PI);
    return goingLeft ? delta : 2 * M_PI - delta;
}

// Checks if a circle center is left of the line from point F to point S
bool GenerateForwardPath::leftleave(const Eigen::Vector2d& F, const Eigen::Vector2d& S, const Eigen::Vector2d& C) {
    return (S.x() - F.x()) * (C.y() - F.y()) - (S.y() - F.y()) * (C.x() - F.x()) > 0;
}

// Determines which tangent lines can be used without changing direction
std::set<int> GenerateForwardPath::whichleftleave(const std::vector<std::array<Eigen::Vector2d, 2>>& tangents, const Eigen::Vector2d& centerpoint, bool inverse) {
    std::set<int> valid;
    for (size_t i = 0; i < tangents.size(); ++i) {
        bool isLeft = leftleave(tangents[i][0], tangents[i][1], centerpoint);
        if ((inverse && !isLeft) || (!inverse && isLeft)) {
            valid.insert(i);
        }
    }
    return valid;
}

// Checks if two circles are close enough
bool GenerateForwardPath::circles_close_enough(const Eigen::Vector2d& C1, const Eigen::Vector2d& C2, double r) {
    return (C2 - C1).norm() < 4 * r;
}

// Method to get paths between two circles
std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> GenerateForwardPath::GetPathTwoCircles(
    const Eigen::Vector2d& CS, const Eigen::Vector2d& CE, bool startleft, bool endleft, double rmin) {
    
    // Get tangent lines between the two circles
    auto tangents = getRaaklijnen(CS, CE, rmin);

    // Determine valid tangent lines for start and end points
    std::set<int> valid_start = whichleftleave(tangents, CS, !startleft);
    std::set<int> valid_end = whichleftleave(tangents, CE, !endleft);

    // Find intersection of valid start and end paths
    std::set<int> valid_paths;
    for (const auto& vs : valid_start) {
        if (valid_end.find(vs) != valid_end.end()) {
            valid_paths.insert(vs);
        }
    }

    // Generate valid paths
    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> paths;
    for (int elem : valid_paths) {
        double angle_start = anglearc(CS, P1_, tangents[elem][0], startleft);
        double angle_end = anglearc(CE, tangents[elem][1], P2_, endleft);
        double linear_part = (tangents[elem][0] - tangents[elem][1]).norm();
        double total_length = (angle_start + angle_end) * rmin + linear_part;

        // Create the path components
        std::vector<std::shared_ptr<void>> path;
        path.push_back(std::make_shared<Rotate>(angle_start, rmin, true, !startleft));
        path.push_back(std::make_shared<Translate>(linear_part, true));
        path.push_back(std::make_shared<Rotate>(angle_end, rmin, true, !endleft));

        // Store the path and its length
        paths.emplace_back(path, total_length);
    }

    return paths;
}

// Calculate the positions of the two potential third circle centers between two circles
std::tuple<std::vector<Eigen::Vector2d>, double, Eigen::Vector2d, Eigen::Vector2d>
GenerateForwardPath::ThirdCircle(const Eigen::Vector2d& C1, const Eigen::Vector2d& C2, double r) {
    double g = (C2 - C1).norm();
    Eigen::Vector2d center = C2 - (C2 - C1) / 2.0;
    double offset = std::sqrt(std::pow(2 * r, 2) - std::pow(g / 2.0, 2));
    Eigen::Vector2d mydir = (C2 - C1).normalized();

    // Calculate the two third circle centers
    Eigen::Vector2d CT1 = center + Eigen::Vector2d(-mydir.y(), mydir.x()) * offset;
    Eigen::Vector2d CT2 = center - Eigen::Vector2d(-mydir.y(), mydir.x()) * offset;

    // Calculate angles
    double a = std::acos(g / (4.0 * r));
    double twoa = 2 * a;
    double angledir = std::atan2(mydir.y(), mydir.x());

    // Angles relative to C1 and C2
    Eigen::Vector2d anglesC1(angledir + a, angledir - a);
    Eigen::Vector2d anglesC2(-M_PI + angledir - a, -M_PI + angledir + a);

    // Return both potential third circle centers and relevant angles
    return std::make_tuple(std::vector<Eigen::Vector2d>{CT1, CT2}, twoa, anglesC1, anglesC2);
}

// Method to generate paths using three circles
std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> GenerateForwardPath::GetPathThreeCircles(
    const Eigen::Vector2d& CS, const Eigen::Vector2d& CE, bool start_and_endleft, double rmin) {

    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> paths;

    if (!circles_close_enough(CS, CE, rmin)) {
        return paths;
    }

    auto [CTs, twoa, anglesC1, anglesC2] = ThirdCircle(CS, CE, rmin);

    double angle_P1 = anglePointOnCircle(CS, P1_);
    double angle_P2 = anglePointOnCircle(CE, P2_);

    if (start_and_endleft) {
        // Left corner using CT1
        double angle_start = anglearc_angles_in(angle_P1, anglesC1[0], true);
        double angle_end = anglearc_angles_in(anglesC2[0], angle_P2, true);
        if (!strictly_forward_) {
            double total_length = (angle_start + angle_end) * rmin + (M_PI - twoa) * rmin;
            std::vector<std::shared_ptr<void>> path = {
                std::make_shared<Rotate>(angle_start, rmin, true, false),
                std::make_shared<Rotate>(M_PI - twoa, rmin, false, false),
                std::make_shared<Rotate>(angle_end, rmin, true, false)
            };
            paths.emplace_back(path, total_length);
        } else {
            double total_length = (angle_start + angle_end) * rmin + (M_PI + twoa) * rmin;
            std::vector<std::shared_ptr<void>> path = {
                std::make_shared<Rotate>(angle_start, rmin, true, false),
                std::make_shared<Rotate>(M_PI + twoa, rmin, true, true),
                std::make_shared<Rotate>(angle_end, rmin, true, false)
            };
            paths.emplace_back(path, total_length);
        }
    } else {
        // Right corner using CT2
        double angle_start = anglearc_angles_in(angle_P1, anglesC1[1], false);
        double angle_end = anglearc_angles_in(anglesC2[1], angle_P2, false);
        if (!strictly_forward_) {
            double total_length = (angle_start + angle_end) * rmin + (M_PI - twoa) * rmin;
            std::vector<std::shared_ptr<void>> path = {
                std::make_shared<Rotate>(angle_start, rmin, true, true),
                std::make_shared<Rotate>(M_PI - twoa, rmin, false, true),
                std::make_shared<Rotate>(angle_end, rmin, true, true)
            };
            paths.emplace_back(path, total_length);
        } else {
            double total_length = (angle_start + angle_end) * rmin + (M_PI + twoa) * rmin;
            std::vector<std::shared_ptr<void>> path = {
                std::make_shared<Rotate>(angle_start, rmin, true, true),
                std::make_shared<Rotate>(M_PI + twoa, rmin, true, false),
                std::make_shared<Rotate>(angle_end, rmin, true, true)
            };
            paths.emplace_back(path, total_length);
        }
    }

    return paths;
}

std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> GenerateForwardPath::GetAllPaths() {
    // Get left and right circles for start and end points
    auto [CLS, CRS] = getCircles(P1_, V1_, rmin_);
    auto [CLE, CRE] = getCircles(P2_, V2_, rmin_);

    // Collect all possible paths
    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> paths;
    
    auto llpaths = GetPathTwoCircles(CLS, CLE, true, true, rmin_);
    auto lrpaths = GetPathTwoCircles(CLS, CRE, true, false, rmin_);
    auto rlpaths = GetPathTwoCircles(CRS, CLE, false, true, rmin_);
    auto rrpaths = GetPathTwoCircles(CRS, CRE, false, false, rmin_);

    // Combine all paths
    paths.insert(paths.end(), llpaths.begin(), llpaths.end());
    paths.insert(paths.end(), lrpaths.begin(), lrpaths.end());
    paths.insert(paths.end(), rlpaths.begin(), rlpaths.end());
    paths.insert(paths.end(), rrpaths.begin(), rrpaths.end());

    return paths;
}

std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> GenerateForwardPath::GetAllThreeCirclePaths() {
    // Get left and right circles for start and end points
    auto [CLS, CRS] = getCircles(P1_, V1_, rmin_);
    auto [CLE, CRE] = getCircles(P2_, V2_, rmin_);

    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> paths;

    // Get paths using three circles
    auto llpaths = GetPathThreeCircles(CLS, CLE, true, rmin_);
    auto rrpaths = GetPathThreeCircles(CRS, CRE, false, rmin_);

    // Combine left-left and right-right paths
    paths.insert(paths.end(), llpaths.begin(), llpaths.end());
    paths.insert(paths.end(), rrpaths.begin(), rrpaths.end());

    return paths;
}


std::pair<std::vector<std::shared_ptr<void>>, double> GenerateForwardPath::GetShortestPath() {
    // Get all paths using two circles and three circles
    auto paths1 = GetAllPaths();
    auto paths2 = GetAllThreeCirclePaths();

    // Combine both sets of paths
    std::vector<std::pair<std::vector<std::shared_ptr<void>>, double>> paths;
    paths.insert(paths.end(), paths1.begin(), paths1.end());
    paths.insert(paths.end(), paths2.begin(), paths2.end());

    // Return if no paths are available
    if (paths.empty()) {
        return {{}, 0.0};
    }

    // Find the shortest path by comparing lengths
    auto shortest = std::min_element(paths.begin(), paths.end(),
        [](const auto& p1, const auto& p2) {
            return p1.second < p2.second;
        });

    // Return the shortest path
    return *shortest;
}

} //end namespace