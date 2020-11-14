////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include "conveyor_manip_heuristic.h"

// standard includes
#include <cmath>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>

namespace smpl {

static const char* LOG = "heuristic.euclid_dist";

static inline
double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool ConveyorManipHeuristic::init(RobotPlanningSpace* space)
{
    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_point_ext = space->getExtension<PointProjectionExtension>();
    if (m_point_ext) {
        SMPL_DEBUG_NAMED(LOG, "Got Point Projection Extension!");
    }
    m_pose_ext = space->getExtension<PoseProjectionExtension>();
    if (m_pose_ext) {
        SMPL_DEBUG_NAMED(LOG, "Got Pose Projection Extension!");
    }
    if (!m_pose_ext && !m_point_ext) {
        SMPL_WARN_NAMED(LOG, "ConveyorManipHeuristic recommends PointProjectionExtension or PoseProjectionExtension");
    }
    m_ers = space->getExtension<ExtractRobotStateExtension>();
    if (!m_ers) {
        SMPL_WARN_NAMED(LOG, "ConveyorManipHeuristic recommends ExtractRobotStateExtension");
    }
    m_ecos = space->getExtension<ConveyorObjectStateExtension>();
    if (!m_ecos) {
        SMPL_WARN_NAMED(LOG, "ConveyorManipHeuristic recommends ConveyorObjectStateExtension");
    }
    m_fd_iface = space->robot()->getExtension<ForwardVelocityInterface>();
    if (!m_fd_iface) {
        SMPL_WARN_NAMED(LOG, "ConveyorManipHeuristic recommends ForwardVelocityInterface");
    }

    return true;
}

void ConveyorManipHeuristic::setWeightX(double wx)
{
    m_x_coeff = wx;
}

void ConveyorManipHeuristic::setWeightY(double wy)
{
    m_y_coeff = wy;
}

void ConveyorManipHeuristic::setWeightZ(double wz)
{
    m_z_coeff = wz;
}

void ConveyorManipHeuristic::setWeightRot(double wr)
{
    m_rot_coeff = wr;
}

double ConveyorManipHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    auto& goal_pose = planningSpace()->goal().pose;
    return EuclideanDistance(
            x, y, z,
            goal_pose.translation()[0],
            goal_pose.translation()[1],
            goal_pose.translation()[2]);
}

double ConveyorManipHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: implement
    return 0.0;
}

Extension* ConveyorManipHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

static
double GetTimeToIntercept1(
    Eigen::Vector3d target_pos,
    Eigen::Vector3d target_vel,
    Eigen::Vector3d interceptor_pos,
    double interceptor_speed)
{
    double k = target_vel.norm() / interceptor_speed;
    double c = (interceptor_pos - target_pos).norm();
 
    auto b_hat = target_vel;
    auto c_hat = interceptor_pos - target_pos;
 
    // CAB = angle_between(b_hat, c_hat)
    auto CAB = std::acos(b_hat.dot(c_hat) / (b_hat.norm() * c_hat.norm()));

    auto ABC = std::asin(std::sin(CAB) * k);
    auto ACB = M_PI - (CAB + ABC);
 
    auto j = c / std::sin(ACB);
    auto a = j * std::sin(CAB);
    auto b = j * std::sin(ABC);
 
    if (b > 1e6) {
        return 1e6;
    }
 
    auto t = b / target_vel.norm();

    // auto collision_pos = target_pos + (target_vel * t);
    // printf("collision_pos y: %.3f\n", collision_pos[1]);
    return t;
}

static
double GetTimeToIntercept2(
    Eigen::Vector3d vo,
    Eigen::Vector3d ve)
{
    // printf("vo: %.2f %.2f %.2f\n", vo[0], vo[1], vo[2]);
    // printf("ve: %.2f %.2f %.2f\n", ve[0], ve[1], ve[2]);
    const double ae_max = 0.5;
    auto diff = vo - ve;
    double t = std::fabs(diff.norm()/ae_max);
    // double vo_norm = vo.norm();
    // double ve_norm = ve.norm();
    // double t = std::fabs((vo_norm - ve_norm)/ae_max);
    return t;
}

static
double GetTimeToIntercept3(
    Eigen::Vector3d po,
    Eigen::Vector3d vo,
    Eigen::Vector3d pe,
    Eigen::Vector3d ve)
{
    const double ae_max = 0.5;
    auto line = po - pe;

    auto line_norm = line.norm();
    // project vo

    auto vo_p = (vo.dot(line) / (line_norm * line_norm)) * line;
    // printf("%f %f %f\n", vo_p[0], vo_p[1], vo_p[2]);
    double vo_p_norm = vo_p.norm();
    printf("speed o: %.2f\n", vo_p_norm);

    // project ve
    auto ve_p = (ve.dot(line) / (line_norm * line_norm)) * line;
    // printf("%f %f %f\n", ve_p[0], ve_p[1], ve_p[2]);
    double ve_p_norm = ve_p.norm();
    printf("speed e: %.2f\n", ve_p_norm);

    // # profile
    double d = line.norm();
    printf("distance: %.2f\n", d);

    // compute ve_max
    double ve_max = std::sqrt((2 * ae_max * d + ve_p_norm*ve_p_norm + vo_p_norm*vo_p_norm ) / 2) - 1e-6;
    printf("ve_max: %.2f\n", ve_max);
    if (ve_p_norm > ve_max) {
    // if (d < 0.6) {
        double t_reach = (vo_p_norm - ve_p_norm) / -ae_max;
        printf("t_reach (ramp down): %f\n", t_reach); getchar();
        return t_reach;
    }

    // printf("%f %f %f\n", ve_max*ve_max, ve_p_norm*ve_p_norm, (ve_max*ve_max - ve_p_norm*ve_p_norm));
    double d_rampup = (ve_max*ve_max - ve_p_norm*ve_p_norm) / (2 * ae_max);
    double t_rampup = (ve_max - ve_p_norm) / ae_max;
    double d_rampdown = (vo_p_norm*vo_p_norm - ve_max*ve_max) / (2 * -ae_max);
    double t_rampdown = (vo_p_norm - ve_max) / -ae_max;
    double d_const = d - (d_rampup + d_rampdown);
    printf("d_rampup: %.2f , t_rampup: %.2f , d_rampdown: %.2f , t_rampdown: %.2f , d_const: %.2f\n",
        d_rampup, t_rampup, d_rampdown, t_rampdown, d_const);

    if (d_const > 0.0) {
        double t_const = d_const / ve_max;
        double t_reach = t_const + t_rampup + t_rampdown;
        printf("t_reach: %f\n\n", t_reach);
        return t_reach;
        // print("t_reach hitting ve_max", t_reach)
    }
    else {
        printf("Profiling problem\n"); getchar();
    }
}

int ConveyorManipHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    if (!m_ers) {
        return 0;
    }

    auto sqrd = [](double d) { return d * d; };

    // infer object position
    const RobotState& state = m_ers->extractState(state_id);
    Eigen::Affine3d object_pose;
    Eigen::Vector3d object_velocity;
    std::tie(object_pose, object_velocity) = m_ecos->extractConveyorObjectState(state.back());

    // if (object_pose.translation().y() < -0.2) {
    //     return 1e6;
    // }
    if (m_pose_ext) {
        Affine3 p;
        if (!m_pose_ext->projectToPose(state_id, p)) {
            return 0;
        }

        Affine3 offset = Eigen::Affine3d::Identity();

        // move pregrasp to yaml
        offset.translation().x() -= 0.1;
        // offset.translation().y() += 0.03;
        object_pose = object_pose * offset;
        // object_pose.translation().y() -= 0.02;
        if (object_pose.translation().y() < -0.6) {
            return 1e5;
        }
        
        // get cartesian velocity
        RobotState positions(7);
        RobotState velocities(7);
        std::copy(state.begin(), state.begin() + 7, positions.begin());
        std::copy(state.begin() + 7, state.begin() + 14, velocities.begin());
        std::vector<double> cart_vels;
        m_fd_iface->computeForwardVelocity(positions, velocities, cart_vels);
        // printf("%0.3f %0.3f %0.3f\n", cart_vels[0], cart_vels[1], cart_vels[2]);
        Eigen::Vector3d ee_velocity(cart_vels[0], cart_vels[1], cart_vels[2]);
        double t1 = GetTimeToIntercept1(object_pose.translation(), object_velocity, p.translation(), 0.3);
        double t2 = GetTimeToIntercept2(object_velocity, ee_velocity);

        auto line = object_pose.translation() - p.translation();
        double distance = line.norm();
        // double t = GetTimeToIntercept3(object_pose.translation(), object_velocity, p.translation(), ee_velocity);


        // double w = 2.5;
        double w = 5.0;
        // printf("state: %d, angular distance %.2f time %.2f\n", state_id, computeAngularDistance(p, object_pose), w * t);
        // getchar();
        // double w1 = 1.0;
        // double w2 = 2.0;
        // double w3 = 0.5;
        // double w1 = 2.5;
        // double w2 = 1.5;
        // double w3 = 1.5;
        double w1 = 2.5;
        double w2 = 1.5;
        double w3 = 1.5;
        double rot_dist = computeAngularDistance(p, object_pose);

        // dist = std::max(dist, w3 * rot_dist);
        // double h1 = std::max(w1 * t1, w2 * t2);

        double h1 = w1 * t1;
        double h2 = w2 * t2 + w3 * rot_dist;
        double dist = std::max(h1, h2);
        // const double dist = t;
        // const double dist = computeAngularDistance(p, object_pose);
        // const double dist = computeAngularDistance(p, object_pose) + w * t;
        // const double dist = std::max(computeAngularDistance(p, object_pose), w * t);

        const int h = FIXED_POINT_RATIO * dist;
        // printf("state: %d | t1: %0.3f t2: %0.3f ad: %0.3f h %d\n", state_id, w1 * t1, w2 * t2, w3 * rot_dist, h);

        // double Y, P, R;
        // angles::get_euler_zyx(p.rotation(), Y, P, R);
        // SMPL_DEBUG_NAMED(LOG, "h(%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f) = %d", p.translation()[0], p.translation()[1], p.translation()[2], Y, P, R, h);
        return h;
    } else if (m_point_ext) {
        Vector3 p;
        if (!m_point_ext->projectToPoint(state_id, p)) {
            return 0;
        }

        auto& goal_pose = planningSpace()->goal().pose;
        Vector3 gp(goal_pose.translation());

        double dist = computeDistance(p, gp);

        const int h = FIXED_POINT_RATIO * dist;
        SMPL_DEBUG_NAMED(LOG, "h(%d) = %d", state_id, h);
        return h;
    } else {
        return 0;
    }
}

int ConveyorManipHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int ConveyorManipHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    // printf("FROM %d TO %d\n", from_id, to_id);
    if (from_id == planningSpace()->getGoalStateID()) {
        return 0;
    } else if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        const RobotState& from_state = m_ers->extractState(from_id);
        const RobotState& to_state = m_ers->extractState(to_id);

        double diff_time = to_state.back() - from_state.back();
        if (diff_time < 0) {
            return 10000000;    //infinity causes problems
        }

        // add time steps respecting velocity limits
        double max_time = 0.0;
        for (size_t j = 0; j < planningSpace()->robot()->jointVariableCount(); ++j) {
            auto from_pos = from_state[j];
            auto to_pos = to_state[j];
            auto vel = planningSpace()->robot()->velLimit(j);
            if (vel <= 0.0) {
                continue;
            }
            auto t = 0.0;
            if (planningSpace()->robot()->isContinuous(j)) {
                t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
            } else {
                t = fabs(to_pos - from_pos) / vel;
            }
            max_time = std::max(max_time, t);
        }

        if (max_time > diff_time) {
            return 10000000;
        }

        // Affine3 from_p;
        // Affine3 to_p;
        // if (!m_pose_ext->projectToPose(from_id, from_p) || !m_pose_ext->projectToPose(to_id, to_p)) {
        //     return 0;
        // }

        double w = 5.0;
        // const double dist = std::max(computeAngularDistance(from_p, to_p), w * max_time);
        const double dist = w * max_time;

        const int h = FIXED_POINT_RATIO * dist;
        return h;
    }

    // TODO:
    // Find time difference?
    if (m_pose_ext) {
        if (from_id == planningSpace()->getGoalStateID()) {
            auto& gp = planningSpace()->goal().pose;
            Affine3 p;
            if (!m_pose_ext->projectToPose(to_id, p)) {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(gp, p));
        } else if (to_id == planningSpace()->getGoalStateID()) {
            auto& gp = planningSpace()->goal().pose;
            Affine3 p;
            if (!m_pose_ext->projectToPose(from_id, p)) {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(p, gp));
        } else {
            Affine3 a, b;
            if (!m_pose_ext->projectToPose(from_id, a) ||
                !m_pose_ext->projectToPose(to_id, b))
            {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(a, b));
        }
    } else if (m_point_ext) {
        if (from_id == planningSpace()->getGoalStateID()) {
            Vector3 gp(planningSpace()->goal().pose.translation());
            Vector3 p;
            if (!m_pose_ext->projectToPoint(to_id, p)) {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(gp, p));
        } else if (to_id == planningSpace()->getGoalStateID()) {
            Vector3 gp(planningSpace()->goal().pose.translation());
            Vector3 p;
            if (!m_pose_ext->projectToPoint(from_id, p)) {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(p, gp));
        } else {
            Vector3 a, b;
            if (!m_pose_ext->projectToPoint(from_id, a) ||
                !m_pose_ext->projectToPoint(to_id, b))
            {
                return 0;
            }
            return (int)(FIXED_POINT_RATIO * computeDistance(a, b));
        }
    } else {
        return 0;
    }
}

Affine3 ConveyorManipHeuristic::createPose(
    const std::vector<double> &pose) const
{
    return createPose(pose[0], pose[1], pose[2], pose[5], pose[4], pose[3]);
}

Vector3 ConveyorManipHeuristic::createPoint(
    const std::vector<double>& point) const
{
    return Vector3(point[0], point[1], point[2]);
}

Affine3 ConveyorManipHeuristic::createPose(
    double x, double y, double z,
    double Y, double P, double R) const
{
    return Affine3(
            Translation3(x, y, z) *
            AngleAxis(Y, Vector3::UnitZ()) *
            AngleAxis(P, Vector3::UnitY()) *
            AngleAxis(R, Vector3::UnitX()));
}

double ConveyorManipHeuristic::computeAngularDistance(
    const Affine3& a,
    const Affine3& b) const
{
    Quaternion qb(b.rotation());
    Quaternion qa(a.rotation());

    double dot = qa.dot(qb);
    if (dot < 0.0) {
        qb = Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
        dot = qa.dot(qb);
    }

    dot = std::min(dot, 1.0);

    double dr2 = angles::normalize_angle(2.0 * std::acos(dot));
    dr2 *= (m_rot_coeff * dr2);

    SMPL_DEBUG_NAMED(LOG, "Compute Angular Distance: ", dr2);

    return dr2;
}

double ConveyorManipHeuristic::computeDistance(
    const Affine3& a,
    const Affine3& b) const
{
    auto sqrd = [](double d) { return d * d; };

    Vector3 diff = b.translation() - a.translation();

    double dp2 =
            m_x_coeff * sqrd(diff.x()) +
            m_y_coeff * sqrd(diff.y()) +
            m_z_coeff * sqrd(diff.z());

    Quaternion qb(b.rotation());
    Quaternion qa(a.rotation());

    double dot = qa.dot(qb);
    if (dot < 0.0) {
        qb = Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
        dot = qa.dot(qb);
    }

    dot = std::min(dot, 1.0);

    double dr2 = angles::normalize_angle(2.0 * std::acos(dot));
    dr2 *= (m_rot_coeff * dr2);

    SMPL_DEBUG_NAMED(LOG, "Compute Distance: sqrt(%f + %f)", dp2, dr2);

    return std::sqrt(dp2 + dr2);
}

double ConveyorManipHeuristic::computeDistance(
    const Vector3& u,
    const Vector3& v) const
{
    auto sqrd = [](double d) { return d * d; };
    Vector3 diff = v - u;
    return m_x_coeff * sqrd(diff.x()) +
            m_y_coeff * sqrd(diff.y()) +
            m_z_coeff * sqrd(diff.z());
}

} // namespace smpl
