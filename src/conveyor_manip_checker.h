#ifndef sbpl_collision_object_manip_checker_h
#define sbpl_collision_object_manip_checker_h

#include <fcl/narrowphase/collision_object.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/collision_checker.h>
#include <smpl/spatial.h>

#include <smpl_urdf_robot_model/smpl_urdf_robot_model.h>

namespace smpl {
namespace collision {
class CollisionSpace;
}
}

struct LinkCollision
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // fixed transform from link origin to geometry origin
    Eigen::Affine3d origin;

    // the collision object, storing the geometry and its current transform
    std::unique_ptr<fcl::CollisionObjectf> collision_object;

    std::string link_name;
};

// The collision model for a link, as a set of collision geometries
struct LinkCollisionModel
{
    std::vector<LinkCollision> models;
};

class ConveyorManipChecker : public smpl::CollisionChecker
{
public:

    smpl::collision::CollisionSpace* parent = NULL;

    bool isStateValid(
        const smpl::RobotState& state,
        bool verbose = false) override;

    bool isStateToStateValid(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        bool verbose = false) override;

    bool interpolatePath(
        const smpl::RobotState& start_positions,
        const smpl::RobotState& finish_positions,
        std::vector<smpl::RobotState>& path) override;

    auto getCollisionModelVisualization(const smpl::RobotState& state)
        -> std::vector<smpl::visual::Marker> override;

    auto getExtension(size_t class_code) -> smpl::Extension* override;

    void setObjectInitialPose(const Eigen::Affine3d& pose);

    smpl::ForwardKinematicsInterface* m_fk_iface = nullptr;

    Eigen::Affine3d object_init_pose;
    Eigen::Vector3d object_velocity;
    smpl::collision::CollisionObject* collision_object_ptr;

    // fcl checking
    std::map<std::string, std::string> link_to_mesh_map;
    std::vector<LinkCollisionModel> link_collision_models;
    std::unique_ptr<fcl::CollisionObjectf> obj_conveyor;

    // for gripper visualization
    std::map<std::string, Eigen::Affine3d> link_to_pose_map;

    double dist_thresh_fcl;

private:

    void updateGripperMeshesState();
    void updateObjectSpheresState(const smpl::RobotState& state);
    bool checkStateFCL(
        const smpl::RobotState& state,
        const Eigen::Affine3d& pose_object);
};

bool Init(
    ConveyorManipChecker* checker,
    smpl::RobotModel* _robot,
    smpl::urdf::RobotModel* robot,
    smpl::collision::CollisionSpace* parent,
    smpl::collision::CollisionObject* collision_object,
    Eigen::Vector3d object_velocity);

#endif
