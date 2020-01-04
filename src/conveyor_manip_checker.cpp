#include "conveyor_manip_checker.h"

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision_request.h> // CollisionRequest, CollisionResult
#include <geometric_shapes/mesh_operations.h>
#include <sbpl_collision_checking/shape_visualization.h>
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_conversions.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/stl/memory.h>

static
void transformEigenToFCL(const Eigen::Affine3d& A, fcl::Transform3f& B)
{
    fcl::Vector3f T(fcl::Vector3f(A.translation().x(), A.translation().y(), A.translation().z()));
    Eigen::Quaterniond q(A.rotation());
    fcl::Matrix3f R(fcl::Quaternionf(q.w(), q.x(), q.y(), q.z()));
    B.linear() = R;
    B.translation() = T;
}

bool Init(
    ConveyorManipChecker* checker,
    smpl::RobotModel* _robot,
    smpl::urdf::RobotModel* robot,
    smpl::collision::CollisionSpace* parent,
    smpl::collision::CollisionObject* collision_object,
    Eigen::Vector3d object_velocity)
{
    checker->parent = parent;
    // checker->object_init_pose = collision_object->shape_poses[0];
    checker->object_velocity = object_velocity;
    checker->collision_object_ptr = collision_object;
    checker->dist_thresh_fcl = 0.3;

    checker->m_fk_iface = _robot->getExtension<smpl::ForwardKinematicsInterface>();

    checker->link_to_mesh_map["r_gripper_palm_link"] = "package://pr2_description/meshes/gripper_v0/gripper_palm.stl";
    checker->link_to_mesh_map["r_gripper_l_finger_link"] = "package://pr2_description/meshes/gripper_v0/l_finger.stl";
    checker->link_to_mesh_map["r_gripper_r_finger_link"] = "package://pr2_description/meshes/gripper_v0/l_finger.stl";
    checker->link_to_mesh_map["r_gripper_l_finger_tip_link"] = "package://pr2_description/meshes/gripper_v0/l_finger_tip.stl";
    checker->link_to_mesh_map["r_gripper_r_finger_tip_link"] = "package://pr2_description/meshes/gripper_v0/l_finger_tip.stl";

    // fcl stuff
    checker->link_collision_models.resize(GetLinkCount(robot));

    for (auto& link : smpl::urdf::Links(robot)) {
        if (checker->link_to_mesh_map.find(link.name) == checker->link_to_mesh_map.end()) {
            continue;
        }
        for (auto& c : link.collision) {
            auto* mesh_smpl = static_cast<smpl::urdf::Mesh*>(c.shape);
            if (c.shape->type != smpl::urdf::ShapeType::Mesh) {
                SMPL_ERROR("Link is not a mesh");
                return false;
            }
            std::unique_ptr<shapes::Mesh> mesh;
            mesh.reset(shapes::createMeshFromResource(mesh_smpl->filename, mesh_smpl->scale));
            std::vector<fcl::Vector3f> vertices;
            std::vector<fcl::Triangle> triangles;

            // ...copy vertices
            vertices.reserve(mesh->vertex_count);
            for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
                double x = mesh->vertices[3 * i    ];
                double y = mesh->vertices[3 * i + 1];
                double z = mesh->vertices[3 * i + 2];
                vertices.push_back(fcl::Vector3f(x, y, z));
            }

            // ...copy triangles
            triangles.reserve(mesh->triangle_count);
            for (unsigned int i = 0; i < mesh->triangle_count; ++i) {
                fcl::Triangle t;
                t[0] = mesh->triangles[3 * i];
                t[1] = mesh->triangles[3 * i + 1];
                t[2] = mesh->triangles[3 * i + 2];
                triangles.push_back(t);
            }

            typedef fcl::BVHModel<fcl::OBBRSSf> Model;
            std::shared_ptr<Model> mesh_geom = std::make_shared<Model>();

            mesh_geom->beginModel();
            mesh_geom->addSubModel(vertices, triangles);
            mesh_geom->endModel();

            auto lc = LinkCollision();
            lc.origin = c.origin;
            lc.collision_object = smpl::make_unique<fcl::CollisionObjectf>(mesh_geom);
            lc.link_name = link.name;

            auto lidx = GetLinkIndex(robot,
                    GetLink(robot, link.name.c_str()));
            checker->link_collision_models[lidx].models.push_back(std::move(lc));
        }
    }

    // TODO: might have to set poses here

    switch (collision_object->shapes[0]->type) {
    case smpl::collision::ShapeType::Cylinder:
    {
        auto* cyl = static_cast<const smpl::collision::CylinderShape*>(collision_object->shapes[0]);
        auto cylinder_geom = std::make_shared<fcl::Cylinderf>(
                cyl->radius, cyl->height);
        fcl::Transform3f cylinder_pose;
        checker->obj_conveyor = smpl::make_unique<fcl::CollisionObjectf>(cylinder_geom, cylinder_pose);
        break;
    }
    case smpl::collision::ShapeType::Box:
    {
        auto* box = static_cast<const smpl::collision::BoxShape*>(collision_object->shapes[0]);
        checker->box_geom_actual = std::make_shared<fcl::Boxf>(
                box->size[0], box->size[1], box->size[2]);
        double inflation = 1.1;
        checker->box_geom_inflated = std::make_shared<fcl::Boxf>(
                box->size[0] * inflation, box->size[1] * inflation, box->size[2] * inflation);
        fcl::Transform3f box_pose;
        checker->obj_conveyor = smpl::make_unique<fcl::CollisionObjectf>(checker->box_geom_inflated, box_pose);
        break;
    }
    default:
    {
        SMPL_ERROR("Unknown shape type.");
        return false;
    }
    }

    return true;
}

void ConveyorManipChecker::setObjectInitialPose(const Eigen::Affine3d& pose)
{
    object_init_pose = pose;
}

void ConveyorManipChecker::inflateCollisionObject()
{
    fcl::Transform3f box_pose;
    obj_conveyor = smpl::make_unique<fcl::CollisionObjectf>(box_geom_inflated, box_pose);
}

void ConveyorManipChecker::deflateCollisionObject()
{
    fcl::Transform3f box_pose;
    obj_conveyor = smpl::make_unique<fcl::CollisionObjectf>(box_geom_actual, box_pose);
}

void ConveyorManipChecker::updateGripperMeshesState()
{
    // open gripper in rcs
    parent->m_rcs->setJointVarPosition("r_gripper_r_finger_joint", 0.548);
    parent->m_rcs->setJointVarPosition("r_gripper_l_finger_joint", 0.548);
    parent->m_rcs->setJointVarPosition("r_gripper_r_finger_tip_joint", 0.548);
    parent->m_rcs->setJointVarPosition("r_gripper_l_finger_tip_joint", 0.548);

    parent->m_rcs->updateLinkTransform("r_gripper_r_finger_link");
    parent->m_rcs->updateLinkTransform("r_gripper_l_finger_link");
    parent->m_rcs->updateLinkTransform("r_gripper_r_finger_tip_link");
    parent->m_rcs->updateLinkTransform("r_gripper_l_finger_tip_link");

    for (auto lidx = 0; lidx < link_collision_models.size(); ++lidx) {
        auto* model = &link_collision_models[lidx];
        for (auto& object : model->models) {
            auto& link_pose = parent->m_rcs->linkTransform(object.link_name) * object.origin;
            fcl::Transform3f T;
            transformEigenToFCL(link_pose, T);
            object.collision_object->setTransform(T);
            object.collision_object->computeAABB();

            // for vis
            link_to_pose_map[object.link_name] = link_pose;
        }
    }
}

void ConveyorManipChecker::updateObjectSpheresState(const smpl::RobotState& state)
{
    // open gripper in cc
    parent->setJointPosition("r_gripper_r_finger_joint", 0.548);
    parent->setJointPosition("r_gripper_l_finger_joint", 0.548);
    parent->setJointPosition("r_gripper_r_finger_tip_joint", 0.548);
    parent->setJointPosition("r_gripper_l_finger_tip_joint", 0.548);

    auto pose = object_init_pose;
    pose.translation().x() += object_velocity[0] * state.back();
    pose.translation().y() += object_velocity[1] * state.back();
    pose.translation().z() += object_velocity[2] * state.back();

    Eigen::Quaterniond q(pose.rotation());
    parent->setJointPosition("conveyor_object_joint/trans_x", pose.translation().x());
    parent->setJointPosition("conveyor_object_joint/trans_y", pose.translation().y());
    parent->setJointPosition("conveyor_object_joint/trans_z", pose.translation().z());
    parent->setJointPosition("conveyor_object_joint/rot_x", q.x());
    parent->setJointPosition("conveyor_object_joint/rot_y", q.y());
    parent->setJointPosition("conveyor_object_joint/rot_z", q.z());
    parent->setJointPosition("conveyor_object_joint/rot_w", q.w());
}

bool ConveyorManipChecker::checkStateFCL(
    const smpl::RobotState& state,
    const Eigen::Affine3d& pose_object)
{
    // update fcl object pose
    fcl::Transform3f T;
    transformEigenToFCL(pose_object, T);
    obj_conveyor->setTransform(T);
    obj_conveyor->computeAABB();

    // Comment: this function updates the robot spheres state:
    auto ma_collision_model = parent->getCollisionModelVisualization(state);

    updateGripperMeshesState();

    for (auto lidx = 0; lidx < link_collision_models.size(); ++lidx) {
        auto* model = &link_collision_models[lidx];
        for (auto& lo : model->models) {
            fcl::CollisionRequest<float> request;
            fcl::CollisionResult<float> result;

            fcl::collide(lo.collision_object.get(), obj_conveyor.get(), request, result);

            if (result.isCollision()) {
                // if (collisionPositions != nullptr) {
                //     for (size_t i = 0; i < result.numContacts(); i++) {
                //         const fcl::Contact& contact = result.getContact(i);
                //         collisionPositions->push_back({contact.pos[0], contact.pos[1], contact.pos[2]});
                //     }
                // }
                return false;
            }
        }
    }
    return true;
}

static
double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool ConveyorManipChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    // Three step collision checking:
    // 1. Do a regular collision check of the edge from start to finish
    // 2. If away from goal then do crude check of start and finish states only
    // 3. If close to goal then do fine check of start and finish states only

    smpl::RobotState state_positions(state.size() - 1);
    std::copy(state.begin(), state.begin() + state.size() - 1, state_positions.begin());
    
    Eigen::Affine3d pose_object;

    pose_object = object_init_pose;
    pose_object.translation().x() += object_velocity[0] * state.back();
    pose_object.translation().y() += object_velocity[1] * state.back();
    pose_object.translation().z() += object_velocity[2] * state.back();

    auto pose_robot = m_fk_iface->computeFK(state_positions);
    double intercept_dist = EuclideanDistance(
                                pose_object.translation()[0],
                                pose_object.translation()[1],
                                pose_object.translation()[2],
                                pose_robot.translation()[0],
                                pose_robot.translation()[1],
                                pose_robot.translation()[2]);

    if (!intercept_dist > dist_thresh_fcl) {
        updateObjectSpheresState(state);
        // if (!parent->isStateValid(state_positions)) {
        //     return false;
        // }
    }
    else {
        if (!checkStateFCL(state_positions, pose_object)) {
            return false;
        }
    }

    return true;
}

bool ConveyorManipChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    bool verbose)
{
    bool vis = false;

    // if (verbose) {
    //     printf("Shortcut %f %f\n", start.back(), finish.back());
    //     // if (fabs(start.back() - 3.6) < 1e-6 && fabs(finish.back() - 3.8) < 1e-6) {
    //     if (fabs(start.back() - 0.0) < 1e-3 && fabs(finish.back() - 3.71017) < 1e-3) {
    //         printf("bad edge\n");
    //         vis = true;
    //     }
    // }

    // Three step collision checking:
    // 1. Do a regular collision check of the edge from start to finish
    // 2. If away from goal then do crude check of start and finish states only
    // 3. If close to goal then do fine check of start and finish states only

    smpl::RobotState start_positions(start.size() - 1);
    std::copy(start.begin(), start.begin() + start.size() - 1, start_positions.begin());

    smpl::RobotState finish_positions(finish.size() - 1);
    std::copy(finish.begin(), finish.begin() + finish.size() - 1, finish_positions.begin());

    // Interpolate path
    std::vector<smpl::RobotState> path;
    interpolatePath(start_positions, finish_positions, path);

    double time_inc = (finish.back() - start.back()) / (path.size() - 1);
    double state_time = start.back();
    
    for (size_t i = 0; i < path.size(); ++i) {

        auto state_full = path[i];
        state_full.push_back(state_time);

        if (vis) {
            SV_SHOW_INFO(getCollisionModelVisualization(state_full));
            printf("showing %zu of total %zu\n", i, path.size());
            getchar();
        }
        Eigen::Affine3d pose_object;

        pose_object = object_init_pose;
        pose_object.translation().x() += object_velocity[0] * state_time;
        pose_object.translation().y() += object_velocity[1] * state_time;
        pose_object.translation().z() += object_velocity[2] * state_time;

        auto pose_robot = m_fk_iface->computeFK(path[i]);
        double intercept_dist = EuclideanDistance(
                                    pose_object.translation()[0],
                                    pose_object.translation()[1],
                                    pose_object.translation()[2],
                                    pose_robot.translation()[0],
                                    pose_robot.translation()[1],
                                    pose_robot.translation()[2]);

        if (intercept_dist > dist_thresh_fcl) {
            updateObjectSpheresState(state_full);
            // Comment: updateState is called in isStateValid
            // if (!parent->isStateValid(path[i])) {
            //     // printf("i %zu size %zu spheres collision \n", i, path.size());
            //     return false;
            // }
        }
        else {
            if (!checkStateFCL(path[i], pose_object)) {
                // printf("i %zu size %zu fcl collision \n", i, path.size());
                return false;
            }
        }
        state_time += time_inc;
    }

    return true;
}

bool ConveyorManipChecker::interpolatePath(
    const smpl::RobotState& start_positions,
    const smpl::RobotState& finish_positions,
    std::vector<smpl::RobotState>& path)
{
    if (!parent->interpolatePath(start_positions, finish_positions, path)) {
        return false;
    }

    return true;
}

auto ConveyorManipChecker::getCollisionModelVisualization(const smpl::RobotState& state)
    -> std::vector<smpl::visual::Marker>
{
    std::vector<smpl::visual::Marker> mm;
    updateObjectSpheresState(state);

    // Spheres model
    smpl::RobotState state_positions(state.size() - 1);
    std::copy(state.begin(), state.begin() + state.size() - 1, state_positions.begin());
    auto ma_collision_model = parent->getCollisionModelVisualization(state_positions);  // updates spheres
    ma_collision_model.clear();
    updateGripperMeshesState();

    // Gripper mesh markers
    visualization_msgs::MarkerArray ma;
    int id = ma_collision_model.back().id;
    for (auto const& it : link_to_pose_map)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = parent->m_grid->getReferenceFrame();
        marker.header.stamp = ros::Time();
        marker.ns = "gripper";
        marker.id = id;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        Eigen::Quaterniond q(it.second.rotation());
        marker.pose.position.x = it.second.translation().x();
        marker.pose.position.y = it.second.translation().y();
        marker.pose.position.z = it.second.translation().z();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.mesh_resource = link_to_mesh_map[it.first];
        ma.markers.push_back(marker);
        id++;
    }
    auto ma_gripper = smpl::visual::ConvertMarkerArrayToMarkers(ma);
    ma_collision_model.insert(ma_collision_model.end(), ma_gripper.begin(), ma_gripper.end());

    // Object geometry marker
    Eigen::Affine3d pose_object;
    pose_object = object_init_pose;
    pose_object.translation().x() += object_velocity[0] * state.back();
    pose_object.translation().y() += object_velocity[1] * state.back();
    pose_object.translation().z() += object_velocity[2] * state.back();

    visualization_msgs::Marker marker;
    smpl::collision::MakeCollisionShapeMarker(*(collision_object_ptr->shapes[0]), marker);
    marker.header.frame_id = parent->m_grid->getReferenceFrame();
    marker.header.stamp = ros::Time();
    marker.ns = "object";
    marker.id = id;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    tf::poseEigenToMsg(pose_object, marker.pose);
    smpl::visual::Marker smpl_marker;
    ConvertMarkerMsgToMarker(marker, smpl_marker);
    ma_collision_model.push_back(smpl_marker);

    return ma_collision_model;
}

auto ConveyorManipChecker::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<CollisionChecker>()) {
        return this;
    }
    return NULL;
}