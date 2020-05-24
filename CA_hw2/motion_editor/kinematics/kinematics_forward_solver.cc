#include "kinematics_forward_solver.h"
#include <algorithm>
#include "math_utils.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "helper_forward_kinematics.h"
#include "kinematics_artic_idx.h"
#include "kinematics_pose.h"

namespace kinematics {

// public func.

ForwardSolver::ForwardSolver()
    :skeleton_(nullptr),
    motion_(nullptr),
    artic_path_(new ArticIdxColl_t),
    helper_fk_(new helper::ForwardKinematics)
{
}

ForwardSolver::~ForwardSolver()
{
}

std::shared_ptr<acclaim::Skeleton> ForwardSolver::skeleton() const
{
    return skeleton_;
}

std::shared_ptr<acclaim::Motion> ForwardSolver::motion() const
{
    return motion_;
}

void ForwardSolver::set_skeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton)
{
    skeleton_ = skeleton;
    helper_fk_->set_skeleton(skeleton_);
}

void ForwardSolver::set_motion(const std::shared_ptr<acclaim::Motion> &motion)
{
    motion_ = motion;
    helper_fk_->set_skeleton(skeleton_);
}

void ForwardSolver::ConstructArticPath()
{
    helper_fk_->ConstructArticPath();
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const int32_t frame_idx)
{
    return this->ComputeSkeletonPose(motion_->joint_spatial_pos(frame_idx));
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const math::Vector6dColl_t &joint_spatial_pos)
{
	PoseColl_t results;
	results.resize(joint_spatial_pos.size());
	
	std::shared_ptr<acclaim::Skeleton> skeleton = this->skeleton();
	const int rootIdx = skeleton->root_idx();
	const acclaim::Bone* root = skeleton->bone_ptr(rootIdx);
	const math::Vector3d_t rootPosition = joint_spatial_pos[rootIdx].linear_vector();

	std::function<void(const acclaim::Bone*, math::Vector3d_t, math::Quaternion_t)> Traversal = [&](const acclaim::Bone* parentBone, math::Vector3d_t parentPosition, math::Quaternion_t parentRotation) {
		for (int i = 0; i < skeleton->bone_num(); ++i) {
			auto boneIdx = i;
			auto bone = skeleton->bone_ptr(boneIdx);
			auto joint_local_pos_rot = joint_spatial_pos[bone->idx];
			auto joint_local_rot = joint_local_pos_rot.angular_vector();
			auto joint_local_pos = joint_local_pos_rot.linear_vector();
			auto localPosition = joint_local_pos;
			auto localRotation = math::ComputeQuaternionXyz(joint_local_rot.x(), joint_local_rot.y(), joint_local_rot.z());

			if (bone->parent == parentBone) {

				auto currentRotation = parentRotation * localRotation; // Calculate current Rotation
				auto currentPosition = parentPosition + currentRotation * localPosition; // Calculate current Position

				// store the result into "results"
				results[boneIdx] = Pose(
					currentPosition, 
					currentPosition + bone->dir * bone->length, 
					currentRotation.toRotationMatrix()
				);

				std::cout << bone->name << " -> " << parentBone->name << std::endl;
				Traversal(bone, currentPosition, currentRotation);
			}
		}
	};
	
	Traversal(root, rootPosition, math::Quaternion_t()); // DFS Traversal on the bone trees
	
	// results = helper_fk_->ComputeSkeletonPose(joint_spatial_pos);

    // TO DO
	return results;
}

// protected func.

// private func.

} // namespace kinematics {
