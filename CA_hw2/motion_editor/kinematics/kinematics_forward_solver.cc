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
	PoseColl_t results;	// return instance
	
	std::shared_ptr<acclaim::Skeleton> skeleton = this->skeleton();
	const acclaim::Bone* root = skeleton->bone_ptr(skeleton->root_idx());

	std::function<void(const acclaim::Bone*)> Traversal = [&](const acclaim::Bone* parentBone) {
		for (int i = 0; i < skeleton->bone_num(); ++i) {
			auto bone = skeleton->bone_ptr(i);
			if (bone->parent == parentBone) {
				std::cout << bone->name << " -> " << parentBone->name << std::endl;
				Traversal(bone);
			}
		}
	};
	
	Traversal(root); // DFS Traversal on the bone trees
	
	results = helper_fk_->ComputeSkeletonPose(joint_spatial_pos);

    // TO DO
	return results;
}

// protected func.

// private func.

} // namespace kinematics {