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

	const math::Vector3d_t root_joint_pos = joint_spatial_pos[rootIdx].linear_vector();
	const math::Vector3d_t root_joint_rot = joint_spatial_pos[rootIdx].angular_vector();
	
	// const math::Vector3d_t rootLocalPosition = root_joint_pos; // simple rename to make varialble naming consistent
	// const math::Quaternion_t rootRotation = math::ComputeQuaternionXyz(root_joint_rot.x(), root_joint_rot.y(), root_joint_rot.z());

	// const math::Vector3d_t rootLocalPosition = skeleton->root_pos();

	auto ComputeQuaternionXyz = [](math::Vector3d_t v) ->math::Quaternion_t {
		return math::ComputeQuaternionXyz(v.x(), v.y(), v.z());
	};

	std::function<void(const acclaim::Bone*, math::Vector3d_t, math::Quaternion_t)> Traversal = 
		[&](const acclaim::Bone* parentBone, math::Vector3d_t parentWorldPosition, math::Quaternion_t parentWorldRotation) {
		
		for (int i = 0; i < skeleton->bone_num(); ++i) {
			
			const int boneIdx = i;
			const acclaim::Bone* bone = skeleton->bone_ptr(boneIdx);
			const math::Vector6d_t joint_local_pos_rot = joint_spatial_pos[boneIdx];
			const math::Vector3d_t joint_local_rot = joint_local_pos_rot.angular_vector();
			const math::Vector3d_t joint_local_pos = joint_local_pos_rot.linear_vector();

			const math::Vector3d_t   localPosition = joint_local_pos; // simple rename to make varialble naming consistent
			const math::Quaternion_t localRotation = ComputeQuaternionXyz(joint_local_rot);

			if (bone->parent == parentBone) {

				// Calculate current Position & Rotation
				// math::Quaternion_t worldRotation = parentWorldRotation * localRotation;
				// math::Quaternion_t inverseBindPoseMatrix = worldRotation.inverse();
				// math::Vector3d_t   worldPosition = worldRotation * localPosition + worldRotation * bone->dir * bone->length + parentWorldPosition;

				const math::Vector3d_t   boneAxis = bone->axis; // Note it is DEGREE !!!!!!!!!!!!!!!!!!!!!
				const math::Quaternion_t boneWorldRotation = ComputeQuaternionXyz(math::ToRadian(boneAxis));

				math::Quaternion_t worldRotation = nullptr;
				math::Vector3d_t   worldPosition = boneWorldRotation * bone->dir * bone->length + parentWorldPosition;

				// store the result into "results"
				results[boneIdx] = Pose(
					parentWorldPosition,
					worldPosition,
					boneWorldRotation.toRotationMatrix()
				);

				// std::cout << bone->name << " -> " << parentBone->name << std::endl;
				Traversal(bone, worldPosition, boneWorldRotation);
			}
		}
	};

	const math::Vector3d_t   rootBoneAxis = root->axis;
	const math::Quaternion_t rootLocalRotation = ComputeQuaternionXyz(math::ToRadian(rootBoneAxis));	
	const math::Vector3d_t   rootWorldPosition = skeleton->root_pos() + rootLocalRotation * root->dir * root->length;
	const math::Quaternion_t rootWorldRotation = rootLocalRotation;
	
	results[rootIdx] = Pose(
		skeleton->root_pos(),
		rootWorldPosition,
		rootWorldRotation.toRotationMatrix()
	);

	Traversal(root, rootWorldPosition, rootWorldRotation); // DFS Traversal on the bone trees

	// results = helper_fk_->ComputeSkeletonPose(joint_spatial_pos);

    // TO DO
	return results;
}

// protected func.

// private func.

} // namespace kinematics {
