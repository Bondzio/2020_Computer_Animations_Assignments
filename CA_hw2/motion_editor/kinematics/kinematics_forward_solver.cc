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
	std::function<math::Quaternion_t(math::Vector3d_t)> ComputeQuaternionXyz = [](math::Vector3d_t v) -> math::Quaternion_t {
		return math::ComputeQuaternionXyz(v.x(), v.y(), v.z());
	};

	PoseColl_t results;
	results.resize(joint_spatial_pos.size());
	
	std::shared_ptr<acclaim::Skeleton> skeleton = this->skeleton();
	
	std::function<void(const acclaim::Bone*, math::Vector3d_t, math::Quaternion_t)> Traversal = [&Traversal, &results, &skeleton, &joint_spatial_pos, &ComputeQuaternionXyz]
		(const acclaim::Bone* parentBone, math::Vector3d_t parentEndEffectorWorldPosition, math::Quaternion_t accumulated_R) -> void {
		
		for (int i = 0; i < skeleton->bone_num(); ++i) {
			
			const int boneIdx = i;
			const acclaim::Bone* bone = skeleton->bone_ptr(boneIdx);
			const math::Vector6d_t joint_local_pos_rot = joint_spatial_pos[boneIdx];
			
			const math::Vector3d_t jointPosition = joint_local_pos_rot.linear_vector();
			const math::Quaternion_t jointRotation = ComputeQuaternionXyz(math::ToRadian(joint_local_pos_rot.angular_vector())); // Note it is DEGREE

			if (bone->parent == parentBone) {

				math::Quaternion_t R_asf, R_amc;

				const math::Vector3d_t   boneAxis = bone->axis; // Note it is DEGREE
				const math::Quaternion_t worldBoneRotation = ComputeQuaternionXyz(math::ToRadian(boneAxis));

				// Compute R_asf
				if (bone->parent == nullptr) { // root
					R_asf = worldBoneRotation;
				}
				else {
					const math::Vector3d_t   parentBoneAxis = bone->parent->axis;
					const math::Quaternion_t parentBoneWorldBoneRotation = ComputeQuaternionXyz(math::ToRadian(parentBoneAxis));
					R_asf = parentBoneWorldBoneRotation.inverse() * worldBoneRotation;
				}
				
				// Compute R_amc
				R_amc = jointRotation;

				// R accumlates (R_asf * R_amc), note the order of multiplication differs when calculating endPosition
				const math::Quaternion_t R = accumulated_R * R_asf * R_amc;
				const math::Vector3d_t   endPosition = accumulated_R * R_asf * (R_amc * bone->dir * bone->length + jointPosition) + parentEndEffectorWorldPosition;

				// deal special cases if dealing root bone (in this case parentEndEffectorWorldPosition equals skeleton->root_pos())
				const math::Vector3d_t startPosition = (bone->parent == nullptr) ? (R_asf * jointPosition + parentEndEffectorWorldPosition) : parentEndEffectorWorldPosition;

				// Store results
				results[boneIdx] = Pose(
					startPosition,
					endPosition,
					R.toRotationMatrix()
				);

				Traversal(bone, endPosition, R);
			}
		}
	};

	Traversal(nullptr, skeleton->root_pos(), math::Quaternion_t::Identity()); // DFS Traversal on the bone trees

	return results;
}

// protected func.

// private func.

} // namespace kinematics {
