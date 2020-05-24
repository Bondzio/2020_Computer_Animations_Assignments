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
	std::function<math::Quaternion_t(math::Vector3d_t)> ComputeQuaternionXyz = [](math::Vector3d_t v) ->math::Quaternion_t {
		return math::ComputeQuaternionXyz(v.x(), v.y(), v.z());
	};

	PoseColl_t results;
	results.resize(joint_spatial_pos.size());
	
	std::shared_ptr<acclaim::Skeleton> skeleton = this->skeleton();
	
	const int rootIdx = skeleton->root_idx();
	const acclaim::Bone* root = skeleton->bone_ptr(rootIdx);
	
	std::function<void(const acclaim::Bone*, math::Vector3d_t, math::Vector3d_t, math::Quaternion_t)> Traversal =
		// [&Traversal, &results, &skeleton, &joint_spatial_pos, &ComputeQuaternionXyz]
		[&](const acclaim::Bone* parentBone, math::Vector3d_t parentWorldBonePosition, math::Vector3d_t parentWorldPosition, math::Quaternion_t parentWorldRotation) -> void {
		
		for (int i = 0; i < skeleton->bone_num(); ++i) {
			
			const int boneIdx = i;
			const acclaim::Bone* bone = skeleton->bone_ptr(boneIdx);
			const math::Vector6d_t joint_local_pos_rot = joint_spatial_pos[boneIdx];
			
			const math::Vector3d_t jointPosition = joint_local_pos_rot.linear_vector();
			const math::Quaternion_t jointRotation = ComputeQuaternionXyz(math::ToRadian(joint_local_pos_rot.angular_vector()));

			if (bone->parent == parentBone) {

				const math::Vector3d_t   parentBoneAxis = bone->parent->axis; // Note it is DEGREE !!!!!!!!!!!!!!!!!!!!!
				const math::Quaternion_t parentBoneWorldBoneRotation = ComputeQuaternionXyz(math::ToRadian(parentBoneAxis));

				const math::Vector3d_t   boneAxis = bone->axis; // Note it is DEGREE !!!!!!!!!!!!!!!!!!!!!
				const math::Quaternion_t worldBoneRotation = ComputeQuaternionXyz(math::ToRadian(boneAxis));
				const math::Vector3d_t   worldBonePosition = (worldBoneRotation * bone->dir) * bone->length + parentWorldBonePosition; // v_i-1 = R_amc * V0_hat * L

				const math::Quaternion_t R_asf = parentBoneWorldBoneRotation.inverse() * worldBoneRotation;
				const math::Quaternion_t R_amc = jointRotation;

				// Calculate current Position & Rotation in worldSpace
				const math::Quaternion_t worldRotation = parentWorldRotation * R_asf * R_amc;				
				// const math::Vector3d_t   worldPosition = worldRotation * (bone->dir * bone->length + jointPosition) + parentWorldPosition;
				const math::Vector3d_t   worldPosition = parentWorldRotation * R_asf * (R_amc * bone->dir * bone->length + jointPosition) + parentWorldPosition;

				// bind pose only
				/*results[boneIdx] = Pose(
					parentWorldBonePosition,
					worldBonePosition,
					worldBoneRotation.toRotationMatrix()
				);
				Traversal(bone, worldBonePosition, worldPosition, worldBoneRotation);
				continue;*/

				// store the result into "results"
				results[boneIdx] = Pose(
					parentWorldPosition,
					worldPosition,
					worldRotation.toRotationMatrix()
				);

				// std::cout << bone->name << " -> " << parentBone->name << std::endl;
				Traversal(bone, worldBonePosition, worldPosition, worldRotation);
			}
		}
	};

	const math::Vector3d_t   rootBoneAxis = root->axis;
	const math::Quaternion_t rootLocalBoneRotation = ComputeQuaternionXyz(math::ToRadian(rootBoneAxis));
	
	const math::Vector3d_t   rootJointPosition = joint_spatial_pos[rootIdx].linear_vector();
	const math::Quaternion_t rootJointRotation = ComputeQuaternionXyz(math::ToRadian(joint_spatial_pos[rootIdx].angular_vector()));

	const math::Quaternion_t rootWorldBoneRotation = rootLocalBoneRotation;
	const math::Vector3d_t   rootWorldBonePosition = rootLocalBoneRotation * root->dir * root->length + skeleton->root_pos();

	const math::Quaternion_t R_asf = rootWorldBoneRotation;
	const math::Quaternion_t R_amc = rootJointRotation;
	
	const math::Quaternion_t rootWorldRotation = R_asf * R_amc;
	const math::Vector3d_t   rootWorldPosition = R_asf * (R_amc * root->dir * root->length + rootJointPosition) + skeleton->root_pos();

	// const math::Vector3d_t   rootWorldPosition = ;
	
	results[rootIdx] = Pose(
		R_asf * rootJointPosition + skeleton->root_pos(),
		rootWorldPosition,
		rootWorldRotation.toRotationMatrix()
	);

	Traversal(root, rootWorldBonePosition, rootWorldPosition, rootWorldRotation); // DFS Traversal on the bone trees

	// results = helper_fk_->ComputeSkeletonPose(joint_spatial_pos);

    // TO DO
	return results;
}

// protected func.

// private func.

} // namespace kinematics {
