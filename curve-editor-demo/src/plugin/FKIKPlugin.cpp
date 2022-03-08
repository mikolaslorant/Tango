#include <stdlib.h> 
#include <math.h> 
#include <string>
#include <vector>
#include "Plugin.h"
#include "aSplineVec3.h"
#include "aSplineQuat.h"
#include "aBVHController.h"
#include "aJoint.h"
#include "aActor.h"
#include <unordered_map>

struct JointData
{
	int id;
	float localRotation[4];	// Quaternion w x y z
	float localTranslation[3]; // Vector3
};

class FKIKPluginManager
{
public:
	FKIKPluginManager() {}

	std::unordered_map<int, std::unique_ptr<AActor>> mActorPool;
	int mCurrentIndex = 0;

	int CreateActor()
	{
		mActorPool.insert({ mCurrentIndex, std::make_unique<AActor>() });
		mCurrentIndex++;
		return mCurrentIndex - 1;
	}

	void RemoveActor(int id)
	{
		mActorPool.erase(id);
	}

	int CreateJoint(int id, char* name, bool isRoot)
	{
		AJoint* joint = new AJoint(name);
		mActorPool[id]->getSkeleton()->addJoint(joint, isRoot);
		return joint->getID();
	}

	void SetJointData(int id, JointData jointData, int parentID)
	{
		AJoint* child = mActorPool[id]->getSkeleton()->getJointByID(jointData.id);
		vec3 v = vec3(jointData.localTranslation[0], jointData.localTranslation[1], jointData.localTranslation[2]);
		child->setLocalTranslation(v);
		quat q = quat(jointData.localRotation[0], jointData.localRotation[1], jointData.localRotation[2], jointData.localRotation[3]);
		child->setLocalRotation(q.ToRotation());
		if (parentID != -1)
		{
			AJoint* parent = mActorPool[id]->getSkeleton()->getJointByID(parentID);
			AJoint::Attach(parent, child);
		}
	}

	void UpdateSkeleton(int id)
	{
		mActorPool[id]->getSkeleton()->update();
	}

	void UpdateIKSkeleton(int id)
	{
		mActorPool[id]->getIKController()->getIKSkeleton()->copyHierarchy(mActorPool[id]->getSkeleton());
	}

	bool LoadBVH(int id, char* file)
	{
		return mActorPool[id]->getBVHController()->load(file);
	}

	int GetJointSize(int id)
	{
		return mActorPool[id]->getSkeleton()->getNumJoints();
	}

	int GetJointIdByName(int id, char* name)
	{
		AJoint* joint = mActorPool[id]->getSkeleton()->getJointByName(name);
		return joint ? joint->getID() : -1;
	}

	int GetJointIdByParentName(int id, char* pname)
	{
		AJoint* parent = mActorPool[id]->getSkeleton()->getJointByName(pname);
		if (parent == NULL || parent->getNumChildren() > 1)
		{
			return -1;
		}
		AJoint* end = parent->getChildAt(0);
		if (end == NULL || end->getNumChildren() != 0)
		{
			return -1;
		}
		return end->getID();
	}

	void GetJointData(int id, JointData* jointDataArray, int size)
	{
		ASkeleton* skeleton = mActorPool[id]->getSkeleton();
		for (int i = 0; i < size; ++i)
		{
			JointData data = jointDataArray[i];
			AJoint* joint = skeleton->getJointByID(data.id);
			quat q = joint->getLocalRotation().ToQuaternion();
			data.localRotation[0] = q.W();
			data.localRotation[1] = q.X();
			data.localRotation[2] = q.Y();
			data.localRotation[3] = q.Z();
			vec3 v = joint->getLocalTranslation();
			data.localTranslation[0] = v[0];
			data.localTranslation[1] = v[1];
			data.localTranslation[2] = v[2];
			jointDataArray[i] = data;
		}
	}

	void UpdateBVHSkeleton(int id, float t)
	{
		mActorPool[id]->getBVHController()->update(t);
	}

	float GetDuration(int id)
	{
		return mActorPool[id]->getBVHController()->getDuration();
	}

	int GetKeySize(int id)
	{
		return mActorPool[id]->getBVHController()->getKeySize();
	}

	float GetKeyTime(int id, int keyID)
	{
		return mActorPool[id]->getBVHController()->getKeyTime(keyID);
	}

	void SetJointRotation(int id, int jointID, quat value)
	{
		mActorPool[id]->getSkeleton()->getJointByID(jointID)->setLocalRotation(value.ToRotation());
	}

	void SetRootJointTranslation(int id, vec3 value)
	{
		mActorPool[id]->getSkeleton()->getRootNode()->setLocalTranslation(value);
	}

	void SetRootJointRotation(int id, quat value)
	{
		mActorPool[id]->getSkeleton()->getRootNode()->setLocalRotation(value.ToRotation());
	}

	void SolveLimbIK(int id, int jointID, vec3 pos)
	{
		ATarget target;
		target.setGlobalTranslation(pos);
		mActorPool[id]->getIKController()->IKSolver_Limb(jointID, target);
	}

	void SetLeftHandID(int id, int jointID)
	{
		mActorPool[id]->getIKController()->mLhandID = jointID;
	}

	void SetRightHandID(int id, int jointID)
	{
		mActorPool[id]->getIKController()->mRhandID = jointID;
	}

	void SetLeftFootID(int id, int jointID)
	{
		mActorPool[id]->getIKController()->mLfootID = jointID;
	}

	void SetRightFootID(int id, int jointID)
	{
		mActorPool[id]->getIKController()->mRfootID = jointID;
	}

	void SetRootID(int id, int jointID)
	{
		mActorPool[id]->getIKController()->mRootID = jointID;
	}

	void CreateLimbIKChains(int id)
	{
		UpdateIKSkeleton(id);
		mActorPool[id]->getIKController()->createLimbIKchains();
	}

	void SolveFootIK(int id, float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
	{
		mActorPool[id]->solveFootIK(leftHeight, rightHeight, rotateLeft, rotateRight, leftNormal, rightNormal);
	}


	void UpdateGuideJoint(int id, vec3 targetPos)
	{
		mActorPool[id]->updateGuideJoint(targetPos);
	}

	vec3 GetGuideTranslation(int id)
	{
		return mActorPool[id]->getGuideJoint().getGlobalTranslation();
	}
	
	quat GetGuideRotation(int id)
	{
		return mActorPool[id]->getGuideJoint().getGlobalRotation().ToQuaternion();
	}
};


extern "C"
{
	static FKIKPluginManager mFKIKPluginManager;

	// Return the ID of the actor
	EXPORT_API int CreateActor()
	{
		return mFKIKPluginManager.CreateActor();
	}

	EXPORT_API void RemoveActor(int id)
	{
		mFKIKPluginManager.RemoveActor(id);
	}

	// Return the ID of the joint
	EXPORT_API int CreateJoint(int id, char* name, bool isRoot)
	{
		return mFKIKPluginManager.CreateJoint(id, name, isRoot);
	}

	EXPORT_API void SetJointData(int id, JointData jointData, int parentID)
	{
		mFKIKPluginManager.SetJointData(id, jointData, parentID);
	}

	EXPORT_API void UpdateSkeleton(int id)
	{
		mFKIKPluginManager.UpdateSkeleton(id);
	}

	EXPORT_API void UpdateIKSkeleton(int id)
	{
		mFKIKPluginManager.UpdateIKSkeleton(id);
	}

	EXPORT_API bool LoadBVH(int id, char* file)
	{
		return mFKIKPluginManager.LoadBVH(id, file);
	}

	// Return the number of the joints
	EXPORT_API int GetJointSize(int id)
	{
		return mFKIKPluginManager.GetJointSize(id);
	}

	// Return the id of the joint by its name
	EXPORT_API int GetJointIdByName(int id, char* name)
	{
		return mFKIKPluginManager.GetJointIdByName(id, name);
	}

	// Return the id of the end joint by its parent name, 
	// becasue in .BVH file, the end joints do not have name
	// If the joint is not the end joint, it will return -1
	EXPORT_API int GetJointIdByParentName(int id, char* name)
	{
		return mFKIKPluginManager.GetJointIdByParentName(id, name);
	}

	EXPORT_API void GetJointData(int id, JointData* jointDataArray, int size)
	{
		return mFKIKPluginManager.GetJointData(id, jointDataArray, size);
	}

	EXPORT_API void UpdateBVHSkeleton(int id, float t)
	{
		mFKIKPluginManager.UpdateBVHSkeleton(id, t);
	}

	EXPORT_API float GetDuration(int id)
	{
		return mFKIKPluginManager.GetDuration(id);
	}

	EXPORT_API int GetKeySize(int id)
	{
		return mFKIKPluginManager.GetKeySize(id);
	}

	// Return time of the key
	EXPORT_API float GetKeyTime(int id, int keyID)
	{
		return mFKIKPluginManager.GetKeyTime(id, keyID);
	}

	EXPORT_API void SetJointRotation(int id, int jointID, float q[])
	{
		mFKIKPluginManager.SetJointRotation(id, jointID, quat(q[0], q[1], q[2], q[3]));
	}

	EXPORT_API void SetRootJointTranslation(int id, float pos[])
	{
		mFKIKPluginManager.SetRootJointTranslation(id, vec3(pos[0], pos[1], pos[2]));
	}

	EXPORT_API void SetRootJointRotation(int id, float q[])
	{
		mFKIKPluginManager.SetRootJointRotation(id, quat(q[0], q[1], q[2], q[3]));
	}

	EXPORT_API void SolveLimbIK(int id, int jointID, float pos[])
	{
		mFKIKPluginManager.SolveLimbIK(id, jointID, vec3(pos[0], pos[1], pos[2]));
	}

	EXPORT_API void SetLeftHandID(int id, int jointID)
	{
		mFKIKPluginManager.SetLeftHandID(id, jointID);
	}

	EXPORT_API void SetRightHandID(int id, int jointID)
	{
		mFKIKPluginManager.SetRightHandID(id, jointID);
	}

	EXPORT_API void SetLeftFootID(int id, int jointID)
	{
		mFKIKPluginManager.SetLeftFootID(id, jointID);
	}

	EXPORT_API void SetRightFootID(int id, int jointID)
	{
		mFKIKPluginManager.SetRightFootID(id, jointID);
	}

	EXPORT_API void SetRootID(int id, int jointID)
	{
		mFKIKPluginManager.SetRootID(id, jointID);
	}

	EXPORT_API void CreateLimbIKChains(int id)
	{
		mFKIKPluginManager.CreateLimbIKChains(id);
	}

	EXPORT_API void SolveFootIK(int id, float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight,
		float leftNormal[], float rightNormal[])
	{
		mFKIKPluginManager.SolveFootIK(id, leftHeight, rightHeight, rotateLeft, rotateRight,
			vec3(leftNormal[0], leftNormal[1], leftNormal[2]), vec3(rightNormal[0], rightNormal[1], rightNormal[2]));
	}

	EXPORT_API void UpdateGuideJointByTarget(int id, float targetPos[], float* newPos, float* newQuat)
	{
		mFKIKPluginManager.UpdateGuideJoint(id, vec3(targetPos[0], targetPos[1], targetPos[2]));
		vec3 pos = mFKIKPluginManager.GetGuideTranslation(id);
		quat q = mFKIKPluginManager.GetGuideRotation(id);

		newPos[0] = pos[0]; newPos[1] = pos[1]; newPos[2] = pos[2];
		newQuat[0] = q.W(); newQuat[1] = q.X(); newQuat[2] = q.Y(); newQuat[3] = q.Z();
	}

}