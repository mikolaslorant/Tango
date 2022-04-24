#ifndef Actor_H_
#define Actor_H_

//#pragma once

#include "aTransform.h"

#include "aJoint.h"
#include "aSkeleton.h"
#include "aBVHController.h"
#include "aIKController.h"
#include "aBehaviorController.h"


//class BVHController;
//class IKController;
class BehaviorController;

class AActor
{

public:
	AActor();
	AActor(const AActor* actor); 
	virtual ~AActor();

	virtual AActor& operator=(const AActor& actor); 
	void clear();
	void update();


	ASkeleton* getSkeleton();
	void setSkeleton(ASkeleton* pExternalSkeleton);
	void resetSkeleton();
	BVHController* getBVHController();
	IKController* getIKController();
	BehaviorController* getBehaviorController();

	// Update the guide position to the hip/root position and update the guide orientation towards guideTarget
	void updateGuideJoint(vec3 guideTargetPos);


	const AJoint& getGuideJoint() const { return m_Guide; }
	void resetGuide() { m_Guide.setGlobalRotation(IdentityMat3); m_Guide.setGlobalTranslation(vec3(0)); }

	// Solve the foot IK position and orientation based on the height and normal of the terrain
	//void solveFootIK(float leftHeight, float rightHeight, float lastLeftFoot, float lastRightFoot,
	//	float footSpeed, float lastRootOffset, float rootSpeed,
	//	bool rotateLeft, bool rotateRight, 
	//	vec3 leftNormal, vec3 rightNormal);
	void solveFootIK(float leftHeight, float rightHeight,
		bool rotateLeft, bool rotateRight, 
		vec3 leftNormal, vec3 rightNormal);

protected:
	// the actor owns the skeleton and controllers
	ASkeleton* m_pSkeleton;
	ASkeleton* m_pInternalSkeleton;
	BVHController *m_BVHController;
	IKController *m_IKController;
	BehaviorController* m_BehaviorController;
	AJoint m_Guide; // Considered as the parent joint of the hip/root joint
};

#endif