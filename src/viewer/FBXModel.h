#pragma once
#include <ofbx.h>
#include <string>
#include <vector>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm.hpp>
#include <unordered_map>
#include "shader.h"
#include "aActor.h"
#include "aBVHController.h"
#include "drawable.h"
#include "utils.h"

constexpr int MAXJOINTNUM = 6;
struct FBXVertex
{
	glm::vec3 pos;
	glm::vec3 nor;
	int jointNum;
	glm::vec2 jointWeight[MAXJOINTNUM];	// first: joint id, second: weight
};

struct IKTarget
{
	int jointID;
	std::string jointName;
	AJoint* joint;
	float targetPos[3];
	ATarget target;
};

class FBXModel
{
public:
	FBXModel();
	~FBXModel();

	bool loadFBX(const std::string& filename);
	bool loadBVHMotion(const std::string& filename, bool updateShaderBindMats = false);
	bool loadShaders();

	void drawModel(const glm::mat4& projView, const glm::mat4& model,
			const glm::vec3& lightPos, const glm::vec3& color);
	void drawTargets(const glm::mat4& projView, const glm::mat4& model, const glm::vec3& color, float size);
	void drawSkeleton(const glm::mat4& projView, const glm::mat4& model, const glm::vec3& color);

	void createShader(const std::string& vert, const std::string& frag);
	void setShaderBindMats();
	void setShaderJointTransMats();

	void updateDeltaT(float deltaT);	// Update the model by a timestep
	void updateT(float t);	// Update the model to time t;

	void computeIK(int type, IKTarget& ikTarget);



	std::vector<IKTarget> mIKTargets;

	AActor mActor;
	BVHController* mBVHController;
	IKController *mIKController;
	ASkeleton *mSkeleton;

private:
	ofbx::IScene* scene;

	GLuint VAO;	// Vertex buffer obj
	GLuint VBO; // Vertex buffer obj
	GLuint EBO;	// Element/index buffer obj
	int numTriangles;	// Number of triangles

	std::unordered_map<const ofbx::Object*, int> mJointMap;		// Map between joint node and the index of this joint in shader
	std::unordered_map<int, const ofbx::Object*> mSkeletonMap;		// Map between joint id in actor's skeleton and the joint node
	std::unordered_map<int, std::string> mIKJointMap;	// Map between IK joint id and the joint name;

	
	
	// Add all FBX joints to the mJointMap
	void findAllJoints();
	// Recursively add joints
	void traverseJoints(const ofbx::Object* node);

	// Construct the skeleton map and the IK skeleton
	bool constructSkeleton();

	// Find 4 limb joints and the hips joint
	void setLimbJoints(AJoint* joint);

	std::unique_ptr<Drawable> mDrawableSkeleton;
	std::unique_ptr<Drawable> mDrawableTargets;
	std::unique_ptr<Shader> mFBXShader;	// Lambert and skinning shader for the model 
	std::unique_ptr<Shader> mTargetPointShader;	// 3D point shader for targets
	std::unique_ptr<Shader> mSkeletonShader;	// 3D skeleton shader

	float mTime = 0;
};