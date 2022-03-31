#include "FBXModel.h"
#include <fstream>
#include <iostream>
#include <gtc/matrix_transform.hpp>
#include "utils.h"


FBXModel::FBXModel()
{
	mBVHController = mActor.getBVHController();
	mSkeleton = mActor.getSkeleton();
	mIKController = mActor.getIKController();
}

FBXModel::~FBXModel()
{
	if (scene)
	{
		scene->destroy();
	}
}

bool FBXModel::loadFBX(const std::string & filename)
{
	// Load FBX model
	std::ifstream file(filename, std::ios::ate | std::ios::binary);
	if (!file.is_open())
	{
		throw std::runtime_error("failed to open the fbx file.");
		return false;
	}
	size_t fileSize = file.tellg();
	file.seekg(0, file.beg);

	ofbx::u8* buffer = new ofbx::u8[fileSize];
	std::unique_ptr<std::vector<ofbx::u8>> fileBuffer = std::make_unique<std::vector<ofbx::u8>>(fileSize);
	file.read(reinterpret_cast<char*>(fileBuffer->data()), fileSize);

	scene = ofbx::load(fileBuffer->data(), fileSize, static_cast<ofbx::u64>(ofbx::LoadFlags::TRIANGULATE));

	// Construct the mJointMap
	findAllJoints();

	int count = scene->getMeshCount();

	std::vector<int> indicesBuffer;
	std::vector<FBXVertex> vertexBuffer;

	// Iterate all meshes
	for (int i = 0; i < count; ++i)
	{
		const auto* mesh = scene->getMesh(i);
		const auto* geometry = mesh->getGeometry();
		const auto* vertices = geometry->getVertices();
		const auto* normals = geometry->getNormals();
		const auto* skin = geometry->getSkin();

		int vertexCount = geometry->getVertexCount();
		int clusterCount = skin->getClusterCount();

		int vertexOffset = vertexBuffer.size();		// Current vertices number
		// Iterate all vertices
		for (int j = 0; j < vertexCount ; ++j)
		{
			auto v = vertices[j];
			auto n = normals[j];
			vertexBuffer.emplace_back(FBXVertex{ glm::vec3{v.x, v.y, v.z}, glm::vec3{n.x, n.y, n.z}, 0 });
		}
		// Iterate all joints in this cluster that influence the vertices
		for (int j = 0; j < clusterCount; ++j)
		{
			const auto* cluster = skin->getCluster(j);
			const auto* joint = cluster->getLink();
			int indicesCount = cluster->getIndicesCount();
			int weightsCount = cluster->getWeightsCount();
			if (indicesCount == 0) { continue; }

			const auto* indices = cluster->getIndices();
			const auto* weights = cluster->getWeights();
			// Iterate all vertices that are influenced by this joint
			for (int k = 0; k < indicesCount; ++k)
			{
				int index = indices[k] + vertexOffset;
				float weight = static_cast<float>(weights[k]);
				int num = vertexBuffer[index].jointNum;
				vertexBuffer[index].jointWeight[num][0] = mJointMap[joint] + 0.5f;	// To make sure when truncate the float to int, the int is not changed
				vertexBuffer[index].jointWeight[num][1] = weight;
				vertexBuffer[index].jointNum++;
			}
		}
		
		// Construct indices buffer
		const auto* indices = geometry->getFaceIndices();
		for (int j = 0; j < geometry->getIndexCount(); ++j)
		{
			int index = indices[j];
			index = index >= 0 ? index : (-index) - 1;
			indicesBuffer.push_back(vertexOffset + index);
		}
	}

	numTriangles = indicesBuffer.size() / 3;
	// Gen VAO
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);
	// Create a Vector Buffer Object that will store the vertices on video memory
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	// Allocate space and upload the data from CPU to GPU
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, vertexBuffer.size() * sizeof(FBXVertex), vertexBuffer.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(FBXVertex), (void*)0);
	glEnableVertexAttribArray(0);	// pos
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(FBXVertex), (void*)(1 * sizeof(glm::vec3)));
	glEnableVertexAttribArray(1);	// nor
	glVertexAttribIPointer(2, 1, GL_INT, sizeof(FBXVertex), (void*)(2 * sizeof(glm::vec3)));
	glEnableVertexAttribArray(2);	// joint num
	
	for (int i = 3; i < 3 + MAXJOINTNUM; ++i)
	{
		glVertexAttribPointer(i, 2, GL_FLOAT, GL_FALSE, sizeof(FBXVertex), (void*)(2 * sizeof(glm::vec3) + sizeof(int) + (i-3) * sizeof(glm::vec2)));
		glEnableVertexAttribArray(i);	// joint weight (vec2)
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer.size() * sizeof(int), indicesBuffer.data(), GL_STATIC_DRAW);

	return true;
}

bool FBXModel::loadBVHMotion(const std::string & filename, bool updateShaderBindMats)
{
	if (!mBVHController->load(filename))
	{
		throw std::runtime_error("failed to open the bvh file.");
		return false;
	}
	if (!constructSkeleton()) { return false; }
	mBVHController->update(0);
	// Set shader bind matrices
	if (updateShaderBindMats)
	{
		setShaderBindMats();
	}
	// Update target postion
	for (auto& target : mIKTargets)
	{
		vec3 pos = target.joint->getGlobalTranslation();
		target.targetPos[0] = pos[0]; target.targetPos[1] = pos[1]; target.targetPos[2] = pos[2];
	}
	return true;
}

bool FBXModel::loadShaders()
{
	mFBXShader = std::make_unique<Shader>("../shader/betaCharacter.vert.glsl", "../shader/betaCharacter.frag.glsl");
	mTargetPointShader = std::make_unique<Shader>("../shader/point3d.vert.glsl", "../shader/point3d.frag.glsl");
	mSkeletonShader = std::make_unique<Shader>("../shader/skeleton.vert.glsl", "../shader/skeleton.frag.glsl");
	return true;
}

void FBXModel::drawModel(const glm::mat4& projView, const glm::mat4& model,
	const glm::vec3& lightPos, const glm::vec3& color)
{
	if (!mFBXShader)
	{
		throw std::runtime_error("Shader isn't created.");
	}
	vec3 t = mActor.getGuideJoint().getGlobalTranslation();
	mat3 r = mActor.getGuideJoint().getGlobalRotation();
	
	glm::mat4 guideModel = model * toGLMmat4(r, t);

	mFBXShader->use();
	setShaderJointTransMats();
	mFBXShader->setMat4("uProjView", projView);
	mFBXShader->setMat4("uModel", guideModel);
	mFBXShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(guideModel))));
	mFBXShader->setVec3("uLightPos", lightPos);
	mFBXShader->setVec3("color", color);

	glBindVertexArray(VAO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glDrawElements(GL_TRIANGLES, 3 * numTriangles, GL_UNSIGNED_INT, 0);
}

void FBXModel::drawTargets(const glm::mat4 & projView, const glm::mat4 & model, const glm::vec3 & color, float size)
{
	std::vector<float> pos;
	for (const auto& target : mIKTargets)
	{
		for (int i = 0; i < 3; ++i)
		{
			pos.push_back(target.targetPos[i]);
		}
	}
	if (!mDrawableTargets) { mDrawableTargets = std::make_unique<Drawable>(); }

	glDisable(GL_DEPTH_TEST);

	glBindVertexArray(mDrawableTargets->VAO);
	glBindBuffer(GL_ARRAY_BUFFER, mDrawableTargets->VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * pos.size(), pos.data(), GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glEnable(GL_PROGRAM_POINT_SIZE);

	mTargetPointShader->use();
	mTargetPointShader->setMat4("uProjView", projView);
	mTargetPointShader->setMat4("uModel", model);
	mTargetPointShader->setVec3("uColor", color);
	mTargetPointShader->setFloat("uSize", size);

	glDrawArrays(GL_POINTS, 0, pos.size() / 3);
	glDisable(GL_PROGRAM_POINT_SIZE);
	glEnable(GL_DEPTH_TEST);
}

void FBXModel::drawSkeleton(const glm::mat4 & projView, const glm::mat4 & model, const glm::vec3 & color)
{
	std::vector<glm::vec3> pos;
	int jointNum = mSkeleton->getNumJoints();
	for (int i = 0; i < jointNum; ++i)
	{
		AJoint* child = mSkeleton->getJointByID(i);
		AJoint* parent = child->getParent();
		if (!parent) { continue; }
		pos.push_back(toGLMvec3(child->getGlobalTranslation()));
		pos.push_back(toGLMvec3(parent->getGlobalTranslation()));
	}
	if (!mDrawableSkeleton) { mDrawableSkeleton = std::make_unique<Drawable>(); }
	glBindVertexArray(mDrawableSkeleton->VAO);
	glBindBuffer(GL_ARRAY_BUFFER, mDrawableSkeleton->VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * pos.size(), pos.data(), GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	mSkeletonShader->use();
	mSkeletonShader->setMat4("uProjView", projView);
	mSkeletonShader->setMat4("uModel", model);
	mSkeletonShader->setVec3("uColor", color);
	glDrawArrays(GL_LINES, 0, pos.size());
}

void FBXModel::findAllJoints()
{
	mJointMap.clear();
	// Find the root limb (hips)
	const ofbx::Object *root = nullptr; // root limb
	int i = 0;
	while (const ofbx::Object* child = scene->getRoot()->resolveObjectLink(i))
	{
		if (child->getType() == ofbx::Object::Type::LIMB_NODE) 
		{
			root = child;
			break;
		}
		++i;
	}
	traverseJoints(root);
}

void FBXModel::traverseJoints(const ofbx::Object * node)
{
	if (node->getType() != ofbx::Object::Type::LIMB_NODE)
	{
		return;
	}
	mJointMap.emplace(node, mJointMap.size());	
	int i = 0;
	while (const ofbx::Object* child = node->resolveObjectLink(i))
	{
		traverseJoints(child);
		i++;
	}
}

bool FBXModel::constructSkeleton()
{
	mSkeletonMap.clear();
	mIKTargets.clear();
	ASkeleton* skeleton = mBVHController->getSkeleton();

	for (const auto& pair : mJointMap)
	{
		const auto* joint = pair.first;
		AJoint* actorJoint = skeleton->getJointByName(joint->name);
		if (!actorJoint) { return false; }
		setLimbJoints(actorJoint);
		//std::cout << actorJoint->getName() << " id:" << actorJoint->getID() << std::endl;
		mSkeletonMap.emplace(actorJoint->getID(), joint);
	}

	// Set uo the IK Skeleton and create 4 limb IK chains
	mIKController->getIKSkeleton()->copyHierarchy(mSkeleton);
	mIKController->createLimbIKchains();
	return true;
}

void FBXModel::setLimbJoints(AJoint * joint)
{
	std::string name = joint->getName();
	int id = joint->getID();
	if (name.compare("Beta:Hips") == 0)
	{
		mIKTargets.emplace_back(IKTarget{ id, name, joint });
		mIKController->mRootID = id;
	}
	else if (name.compare("Beta:LeftHand") == 0)
	{
		mIKTargets.emplace_back(IKTarget{ id, name, joint });
		mIKController->mLhandID = id;
	}
	else if (name.compare("Beta:RightHand") == 0)
	{
		mIKTargets.emplace_back(IKTarget{ id, name, joint });
		mIKController->mRhandID = id;
	}
	else if (name.compare("Beta:LeftFoot") == 0)
	{
		mIKTargets.emplace_back(IKTarget{ id, name, joint });
		mIKController->mLfootID = id;
	}
	else if (name.compare("Beta:RightFoot") == 0)
	{
		mIKTargets.emplace_back(IKTarget{ id, name, joint });
		mIKController->mRfootID = id;
	}
}

void FBXModel::createShader(const std::string & vert, const std::string & frag)
{
	mFBXShader = std::make_unique<Shader>(vert.c_str(), frag.c_str());
}

void FBXModel::setShaderBindMats()
{
	mFBXShader->use();
	std::vector<glm::mat4> mats(mJointMap.size());
	for (const auto& pair : mSkeletonMap)
	{
		AJoint* joint = mSkeleton->getJointByID(pair.first);
		const auto* node = pair.second;
		int index = mJointMap[node];

		mat3 rot = joint->getGlobalRotation();
		vec3 tran = joint->getGlobalTranslation();

		ofbx::Matrix mat = node->getGlobalTransform();
		glm::mat4 model = toGLMmat4(rot, tran);
		mats[index] = glm::inverse(model);
	}
	mFBXShader->setMatN("uBindMats", mats, mats.size());
}

void FBXModel::setShaderJointTransMats()
{
	mFBXShader->use();
	std::vector<glm::mat4> mats(mJointMap.size());
	for (const auto& pair : mSkeletonMap)
	{
		AJoint* joint = mSkeleton->getJointByID(pair.first);
		const auto* node = pair.second;
		int index = mJointMap[node];

		mat3 rot = joint->getGlobalRotation();
		vec3 tran = joint->getGlobalTranslation();
		glm::mat4 model = toGLMmat4(rot, tran);
		mats[index] = model;
	}
	mFBXShader->setMatN("uJointTransMats", mats, mats.size());
}

void FBXModel::updateDeltaT(float deltaT)
{
	mTime += deltaT;
	float duration = mBVHController->getDuration();
	if (mTime >= duration)
	{
		mTime = 0;
		//mActor.updateGuideJoint(vec3{ 100, 0, 0 });
	}
	mBVHController->update(mTime);
}

void FBXModel::updateT(float t)
{
	mBVHController->update(t, false);
}

void FBXModel::computeIK(int type, IKTarget & target)
{
	IKController::IKType ikType = static_cast<IKController::IKType>(type);
	vec3 pos{ target.targetPos[0], target.targetPos[1], target.targetPos[2] };
	target.target.setGlobalTranslation(pos);
	switch (ikType)
	{
	case IKController::CCD:
		mIKController->IKSolver_CCD(target.jointID, target.target);
		break;
	case IKController::PSEDUOINV:
		mIKController->IKSolver_PseudoInv(target.jointID, target.target);
		break;
	case IKController::LIMB:
	default:
		mIKController->IKSolver_Limb(target.jointID, target.target);
		break;
	}
	
}
