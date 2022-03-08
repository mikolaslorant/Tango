#pragma once
#include "viewer.h"
#include "FBXModel.h"
#include "objmodel.h"

class BehaviorViewer : public Viewer
{
public:
	BehaviorViewer(const std::string& name);
	virtual ~BehaviorViewer();

	virtual void drawScene() override;
	virtual void createGUIWindow() override;

	virtual void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) override;
	virtual void cursorPosCallback(GLFWwindow* window, double xpos, double ypos)override;
	virtual void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) override;


private:
	void reset();
	void updateActors(float deltaT);

	void drawTargetPoint(const glm::mat4& projView, const glm::mat4& model);
	void drawObstacles(const glm::mat4& projView, const glm::mat4& model);
	void drawDebugLines(const glm::mat4& projView, const glm::mat4& model);
	void drawLine(const glm::mat4& projView, const glm::mat4& model, const glm::vec3 p0, const glm::vec3 p1, const glm::vec3 color);
	void drawDebugCone(const glm::mat4& projView, const glm::mat4& model);
	void drawPlane(const glm::mat4& projView, const glm::mat4& model);

	FBXModel mFBXModel;

	std::unique_ptr<ObjModel> mObstacleModel;
	std::unique_ptr<ObjModel> mConeModel;
	std::unique_ptr<DrawablePoint> mTargetPoint;
	std::unique_ptr<DrawableLine> mDebugLine;
	std::unique_ptr<ObjModel> mPlaneModel;
	std::unique_ptr<ObjModel> mTargetSphere;
	std::unique_ptr<Shader> mPoint3dShader;
	std::unique_ptr<Shader> mLineShader;
	std::unique_ptr<Shader> mModelFlatShader;
	ATarget mBehaviorTarget;
	bool mPicked = false;
	bool mDebug = true;
	bool mPause = false;
	bool mReset = false;
	bool mHoldLShift = false;
	bool mDrawPlane = true;

	int mActorNum = 1;
	int mObstacleNum = 10;
	BehaviorController* m_pBehaviorController;
	std::vector<AActor> mActors;
	std::vector<float> mActorTimes;
	std::vector<Obstacle> mObstacles;

	int mBehaviorType;
	


};