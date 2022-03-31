#pragma once
#include "viewer.h"
#include "aParticleSystem.h"
#include "aFireworks.h"
#include "objmodel.h"

class ParticleViewer : public Viewer
{
public:
	ParticleViewer(const std::string& name);
	virtual ~ParticleViewer();

	virtual void drawScene()override;
	virtual void createGUIWindow() override;
	virtual void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) override;

private:
	int mDemo = 0;	// 0 for particles, 1 for fireworks

	int mParticleModelType = 0; // 0 for cube, 1 for sphere

	AParticleSystem mParticles;
	AFireworks mFireworks;

	std::unique_ptr<ObjModel> mParticleModel;
	std::unique_ptr<ObjModel> mParticleModelSphere;
	std::unique_ptr<ObjModel> mRocketModel;

	void drawParticles(const glm::mat4& projView);
	void drawFireworks(const glm::mat4& projView);

	// Fireworks parameters
	bool mWindActive = false;
	bool mDragActive= false;
	bool mAttractorActive = false;
	bool mRepellerActive = false;
	bool mRandomActive = false;
	bool mExtSparkForcesActive = false;
	bool mExtRocketForcesActive = false;
	int mExtForceMode = 0;
};