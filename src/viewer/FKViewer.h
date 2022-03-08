#pragma once
#include "viewer.h"
#include "FBXModel.h"
#include "objmodel.h"

class FKViewer : public Viewer
{
public:
	FKViewer(const std::string& name);
	virtual ~FKViewer();
	virtual void createGUIWindow() override;
	virtual void drawScene() override;

	virtual void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) override;
	virtual void cursorPosCallback(GLFWwindow* window, double xpos, double ypos)override;

private:
	FBXModel mFBXModel;

	int mCurrentBVHFileIndex = 2;	// Default "Beta.bvh"
	float mTimeScale = 1.0f;
	float mTime = 0;
	float mLastTime = 0;	// last gflw time

	std::vector<std::string> mBVHFilePaths;		// The relative paths of the files
	std::vector<std::string> mBVHFileStems;		// Filenames that show in the list box
	
	int mFKIKMode = 0;	// 0 for FK, 1 for IK
	int mIKType = 0;	// 0 for Limb, 1 for CCD, 2 for Pseudo Inverse
	bool mLoaded = true;
	bool mShowSkeleton = false;

	void loadBVHFile(int index);
	void reset();

	IKTarget* mPickedTarget;
	float mPickedRayT;	// Store the t of the casted ray when the target is picked
};