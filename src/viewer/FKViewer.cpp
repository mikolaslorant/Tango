#include "FKViewer.h"
#include <filesystem>

FKViewer::FKViewer(const std::string & name) :
	Viewer(name)
{
	mFBXModel.loadFBX("../fbx/BetaCharacter.fbx");
	mFBXModel.loadShaders();
	//mFBXModel.createShader("../shader/betaCharacter.vert.glsl", "../shader/betaCharacter.frag.glsl");
	mFBXModel.loadBVHMotion("../motions/Beta/Beta.bvh", true);	// Use it to set the bind matrices

	std::string path = "../motions/Beta";
	for (const auto & entry : std::experimental::filesystem::directory_iterator(path))
	{		
		std::string extension = entry.path().extension().generic_string();
		
		if (extension.compare(".bvh") == 0)
		{
			mBVHFilePaths.push_back(entry.path().generic_string());
			mBVHFileStems.push_back(entry.path().stem().generic_string());
		}
	}
	mPickedTarget = nullptr;
}

FKViewer::~FKViewer()
{
}

void FKViewer::createGUIWindow()
{
	ImGui::Begin("FKIK Editor");
	//Viewer::createGUIWindow();
	if (ImGui::RadioButton("FK", &mFKIKMode, 0)) { reset(); }
	ImGui::SameLine();
	if (ImGui::RadioButton("IK", &mFKIKMode, 1)) { reset(); }
	ImGui::Checkbox("Show Skeleton", &mShowSkeleton);
	if (ImGui::Button("Reset")) { reset(); }
	if (mFKIKMode == 0)	// FK
	{
		ImGui::SliderFloat("Time Scale", &mTimeScale, 0, 2);
		ImGui::Separator();
		// List box
		ImGui::Text("Motions");
		std::vector<const char*> filesChar(mBVHFilePaths.size());
		for (int i = 0; i < filesChar.size(); ++i)
		{
			filesChar[i] = mBVHFileStems[i].c_str();
		}
		if (ImGui::ListBox("BVH Files", &mCurrentBVHFileIndex, filesChar.data(), mBVHFilePaths.size(), 10))
		{
			loadBVHFile(mCurrentBVHFileIndex);
		}
		if (!mLoaded)
		{
			ImGui::Text("Loading failed!");
			ImGui::Text("FBX joints names and BVH joints names are not the same!");
		}
	}
	else if (mFKIKMode == 1)	// IK
	{
		const char* IKSolvers[] = { "Limb-based", "CCD", "Pseudo Inverse" };
		ImGui::Combo("IK Solver", &mIKType, IKSolvers, IM_ARRAYSIZE(IKSolvers));

		for (auto& target : mFBXModel.mIKTargets)
		{
			if (ImGui::DragFloat3(target.jointName.c_str(), target.targetPos))
			{
				mFBXModel.computeIK(mIKType, target);
			}
		}
	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();

}

void FKViewer::drawScene()
{
	glEnable(GL_DEPTH_TEST);
	
	glm::mat4 model = glm::mat4(1.0f);
	glm::mat4 projView = mCamera.getProjView();
	// Draw Model
	if (mFKIKMode == 0)
	{
		float currentTime = glfwGetTime();
		mFBXModel.updateDeltaT((currentTime - mLastTime) * mTimeScale);
		mLastTime = currentTime;	
	}
	mShowSkeleton?
		mFBXModel.drawSkeleton(projView, model, glm::vec3(1, 1, 1)) :
		mFBXModel.drawModel(projView, model, mLightPos, glm::vec3(0.2, 0.9, 1.0));
	if (mFKIKMode == 1)	// IK
	{
		mFBXModel.drawTargets(projView, model, glm::vec3(0.5, 1.0, 0.4), 20);
	}
	drawGridGround(projView);
}

void FKViewer::mouseButtonCallback(GLFWwindow * window, int button, int action, int mods)
{
	Viewer::mouseButtonCallback(window, button, action, mods);

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) { mPickedTarget = nullptr; }
	if (mFKIKMode == 0) { return; }

	// Raycast pick targets
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		// Normalised Device Coordinates
		xpos = (2.0f * xpos) / windowWidth - 1.0f;
		ypos = 1.0f - (2.0f * ypos) / windowHeight;
		glm::vec4 ray = glm::vec4(xpos, ypos, -1, 1);
		ray = glm::inverse(mCamera.getProj()) * ray;
		ray = glm::vec4(ray.x, ray.y, -1, 0);
		glm::vec3 rayWorld = glm::vec3(glm::inverse(mCamera.getView()) * ray);
		rayWorld = glm::normalize(rayWorld);	// Direction of the ray
		glm::vec3 origin = mCamera.getEye();
		mPickedTarget = nullptr;

		for (auto& target : mFBXModel.mIKTargets)
		{
			float radius = 10;
			glm::vec3 center = glm::vec3{ target.targetPos[0], target.targetPos[1], target.targetPos[2] };
			float a = glm::dot(rayWorld, rayWorld);
			float b = 2 * glm::dot(rayWorld, origin - center);
			float c = glm::dot(origin - center, origin - center) - radius * radius;
			float discriminant = b * b - 4 * a * c;
			if (discriminant >= 0)
			{
				mPickedTarget = &target;
				mPickedRayT = (-b - std::sqrt(discriminant)) / (2 * a);
				std::cout << target.jointName << " t:" << mPickedRayT << std::endl;
				break;
			}
		}
	}
}

void FKViewer::cursorPosCallback(GLFWwindow * window, double xpos, double ypos)
{
	Viewer::cursorPosCallback(window, xpos, ypos);	// Call parent method

	if (mPickedTarget && mPickedRayT > 0)
	{
		xpos = (2.0f * xpos) / windowWidth - 1.0f;
		ypos = 1.0f - (2.0f * ypos) / windowHeight;
		glm::vec4 ray = glm::vec4(xpos, ypos, -1, 1);
		ray = glm::inverse(mCamera.getProj()) * ray;
		ray = glm::vec4(ray.x, ray.y, -1, 0);
		glm::vec3 rayWorld = glm::vec3(glm::inverse(mCamera.getView()) * ray);
		rayWorld = glm::normalize(rayWorld);	// Direction of the ray
		glm::vec3 origin = mCamera.getEye();
		glm::vec3 newPos = origin + mPickedRayT * rayWorld;
		mPickedTarget->targetPos[0] = newPos[0];
		mPickedTarget->targetPos[1] = newPos[1];
		mPickedTarget->targetPos[2] = newPos[2];
		mFBXModel.computeIK(mIKType, *mPickedTarget);
	}

}

void FKViewer::loadBVHFile(int index)
{
	mLoaded = mFBXModel.loadBVHMotion(mBVHFilePaths[index], false);
}

void FKViewer::reset()
{
	mFBXModel.mActor.resetGuide();
	mFBXModel.loadBVHMotion("../motions/Beta/Beta.bvh");
}

