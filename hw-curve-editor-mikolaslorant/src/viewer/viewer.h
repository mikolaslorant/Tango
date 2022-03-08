#pragma once
#define IMGUI_IMPL_OPENGL_LOADER_GLAD

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <memory>

#include "camera.h"
#include "shader.h"
#include "drawable.h"

class Viewer
{
public:
	Viewer(const std::string& name);
	virtual ~Viewer();

	// The main loop of the viewer
	virtual void mainLoop();

	// Create GUI here
	virtual void createGUIWindow();

	// Draw all of the scene here
	virtual void drawScene();

	// Set all callback functions
	virtual void setCallbacks();

	// Some callback functions
	virtual void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
	virtual void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
	virtual void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
	virtual void scrollCallBack(GLFWwindow* window, double xoffset, double yoffset);

protected:
	virtual void createGridGround();
	virtual void drawGridGround(const glm::mat4& projViewModel);

	ImVec4 clearColor = ImVec4(0.2f, 0.2f, 0.2f, 1.00f);
	GLFWwindow* window;

	// Some shaders
	std::unique_ptr<Shader> mPointShader;
	std::unique_ptr<Shader> mCurveShader;
	std::unique_ptr<Shader> mModelShader;
	std::unique_ptr<Shader> mGridShader;

	// Some drawables
	std::unique_ptr<Drawable> mGridGround;
	

	// Screen size, update in each frame
	int windowWidth = 1920;
	int windowHeight = 1080;
	
	glm::vec3 mLightPos;

	// Camera control
	Camera mCamera;
	bool mFirstMouse = true;
	double mLastX, mLastY;
	bool mHoldLeftButton, mHoldMidButton, mHoldRightButton;
	bool mHoldLCtrl;

private:
	bool showDemoWindow = true;
	bool showAnotherWindow = false;
};