#include "viewer.h"


static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

// Copy from ImGUI/examples/example_glfw_opengl3
Viewer::Viewer(const std::string& name) :
	windowWidth(1920), windowHeight(1080),
	//changed camerea position value to (0, 500, 800) from (0, 100, 500)
	mCamera(windowWidth, windowHeight, glm::vec3(0, 500, 800), glm::vec3(0, 100, 0), glm::vec3(0, 1, 0)),
	mLightPos(glm::vec3(200, 600, 400)),
	mHoldLeftButton(false),
	mHoldMidButton(false),
	mHoldRightButton(false),
	mHoldLCtrl(false)
{
	// Setup window
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
	{
		std::cerr << "Failed to init glfw" << std::endl;
		return;
	}

	// Decide GL+GLSL versions
#if __APPLE__
	// GL 3.2 + GLSL 150
	const char* glsl_version = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
	// GL 4.6 + GLSL 130
	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

	// Create window with graphics context
	window = glfwCreateWindow(windowWidth, windowHeight, name.c_str(), NULL, NULL);
	if (window == NULL)
	{
		std::cerr << "Failed to create a window" << std::endl;
		return;
	}
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); // Enable vsync

	// Initialize OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
	bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
	bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
	bool err = gladLoadGL() == 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING2)
	bool err = false;
	glbinding::Binding::initialize();
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING3)
	bool err = false;
	glbinding::initialize([](const char* name) { return (glbinding::ProcAddress)glfwGetProcAddress(name); });
#else
	bool err = false; // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to requires some form of initialization.
#endif
	if (err)
	{
		std::cerr << "Failed to initialize OpenGL loader!" << std::endl;
	}

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// Load Fonts
	// - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
	// - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
	// - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
	// - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
	// - Read 'docs/FONTS.md' for more instructions and details.
	// - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
	io.Fonts->AddFontDefault();
	io.FontGlobalScale = 1.25f;
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
	//ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
	//IM_ASSERT(font != NULL);

	/* Our initializaton */
	setCallbacks();

	mPointShader = std::make_unique<Shader>("../shader/point.vert.glsl", "../shader/point.frag.glsl");
	mCurveShader = std::make_unique<Shader>("../shader/curve.vert.glsl", "../shader/curve.frag.glsl",
		"../shader/curve.geom.glsl");
	mModelShader = std::make_unique<Shader>("../shader/model.vert.glsl", "../shader/model.frag.glsl");
	mGridShader = std::make_unique<Shader>("../shader/grid.vert.glsl", "../shader/grid.frag.glsl");
	createGridGround();

}

Viewer::~Viewer()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
}

void Viewer::mainLoop()
{
	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// Create the GUI window
		createGUIWindow();

		// Rendering
		ImGui::Render();
		glfwGetFramebufferSize(window, &windowWidth, &windowHeight);
		glViewport(0, 0, windowWidth, windowHeight);
		glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		drawScene();
		// Draw gui
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
}

void Viewer::createGUIWindow()
{
	// 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
	if (showDemoWindow)
		ImGui::ShowDemoWindow(&showDemoWindow);

	// 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
	{
		static float f = 0.0f;
		static int counter = 0;

		ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

		ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
		ImGui::Checkbox("Demo Window", &showDemoWindow);      // Edit bools storing our window open/close state
		ImGui::Checkbox("Another Window", &showAnotherWindow);

		ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
		ImGui::ColorEdit3("clear color", (float*)&clearColor); // Edit 3 floats representing a color

		if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
			counter++;
		ImGui::SameLine();
		ImGui::Text("counter = %d", counter);

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
	}

	// 3. Show another simple window.
	if (showAnotherWindow)
	{
		ImGui::Begin("Another Window", &showAnotherWindow);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::Text("Hello from another window!");
		if (ImGui::Button("Close Me"))
			showAnotherWindow = false;
		ImGui::End();
	}
}

void Viewer::drawScene()
{
	glm::mat4 model = glm::mat4(1.0f);
	glm::mat4 projView = mCamera.getProjView();
	drawGridGround(projView * model);
}

void Viewer::setCallbacks()
{
	glfwSetWindowUserPointer(window, this);
	glfwSetKeyCallback(window,
		[](GLFWwindow* window, int key, int scancode, int action, int mods)
	{
		static_cast<Viewer*>(glfwGetWindowUserPointer(window))->keyCallback(window, key, scancode, action, mods);
	});
	glfwSetCursorPosCallback(window,
		[](GLFWwindow* window, double xpos, double ypos)
	{
		static_cast<Viewer*>(glfwGetWindowUserPointer(window))->cursorPosCallback(window, xpos, ypos);
	});
	glfwSetScrollCallback(window,
		[](GLFWwindow* window, double xoffset, double yoffset)
	{
		static_cast<Viewer*>(glfwGetWindowUserPointer(window))->scrollCallBack(window, xoffset, yoffset);
	});
	glfwSetMouseButtonCallback(window,
		[](GLFWwindow* window, int button, int action, int mods)
	{
		static_cast<Viewer*>(glfwGetWindowUserPointer(window))->mouseButtonCallback(window, button, action, mods);
	});
}

void Viewer::mouseButtonCallback(GLFWwindow * window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) { mHoldLeftButton = true; }
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) { mHoldLeftButton = false; }
	else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) { mHoldMidButton = true; }
	else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_RELEASE) { mHoldMidButton = false; }
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) { mHoldRightButton = true; }
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) { mHoldRightButton = false; }
}

void Viewer::keyCallback(GLFWwindow * window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_LEFT_CONTROL && action == GLFW_PRESS) { mHoldLCtrl = true; }
	else if (key == GLFW_KEY_LEFT_CONTROL && action == GLFW_RELEASE) { mHoldLCtrl = false; }
}

void Viewer::cursorPosCallback(GLFWwindow * window, double xpos, double ypos)
{
	if (mHoldLCtrl)
	{
		if (mFirstMouse)
		{
			mLastX = xpos;
			mLastY = ypos;
			mFirstMouse = false;
		}

		int deltaX = xpos - mLastX;
		int deltaY = mLastY - ypos; // reversed since y-coordinates go from bottom to top
		bool moveLeftRight = std::abs(deltaX) > std::abs(deltaY);
		bool moveUpDown = !moveLeftRight;
		if (mHoldLeftButton)  // Rotate
		{
			if (moveLeftRight) mCamera.PolarRotateAboutY(-deltaX * 0.1f);
			else if (moveUpDown) mCamera.PolarRotateAboutX(deltaY * 0.1f);
		}
		else if (mHoldMidButton) // Zoom
		{
			if (moveUpDown) mCamera.PolarZoom(deltaY);
		}
		else if (mHoldRightButton)
		{
			if (moveLeftRight) mCamera.PolarPanX(-deltaX * 0.5);
			else if (moveUpDown) mCamera.PolarPanY(-deltaY * 0.5);
		}

		mLastX = xpos;
		mLastY = ypos;
	}
}

void Viewer::scrollCallBack(GLFWwindow * window, double xoffset, double yoffset)
{
}

void Viewer::createGridGround()
{
	mGridGround = std::make_unique<Drawable>();	
	int lineNum = 21;	// Number of grid line
	float length = 500;
	float padding = 50;
	mGridGround->elementCount = lineNum * 2 * 2;	// X direction and Z direction, each has two vertices
	std::vector<glm::vec3> buffer;	// Position and color
	for (int i = -lineNum / 2; i <= lineNum / 2; ++i)
	{
		glm::vec3 color{ 0.75, 0.75, 0.75 };
		// X direction
		if (i == 0) { color = glm::vec3(1, 0, 0); }
		buffer.emplace_back(glm::vec3{ -length, 0, i * padding });
		buffer.emplace_back(color);
		buffer.emplace_back(glm::vec3{ length, 0, i * padding });
		buffer.emplace_back(color);
		// Z direction
		if (i == 0) { color = glm::vec3(0, 1, 0); }
		buffer.emplace_back(glm::vec3{ i * padding, 0, -length });
		buffer.emplace_back(color);
		buffer.emplace_back(glm::vec3{ i * padding, 0, length });
		buffer.emplace_back(color);
	}
	glBindVertexArray(mGridGround->VAO);
	glBindBuffer(GL_ARRAY_BUFFER, mGridGround->VBO);
	glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(glm::vec3), buffer.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);	// pos
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)(sizeof(glm::vec3)));
	glEnableVertexAttribArray(1);	// col
}

void Viewer::drawGridGround(const glm::mat4 & projViewModel)
{
	mGridShader->use();
	mGridShader->setMat4("uProjViewModel", projViewModel);
	glBindVertexArray(mGridGround->VAO);
	glDrawArrays(GL_LINES, 0, mGridGround->elementCount);
}
