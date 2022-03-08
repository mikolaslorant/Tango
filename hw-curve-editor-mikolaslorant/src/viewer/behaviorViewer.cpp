#include "behaviorViewer.h"
#include "utils.h"

BehaviorViewer::BehaviorViewer(const std::string & name):
	Viewer(name)
{
	mFBXModel.loadFBX("../fbx/BetaCharacter.fbx");
	mFBXModel.loadShaders();
	//mFBXModel.createShader("../shader/betaCharacter.vert.glsl", "../shader/betaCharacter.frag.glsl");
	mFBXModel.loadBVHMotion("../motions/Beta/Beta.bvh", true);	// Use it to set the bind matrices
	mFBXModel.loadBVHMotion("../motions/Beta/walking.bvh", false);

	mObstacleModel = std::make_unique<ObjModel>();
	mObstacleModel->loadObj("../obj/cone2.obj");
	mConeModel = std::make_unique<ObjModel>();
	mConeModel->loadObj("../obj/cone.obj");
	mTargetSphere = std::make_unique<ObjModel>();
	mTargetSphere->loadObj("../obj/sphere.obj");
	mPlaneModel = std::make_unique<ObjModel>();
	mPlaneModel->loadObj("../obj/plane.obj");

	mPoint3dShader = std::make_unique<Shader>("../shader/point3d.vert.glsl", "../shader/point3d.frag.glsl");
	mLineShader = std::make_unique<Shader>("../shader/skeleton.vert.glsl", "../shader/skeleton.frag.glsl");
	mModelFlatShader = std::make_unique<Shader>("../shader/model.vert.glsl", "../shader/modelFlat.frag.glsl");
	mTargetPoint = std::make_unique<DrawablePoint>(glm::vec3(0, 0, 0));
	mDebugLine = std::make_unique<DrawableLine>(glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));

	mBehaviorType = BehaviorType::ARRIVAL;
	reset();
}

BehaviorViewer::~BehaviorViewer()
{
}

void BehaviorViewer::drawScene()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	glm::mat4 model = glm::mat4(1.0f);
	glm::mat4 projView = mCamera.getProjView();
	//float deltaT = 1.0 / ImGui::GetIO().Framerate;
	if (mDrawPlane)
	{
		drawPlane(projView, glm::translate(glm::mat4(1.0f), glm::vec3(0, -0.5f, 0)));
	}
	float deltaT = 1.0 / 60.0;
	for (int i = 0; i < mActorNum; ++i)
	{
		float dt = mPause ? 0 : deltaT * mActors[i].getBehaviorController()->getVelocity().Length() / 164.0f;
		mActorTimes[i] += dt;
	}
	if (!mPause)
	{
		updateActors(deltaT);
	}

	for (int i = 0; i < mActorNum; ++i)
	{
		auto& actor = mActors[i];
		mat3 rot;
		rot.FromEulerAngles(mat3::YXZ, actor.getBehaviorController()->getOrientation());
		model = toGLMmat4(rot, actor.getBehaviorController()->getPosition());
		if (mDebug)
		{
			drawDebugCone(projView, model);
		}
		else
		{
			mFBXModel.updateT(mActorTimes[i]);
			mFBXModel.drawModel(projView, model, mLightPos, glm::vec3(0.2, 0.9, 1.0));
		}
	}
	if (static_cast<BehaviorType>(mBehaviorType) == BehaviorType::AVOID)
	{
		drawObstacles(projView, glm::mat4(1.0));
	}
	if (mDebug)
	{
		drawDebugLines(projView, glm::mat4(1.0));
	}
	drawGridGround(projView);
	drawTargetPoint(projView, glm::translate(glm::mat4(1.0f), toGLMvec3(mBehaviorTarget.getGlobalTranslation())));
	
	
}

void BehaviorViewer::createGUIWindow()
{
	ImGui::Begin("Behavior Editor");
	ImGui::Text("Behaviors");
	const char* behaviorTypes[] = { "Seek", "Flee", "Arrival", "Departure", "Avoid", "Wander", "Alignment",
		"Seperation", "Cohesion", "Flocking", "Leader" };

	// Set spline type
	ImGui::Combo("Type", &mBehaviorType, behaviorTypes, IM_ARRAYSIZE(behaviorTypes), 13);
	ImGui::Checkbox("Pause", &mPause); 
	ImGui::SameLine(); 
	ImGui::Checkbox("Debug", &mDebug);
	ImGui::SameLine();
	ImGui::Checkbox("Draw Plane", &mDrawPlane);
	
	if (ImGui::Button("Reset"))
	{
		reset();
		mReset = false;
	}

	if (ImGui::SliderInt("Actor Number", &mActorNum, 1, 5)) { reset(); }
	ImGui::Text("Gains");
	double sMin = 0, sMax = 2000;
	ImGui::SliderScalar("Max Speed", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::gMaxSpeed,
		&sMin, &sMax, "%.3lf");
	ImGui::SliderScalar("Max Angular Speed", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::gMaxAngularSpeed,
		&sMin, &sMax, "%.3lf");
	ImGui::SliderScalar("Max Force", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::gMaxForce,
		&sMin, &sMax, "%.3lf");
	ImGui::SliderScalar("Max Toruqe", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::gMaxTorque,
		&sMin, &sMax, "%.3lf");
	ImGui::SliderScalar("Neighborhood", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::gKNeighborhood,
		&sMin, &sMax, "%.3lf");

	//TODO: Add your code here to create additional GUI Variables
	ImGui::Text("You can add additional GUI Variables here");
	ImGui::SliderScalar("AgentRadius", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::gAgentRadius,
		&sMin, &sMax, "%.3lf");
	ImGui::SliderScalar("ObstacleOffset", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::obstacleOffset,
		&sMin, &sMax, "%.3lf");
	ImGui::SliderScalar("Tavoid", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::TAvoid,
		&sMin, &sMax, "%.3lf");
	ImGui::SliderScalar("Kavoid", ImGuiDataType_::ImGuiDataType_Double, &BehaviorController::KAvoid,
		&sMin, &sMax, "%.3lf");

	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();
}

void BehaviorViewer::mouseButtonCallback(GLFWwindow * window, int button, int action, int mods)
{
	Viewer::mouseButtonCallback(window, button, action, mods);
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) { mPicked = false; }

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
		if (mHoldLShift && rayWorld.y != 0)
		{
			float t = -origin.y / rayWorld.y;
			glm::vec3 newPos = origin + t * rayWorld;
			newPos = glm::clamp(newPos, glm::vec3(-500, 0, -500), glm::vec3(500, 0, 500));
			mBehaviorTarget.setGlobalTranslation(vec3(newPos[0], 0, newPos[2]));
		}
		float radius = 20;
		glm::vec3 center = toGLMvec3(mBehaviorTarget.getGlobalTranslation());
		float a = glm::dot(rayWorld, rayWorld);
		float b = 2 * glm::dot(rayWorld, origin - center);
		float c = glm::dot(origin - center, origin - center) - radius * radius;
		float discriminant = b * b - 4 * a * c;
		if (discriminant >= 0)
		{
			mPicked = true;

		}
	}
}

void BehaviorViewer::cursorPosCallback(GLFWwindow * window, double xpos, double ypos)
{
	Viewer::cursorPosCallback(window, xpos, ypos);	// Call parent method
	if (mPicked)
	{
		xpos = (2.0f * xpos) / windowWidth - 1.0f;
		ypos = 1.0f - (2.0f * ypos) / windowHeight;
		glm::vec4 ray = glm::vec4(xpos, ypos, -1, 1);
		ray = glm::inverse(mCamera.getProj()) * ray;
		ray = glm::vec4(ray.x, ray.y, -1, 0);
		glm::vec3 rayWorld = glm::vec3(glm::inverse(mCamera.getView()) * ray);
		rayWorld = glm::normalize(rayWorld);	// Direction of the ray
		glm::vec3 origin = mCamera.getEye();
		if (rayWorld.y != 0)
		{
			float t = -origin.y / rayWorld.y;
			glm::vec3 newPos = origin + t * rayWorld;
			newPos = glm::clamp(newPos, glm::vec3(-500, 0, -500), glm::vec3(500, 0, 500));
			mBehaviorTarget.setGlobalTranslation(vec3(newPos[0], 0, newPos[2]));
		}
	}
}

void BehaviorViewer::keyCallback(GLFWwindow * window, int key, int scancode, int action, int mods)
{
	Viewer::keyCallback(window, key, scancode, action, mods);
	if (key == GLFW_KEY_LEFT_SHIFT && action == GLFW_PRESS) { mHoldLShift = true; }
	else if (key == GLFW_KEY_LEFT_SHIFT && action == GLFW_RELEASE) { mHoldLShift = false; }

}

void BehaviorViewer::reset()
{
	mActors.clear();
	mObstacles.clear();
	mActorTimes.resize(mActorNum, 0);
	mActors.resize(mActorNum);
	mObstacles.resize(mObstacleNum);

	mBehaviorTarget.setGlobalTranslation(vec3(0, 0, 0));

	// set up behavior controllers
	for (int i = 0; i < mActorNum; i++)
	{
		BehaviorController* behavior = mActors[i].getBehaviorController();
		behavior->setTarget(mBehaviorTarget);
		behavior->createBehaviors(mActors, mObstacles);
		behavior->setActiveBehaviorType(static_cast<BehaviorType>(mBehaviorType));
		if (i == 0)
		{
			behavior->setLeader(true);
		}
		behavior->reset();
	}

	// create obstacles
	vec3 obstaclePositions[5] = { vec3(300, 0, 300), vec3(300, 0, -300), vec3(-300, 0, -300), vec3(-300, 0, 300), vec3(0, 0, 0) };
	
/*
	for (int i = 0; i < mObstacleNum; i++)
	{
		mObstacles[i].m_Radius = Random::GetRandom(50, 80);
		if (i < 5)
		{
			mObstacles[i].m_Center.setGlobalTranslation(obstaclePositions[i]);
		}
		else
		{
			mObstacles[i].m_Center.setGlobalTranslation(vec3(Random::GetRandom(-500, 500), 0.0f, Random::GetRandom(-500, 500)));
		}
		
		vec3 p = mObstacles[i].m_Center.getGlobalTranslation();
	}
	*/

	for (int i = 0; i < mObstacleNum; i++)
	{
		mObstacles[i].m_Radius = 10.0 + 80.0 * (((double)rand()) / RAND_MAX);
	
		double x = ((double)rand() / RAND_MAX) - 0.5;
		double z = ((double)rand() / RAND_MAX) - 0.5;


		mObstacles[i].m_Center.setGlobalTranslation(vec3(x*1000.0, 0.0, z*1000.0));
	}


}

void BehaviorViewer::updateActors(float deltaT)
{
	// SENSE PHASE - all agents sense the state of the world in the sense phase
	for (auto& actor : mActors)
	{
		BehaviorController* behavior = actor.getBehaviorController();
		behavior->setActiveBehaviorType(static_cast<BehaviorType>(mBehaviorType));
		behavior->setTarget(mBehaviorTarget);  // sets target of ith agent
		behavior->sense(deltaT);
	}

	// CONTROL PHASE - given the state of the world all agents determine what to do in control phase
	for (auto& actor : mActors)
	{
		actor.getBehaviorController()->control(deltaT);
	}

	// ACT PHASE - control is applied and agent state is updated in act phase
	for (auto& actor : mActors)
	{
		actor.getBehaviorController()->act(deltaT);
	}

}

void BehaviorViewer::drawTargetPoint(const glm::mat4 & projView, const glm::mat4 & model)
{
	//glDisable(GL_DEPTH_TEST);
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//glBindVertexArray(mTargetPoint->VAO);
	//glEnable(GL_PROGRAM_POINT_SIZE);
	//mPoint3dShader->use();
	//mPoint3dShader->setMat4("uProjView", projView);
	//mPoint3dShader->setMat4("uModel", model);
	//mPoint3dShader->setVec3("uColor", glm::vec3(1, 0, 0));
	//mPoint3dShader->setFloat("uSize", 30);

	//glDrawArrays(GL_POINTS, 0, mTargetPoint->elementCount);
	//glDisable(GL_PROGRAM_POINT_SIZE);
	//glEnable(GL_DEPTH_TEST);
	//glDisable(GL_BLEND);

	mModelFlatShader->use();
	mModelFlatShader->setMat4("uProjView", projView);
	mModelFlatShader->setVec3("uLightPos", mLightPos);
	mModelFlatShader->setVec3("color", glm::vec3(0.8, 0.1, 0.2));
	glm::mat4 obModel = model * glm::scale(glm::mat4(1.0), glm::vec3(8, 8, 8));
	mModelFlatShader->setMat4("uModel", obModel);
	mModelFlatShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(obModel))));
	mTargetSphere->drawObj();

}

void BehaviorViewer::drawObstacles(const glm::mat4 & projView, const glm::mat4 & model)
{
	mModelShader->use();
	mModelShader->setMat4("uProjView", projView);
	mModelShader->setVec3("uLightPos", mLightPos);
	mModelShader->setVec3("color", glm::vec3(0.8, 0.7, 0));
	for (auto& obstacle : mObstacles)
	{
		vec3 pos = obstacle.m_Center.getGlobalTranslation();
		float scale = obstacle.m_Radius;
		glm::mat4 obModel = model * glm::translate(glm::mat4(1.0), toGLMvec3(pos)) * glm::scale(glm::mat4(1.0), glm::vec3(scale, scale, scale));
		mModelShader->setMat4("uModel", obModel);
		mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(obModel))));
		
		mObstacleModel->drawObj();
	}
}

void BehaviorViewer::drawDebugLines(const glm::mat4 & projView, const glm::mat4 & model)
{
	glDisable(GL_DEPTH_TEST);
	for (auto& actor : mActors)
	{
		BehaviorController* behavior = actor.getBehaviorController();
		vec3 pos = behavior->getPosition();
		double scale = 1.0;
		vec3 vel = scale * behavior->getVelocity();
		double velMag = vel.Length();
		vec3 dvel = scale * behavior->getDesiredVelocity();
		

		drawLine(projView, model, glm::vec3(pos[0], pos[1], pos[2]),
			glm::vec3(pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2]), glm::vec3(1, 0, 0));
		drawLine(projView, model, glm::vec3(pos[0], pos[1], pos[2]),
			glm::vec3(pos[0] + dvel[0], pos[1] + dvel[1], pos[2] + dvel[2]), glm::vec3(0, 1, 0));

		if (static_cast<BehaviorType>(mBehaviorType) == BehaviorType::AVOID)
		{
			// avoid lookahead
			vec3 lookahead = vel.Normalize() * BehaviorController::lookAhead;
			drawLine(projView, model, glm::vec3(pos[0], pos[1], pos[2]),
				glm::vec3(pos[0] + lookahead[0], pos[1] + lookahead[1], pos[2] + lookahead[2]), glm::vec3(0, 0, 1));
			vec3 angle = behavior->getOrientation();
			vec3 dir = vec3(cos(angle[1]), 0, sin(angle[1]));
			vec3 opos = dynamic_cast<Avoid*>(behavior->getActiveBehavior())->m_obstaclePos;
			drawLine(projView, model, glm::vec3(pos[0], pos[1], pos[2]),
				glm::vec3(opos[0], opos[1], opos[2]), glm::vec3(1, 1, 1));
		}
	}
	glEnable(GL_DEPTH_TEST);

}

void BehaviorViewer::drawLine(const glm::mat4 & projView, const glm::mat4 & model, const glm::vec3 p0, const glm::vec3 p1, const glm::vec3 color)
{
	std::vector<glm::vec3> pos;
	pos.push_back(p0);
	pos.push_back(p1);
	glBindVertexArray(mDebugLine->VAO);
	glBindBuffer(GL_ARRAY_BUFFER, mDebugLine->VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * pos.size(), pos.data(), GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	mLineShader->use();
	mLineShader->setMat4("uProjView", projView);
	mLineShader->setMat4("uModel", model);
	mLineShader->setVec3("uColor", color);
	glDrawArrays(GL_LINES, 0, pos.size());
}

void BehaviorViewer::drawDebugCone(const glm::mat4 & projView, const glm::mat4 & model)
{
	mModelShader->use();
	mModelShader->setMat4("uProjView", projView);
	mModelShader->setVec3("uLightPos", mLightPos);
	mModelShader->setVec3("color", glm::vec3(0, 1.0, 1.0));

	glm::mat4 obModel = model * glm::rotate(glm::mat4(1.0), glm::radians(90.0f), glm::vec3(1, 0, 0)) * glm::scale(glm::mat4(1.0), glm::vec3(10));
	mModelShader->setMat4("uModel", obModel);
	mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(obModel))));

	mConeModel->drawObj();
}

void BehaviorViewer::drawPlane(const glm::mat4 & projView, const glm::mat4 & model)
{
	mModelShader->use();
	mModelShader->setMat4("uProjView", projView);
	mModelShader->setVec3("uLightPos", mLightPos);
	mModelShader->setVec3("color", glm::vec3(0.5, 0.5, 0.5));

	glm::mat4 obModel = model * glm::scale(glm::mat4(1.0), glm::vec3(500));
	mModelShader->setMat4("uModel", obModel);
	mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(obModel))));
	mPlaneModel->drawObj();
}
