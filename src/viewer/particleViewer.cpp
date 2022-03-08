#include "particleViewer.h"
#include "utils.h"

ParticleViewer::ParticleViewer(const std::string& name) :
	Viewer(name)
{
	clearColor = ImVec4(0.6f, 0.6f, 0.6f, 1.00f);
	mParticleModel = std::make_unique<ObjModel>();
	mRocketModel = std::make_unique<ObjModel>();
	mParticleModelSphere = std::make_unique<ObjModel>();
	mParticleModel->loadObj("../obj/cube.obj");
	mRocketModel->loadObj("../obj/cone.obj");
	mParticleModelSphere->loadObj("../obj/sphere.obj");
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

ParticleViewer::~ParticleViewer()
{
}

void ParticleViewer::drawScene()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	float deltaT = 1.0 / ImGui::GetIO().Framerate;
	
	
	glm::mat4 projView = mCamera.getProjView();
	
	drawGridGround(projView);
	if (mDemo == 0)
	{
		mParticles.update(deltaT);
		drawParticles(projView);
	}
	else if (mDemo == 1)
	{
		int extForceMode = 0;

		if (mExtRocketForcesActive)
			extForceMode = extForceMode | EXT_ROCKETFORCES_ACTIVE;
		if (mExtSparkForcesActive)
			extForceMode = extForceMode | EXT_SPARKFORCES_ACTIVE;
		if (mWindActive)
			extForceMode = extForceMode | WIND_ACTIVE;
		if (mAttractorActive)
			extForceMode = extForceMode | ATTRACTOR_ACTIVE;
		if (mRepellerActive)
			extForceMode = extForceMode | REPELLER_ACTIVE;
		if (mDragActive)
			extForceMode = extForceMode | DRAG_ACTIVE;
		if (mRandomActive)
			extForceMode = extForceMode | RANDOM_ACTIVE;
		mFireworks.update(deltaT, extForceMode);
		drawFireworks(projView);
	}
	glDisable(GL_DEPTH_TEST);
}

void ParticleViewer::createGUIWindow()
{
	ImGui::Begin("Particle Editor");

	ImGui::RadioButton("Particles", &mDemo, 0); ImGui::SameLine();
	ImGui::RadioButton("Fireworks", &mDemo, 1);

	ImGui::RadioButton("Cube", &mParticleModelType, 0); ImGui::SameLine();
	ImGui::RadioButton("Sphere", &mParticleModelType, 1);

	if (mDemo == 0)
	{
		ImGui::Text("Particles Parameters");
		ImGui::Separator();
		ImGui::Checkbox("Infinite", &mParticles.mInfinite);
		ImGui::SliderInt("Max particles", &mParticles.mMaxParticles, 1, 50);
		double lifeMin = 0.1f, lifeMax = 5.0f;
		ImGui::SliderScalar("Lift Time", ImGuiDataType_::ImGuiDataType_Double, &mParticles.mLifeSpan,
			&lifeMin, &lifeMax, "%.3lf");

		double gMin = -50, gMax = 50, pMin = -100, pMax = 100, vMin = -50, vMax = 50;
		ImGui::SliderScalarN("Gravity", ImGuiDataType_::ImGuiDataType_Double, reinterpret_cast<double*>(&mParticles.mGravity),
			3, &gMin, &gMax, "%.3lf");
		ImGui::SliderScalarN("Position", ImGuiDataType_::ImGuiDataType_Double, reinterpret_cast<double*>(&mParticles.mStartPos),
			3, &pMin, &pMax, "%.3lf");
		ImGui::SliderScalarN("Velocity", ImGuiDataType_::ImGuiDataType_Double, reinterpret_cast<double*>(&mParticles.mStartVel),
			3, &vMin, &vMax, "%.3lf");

		float startColor[3] = { mParticles.mStartColor[0], mParticles.mStartColor[1], mParticles.mStartColor[2] };
		if (ImGui::ColorEdit3("Start Color", startColor))
		{
			mParticles.mStartColor = vec3(startColor[0], startColor[1], startColor[2]);
		}
		float endColor[3] = { mParticles.mEndColor[0], mParticles.mEndColor[1], mParticles.mEndColor[2] };
		if (ImGui::ColorEdit3("End Color", endColor))
		{
			mParticles.mEndColor = vec3(endColor[0], endColor[1], endColor[2]);
		}

		double aMin = 0.0f, aMax = 1.0f;
		ImGui::SliderScalar("Start Alpha", ImGuiDataType_::ImGuiDataType_Double, &mParticles.mStartAlpha,
			&aMin, &aMax, "%.3lf");
		ImGui::SliderScalar("End Alpha", ImGuiDataType_::ImGuiDataType_Double, &mParticles.mEndAlpha,
			&aMin, &aMax, "%.3lf");

		double sMin = 0.1f, sMax = 10.0f;
		ImGui::SliderScalar("Start Scale", ImGuiDataType_::ImGuiDataType_Double, &mParticles.mStartScale,
			&sMin, &sMax, "%.3lf");
		ImGui::SliderScalar("End Scale", ImGuiDataType_::ImGuiDataType_Double, &mParticles.mEndScale,
			&sMin, &sMax, "%.3lf");

	}
	else if (mDemo == 1)
	{
		ImGui::Text("Fireworks Parameters");
		ImGui::Separator();
		ImGui::Checkbox("Ext. Rocket Forces", &mExtRocketForcesActive);
		ImGui::Checkbox("Ext. Spark Forces", &mExtSparkForcesActive);
		
		ImGui::Separator();
		ImGui::Checkbox("Active Wind", &mWindActive);
		double wMin = -500, wMax = 500;
		ImGui::SliderScalarN("Wind Force", ImGuiDataType_::ImGuiDataType_Double, reinterpret_cast<double*>(&mFireworks.m_windForce),
			3, &wMin, &wMax, "%.3lf");
		ImGui::Separator();

		ImGui::Checkbox("Active Attract", &mAttractorActive);
		double aMin = -500, aMax = 500;
		ImGui::SliderScalarN("Attractor Pos", ImGuiDataType_::ImGuiDataType_Double, reinterpret_cast<double*>(&mFireworks.m_attractorPos),
			3, &wMin, &wMax, "%.3lf");
		ImGui::Separator();

		ImGui::Checkbox("Active Repel", &mRepellerActive);
		double rMin = -500, rMax = 500;
		ImGui::SliderScalarN("Repeller Pos", ImGuiDataType_::ImGuiDataType_Double, reinterpret_cast<double*>(&mFireworks.m_repellerPos),
			3, &rMin, &rMax, "%.3lf");
		ImGui::Separator();

		ImGui::Checkbox("Active Drag", &mDragActive);
		ImGui::Checkbox("Random Force", &mRandomActive);

	}
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();
}

void ParticleViewer::keyCallback(GLFWwindow * window, int key, int scancode, int action, int mods)
{
	Viewer::keyCallback(window, key, scancode, action, mods);

	// Generate a rocket
	if (mDemo == 1 && key == GLFW_KEY_SPACE && action == GLFW_PRESS)
	{
		vec3 color{ Random::GetRandom(0, 1) , Random::GetRandom(0, 1),  Random::GetRandom(0, 1) };
		vec3 pos{ Random::GetRandom(-50, 50), 0, Random::GetRandom(-50, 50) };
		vec3 vel{ Random::GetRandom(-10, 10), Random::GetRandom(80, 100), Random::GetRandom(-10, 10) };
		mFireworks.fireRocket(pos, vel, color);
	}
}

void ParticleViewer::drawParticles(const glm::mat4& projView)
{
	mModelShader->use();
	mModelShader->setMat4("uProjView", projView);
	mModelShader->setVec3("uLightPos", mLightPos);
	for (int i = 0; i < mParticles.getParticleNum(); ++i)
	{
		AParticle* particle = mParticles.getParticle(i);
		if (particle->isAlive())
		{
			// ramp the color and fade out the particle as its life runs out 
			float u = particle->getTimeToLive() / particle->m_lifeSpan;
			float scale = particle->mStartScale * u + particle->mEndScale * (1 - u);
			float alpha = particle->mStartAlpha * u + particle->mEndAlpha * (1 - u);
			vec3 color = particle->mStartColor * u + particle->mEndColor * (1 - u);
			vec3 pos = particle->m_Pos;
			glm::mat4 model = glm::mat4(1.0f);
			model = glm::scale(model, glm::vec3(5, 5, 5));
			model = glm::translate(model, glm::vec3(pos[0], pos[1], pos[2]));
			model = glm::scale(model, glm::vec3(scale, scale, scale));
			
			mModelShader->setMat4("uModel", model);
			mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
			mModelShader->setVec3("color", glm::vec3(color[0], color[1], color[2]));
			mModelShader->setFloat("uAlpha", alpha);
			mParticleModelType == 0? mParticleModel->drawObj() : mParticleModelSphere->drawObj();
		}
	}
}

void ParticleViewer::drawFireworks(const glm::mat4 & projView)
{
	float fadeTime = 3.0f;
	mModelShader->use();
	mModelShader->setMat4("uProjView", projView);
	mModelShader->setVec3("uLightPos", mLightPos);
	for (auto* spark : mFireworks.sparks)
	{
		if (!spark->m_alive) { continue; }
		vec3 pos = spark->m_Pos;
		
		glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(pos[0], pos[1], pos[2]))
			* glm::scale(glm::mat4(1.0), glm::vec3(2, 2, 2));

		mModelShader->setMat4("uModel", model);
		mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
		mModelShader->setVec3("color", glm::vec3(spark->m_color[0], spark->m_color[1], spark->m_color[2]));
		float alpha = 1.0;
		if (spark->m_state[10] < fadeTime)
		{
			alpha = spark->m_state[10] / 10.0f;
		}
		mModelShader->setFloat("uAlpha", alpha);
		mParticleModel->drawObj();
	}
	for (auto* rocket : mFireworks.rockets)
	{
		if (rocket->m_explosionCount >= 0) { continue; }
		vec3 pos = rocket->m_Pos;
		mat3 rot = mat3::FromToRotation(vec3(0, 1, 0), vec3(rocket->m_Vel[0], rocket->m_Vel[1], rocket->m_Vel[2]).Normalize());;
		glm::mat4 t = toGLMmat4(rot, pos);
		glm::mat4 model = t * glm::scale(glm::mat4(1.0), glm::vec3(5, 5, 5));

		mModelShader->setMat4("uModel", model);
		mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
		mModelShader->setVec3("color", glm::vec3(rocket->m_color[0], rocket->m_color[1], rocket->m_color[2]));
		mModelShader->setFloat("uAlpha", 1.0f);
		mRocketModel->drawObj();
	}
}
