#include "curveViewer.h"
#include <gtc/matrix_transform.hpp>

CurveViewer::CurveViewer(const std::string& name) :
	Viewer(name),
	mShowControlPoint(false), mAnimate(false),
	mHoldLeftMouseButton(false),
	mSplineVec3(), mSplineEuler(), mSplineQuat()
{
	// Create and load objects
	mUnpinnedKeyPoints = std::make_unique<Drawable>();
	mCurveLine = std::make_unique<Drawable>();
	mControlPoints = std::make_unique<Drawable>();
	mStatePoints = std::make_unique<Drawable>();
	mControlPointLine = std::make_unique<Drawable>();
	mAnimatedPoint = std::make_unique<Drawable>();
	mPinnedStates = std::make_unique<Drawable>();
	mPinnedCurveKeys = std::make_unique<Drawable>();

	mRotatedModel = std::make_unique<ObjModel>();

	mRotatedModel->loadObj("../obj/wahoo.obj");

	// Initialize curves
	mSplineQuat.setLooping(true);
	mSplineQuat.setInterpolationType(ASplineQuat::LINEAR);
	mSplineEuler.setLooping(true);
	mSplineEuler.setInterpolationType(ASplineVec3::LINEAR_EULER);

	appendRotationKey(mSplineEuler, mSplineQuat);
	appendRotationKey(mSplineEuler, mSplineQuat);
	appendRotationKey(mSplineEuler, mSplineQuat);
	appendRotationKey(mSplineEuler, mSplineQuat);
	editRotationKey(1, vec3(-180, 0, 0), mSplineEuler, mSplineQuat);
	editRotationKey(2, vec3(0, 60, 0), mSplineEuler, mSplineQuat);

	// Initialize colors
}

CurveViewer::~CurveViewer()
{
}

void CurveViewer::createGUIWindow()
{
	ImGui::Begin("Curve Editor");   // Create a window called "Cuvre Editor" and append into it.

	ImGui::RadioButton("Spline Curve", &mDemo, 0); ImGui::SameLine();
	ImGui::RadioButton("Rotation", &mDemo, 1); 

	if (mDemo == 0)	// SplineVec3
	{
		// Curve Type
		const char* splineTypes[] = { "Linear", "Berstein-Bezier", "Casteljau-Bezier",
			"Matrix-Bezier", "Hermite", "BSpline" };

		// Set spline type
		ImGui::Combo("Type", &mCurrentSplineVec3Type, splineTypes, IM_ARRAYSIZE(splineTypes));
		updateSplineVec3Type(mCurrentSplineVec3Type, mSplineVec3);

		// Reset curve
		ImGui::Checkbox("Show Control Point", &mShowControlPoint);
		ImGui::Checkbox("Animate", &mAnimate);
		if (ImGui::Button("Reset"))
		{
			resetSplineVec3(mSplineVec3);
		}
		ImGui::Separator();
		/*
		if (ImGui::Button("Pin"))
		{
			mPinCurve = 1;
		}
		ImGui::SameLine();
		if (ImGui::Button("UnPin"))
		{
			mPinCurve = 0;
		}
		//ImGui::RadioButton("Pin Curve", &mPinCurve, 1); ImGui::SameLine();
		//ImGui::RadioButton("Unpin Curve", &mPinCurve, 0);
		ImGui::Separator();
		*/
		ImGui::Text("Add: Hold the left CTRL and left-click the mouse");
		ImGui::Text("Edit: Hold the left mouse button and move");
		ImGui::Text("Delete: Right-click the mouse");

	}
	else if (mDemo == 1)	// Rotation
	{
		// Title
		ImGui::Text("Left: Euler Angles");
		ImGui::Text("Right: Quaternion");

		const char* rotOrders[] = { "ZYX", "XYZ", "YZX", "XZY", "YXZ", "ZXY" };
		if (ImGui::Combo("Rotation Order", &mRotOrder, rotOrders, IM_ARRAYSIZE(rotOrders)))
		{
			updateRotationOrder(mRotOrder, mSplineEuler, mSplineQuat);
		}

		// Type
		const char* types[] = { "Linear/Spherical Linear", "Cubic/Spherical Cubic" };
		if (ImGui::Combo("Type", &mCurrentSplineRotationType, types, IM_ARRAYSIZE(types)))
		{
			updateSplineQuatType(mCurrentSplineRotationType, mSplineQuat);
			updateSplineEulerType(mCurrentSplineRotationType, mSplineEuler);
		}

		ImGui::Text("Keys are represented by euler angles (x, y, z)");
		// Append button
		if (ImGui::Button("Append a key"))
		{
			appendRotationKey(mSplineEuler, mSplineQuat);
		}
		// Edit and delete
		for (int i = 0; i < mSplineEuler.getNumKeys(); ++i)
		{
			vec3 euler = mSplineEuler.getKey(i);
			float e[3] = { euler[0], euler[1], euler[2] };
			std::string s = "key" + std::to_string(i);
			if (ImGui::DragFloat3(s.c_str(), e, 5.0f))
			{
				editRotationKey(i, vec3(e[0], e[1], e[2]), mSplineEuler, mSplineQuat);
			}
			ImGui::SameLine();
			if (ImGui::Button((std::string("Delete" + s)).c_str()))
			{
				deleteRotationKey(i, mSplineEuler, mSplineQuat);
			}
		}
	}

	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();
}

void CurveViewer::drawScene()
{
	// Draw the curve
	if (mDemo == 0)
	{
		glDisable(GL_DEPTH_TEST);
		drawCurve(mSplineVec3);
		drawControlPointLine(mSplineVec3);
		drawControlPoints(mSplineVec3);
		drawKeyPoints(mSplineVec3);
		drawAnimatedPoint(mSplineVec3);
		drawStatePoints(mSplineVec3);
		drawPinnedPoints(mSplineVec3);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		drawRotatedModel(mSplineEuler, mSplineQuat);
	}
}

void CurveViewer::updateSplineVec3Type(int newtype, ASplineVec3& spline)
{
	if (spline.getInterpolationType() != static_cast<ASplineVec3::InterpolationType>(newtype))
	{
		spline.setInterpolationType(static_cast<ASplineVec3::InterpolationType>(newtype));
	}
}

void CurveViewer::updateSplineQuatType(int newtype, ASplineQuat& spline)
{
	if (spline.getInterpolationType() != static_cast<ASplineQuat::InterpolationType>(newtype))
	{
		spline.setInterpolationType(static_cast<ASplineQuat::InterpolationType>(newtype));
	}
}

void CurveViewer::updateSplineEulerType(int newtype, ASplineVec3 & spline)
{
	auto type = newtype == 0 ? ASplineVec3::LINEAR_EULER : ASplineVec3::CUBIC_EULER;
	if (spline.getInterpolationType() != type)
	{
		spline.setInterpolationType(type);
	}
}

void CurveViewer::drawPinnedPoints(const ASplineVec3& spline)
{
	// Create vertices vector
	int numOfPins = spline.getNumPinPoints();
	if (numOfPins < 1) { return; }
	std::vector<glm::vec3> points(numOfPins);
	int i = 0;
	for (auto point : spline.getPinPoints()) {
		points[i].x = point[0];
		points[i].y = point[1];
		points[i].z = point[2];
		i++;
	}

	glBindVertexArray(mPinnedStates->VAO);
	// Allocate space and upload the data from CPU to GPU
	glBindBuffer(GL_ARRAY_BUFFER, mPinnedStates->VBO);
	glBufferData(GL_ARRAY_BUFFER, numOfPins * sizeof(glm::vec3), points.data(), GL_DYNAMIC_DRAW);

	// Specify how the data for position can be accessed
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glEnable(GL_PROGRAM_POINT_SIZE);

	mPointShader->use();
	mPointShader->setVec3("color", glm::vec3(0, 1, 0));
	mPointShader->setFloat("size", 20.0f);

	glDrawArrays(GL_POINTS, 0, numOfPins);
	glDisable(GL_PROGRAM_POINT_SIZE);
}

void CurveViewer::drawStatePoints(const ASplineVec3& spline)
{
	// Create vertices vector
	if (spline.getNumKeys() < 2) { return; }
	int numOfStates = (spline.getNumKeys() - 1) * 11;
	std::vector<glm::vec3> points;
	int i = 0;
	for (int frame = 10; i < numOfStates; frame+=10)
	{
		if (frame % 120 == 0)
		{
			continue;
		}
		auto& frameData = spline.mCachedCurve[frame];
		if (!spline.isStatePinned(frame)) {
			points.push_back(glm::vec3(frameData[0], frameData[1], frameData[2]));
		}
		i++;
	}

	glBindVertexArray(mStatePoints->VAO);
	// Allocate space and upload the data from CPU to GPU
	glBindBuffer(GL_ARRAY_BUFFER, mStatePoints->VBO);
	glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(), GL_DYNAMIC_DRAW);

	// Specify how the data for position can be accessed
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glEnable(GL_PROGRAM_POINT_SIZE);

	mPointShader->use();
	mPointShader->setVec3("color", glm::vec3(1, 0, 0));
	mPointShader->setFloat("size", 15.0f);

	glDrawArrays(GL_POINTS, 0, points.size());
	glDisable(GL_PROGRAM_POINT_SIZE);
}

void CurveViewer::drawKeyPoints(const ASplineVec3& spline)
{
	// Create vertices vector
	int pointNum = spline.getNumKeys();
	if (pointNum < 1) { return; }
	std::vector<glm::vec3> pinnedKeys;
	std::vector<glm::vec3> points;
	for (int i = 0; i < pointNum; ++i)
	{
		auto& key = spline.getKey(i);
		if (spline.isKeyPinned(i)) {
			pinnedKeys.push_back(glm::vec3(key[0], key[1], key[2]));
		}
		else {
			points.push_back(glm::vec3(key[0], key[1], key[2]));
		}
	}

	glBindVertexArray(mUnpinnedKeyPoints->VAO);
	// Allocate space and upload the data from CPU to GPU
	glBindBuffer(GL_ARRAY_BUFFER, mUnpinnedKeyPoints->VBO);
	glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(), GL_DYNAMIC_DRAW);

	// Specify how the data for position can be accessed
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glEnable(GL_PROGRAM_POINT_SIZE);

	mPointShader->use();
	mPointShader->setVec3("color", glm::vec3(0, 0, 1));
	mPointShader->setFloat("size", 20.0f);
	
	glDrawArrays(GL_POINTS, 0, points.size());
	glDisable(GL_PROGRAM_POINT_SIZE);


	glBindVertexArray(mPinnedCurveKeys->VAO);
	// Allocate space and upload the data from CPU to GPU
	glBindBuffer(GL_ARRAY_BUFFER, mPinnedCurveKeys->VBO);
	glBufferData(GL_ARRAY_BUFFER, pinnedKeys.size() * sizeof(glm::vec3), pinnedKeys.data(), GL_DYNAMIC_DRAW);

	// Specify how the data for position can be accessed
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glEnable(GL_PROGRAM_POINT_SIZE);

	mPointShader->use();
	mPointShader->setVec3("color", glm::vec3(0, 0.39, 0));
	mPointShader->setFloat("size", 20.0f);

	glDrawArrays(GL_POINTS, 0, pinnedKeys.size());
	glDisable(GL_PROGRAM_POINT_SIZE);
}

void CurveViewer::drawControlPoints(const ASplineVec3 & spline)
{
	if (spline.getInterpolationType() == ASplineVec3::LINEAR || !mShowControlPoint ||
		spline.getNumKeys() < 2)
	{
		return;
	}

	// Create vertices vector
	auto type = spline.getInterpolationType();
	int pointNum = 0;
	std::vector<glm::vec3> points;

	switch (type)
	{
	case ASplineVec3::CUBIC_BERNSTEIN:
	case ASplineVec3::CUBIC_CASTELJAU:
	case ASplineVec3::CUBIC_MATRIX:
		pointNum = spline.getNumControlPoints();
		points.resize(pointNum);
		for (int i = 0; i < pointNum; ++i)
		{
			auto& p = spline.getControlPoint(i);
			points[i] = glm::vec3(p[0], p[1], p[2]);
		}
		break;
	case ASplineVec3::CUBIC_HERMITE:
		pointNum = spline.getNumKeys();
		points.resize(pointNum);
		for (int i = 0; i < spline.getNumKeys(); ++i)
		{
			auto p0 = spline.getKey(i);
			auto slope = spline.getControlPoint(i + 1);
			auto p1 = slope + p0;
			points[i] = glm::vec3(p1[0], p1[1], p1[2]);
		}
		break;
	case ASplineVec3::CUBIC_BSPLINE:
		pointNum = spline.getNumControlPoints() - 2;
		points.resize(pointNum);
		for (int i = 1; i < pointNum + 1; ++i)
		{
			auto& p = spline.getControlPoint(i);
			points[i - 1] = glm::vec3(p[0], p[1], p[2]);
		}
		break;
	case ASplineVec3::LINEAR:
	default:
		break;
	}

	glBindVertexArray(mControlPoints->VAO);

	glBindBuffer(GL_ARRAY_BUFFER, mControlPoints->VBO);
	glBufferData(GL_ARRAY_BUFFER, pointNum * sizeof(glm::vec3), points.data(), GL_DYNAMIC_DRAW);

	// Specify how the data for position can be accessed
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glEnable(GL_PROGRAM_POINT_SIZE);

	mPointShader->use();
	mPointShader->setVec3("color", glm::vec3(1, 1, 0));
	mPointShader->setFloat("size", 16.0f);

	glDrawArrays(GL_POINTS, 0, pointNum);
	glDisable(GL_PROGRAM_POINT_SIZE);
}

void CurveViewer::drawCurve(const ASplineVec3 & spline)
{
	if (spline.getNumKeys() < 2) { return; }
	// Create vertices vector
	int pointNum = spline.getNumCurveSegments();

	glBindVertexArray(mCurveLine->VAO);
	glBindBuffer(GL_ARRAY_BUFFER, mCurveLine->VBO);
	glBufferData(GL_ARRAY_BUFFER, pointNum * sizeof(vec3), const_cast<ASplineVec3*>(&spline)->getCachedCurveData(), GL_DYNAMIC_DRAW);
	
	// Specify how the data for position can be accessed
	glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(double), (void*)0);
	glEnableVertexAttribArray(0);

	mCurveShader->use();
	mCurveShader->setVec3("color", glm::vec3(0, 0.4, 0.8));
	mCurveShader->setVec2("screenSize", glm::vec2(windowWidth, windowHeight));
	mCurveShader->setFloat("thickness", 0.005f);

	glDrawArrays(GL_LINE_STRIP, 0, pointNum);
}

void CurveViewer::drawControlPointLine(const ASplineVec3 & spline)
{
	if (!mShowControlPoint || spline.getNumKeys() < 2) { return; }
	auto type = spline.getInterpolationType();
	int pointNum = 0;
	std::vector<glm::vec3> points;

	switch (type)
	{
	case ASplineVec3::CUBIC_BERNSTEIN:
	case ASplineVec3::CUBIC_CASTELJAU:
	case ASplineVec3::CUBIC_MATRIX:
		pointNum = spline.getNumControlPoints();
		for (int i = 0; i < pointNum; ++i)
		{
			auto& p = spline.getControlPoint(i);
			points.push_back(glm::vec3(p[0], p[1], p[2]));
		}
		break;
	case ASplineVec3::CUBIC_HERMITE:
		pointNum = spline.getNumKeys() * 2;
		for (int i = 0; i < spline.getNumKeys(); ++i)
		{
			auto p0 = spline.getKey(i);
			auto slope = spline.getControlPoint(i + 1);
			auto p1 = slope + p0;
			points.push_back(glm::vec3(p0[0], p0[1], p0[2]));
			points.push_back(glm::vec3(p1[0], p1[1], p1[2]));
		}
		break;
	case ASplineVec3::CUBIC_BSPLINE:
		pointNum = spline.getNumControlPoints() - 2;
		for (int i = 1; i < pointNum + 1; ++i)
		{
			auto& p = spline.getControlPoint(i);
			points.push_back(glm::vec3(p[0], p[1], p[2]));
		}
		break;
	case ASplineVec3::LINEAR:
	default:
		break;
	}

	glBindVertexArray(mControlPointLine->VAO);
	glBindBuffer(GL_ARRAY_BUFFER, mControlPointLine->VBO);
	glBufferData(GL_ARRAY_BUFFER, pointNum * sizeof(glm::vec3), points.data(), GL_DYNAMIC_DRAW);


	// Specify how the data for position can be accessed
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	mCurveShader->use();
	mCurveShader->setVec3("color", glm::vec3(0.9, 0.5, 0.1));
	mCurveShader->setVec2("screenSize", glm::vec2(windowWidth, windowHeight));
	mCurveShader->setFloat("thickness", 0.002f);

	type == ASplineVec3::CUBIC_HERMITE ? glDrawArrays(GL_LINES, 0, pointNum) :  glDrawArrays(GL_LINE_STRIP, 0, pointNum);
}

void CurveViewer::drawAnimatedPoint(const ASplineVec3 & spline)
{
	if (!mAnimate || spline.getNumKeys() < 2) { return; }
	auto value = spline.getValue(glfwGetTime());
	glm::vec3 pos(value[0], value[1], value[2]);


	glBindVertexArray(mAnimatedPoint->VAO);
	glBindBuffer(GL_ARRAY_BUFFER, mAnimatedPoint->VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3), &pos, GL_DYNAMIC_DRAW);

	// Specify how the data for position can be accessed
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glEnable(GL_PROGRAM_POINT_SIZE);

	mPointShader->use();
	mPointShader->setVec3("color", glm::vec3(0.3f, 0.8f, 0.2f));
	mPointShader->setFloat("size", 30.0f);

	glDrawArrays(GL_POINTS, 0, 1);
	glDisable(GL_PROGRAM_POINT_SIZE);
}

void CurveViewer::drawRotatedModel(const ASplineVec3& eulerSpline, const ASplineQuat& quatSpline)
{
	if (eulerSpline.getNumKeys() == 0 || quatSpline.getNumKeys() == 0) { return; }
	mModelShader->use();
	double t = glfwGetTime();
	float ratio = windowWidth == 0 ? 1 : static_cast<float>(windowWidth) / windowHeight;
	glm::mat4 projView = glm::perspective(glm::radians(45.0f), ratio, 0.1f, 200.0f) * 
		glm::lookAt(glm::vec3(0, 0, 20), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	mModelShader->setMat4("uProjView", projView);
	mModelShader->setVec3("uLightPos", glm::vec3(20, 0, 20));

	glm::mat4 model;
	double angle;	// in radians
	vec3 axis;

	// Euler Angles (left)
	model = glm::mat4(1.0);
	vec3 euler = eulerSpline.getValue(t);
	mat3 rot;
	rot.FromEulerAngles(static_cast<mat3::RotOrder>(mRotOrder), euler * Deg2Rad);
	rot.ToAxisAngle(axis, angle);
	model = glm::translate(model, glm::vec3(-5, 0, 0));
	model = glm::rotate(model, static_cast<float>(angle), glm::vec3(axis[0], axis[1], axis[2]));
	
	mModelShader->setMat4("uModel", model);
	mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
	mModelShader->setVec3("color", glm::vec3(0.8, 0, 0));
	mRotatedModel->drawObj();

	// Quaternion (right)
	model = glm::mat4(1.0);
	quat q = quatSpline.getCachedValue(t);
	q.ToAxisAngle(axis, angle);
	if (axis[0] == 0 && axis[1] == 0 && axis[2] == 0) { axis[0] = 1; }
	model = glm::translate(model, glm::vec3(5, 0, 0));
	model = glm::rotate(model, static_cast<float>(angle), glm::vec3(axis[0], axis[1], axis[2]));
	
	mModelShader->setMat4("uModel", model);	
	mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
	mModelShader->setVec3("color", glm::vec3(0, 0, 0.8));
	mRotatedModel->drawObj();
}

void CurveViewer::pickPoint(double screenX, double screenY, const ASplineVec3& spline)
{
	mPickedPointId = -1;
	vec3 clickPos = vec3(screenX, screenY, 0);
	// check for picked states -- needs to be done before data points
	for (int frame = 10; frame < (spline.getNumKeys() - 1) * 120; frame += 10)
	{
		if (frame % 120 == 0)
		{
			continue;
		}
		vec3 pointPos = spline.mCachedCurve[frame];
		if ((clickPos - pointPos).Length() < mPickRadius)
		{
 			mPickedPointType = 2;
			mPickedPointId = frame;
			return;
		}
	}
	// check data points -- needs to be done before ctrl points
	for (int i = 0; i < spline.getNumKeys(); ++i)
	{
		vec3 pointPos = spline.getKey(i);
		if ((clickPos - pointPos).Length() < mPickRadius)
		{
			mPickedPointType = 0;
			mPickedPointId = i;
			return;
		}
	}

	if (!mShowControlPoint) { return; }
	// Check control points
	auto type = spline.getInterpolationType();
	for (int i = 0; i < spline.getNumControlPoints(); i++)
	{
		vec3 pointPos = spline.getControlPoint(i);
		if (type == ASplineVec3::CUBIC_HERMITE && i != 0 && i != spline.getNumControlPoints() - 1)		// Hermite Curve
		{
			vec3 key = spline.getKey(i - 1);
			if ((clickPos - pointPos - key).Length() < mPickRadius)
			{
				mPickedPointType = 1;
				mPickedPointId = i;
				return;
			}
		}
		else if ((clickPos - pointPos).Length() < mPickRadius)
		{
			mPickedPointType = 1;
			mPickedPointId = i;
			return;
		}
	}
}

void CurveViewer::appendKeyPoint(double screenX, double screenY, ASplineVec3 & spline)
{
	spline.appendKey(vec3(screenX, screenY, 0));
}

void CurveViewer::deleteKeyPoint(double screenX, double screenY, ASplineVec3 & spline)
{
	vec3 clickPos = vec3(screenX, screenY, 0);
	int keyId = -1;

	// check data points -- needs to be done before ctrl points
	for (int i = 0; i < spline.getNumKeys(); ++i)
	{
		vec3 pointPos = spline.getKey(i);
		if ((clickPos - pointPos).Length() < mPickRadius)
		{	
			spline.deleteKey(i);
			return;
		}
	}
}

void CurveViewer::pinPoint(double screenX, double screenY, ASplineVec3& spline)
{
	pickPoint(screenX, screenY, spline);
	if (mPickedPointId == -1) { return; }
	vec3 clickPos = vec3(screenX, screenY, 0);
	// if you have selected state point
	if (mPickedPointType == 2) {
		spline.appendPin(mPickedPointId, clickPos);
	}
	else if (mPickedPointType == 0) {
		if (!mPinCurve) {
			spline.pinCurve(mPickedPointId);
			mPickedPointType = -1;
			mPinCurve = 0;
		}
	}
}

void CurveViewer::movePoint(double screenX, double screenY, ASplineVec3 & spline)
{
	if (mPickedPointId == -1) { return; }
	vec3 clickPos = vec3(screenX, screenY, 0);
	auto type = spline.getInterpolationType();
	if (mPickedPointType == 0)
	{
		spline.editKey(mPickedPointId, clickPos);
	}
	else if (mPickedPointType == 1)
	{
		if (type == ASplineVec3::CUBIC_HERMITE)
			spline.editControlPoint(mPickedPointId, clickPos - spline.getKey(mPickedPointId - 1));
		else
			spline.editControlPoint(mPickedPointId, clickPos);
	}
	else
	{
		spline.editStatePoint(mPickedPointId, clickPos);
	}
}

void CurveViewer::resetSplineVec3(ASplineVec3 & spline)
{
	spline.clear();
}

void CurveViewer::appendRotationKey(ASplineVec3 & eulerSpline, ASplineQuat & quatSpline)
{
	float t = eulerSpline.getNumKeys() == 0 ? 0 : eulerSpline.getKeyTime(eulerSpline.getNumKeys() - 1) + 3;
	eulerSpline.appendKey(t, vec3(0, 0, 0));
	mat3 rot;
	rot.FromEulerAngles(static_cast<mat3::RotOrder>(mRotOrder), vec3(0, 0, 0));
	quat q = rot.ToQuaternion();
	quatSpline.appendKey(t, q);
}

void CurveViewer::editRotationKey(int key, vec3 newEuler, ASplineVec3 & eulerSpline, ASplineQuat & quatSpline)
{
	// Degree
	eulerSpline.editKey(key, newEuler);

	// Radian
	mat3 rot;
	rot.FromEulerAngles(static_cast<mat3::RotOrder>(mRotOrder), newEuler * Deg2Rad);
	quat q = rot.ToQuaternion();
	quatSpline.editKey(key, q);
}

void CurveViewer::deleteRotationKey(int key, ASplineVec3 & eulerSpline, ASplineQuat & quatSpline)
{
	eulerSpline.deleteKey(key);
	quatSpline.deleteKey(key);
}

void CurveViewer::updateRotationOrder(int order, ASplineVec3 & eulerSpline, ASplineQuat & quatSpline)
{
	int num = eulerSpline.getNumKeys();
	if (num < 1) { return; }
	// Update quat keys by the new rotation order
	for (int i = 0; i < num; ++i)
	{
		vec3 euler = eulerSpline.getKey(i);
		mat3 rot;
		rot.FromEulerAngles(static_cast<mat3::RotOrder>(order), euler * Deg2Rad);
		quatSpline.editKey(i, rot.ToQuaternion());
	}
}

void CurveViewer::mouseButtonCallback(GLFWwindow * window, int button, int action, int mods)
{
	// Get mouse pos in pixel space
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// convert mouse pos to screen space: [0, width] * [height, 0] -> [-1, 1] * [-1, 1]
	xpos = (xpos / windowWidth - 0.5) * 2.0;
	ypos = (0.5 - ypos / windowHeight) * 2.0;

	// Create a new key
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && mHoldLCtrl)
	{
		appendKeyPoint(xpos, ypos, mSplineVec3);
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS && mHoldLCtrl)
	{
		pinPoint(xpos, ypos, mSplineVec3);
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS && !mHoldLCtrl)
	{
		deleteKeyPoint(xpos, ypos, mSplineVec3);
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && !mHoldLCtrl)
	{
		pickPoint(xpos, ypos, mSplineVec3);
		mHoldLeftMouseButton = true;
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		mHoldLeftMouseButton = false;
		mPickedPointId = -1;
	}
}

void CurveViewer::cursorPosCallback(GLFWwindow * window, double xpos, double ypos)
{
	// convert mouse pos to screen space: [0, width] * [height, 0] -> [-1, 1] * [-1, 1]
	xpos = (xpos / windowWidth - 0.5) * 2.0;
	ypos = (0.5 - ypos / windowHeight) * 2.0;

	if (mHoldLeftMouseButton && mPickedPointId != -1)
	{
		movePoint(xpos, ypos, mSplineVec3);
	}
}
