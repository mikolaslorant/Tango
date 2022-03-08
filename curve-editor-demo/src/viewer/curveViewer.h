#pragma once
#include "viewer.h"
#include "aSplineVec3.h"
#include "aSplineQuat.h"
#include "objmodel.h"
#include "drawable.h"

class CurveViewer : public Viewer
{
public:
	CurveViewer(const std::string& name);
	virtual ~CurveViewer();
	
	virtual void createGUIWindow() override;
	virtual void drawScene() override;

	virtual void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) override;
	virtual void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) override;

protected:
	
	void updateSplineVec3Type(int newtype, ASplineVec3& spline);
	void updateSplineQuatType(int newtype, ASplineQuat& spline);
	void updateSplineEulerType(int newtype, ASplineVec3& spline);

	void drawKeyPoints(const ASplineVec3& spline);
	void drawControlPoints(const ASplineVec3& spline);
	void drawCurve(const ASplineVec3& spline);
	void drawControlPointLine(const ASplineVec3& spline);
	void drawAnimatedPoint(const ASplineVec3& spline);

	void drawRotatedModel(const ASplineVec3& eulerSpline, const ASplineQuat& quatSpline);

	// pos coordinates are in screen space [-1, 1] x [-1, 1]
	void pickPoint(double screenX, double screenY, const ASplineVec3& spline);
	void appendKeyPoint(double screenX, double screenY, ASplineVec3& spline);
	void deleteKeyPoint(double screenX, double screenY, ASplineVec3& spline);
	void movePoint(double screenX, double screenY, ASplineVec3& spline);
	void resetSplineVec3(ASplineVec3& spline);

	void appendRotationKey(ASplineVec3& eulerSpline, ASplineQuat& quatSpline);
	void editRotationKey(int key, vec3 newEuler, ASplineVec3& eulerSpline, ASplineQuat& quatSpline);
	void deleteRotationKey(int key, ASplineVec3& eulerSpline, ASplineQuat& quatSpline);
	void updateRotationOrder(int order, ASplineVec3& eulerSpline, ASplineQuat& quatSpline);

	bool mShowControlPoint;
	bool mAnimate;
	bool mHoldLeftMouseButton;

	ASplineVec3 mSplineVec3;
	ASplineVec3 mSplineEuler;
	ASplineQuat mSplineQuat;

	int mDemo = 0;	// 0 for spline, 1 for rotation

	std::unique_ptr<Drawable> mKeyPoints;
	std::unique_ptr<Drawable> mCurveLine;
	std::unique_ptr<Drawable> mControlPoints;
	std::unique_ptr<Drawable> mControlPointLine;
	std::unique_ptr<Drawable> mAnimatedPoint;

	std::unique_ptr<ObjModel> mRotatedModel;

	int mCurrentSplineVec3Type = 0;
	int mCurrentSplineRotationType = 0; // 0 for linear, 1 for cubic	
	int mRotOrder = mat3::XYZ;	//ZYX, XYZ, YZX, XZY, YXZ, ZXY

	// Record the picked point
	float mPickRadius = 0.02f;
	int mPickedPointId = -1;
	int mPickedPointType = 0;	// 0 for key point, 1 for control point

	// Model for rotation
	//ObjModel mModel;
};