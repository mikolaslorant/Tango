#include <stdlib.h> 
#include <math.h> 
#include "Plugin.h"
#include "aSplineVec3.h"
#include "aSplineQuat.h"
#include <unordered_map>
#include <memory>


class ACurve
{
public:
	ACurve()
		: mSplineVec3(std::make_unique<ASplineVec3>()), 
		  mSplineQuat(std::make_unique<ASplineQuat>()), 
		  mSplineEuler(std::make_unique<ASplineVec3>())
	{
		mSplineVec3->setInterpolationType(ASplineVec3::LINEAR);
		mSplineQuat->setInterpolationType(ASplineQuat::LINEAR);
		mSplineEuler->setInterpolationType(ASplineVec3::LINEAR_EULER);
		mSplineVec3->setLooping(false);
		mSplineQuat->setLooping(false);
		mSplineEuler->setLooping(false);
	}
	std::unique_ptr<ASplineVec3> mSplineVec3;	// Translation vec3(x, y, z)
	std::unique_ptr<ASplineQuat> mSplineQuat;	// Quaternion rotation quat(w, x, y, z)
	std::unique_ptr<ASplineVec3> mSplineEuler;	// Euler angles rotation vec3(x, y, z)
};

struct CurveValue
{
	double vec[3];	// Position (x, y, z)
	double quat[4];		// Quaternion (w, x, y, z)
	double euler[3];	// Euler angles (x, y, z)
};

class CurvePluginManager 
{
public:
	CurvePluginManager() {}

	std::unordered_map<int, ACurve> mCurvePool;
	int mCurrentIndex = 0;

	// Initialize
	int createCurve()
	{
		mCurvePool.insert({ mCurrentIndex, ACurve()});
		mCurrentIndex++;
		return mCurrentIndex - 1;
	}

	void removeCurve(int id)
	{
		mCurvePool.erase(id);
	}

	void resetCurve(int id)
	{
		ACurve& curve = mCurvePool[id];
		curve.mSplineEuler->clear();
		curve.mSplineQuat->clear();
		curve.mSplineVec3->clear();
	}

	// Append key
	void appendVecKey(int id, vec3 value)
	{
		mCurvePool[id].mSplineVec3->appendKey(value);
	}

	void appendQuatKey(int id, double t, quat value)
	{
		mCurvePool[id].mSplineQuat->appendKey(t, value.Normalize());
	}

	void appendEulerKey(int id, double t, vec3 value)
	{
		mCurvePool[id].mSplineEuler->appendKey(t, value);
	}

	// Insert key
	int insertQuatKey(int id, double t, quat value)
	{
		return mCurvePool[id].mSplineQuat->insertKey(t, value.Normalize());
	}

	int insertEulerKey(int id, double t, vec3 value)
	{
		return mCurvePool[id].mSplineEuler->insertKey(t, value);
	}

	// Edit key
	void editVecKey(int id, int keyID, vec3 value)
	{
		mCurvePool[id].mSplineVec3->editKey(keyID, value);
	}

	void editQuatKey(int id, int keyID, quat value)
	{
		mCurvePool[id].mSplineQuat->editKey(keyID, value.Normalize());
	}

	void editEulerKey(int id, int keyID, vec3 value)
	{
		mCurvePool[id].mSplineEuler->editKey(keyID, value);
	}

	void editVecControlPoint(int id, int controlPointID, vec3 value)
	{
		mCurvePool[id].mSplineVec3->editControlPoint(controlPointID, value);
	}

	// Delete Key
	void deleteVecKey(int id, int keyID)
	{
		mCurvePool[id].mSplineVec3->deleteKey(keyID);
	}

	void deleteQuatKey(int id, int keyID)
	{
		mCurvePool[id].mSplineQuat->deleteKey(keyID);
	}

	void deleteEulerKey(int id, int keyID)
	{
		mCurvePool[id].mSplineEuler->deleteKey(keyID);
	}

	// Set interpolation type
	void setVecInterpolationType(int id, int type)
	{
		mCurvePool[id].mSplineVec3->setInterpolationType(static_cast<ASplineVec3::InterpolationType>(type));
	}

	void setQuatInterpolationType(int id, int type)
	{
		mCurvePool[id].mSplineQuat->setInterpolationType(static_cast<ASplineQuat::InterpolationType>(type));
	}
	
	void setEulerInterpolationType(int id, int type)
	{
		// 0: Linear ; 1: Cubic
		mCurvePool[id].mSplineEuler->setInterpolationType(static_cast<ASplineVec3::InterpolationType>(type + 6));
	}

	// Get curve
	void GetControlPoints(int id, double startPoint[], double endPoint[], int& controlPointNum, double*& controlPointPtr)
	{
		ASplineVec3& splineVec = *(mCurvePool[id].mSplineVec3);
		vec3 startPointVec = splineVec.getControlPoint(0);
		startPoint[0] = startPointVec[0];
		startPoint[1] = startPointVec[1];
		startPoint[2] = startPointVec[2];
		vec3 endPointVec = splineVec.getControlPoint(splineVec.getNumControlPoints() - 1);
		endPoint[0] = endPointVec[0];
		endPoint[1] = endPointVec[1];
		endPoint[2] = endPointVec[2];
		
		controlPointNum = splineVec.getNumControlPoints();
		controlPointPtr = reinterpret_cast<double*>(splineVec.getControlPointsData());
	}

	void GetCachedCurve(int id, int &cachedPointNum, double*& cachedPointPtr)
	{
		ASplineVec3& splineVec = *(mCurvePool[id].mSplineVec3);
		cachedPointNum = splineVec.getNumCurveSegments();
		cachedPointPtr = reinterpret_cast<double*>(splineVec.getCachedCurveData());
	}

	void GetValue(int id, double t, CurveValue& curveValue)
	{
		vec3 vec = mCurvePool[id].mSplineVec3->getValue(t);
		quat q = mCurvePool[id].mSplineQuat->getCachedValue(t);
		vec3 euler = mCurvePool[id].mSplineEuler->getValue(t);
		curveValue.vec[0] = vec[0];
		curveValue.vec[1] = vec[1];
		curveValue.vec[2] = vec[2];

		curveValue.quat[0] = q.W();
		curveValue.quat[1] = q.X();
		curveValue.quat[2] = q.Y();
		curveValue.quat[3] = q.Z();

		curveValue.euler[0] = euler[0];
		curveValue.euler[1] = euler[1];
		curveValue.euler[2] = euler[2];
	}

	double getVecDuration(int id)
	{
		return mCurvePool[id].mSplineVec3->getDuration();
	}

	int getVecKeyNum(int id)
	{
		return mCurvePool[id].mSplineVec3->getNumKeys();
	}

};

extern "C"
{
	static CurvePluginManager mCurvePluginManager;

	EXPORT_API int CreateCurve()
	{
		return mCurvePluginManager.createCurve();
	}

	EXPORT_API void RemoveCurve(int id)
	{
		mCurvePluginManager.removeCurve(id);
	}

	EXPORT_API void ResetCurve(int id)
	{
		mCurvePluginManager.resetCurve(id);
	}

	// Append key
	EXPORT_API void AppendVecKey(int id, double pos[])
	{
		mCurvePluginManager.appendVecKey(id, vec3(pos[0], pos[1], pos[2]));
	}

	EXPORT_API void AppendQuatKey(int id, double t, double q[])
	{
		mCurvePluginManager.appendQuatKey(id, t, quat(q[0], q[1], q[2], q[3]));
	}

	EXPORT_API void AppendEulerKey(int id, double t, double angle[])
	{
		mCurvePluginManager.appendEulerKey(id, t, vec3(angle[0], angle[1], angle[2]));
	}

	// Insert key
	EXPORT_API int InsertQuatKey(int id, double t, double q[])
	{
		return mCurvePluginManager.insertQuatKey(id, t, quat(q[0], q[1], q[2], q[3]));
	}

	EXPORT_API int InsertEulerKey(int id, double t, double angle[])
	{
		return mCurvePluginManager.insertEulerKey(id, t, vec3(angle[0], angle[1], angle[2]));
	}

	// Edit key
	EXPORT_API void EditVecKey(int id, int keyID, double pos[])
	{
		mCurvePluginManager.editVecKey(id, keyID, vec3(pos[0], pos[1], pos[2]));
	}

	EXPORT_API void EditQuatKey(int id, int keyID, double q[])
	{
		mCurvePluginManager.editQuatKey(id, keyID, quat(q[0], q[1], q[2], q[3]));
	}

	EXPORT_API void EditEulerKey(int id, int keyID, double angle[])
	{
		mCurvePluginManager.editEulerKey(id, keyID, vec3(angle[0], angle[1], angle[2]));
	}

	EXPORT_API void EditVecControlPoint(int id, int controlPointID, double pos[])
	{
		mCurvePluginManager.editVecControlPoint(id, controlPointID, vec3(pos[0], pos[1], pos[2]));
	}

	// Delete key
	EXPORT_API void DeleteVecKey(int id, int keyID)
	{
		mCurvePluginManager.deleteVecKey(id, keyID);
	}

	EXPORT_API void DeleteQuatKey(int id, int keyID)
	{
		mCurvePluginManager.deleteQuatKey(id, keyID);
	}

	EXPORT_API void DeleteEulerKey(int id, int keyID)
	{
		mCurvePluginManager.deleteEulerKey(id, keyID);
	}

	// Set Interpolation Type
	// Spline Vec: 0 - 5
	EXPORT_API void SetVecInterpolationType(int id, int type)
	{
		mCurvePluginManager.setVecInterpolationType(id, type);
	}

	// Spline Quat: 0 - 1
	EXPORT_API void SetQuatInterpolationType(int id, int type)
	{
		mCurvePluginManager.setQuatInterpolationType(id, type);
	}

	// Spline Euler: 0 - 1
	EXPORT_API void SetEulerInterpolationType(int id, int type)
	{
		mCurvePluginManager.setEulerInterpolationType(id, type);
	}

	// Set the size of the cached curve and the pointer points to the cached point data
	EXPORT_API void GetCachedCurve(int id, int& cachedPointNum, double*& cachedPointPtr)
	{
		mCurvePluginManager.GetCachedCurve(id, cachedPointNum, cachedPointPtr);
	}

	// Write the control points
	// startPoint and endPoint are arrays with size of 3
	// controlPointNum is the number of all control points including start point and end point
	// controlPointPtr is the pointer points to the control point data [x0, y0, z0, x1, y1, z1, ...] excluding start point and end point
	EXPORT_API void GetControlPoints(int id, double startPoint[], double endPoint[], int& controlPointNum, double*& controlPointPtr)
	{
		mCurvePluginManager.GetControlPoints(id, startPoint, endPoint, controlPointNum, controlPointPtr);
	}

	// Get value at time t
	EXPORT_API void GetValue(int id, double t, CurveValue& curveValue)
	{
		mCurvePluginManager.GetValue(id, t, curveValue);
	}


	EXPORT_API double GetVecDuration(int id)
	{
		return mCurvePluginManager.getVecDuration(id);
	}

	EXPORT_API int GetVecKeyNum(int id)
	{
		return mCurvePluginManager.getVecKeyNum(id);
	}
}