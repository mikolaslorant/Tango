#pragma once

#include <unordered_set>
#include <unordered_map>
#include <string>
#include <Eigen\Dense>
#include <Eigen\Core>
#include "aSplineVec3.h"
#include "aVector.h"
#include "mosek.h" /* Include the MOSEK definition file. */


#define THRESHOLD_EPSILON 0.0001
#define MAX_ACCEPTED_DIFFERENCE_K 0.01
#define MAX_ITERS_TO_SOLVE 5
#define REGULARIZATION_LAMBDA 0.000001


#define FPS 120
typedef std::pair<double, vec3> Key;

class CurveSegment;
class State;
class Contact;
class KeyFrame;
class Tangent;


static void MSKAPI printstr(void* handle,
	const char str[])
{
	printf("%s", str);
} /* printstr */

class ASolver
{
public:
	std::unordered_map<std::string, std::unique_ptr<CurveSegment>> curveSegments;
	std::unordered_map<std::string, std::unique_ptr<KeyFrame>> keyFrames;
	std::unordered_map<int, std::unique_ptr<State>>  pins;
	std::unordered_map<std::string, std::unique_ptr<Contact>> contacs;
	// solve for new state S' passed as parameter
	void solve(State& newState, int totalNumberOfKeys);
	void calculateCurveSegmentsThatNeedOptimizing(const State& newState, std::unordered_set<State*>& ro, std::vector<CurveSegment*>& C) const;
	void calculateSolverInputs(const State& newState, const std::unordered_set<State*>& ro, const std::vector<CurveSegment*>& C, 
								int totalNumberOfKeys,
								Eigen::MatrixXd& Q, Eigen::VectorXd& b, Eigen::VectorXd& lowerBounds, Eigen::VectorXd& upperBounds, 
								std::vector<Eigen::Matrix3Xd>& A, Eigen::VectorXd& constraintsBounds);
	void mosekSolve(const Eigen::MatrixXd& Q, const Eigen::VectorXd& b, 
					const Eigen::VectorXd& lowerBounds, const Eigen::VectorXd& upperBounds, 
					const std::vector<Eigen::Matrix3Xd>& A,
					const Eigen::VectorXd& constraintsBounds,
					Eigen::VectorXd& solutionDeltaTangents);
	void updateTangents(std::vector<CurveSegment*>& C, const Eigen::VectorXd& solutionDeltaTangents);
	void parseConstraintMatrixA(const std::vector<Eigen::Matrix3Xd>& A, std::vector<double>& aval, 
						std::vector<MSKint32t>& aptrb, std::vector<MSKint32t>& aptre, std::vector<MSKint32t>& asub) const;
	static std::string getKey(int type, int component, int frameNumber);
	double phi(double ui, const KeyFrame& currentKeyFrame, const KeyFrame& otherKeyFrame) const;
	double psi(double vi, const KeyFrame& currentKeyFrame, const KeyFrame& otherKeyFrame) const;
};


enum CurveType {
	TRANSLATION,
	ROTATION,
	SCALE
};

enum Component {
	X,
	Y,
	Z
};

struct Tangent
{
	Tangent(double x, double y) : x(x), y(y)
	{}
	double x;
	double y;

	Tangent() {}

	~Tangent() {}
};

class KeyFrame
{
public:
	std::string id;
	double value;
	double t;
	int keyFrameNumber;
	int frameNumber;
	Tangent tangentMinus;
	Tangent tangentPlus;

	KeyFrame() : t(0.0), frameNumber(0), tangentMinus(Tangent()), tangentPlus(Tangent()) {}

	KeyFrame(int frameNumber, const std::vector<Key>& mKeys, int component, std::string& id)
	{
		int i = frameNumber / FPS;
		keyFrameNumber = i;
		this->id = id;
		this->value = mKeys[i].second[component];
		this->t = mKeys[i].first;
		this->frameNumber = frameNumber;
		
		vec3 tangentVec;
		if (i == 0)
		{
			tangentVec = (mKeys[i + 1].second - mKeys[i].second);
			this->tangentPlus = Tangent(mKeys[i + 1].first - mKeys[i].first, tangentVec[component]);
		}
		else if (i == mKeys.size() - 1)
		{
			tangentVec = (mKeys[i].second - mKeys[i - 1].second);
			this->tangentMinus = Tangent(mKeys[i].first - mKeys[i - 1].first, tangentVec[component]);
		}
		else {
			tangentVec = (mKeys[i + 1].second - mKeys[i - 1].second) * 0.5;
			this->tangentPlus = Tangent((mKeys[i + 1].first - mKeys[i - 1].first) * 0.5, tangentVec[component]);
			this->tangentMinus = Tangent((mKeys[i + 1].first - mKeys[i - 1].first) * 0.5, tangentVec[component]);
		}
	}
	~KeyFrame() {}
};

class CurveSegment
{
public:
	// translation, rotation
	CurveType type;
	// x, y, z
	int component;
	// id of curve
	std::string id;
	// joint
	std::string joint;
	// frame corresponding to left key
	KeyFrame* keyLeft;
	// frame corresponding to right Key
	KeyFrame* keyRight;

	CurveSegment() : type(TRANSLATION), id(0), joint("test"), keyLeft(nullptr), keyRight(nullptr) {}

	CurveSegment(int component, int frameNumber, const std::vector<Key> &mKeys, ASolver& solver) : component(component), type(TRANSLATION)
	{
		std::string leftKey = ASolver::getKey(TRANSLATION, component, frameNumber);
		id = leftKey;
		std::string rightKey = ASolver::getKey(TRANSLATION, component, frameNumber + FPS);
		
		if (solver.keyFrames.find(leftKey) == solver.keyFrames.end())
		{
			solver.keyFrames.insert({ leftKey, std::make_unique<KeyFrame>(frameNumber, mKeys, component, leftKey) });
		}

		this->keyLeft = solver.keyFrames[leftKey].get();

		if (solver.keyFrames.find(rightKey) == solver.keyFrames.end())
		{
			solver.keyFrames.insert({ rightKey, std::make_unique<KeyFrame>(frameNumber + FPS, mKeys, component, rightKey) });
		}
		this->keyRight = solver.keyFrames[rightKey].get();
	}
	~CurveSegment() {}

	double evaluateBezierGivenTangents(int frameNumber, vec3& tangentPlus, vec3& tangentMinus) const;
	double dCdT(int frameNumber, int component, int index) const;
};

class Contact
{
public:
	int id;
	CurveSegment* curveSegment;
	std::vector<CurveSegment*> orderedAffectedCurveSegments;
	vec3 point;

	Contact() : id(0), curveSegment(nullptr), orderedAffectedCurveSegments(), point(vec3()) {}
	~Contact() {}
};

class State
{
public:
	// Cs or Csj
	int frameNumber;
	vec3 point;
	std::vector<CurveSegment*> orderedAffectedCurveSegments;
	bool isPinned;

	State() : frameNumber(0), point(vec3()), orderedAffectedCurveSegments(), isPinned(false) {}
	State(int frameNumber, const vec3& value, bool isPinned, const std::vector<Key>& mKeys, ASolver& solver)
		: frameNumber(frameNumber), point(value), isPinned(isPinned)
	{
		for (int i = 0; i < 3; i++)
		{

			std::string key = ASolver::getKey(TRANSLATION, i, frameNumber);
			if (solver.curveSegments.find(key) == solver.curveSegments.end())
			{
				solver.curveSegments.insert({key, std::make_unique<CurveSegment>(i, frameNumber, mKeys, solver) });
			}
			// set curve segment for my mActiveState
			this->orderedAffectedCurveSegments.push_back(solver.curveSegments[key].get());
		}
	}
	~State() {}

	vec3 getCurrentValue() const;
};



