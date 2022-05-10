#pragma once

#include <unordered_set>
#include <unordered_map>
#include <string>
#include <Eigen\Dense>
#include <Eigen\Core>
#include "aSplineVec3.h"
#include "aVector.h"
#include "mosek.h" /* Include the MOSEK definition file. */


//#define DEBUG
#define THRESHOLD_EPSILON 0.0001
#define MAX_ACCEPTED_DIFFERENCE_K 3000
#define MAX_ITERS_TO_SOLVE 5
#define REGULARIZATION_LAMBDA 0.000001
#define TAU 6.28318530718
#define FPS 30

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
	MSKenv_t env;
	MSKrescodee r;
	std::unordered_map<std::string, std::unique_ptr<CurveSegment>> curveSegments;
	std::unordered_map<std::string, std::unique_ptr<KeyFrame>> keyFrames;
	std::unordered_map<int, std::unique_ptr<State>>  pins;
	std::unordered_map<std::string, std::unique_ptr<Contact>> contacs;
	// solve for new state S' passed as parameter
	ASolver();
	~ASolver();

	void solve(State& newState, std::vector<CurveSegment*>& C, int totalNumberOfKeys);
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

	KeyFrame(std::string& id, double value, double frameNumber, double t, int index, Tangent& tangentMinus, Tangent& tangentPlus)
	{
		
		this->id = id;
		this->value = value;
		this->t = t;
		this->keyFrameNumber = index;
		this->frameNumber = frameNumber;
		this->tangentMinus = tangentMinus;
		this->tangentPlus = tangentPlus;
	}
	~KeyFrame() {}
};

class CurveSegment
{
public:
	// id of curve
	std::string id;
	// translation, rotation
	CurveType type;
	// x, y, z
	int component;
	// joint
	std::string joint;
	// frame corresponding to left key
	KeyFrame* keyLeft;
	// frame corresponding to right Key
	KeyFrame* keyRight;
	// reference to maya MFnAnimCurve
	std::string mfnAnimCurveName;

	CurveSegment() : type(TRANSLATION), id(0), joint("test"), keyLeft(nullptr), keyRight(nullptr) {}

	CurveSegment(std::string& id, CurveType type, int component, std::string& joint, 
					KeyFrame *leftKey, KeyFrame *rightKey, std::string& mfnAnimCurveName) :
		id(id), type(type), component(component), joint(joint), keyLeft(leftKey), keyRight(rightKey),
		mfnAnimCurveName(mfnAnimCurveName)
	{}
	~CurveSegment() {}

	double evaluateBezierGivenTangents(int frameNumber, const vec3& tangentPlus, const vec3& tangentMinus) const;
	// method based on stackoverflow post: 
	// https://stackoverflow.com/questions/51879836/cubic-bezier-curves-get-y-for-given-x-special-case-where-x-of-control-points
	double getTGivenX(double x, double pa, double pb, double pc, double pd) const;
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
	State(int frameNumber, const vec3& value, bool isPinned)
		: frameNumber(frameNumber), point(value), isPinned(isPinned)
	{}

	~State() {}

	vec3 getCurrentValue() const;
};



