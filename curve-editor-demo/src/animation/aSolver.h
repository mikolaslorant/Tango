#pragma once
#include <unordered_map>
#include "aVector.h"

#define FPS 120
typedef std::pair<double, vec3> Key;

class CurveSegment;
class State;
class Contact;
class KeyFrame;
class Tangent;

class ASolver
{
public:
	// C : all curve segments affected

	std::unordered_map<int, std::unique_ptr<CurveSegment>> curveSegments;
	std::unordered_map<int, std::unique_ptr<State>>  pins;
	std::unordered_map<int, std::unique_ptr<Contact>> contacs;
	std::unordered_map<int, std::unique_ptr<KeyFrame>> keyFrames;
	std::unordered_map<int, std::unique_ptr<Tangent>> tangents;
	// solve for new state S' passed as parameter
	void solve(State& newState);
};


enum CurveType {
	TRANSLATION,
	ROTATION,
	SCALE
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
	double t;
	int frameNumber;
	Tangent tangentMinus;
	Tangent tangentPlus;

	KeyFrame() : t(0.0), frameNumber(0), tangentMinus(Tangent()), tangentPlus(Tangent()) {}
	KeyFrame(int frameNumber, const std::vector<Key> &mKeys) {
		this->frameNumber = frameNumber;
		int i = frameNumber / FPS;
		this->t = mKeys[i].first;
		vec3 tangentVec;
		if (i == 0)
		{
			tangentVec = (mKeys[i + 1].second - mKeys[i].second);
			this->tangentPlus = Tangent(tangentVec[0], tangentVec[1]);
		}
		else if (i == mKeys.size() - 1)
		{
			tangentVec = (mKeys[i].second - mKeys[i - 1].second);
			this->tangentMinus = Tangent(tangentVec[0], tangentVec[1]);
		}
		else {
			tangentVec = (mKeys[i + 1].second - mKeys[i - 1].second) * 0.5;
			this->tangentPlus = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentMinus = Tangent(tangentVec[0], tangentVec[1]);
		}
	}
	~KeyFrame() {}
};

class CurveSegment
{
public:
	// translation, rotation
	CurveType type;
	// id of curve
	int id;
	// joint
	std::string joint;
	// frame corresponding to left key
	KeyFrame* keyLeft;
	// frame corresponding to right Key
	KeyFrame* keyRight;

	CurveSegment() : type(TRANSLATION), id(0), joint("test"), keyLeft(nullptr), keyRight(nullptr) {}

	CurveSegment(int frameNumber, const std::vector<Key> &mKeys, ASolver& solver) : type(TRANSLATION), id((frameNumber / FPS) * FPS)
	{
		if (solver.keyFrames.find(this->id) == solver.keyFrames.end())
		{
			solver.keyFrames.insert({ this->id, std::make_unique<KeyFrame>(this->id, mKeys) });
		}
		if (solver.keyFrames.find(this->id + FPS) == solver.keyFrames.end())
		{
			solver.keyFrames.insert({ this->id + FPS, std::make_unique<KeyFrame>(this->id + FPS, mKeys) });
		}
		this->keyLeft = solver.keyFrames[this->id].get();
		this->keyRight = solver.keyFrames[this->id + FPS].get();
	}
	~CurveSegment() {}
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
	CurveSegment* curveSegment;
	vec3 point;
	std::vector<CurveSegment*> orderedAffectedCurveSegments;
	bool isPinned;

	State() : frameNumber(0), curveSegment(nullptr), point(vec3()), orderedAffectedCurveSegments(), isPinned(false) {}
	State(int frameNumber, const vec3& value, bool isPinned, const std::vector<Key>& mKeys, ASolver& solver)
		: frameNumber(frameNumber), point(value), isPinned(isPinned)
	{
		if (solver.curveSegments.find(frameNumber) == solver.curveSegments.end())
		{
			solver.curveSegments.insert({frameNumber, std::make_unique<CurveSegment>(frameNumber, mKeys, solver)});
		}
		// set curve segment for my mActiveState
		this->curveSegment = solver.curveSegments[frameNumber].get();
	}
	~State() {}
};



