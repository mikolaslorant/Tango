#pragma once
#include <unordered_map>
#include "aVector.h"

#define FPS 120
typedef std::pair<double, vec3> Key;

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
	Tangent tangentMinus[3];
	Tangent tangentPlus[3];

	KeyFrame() : t(0.0), frameNumber(0), tangentMinus{ Tangent(), Tangent(), Tangent() }, tangentPlus{ Tangent(), Tangent(), Tangent() } {}
	KeyFrame(int i, std::vector<Key> &mKeys) {
		this->t = mKeys[i].first;
		this->frameNumber = i * FPS;
		vec3 tangentVec;
		if (i == 0)
		{
			tangentVec = (mKeys[i + 1].second - mKeys[i].second);

			this->tangentPlus[0] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentPlus[1] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentPlus[2] = Tangent(tangentVec[0], tangentVec[1]);
		}
		else if (i == mKeys.size() - 1)
		{
			tangentVec = (mKeys[i].second - mKeys[i - 1].second);
			this->tangentMinus[0] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentMinus[1] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentMinus[2] = Tangent(tangentVec[0], tangentVec[1]);
		}
		else {
			tangentVec = (mKeys[i + 1].second - mKeys[i - 1].second) * 0.5;
			this->tangentPlus[0] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentPlus[1] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentPlus[2] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentMinus[0] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentMinus[1] = Tangent(tangentVec[0], tangentVec[1]);
			this->tangentMinus[2] = Tangent(tangentVec[0], tangentVec[1]);
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
	CurveSegment(int frameNumber, std::vector<Key> &mKeys) {
		// Assume translation
		this->type = TRANSLATION;
		
		// we set curve segment id to left keyframe number
		this->id = frameNumber / FPS;
		
		// set keyframes for curve segment
		std::unique_ptr<KeyFrame> leftKeyframe = std::make_unique<KeyFrame>(this->id, mKeys);
		std::unique_ptr<KeyFrame> rightKeyframe = std::make_unique<KeyFrame>(this->id + 1, mKeys);
		this->keyLeft = leftKeyframe.get();
		this->keyRight = rightKeyframe.get();
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
	State(int frameNumber, const vec3& value, std::vector<Key> &mKeys, bool isPinned) {
		std::unique_ptr<CurveSegment> curveSegment = std::make_unique<CurveSegment>(frameNumber, mKeys);
		this->frameNumber = frameNumber;
		this->point = value;
		this->isPinned = isPinned;
		// set curve segment for my mActiveState
		this->curveSegment = curveSegment.get();
	}
	~State() {}
};

//class Pin : public State
//{
//public:
//	Pin() : State() {}
//	~Pin() {}
//};



class ASolver
{
public:
	// C : all curve segments affected
	std::vector<std::unique_ptr<State>> pins;
	std::vector<std::unique_ptr<Contact>> contacts;
	// solve for new state S' passed as parameter
	void solve(const State *newState);
};

