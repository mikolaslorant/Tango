#pragma once
#include <unordered_map>
#include "aVector.h"

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
	std::vector<CurveSegment*> orderedAffectedCurveSegments;
	vec3 point;
	

	State() : frameNumber(0), curveSegment(nullptr), orderedAffectedCurveSegments(), point(vec3()) {}
	~State() {}
};

class Pin : public State
{
public:
	Pin() : State() {}
	~Pin() {}
};



class ASolver
{
public:
	// C : all curve segments affected
	std::vector<std::unique_ptr<CurveSegment>> curveSegments;
	//std::vector<std::unique_ptr<Pin>>  pins;
	std::vector<std::unique_ptr<State>>  pins;
	std::vector<std::unique_ptr<Contact>> contacs;
	std::vector<std::unique_ptr<KeyFrame>> keys;
	// solve for new state S' passed as parameter
	void solve(const State& newState);
	//void addPin(const State& state);
	//void addContact(const Contact& contact);
	//void getKey(int i);
};

