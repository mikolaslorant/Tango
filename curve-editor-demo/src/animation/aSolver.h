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
	Tangent() : x(0.f), y(0.f)
	{}
	Tangent(double x, double y) : x(x), y(y)
	{}
	double x;
	double y;
};

class KeyFrame
{
public:
	double t;
	int frameNumber;
	Tangent tangentMinus[3];
	Tangent tangentPlus[3];
	
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
	
};

class Contact
{
public:
	int id;
	CurveSegment* curveSegment;
	std::vector<CurveSegment*> orderedAffectedCurveSegments;
	vec3 point;
};

class State
{
public:
	// Cs or Csj
	std::vector<CurveSegment*> orderedAffectedCurveSegments;
	CurveSegment* curveSegment;
	vec3 point;
	int frameNumber;
};

class Pin : public State
{
	
};



class ASolver
{
public:
	// C : all curve segments affected
	std::vector<std::unique_ptr<CurveSegment>> curveSegments;
	std::vector<std::unique_ptr<Pin>>  pins;
	std::vector<std::unique_ptr<Contact>> contacs;
	std::vector<std::unique_ptr<KeyFrame>> keys;
	// solve for new state S' passed as parameter
	void solve(const State& newState);
	//void addPin(const State& state);
	//void addContact(const Contact& contact);
	//void getKey(int i);
};

