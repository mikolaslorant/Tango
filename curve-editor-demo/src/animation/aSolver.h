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
	double x;
	double y;
};

class Key
{
public:
	double t;
	int number;
	Tangent tangentMinus;
	Tangent tangentPlus;

	Key(int number);
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
	Key* keyLeft;
	// frame corresponding to right Key
	Key* keyRight;
	
};

class Contact
{
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
};

class Pin : public State
{
	
};



class ASolver
{
public:
	// C : all curve segments affected
	std::unordered_map<int, std::unique_ptr<CurveSegment>> curveSegments;
	std::vector<std::unique_ptr<Pin>>  pins;
	std::vector<std::unique_ptr<Contact>> contacs;
	std::vector<std::unique_ptr<Key>> keys;
	// solve for new state S' passed as parameter
	void solve(const State& newState);
	void addPin(const State& state);
	void addContact(const Contact& contact);
	void getKey(int i);
};

