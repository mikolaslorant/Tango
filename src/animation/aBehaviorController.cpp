#include "aBehaviorController.h"

#include "aVector.h"
#include "aRotation.h"
#include <Windows.h>
#include <algorithm>



#define Truncate(a, b, c) (a = max<double>(min<double>(a,c),b))

double BehaviorController::gMaxSpeed = 200.0; //default = 1000.0, plugin = 6.0
double BehaviorController::gMinSpeed = 1.0;
double BehaviorController::gMaxAngularSpeed = 200.0; // default = 200.0 
double BehaviorController::gMaxForce = 2000.0;   // default = 2000.0
double BehaviorController::gMaxTorque = 2000.0;  // default = 2000.0
double BehaviorController::gKNeighborhood = 500.0; //default = 500.0, plugin = 100.0     
double BehaviorController::gOriKv = 32.0; //default = 32.0;    
double BehaviorController::gOriKp = 256.0; //default = 256.0;  
double BehaviorController::gVelKv = 10.0; //default = 10.0;    
double BehaviorController::gAgentRadius = 20.0; // default = 80.0, plugin = 2.0 


double BehaviorController::gMass = 1;
double BehaviorController::gInertia = 1;
double BehaviorController::KArrival = 1.0; 
double BehaviorController::KDeparture = 12000.0;
double BehaviorController::KNoise = 15.0;
double BehaviorController::KWander = 80.0;   
double BehaviorController::KAvoid = 200.0;  // 600.0;
double BehaviorController::TAvoid = 100.0;   
double BehaviorController::KSeparation = 12000.0; 
double BehaviorController::KAlignment = 1.0;  
double BehaviorController::KCohesion = 1.0;  
double BehaviorController::leaderFollowDistance = 200.0;
double BehaviorController::obstacleOffset = 100.0;
double BehaviorController::lookAhead = 0.0;

const double M2_PI = M_PI * 2.0;

BehaviorController::BehaviorController() 
{
	m_state.resize(m_stateDim);
	m_stateDot.resize(m_stateDim);
	m_controlInput.resize(m_controlDim);

	vec3 m_Pos0 = vec3(0, 0, 0);
	vec3 m_Vel0 = vec3(0, 0, 0);
	vec3 m_lastVel0 = vec3(0, 0, 0);
	vec3 m_Euler = vec3(0, 0, 0);
	vec3 m_VelB = vec3(0, 0, 0);
	vec3 m_AVelB = vec3(0, 0, 0);
	
	m_Vdesired = vec3(0, 0, 0);
	m_lastThetad = 0.0;

	m_Active = true; 
	mpActiveBehavior = NULL;
	mLeader = false;
	m_leaderIndex = 0;

	reset();
}

AActor* BehaviorController::getActor()
{
	return m_pActor;
}

void BehaviorController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();
}


void BehaviorController::createBehaviors(vector<AActor>& agentList, vector<Obstacle>& obstacleList)
{
	m_AgentList = &agentList;
	m_ObstacleList = &obstacleList;

	m_BehaviorList.clear();
	m_BehaviorList[SEEK] = new Seek(m_pBehaviorTarget);
	m_BehaviorList[FLEE] = new Flee(m_pBehaviorTarget);
	m_BehaviorList[ARRIVAL] = new Arrival(m_pBehaviorTarget);
	m_BehaviorList[DEPARTURE] = new Departure(m_pBehaviorTarget);
	m_BehaviorList[WANDER] = new Wander();
	m_BehaviorList[COHESION] = new Cohesion(m_AgentList);
	m_BehaviorList[ALIGNMENT] = new Alignment(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[SEPARATION] = new Separation(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[LEADER] = new Leader(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[FLOCKING] = new Flocking(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[AVOID] = new Avoid(m_pBehaviorTarget, m_ObstacleList);
}

BehaviorController::~BehaviorController()
{
	mpActiveBehavior = NULL;
}

void BehaviorController::reset()
{
	vec3 startPos;
	startPos[0] = ((double)rand()) / RAND_MAX;
	startPos[1] =  ((double)rand()) / RAND_MAX, 
	startPos[2] = ((double)rand()) / RAND_MAX;
	startPos = startPos - vec3(0.5, 0.5, 0.5);

	startPos[1] = 0.0; // set equal to zero for 2D case (assume y is up)
	mat3 Rmat;
	Rmat.Identity();  // sets initial orientation to be aligned with world coords

	m_Guide.setLocalTranslation(startPos * 500.0);
	m_Guide.setLocalRotation(Rmat);
	m_Guide.setGlobalTranslation(startPos * 500.0);
	m_Guide.setGlobalRotation(Rmat);
	
	for (int i = 0; i < m_stateDim; i++)
	{
		m_state[i] = 0.0;
		m_stateDot[i] = 0.0;
	}

	m_state[0] = m_Guide.getGlobalTranslation();
	m_force = 0.0;
	m_torque = 0.0;
	m_thetad = 0.0;
	m_vd = 0.0;
}

///////////////////////////////////////////////////

inline void ClampAngle(double& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}

void BehaviorController::sense(double deltaT)
{
	if (mpActiveBehavior)
	{
		// find the agents in the neighborhood of the current character.
	}
}

void BehaviorController::control(double deltaT)
// Given the active behavior this function calculates a desired velocity vector (Vdesired).  
// The desired velocity vector is then used to compute the desired speed (vd) and direction (thetad) commands
{
	if (mpActiveBehavior)
	{
		m_Vdesired = mpActiveBehavior->calcDesiredVel(this);
		m_Vdesired[1] = 0;
		double speed_d;  // desired speed
		m_vd = speed_d = m_Vdesired.Length();
		Truncate(speed_d, 0, gMaxSpeed);

		if (speed_d < gMinSpeed)
		{
			m_vd = speed_d = 0.0;
			m_thetad = m_lastThetad;
		}
		else
		{
			m_thetad = atan2(m_Vdesired[_X], m_Vdesired[_Z]);
			m_lastThetad = m_thetad;
		}
		ClampAngle(m_thetad);
		double thetad_degrees = m_thetad * (180.0 / M_PI);

		//  force and torque inputs are computed from vd and thetad as follows:
		//              Velocity P controller : force = mass * Kv * (vd - v)
		//              Heading PD controller : torque = Inertia * (-Kv * thetaDot -Kp * (thetad - theta))
		//  where the values of the gains Kv and Kp are different for each controller

		// TODO: insert your code here to compute m_force and m_torque


		// when agent desired agent velocity and actual velocity < gMinSpeed then stop moving
		vec3 Vb = m_state[VEL];
		if (m_vd < gMinSpeed &&  Vb.Length() < gMinSpeed)
		{
			m_force[2] = 0.0;
			m_torque[1] = 0.0;
		}
	}
	else
	{
		m_force[2] = 0.0;
		m_torque[1] = 0.0;
	}

	// set control inputs to current force and torque values
	m_controlInput[0] = m_force;
	m_controlInput[1] = m_torque;
}

void BehaviorController::act(double deltaT)
{
	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);
	
	int EULER = 0;
	int RK2 = 1;
	updateState(deltaT, EULER);
}


void BehaviorController::computeDynamics(vector<vec3>& state, vector<vec3>& controlInput, vector<vec3>& stateDot, double deltaT)
// Compute stateDot vector given the control input and state vectors
//  This function sets derive vector to appropriate values after being called
{
	vec3& force = controlInput[0];
	vec3& torque = controlInput[1];
	// TODO: add your code here

}

void BehaviorController::updateState(float deltaT, int integratorType)
//  Update the state vector given the m_stateDot vector
//  Perform validation check to make sure all values are within MAX values
{
	// Update the state vector given the m_stateDot vector using Euler (integratorType = 0) or RK2 (integratorType = 1) integration
	// this should be similar to what you implemented in the particle system assignment

	// TODO: add your code here

	//  given the new values in m_state, these are the new component state values 
	m_Pos0 = m_state[POS];
	m_Euler = m_state[ORI];
	m_VelB = m_state[VEL];
	m_AVelB = m_state[AVEL];
	m_Vel0 = m_stateDot[POS];
	// Perform validation check to make sure all values are within MAX values
	// TODO: add your code here

	// update the guide orientation
	// compute direction from nonzero velocity vector
	vec3 dir;
	if (m_Vel0.Length() < 1.0)
	{
		dir = m_lastVel0;
		dir.Normalize();;
		m_state[ORI] = atan2(dir[_X], dir[_Z]);
	}
	else
	{
		dir = m_Vel0;
		m_lastVel0 = m_Vel0;
	}

	dir.Normalize();
	vec3 up(0.0, 1.0, 0.0);
	vec3 right = up.Cross(dir);
	right.Normalize();
	mat3 rot(right, up, dir);
	m_Guide.setGlobalRotation(rot.Transpose());
	m_Guide.setGlobalTranslation(m_Guide.getGlobalTranslation() + m_Vel0 * deltaT);
}


void BehaviorController::setTarget(AJoint& target)
{
	m_pBehaviorTarget = &target;
	for (unsigned int i = 0; i < m_BehaviorList.size(); i++)
	{
		BehaviorType index = (BehaviorType) i;
		m_BehaviorList[index]->setTarget(m_pBehaviorTarget);
	}
}

void BehaviorController::setActiveBehavior(Behavior* pBehavior)
{
	mpActiveBehavior = pBehavior;
}

void BehaviorController::setActiveBehaviorType(BehaviorType type)
{
	m_BehaviorType = type;
	Behavior* pActiveBehavior = m_BehaviorList[type];
	setActiveBehavior(pActiveBehavior);

}

vec3 BehaviorController::getForce(RefFrameType type)
{
	vec3 force;
	if (type == WORLD)
		 force = m_Guide.getGlobalRotation()*m_force;
	else force = m_force;

	return force;
}

vec3 BehaviorController::getTorque(RefFrameType type)
{
	vec3 torque;
	if (type == WORLD)
		torque = m_Guide.getGlobalRotation()*m_torque;
	else torque = m_torque;

	return torque;
}


