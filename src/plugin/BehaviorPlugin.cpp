#include "Plugin.h"
#include "aBehaviorController.h"

#include <string>
#include <float.h>
#include "aVector.h"
#include "aRotation.h"

inline vec3 floatArrayToVec3(float v[3])
{
	return vec3(v[0], v[1], v[2]);
}

// q: (w, x, y, z)
inline quat floatArrayToQuat(float q[4])
{
	return quat(q[0], q[1], q[2], q[3]);
}

struct ActorData
{
	int id;
	float globalPosition[3];
	float globalRotation[4];	// represented by quaternion (w, x, y, z)
	float linearVelocity[3];
	float angularVelocity[3];
	float globalInertialTensor[4];	// represented by quaternion (w, x, y, z)
	float mass;
};

struct ObstacleData
{
	float globalPosition[3];
	float radius;
};


class BehaviorPluginManager
{
public:
	BehaviorPluginManager(int actorNum = 5, int obstacleNum = 3)
		: m_agentNum(actorNum), m_obstacleNum(obstacleNum)
	{
		reset(m_agentNum, m_obstacleNum);
	}

public:
	/* Reset */
	void reset(int agentNum, int obstacleNum)
	{
		resetAgents(agentNum);
		resetObstacles(obstacleNum);
		resetGains();
		m_leaderIndex = 0;
	}

	void resetAgents(int agentNum)
	{
		m_agentNum = agentNum;
		m_agents.clear();
		m_agents.resize(m_agentNum);

		// set up behavior controllers
		for (unsigned int i = 0; i < m_agentNum; i++)
		{
			m_agents[i].getBehaviorController()->createBehaviors(m_agents, m_obstacles);
			m_agents[i].getBehaviorController()->setActiveBehaviorType(m_activeBehavior);
			m_agents[i].getBehaviorController()->setTarget(m_target);

			if (i == 0)
			{
				m_agents[i].getBehaviorController()->setLeader(true);
			}
			// initialize behavior controller states and parameters
			m_agents[i].getBehaviorController()->reset();
		}
	}

	void resetGains()
	{
		BehaviorController::gMaxSpeed = 1000.0;          //default = 1000.0;
		BehaviorController::gMaxAngularSpeed = 200.0; // default = 200.0; 
		BehaviorController::gMaxForce = 2000.0;       // default = 2000.0;
		BehaviorController::gMaxTorque = 2000.0;      // default = 2000.0;
		BehaviorController::gKNeighborhood = 500.0;   //default = 500.0; 
		BehaviorController::gOriKv = 32.0;            //default = 32.0;    
		BehaviorController::gOriKp = 256.0;           //default = 256.0;  
		BehaviorController::gVelKv = 10.0;            //default = 10.0;    
		BehaviorController::gAgentRadius = 80.0;       // default = 80.0;

		BehaviorController::gMass = 1.0;               // default = 1.0;
		BehaviorController::gInertia = 1.0;            // default = 1.0;
		BehaviorController::KArrival = 1.0;            // default = 1.0;
		BehaviorController::KDeparture = 12000.0;      // default = 12000.0;
		BehaviorController::KNoise = 15.0;             // default = 15.0;
		BehaviorController::KWander = 80.0;            // default = 80.0;
		BehaviorController::KAvoid = 600.0;            // default = 600.0;
		BehaviorController::TAvoid = 1000.0;           // default = 1000.0;
		BehaviorController::KSeparation = 12000.0;     // default = 12000.0;
		BehaviorController::KAlignment = 1.0;          // default = 1.0;
		BehaviorController::KCohesion = 1.0;           // default = 1.0;
		BehaviorController::leaderFollowDistance = 200.0; // default = 200.0;
	}
	
	void resetObstacles(int obstacleNum)
	{
		m_obstacleNum = obstacleNum;
		m_obstacles.clear();
		m_obstacles.resize(m_obstacleNum);
	}

	/* Setters */
	void setTarget(vec3 target)
	{
		m_target.setGlobalTranslation(target);
		mat3 I;    I.Identity();
		m_target.setGlobalRotation(I);
	}

	void setObstacles(ObstacleData* obstacleDataArray, int obstacleNum)
	{
		m_obstacles.resize(obstacleNum);
		for (int i = 0; i < obstacleNum; ++i)
		{
			m_obstacles[i].m_Radius = obstacleDataArray[i].radius;
			m_obstacles[i].m_Center.setGlobalTranslation(floatArrayToVec3(obstacleDataArray[i].globalPosition));
		}
	}

	void clearObstacles()
	{
		m_obstacles.clear();
	}

	void setGains(float* gains, int gainNum)
	{
		BehaviorController::gMaxSpeed = gains[0];          //default = 1000.0;
		BehaviorController::gMaxAngularSpeed = gains[1]; // default = 200.0; 
		BehaviorController::gMaxForce = gains[2];       // default = 2000.0;
		BehaviorController::gMaxTorque = gains[3];      // default = 2000.0;
		BehaviorController::gKNeighborhood = gains[4];   //default = 500.0; 
		BehaviorController::gOriKv = gains[5];            //default = 32.0;    
		BehaviorController::gOriKp = gains[6];           //default = 256.0;  
		BehaviorController::gVelKv = gains[7];            //default = 10.0;    
		BehaviorController::gAgentRadius = gains[8];       // default = 80.0;

		BehaviorController::gMass = gains[9];               // default = 1.0;
		BehaviorController::gInertia = gains[10];            // default = 1.0;
		BehaviorController::KArrival = gains[11];            // default = 1.0;
		BehaviorController::KDeparture = gains[12];      // default = 12000.0;
		BehaviorController::KNoise = gains[13];             // default = 15.0;
		BehaviorController::KWander = gains[14];            // default = 80.0;
		BehaviorController::KAvoid = gains[15];            // default = 600.0;
		BehaviorController::TAvoid = gains[16];           // default = 1000.0;
		BehaviorController::KSeparation = gains[17];     // default = 12000.0;
		BehaviorController::KAlignment = gains[18];          // default = 1.0;
		BehaviorController::KCohesion = gains[19];           // default = 1.0;
		BehaviorController::leaderFollowDistance = gains[20];// default = 200.0;
	}

	// Update agents data
	void update(float timestep, ActorData* actorDataArray, int actorNum)
	{
		assert(actorNum == m_agentNum);

		// SENSE PHASE - all agents sense the state of the world in the sense phase
		for (unsigned int i = 0; i < m_agentNum; ++i)
		{
			AJoint guideTransform;
			//BehaviorController* pBehaviorController = m_AgentList[i].getBehaviorController();

			// set agent target (m_target is set in Unity Plugin function setTarget)
			m_agents[i].getBehaviorController()->setTarget(m_target);
			// set agent active behavior type 
			m_agents[i].getBehaviorController()->setActiveBehaviorType(m_activeBehavior);
			// set leader index
			m_agents[i].getBehaviorController()->setLeaderIndex(m_leaderIndex);


			// get Unity Agent state data
			vec3 pos0 = floatArrayToVec3(actorDataArray[i].globalPosition);
			vec3 angles;
			mat3 Rmat = floatArrayToQuat(actorDataArray[i].globalRotation).ToRotation();
			Rmat.ToEulerAngles(mat3::YXZ, angles);

			// set Agent Guide position and orientation
			guideTransform.setGlobalTranslation(pos0);
			guideTransform.setGlobalRotation(Rmat);
			m_agents[i].getBehaviorController()->setGuide(guideTransform);

			// set Agent State for simulation
			vector<vec3> state = m_agents[i].getBehaviorController()->getState();
			//only transfer Agent position and orientation state data to plugin
			state[0] = pos0;
			state[1] = angles;
			m_agents[i].getBehaviorController()->setState(state);

			// call behavior sense function
			m_agents[i].getBehaviorController()->sense(timestep);
		}

		// CONTROL PHASE - given the state of the world all agents determine what to do in control phase
		for (unsigned int i = 0; i < m_agentNum; ++i)
		{
			m_agents[i].getBehaviorController()->control(timestep);
		}

		// ACT PHASE - control is applied and agent state is updated in act phase
		for (unsigned int i = 0; i < m_agentNum; ++i)
		{
			m_agents[i].getBehaviorController()->act(timestep);

			// update UnityAgentList state data
			mat3 Rmat;
			quat rot;
			vector<vec3>& state = m_agents[i].getBehaviorController()->getState();

			vec3 pos0 = state[0];
			vec3 eulerAngles = state[1];
			vec3 velB = state[2];
			vec3 angVelB = state[3];

			actorDataArray[i].globalPosition[0] = pos0[0];
			actorDataArray[i].globalPosition[1] = pos0[1];
			actorDataArray[i].globalPosition[2] = pos0[2];
			Rmat.FromEulerAngles(mat3::YZX, eulerAngles);
			rot.FromRotation(Rmat);
			actorDataArray[i].globalRotation[0] = rot.W();
			actorDataArray[i].globalRotation[1] = rot.X();
			actorDataArray[i].globalRotation[2] = rot.Y();
			actorDataArray[i].globalRotation[3] = rot.Z();
			vec3 lVel = Rmat * velB;
			vec3 aVel = Rmat * angVelB;
			actorDataArray[i].linearVelocity[0] = lVel[0];
			actorDataArray[i].linearVelocity[1] = lVel[1];
			actorDataArray[i].linearVelocity[2] = lVel[2];
			actorDataArray[i].angularVelocity[0] = aVel[0];
			actorDataArray[i].angularVelocity[1] = aVel[1];
			actorDataArray[i].angularVelocity[2] = aVel[2];
		}
	}

	void setActiveBehavior(int activeBehavior) { m_activeBehavior = (BehaviorType)activeBehavior; }
	int getActiveBehavior() { return (int)m_activeBehavior; }
	void setAgentActiveBehavior(int agentID, int activeBehavior)
	{
		m_agents[agentID].getBehaviorController()->setActiveBehaviorType((BehaviorType)activeBehavior);
	}
	int getGainNum() { return m_gainNum; }

	void setLeaderIndex(int index) { m_leaderIndex = index; }
	int getLeaderIndex() { return m_leaderIndex; }


protected:
	std::vector<AActor> m_agents;
	std::vector<Obstacle> m_obstacles;

	AJoint m_target;
	bool m_showObstacles;
	int m_agentNum;
	int m_obstacleNum;
	int m_gainNum = 25;

	//enum BehaviorType { SEEK, FLEE, ARRIVAL, DEPARTURE, AVOID, WANDER, ALIGNMENT, SEPARATION, COHESION, FLOCKING, LEADER };
	BehaviorType m_activeBehavior;
	int m_leaderIndex;

};

extern "C"
{
	static BehaviorPluginManager m_behaviorPluginManager;

	EXPORT_API void InitializeBehaviorPlugin(int agentNum, int obstacleNum, int activeBehavior)
	{
		m_behaviorPluginManager.setActiveBehavior(activeBehavior);
		m_behaviorPluginManager.reset(agentNum, obstacleNum);
	}

	EXPORT_API void SetTarget(float target[])
	{
		m_behaviorPluginManager.setTarget(vec3(target[0], target[1], target[2]));
	}

	EXPORT_API void SetObstacleData(ObstacleData* obstacleDataArray, int obstacleNum)
	{
		m_behaviorPluginManager.setObstacles(obstacleDataArray, obstacleNum);
	}

	EXPORT_API void ClearObstacles()
	{
		m_behaviorPluginManager.clearObstacles();
	}

	EXPORT_API void SetControllerGains(float gains[], int gainsNum)
	{
		m_behaviorPluginManager.setGains(gains, gainsNum);
	}

	EXPORT_API void UpdateActorData(float timestep, ActorData* actorDataArray, int actorNum)
	{
		m_behaviorPluginManager.update(timestep, actorDataArray, actorNum);
	}

	EXPORT_API int GetGainNum()
	{
		return m_behaviorPluginManager.getGainNum();
	}

	EXPORT_API void SetLeaderIndex(int index)
	{
		m_behaviorPluginManager.setLeaderIndex(index);
	}

	EXPORT_API int GetLeaderIndex()
	{
		return m_behaviorPluginManager.getLeaderIndex();
	}


}