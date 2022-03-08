#include "Plugin.h"
#include "aParticleSystem.h"
#include <vector>
#include <unordered_map>

inline vec3 floatArrayToVec3(float v[3])
{
	return vec3(v[0], v[1], v[2]);
}

struct ParticleData
{
	bool isAlive;
	float position[3];	// x y z
	float color[3];
	float scale;
	float alpha;
};

struct ParticleSystemParameter
{
	bool infinite;
	int maxParticles;
	float lifeTime;
	float gravity[3];
	float position[3];
	float velocity[3];
	float startColor[3];
	float endColor[3];
	float startAlpha;
	float startSize;
	float endSize;
	float positionJitter[2]; // min, max
	float velocityJitter[2]; // min, max
};

class ParticlePluginManager
{
public:
	ParticlePluginManager() {}

	//std::vector<AParticleSystem> mParticleSystemPool;
	std::unordered_map<int, AParticleSystem> mParticleSystemPool;
	int mCurrentIndex = 0;

	int createParticleSystem()
	{
		mParticleSystemPool.insert({ mCurrentIndex, AParticleSystem() });
		mCurrentIndex++;
		return mCurrentIndex - 1;
	}

	void removeParticleSystem(int id)
	{
		mParticleSystemPool.erase(id);
	}

	int getParticleNum(int id)
	{
		return mParticleSystemPool[id].getParticleNum();
	}

	int getMaxParticleNum(int id)
	{
		return mParticleSystemPool[id].mMaxParticles;
	}

	void updateParticleSystem(int id, float deltaT)
	{
		mParticleSystemPool[id].update(deltaT);
	}

	void setParticleSystemParameters(int id, ParticleSystemParameter parm)
	{
		AParticleSystem& particles = mParticleSystemPool[id];
		particles.mInfinite = parm.infinite;
		particles.mMaxParticles = parm.maxParticles;
		particles.mLifeSpan = parm.lifeTime;
		particles.mGravity = floatArrayToVec3(parm.gravity);
		particles.mStartPos = floatArrayToVec3(parm.position);
		particles.mStartVel = floatArrayToVec3(parm.velocity);
		particles.mStartColor = floatArrayToVec3(parm.startColor);
		particles.mEndColor = floatArrayToVec3(parm.endColor);
		particles.mStartAlpha = parm.startAlpha;
		particles.mStartScale = parm.startSize;
		particles.mEndScale = parm.endSize;
		particles.mPositionJitter = AJitter(parm.positionJitter[0], parm.positionJitter[1]);
		particles.mVelocityJitter = AJitter(parm.velocityJitter[0], parm.velocityJitter[1]);
	}

	bool isAlive(int id)
	{
		return mParticleSystemPool[id].isAlive();
	}

	void writeParticleData(ParticleData* particleDataArray, int id, int size)
	{
		AParticleSystem& particles = mParticleSystemPool[id];
		assert(size <= particles.getParticleNum());
		for (int i = 0; i < size; ++i)
		{
			AParticle* particle = particles.getParticle(i);
			ParticleData& data = particleDataArray[i];
			bool isAlive = particle->isAlive();
			data.isAlive = isAlive;
			if (!isAlive) { continue; }
			// ramp the color and fade out the particle as its life runs out 
			double u = particle->getTimeToLive() / particle->m_lifeSpan;			
			double scale = particle->mStartScale * u + particle->mEndScale * (1 - u);
			double alpha = particle->mStartAlpha * u + particle->mEndAlpha * (1 - u);
			vec3 color = particle->mStartColor * u + particle->mEndColor * (1 - u);
			vec3 pos = particle->m_Pos;

			data.position[0] = pos[0];
			data.position[1] = pos[1];
			data.position[2] = pos[2];
			data.color[0] = color[0];
			data.color[1] = color[1];
			data.color[2] = color[2];
			data.scale = scale;
			data.alpha = alpha;
		}
	}


};

extern "C"
{
	static ParticlePluginManager mParticlePluginManager;

	// Create a new particle system and return its id
	EXPORT_API int CreateParticleSystem()
	{
		return mParticlePluginManager.createParticleSystem();
	}

	// Remove the particle system specified by its id
	EXPORT_API void RemoveParticleSystem(int id)
	{
		mParticlePluginManager.removeParticleSystem(id);
	}

	// Return the number of particles 
	EXPORT_API int GetParticleNum(int id)
	{
		return mParticlePluginManager.getParticleNum(id);
	}

	// Return the maximum number of particles
	EXPORT_API int GetMaxParticleNum(int id)
	{
		return mParticlePluginManager.getMaxParticleNum(id);
	}

	// Get states of particles
	EXPORT_API void GetParticleData(ParticleData* particleDataArray, int id, int size)
	{
		mParticlePluginManager.writeParticleData(particleDataArray, id, size);
	}

	// Update the states of the particles by deltaT
	EXPORT_API void UpdateParticleSystem(int id, float deltaT)
	{
		mParticlePluginManager.updateParticleSystem(id, deltaT);
	}

	// Set parameters
	EXPORT_API void SetParticleSystemParameters(int id, ParticleSystemParameter parm)
	{
		mParticlePluginManager.setParticleSystemParameters(id, parm);
	}

	// Check if the particle system is alive
	EXPORT_API bool IsAlive(int id)
	{
		return mParticlePluginManager.isAlive(id);
	}
}
