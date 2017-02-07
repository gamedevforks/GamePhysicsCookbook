#ifndef _H_ACCUMULATED_IMPULSE_
#define _H_ACCUMULATED_IMPULSE_

#include "Rigidbody.h"
#include "RigidbodyVolume.h"
#include <map>

struct AIContact {
	// Contact point info
	vec3 position;
	vec3 normal;
	float separation;
	// Accumulated impulse info
	float Pn;	// accumulated normal impulse
	float Pt;	// accumulated tangent impulse
	float Pnb;	// accumulated normal impulse for position bias
};

struct AIArbiter {
	RigidbodyVolume* body1; // Body 1
	RigidbodyVolume* body2; // Body 2
	std::vector<AIContact> contacts; // Contacts

	inline AIArbiter(RigidbodyVolume* b1, RigidbodyVolume* b2) {
		body1 = b1;
		body2 = b2;
		contacts.clear();
	}

	inline AIArbiter(RigidbodyVolume* b1, RigidbodyVolume* b2, const CollisionManifold& c) {
		body1 = b1;
		body2 = b2;

		contacts.resize(c.contacts.size());
		for (int i = 0; i < c.contacts.size(); ++i) {
			contacts[i].position = c.contacts[i];
			contacts[i].normal = c.normal;
			contacts[i].separation = c.depth;

			contacts[i].Pn = 0.0f;
			contacts[i].Pt = 0.0f;
			contacts[i].Pnb = 0.0f;
		}
	}
};

struct AIArbiterKey {
	Rigidbody* body1;
	Rigidbody* body2;

	inline AIArbiterKey(Rigidbody* b1, Rigidbody* b2) {
		if (b1 < b2) {
			body1 = b1; body2 = b2;
		}
		else {
			body1 = b2; body2 = b1;
		}
	}
};

bool operator < (const AIArbiterKey& a1, const AIArbiterKey& a2);
typedef std::map<AIArbiterKey, AIArbiter>::iterator AIArbIterator;
typedef std::pair<AIArbiterKey, AIArbiter> AIArbPair;

class AIPhysicsSystem {
protected:
	std::vector<Rigidbody*> bodies;
	std::map<AIArbiterKey, AIArbiter> arbiters;
public:
	bool DebugRender;
	float ContactTolerance;
	float AllowedPenetration;
	float BiasFactor;
public:
	AIPhysicsSystem();

	void Update(float deltaTime);
	void Render();

	void AddRigidbody(RigidbodyVolume* body);
	void ClearRigidbodys();
protected: // Update loop support functions (Mostly the same as Box2D Lite!)
	// Same as BoardPahse();
	void UpdateArbiters();
	// Same as Integrate forces
	void IntegrateForces(float deltaTime);
	// Same as Perform pre-step
	void ApplyAccumulatedImpulse();
	// Same as Perform Iterations
	void ApplyImpulse(float inv_dt);
	// Same as Integrate Velocities
	void IntegrateVelocities(float deltaTime);
};

#endif
