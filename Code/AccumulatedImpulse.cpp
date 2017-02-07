#include "AccumulatedImpulse.h"
#include "FixedFunctionPrimitives.h"
#include "Compare.h"
#include "glad/glad.h"
#include <iostream>

bool operator < (const AIArbiterKey& a1, const AIArbiterKey& a2) {
	if (a1.body1 < a2.body1) {
		return true;
	}
	if (a1.body1 == a2.body1 && a1.body2 < a2.body2) {
		return true;
	}
	return false;
}

// Public interface of physics system
AIPhysicsSystem::AIPhysicsSystem() {
	DebugRender = false;
	ContactTolerance = 0.01f;
	AllowedPenetration = 0.01f;
	BiasFactor = 0.2f;
}

void AIPhysicsSystem::Update(float deltaTime) {
	float inv_dt = deltaTime > 0.0f ? 1.0f / deltaTime : 0.0f;

	UpdateArbiters();
	IntegrateForces(deltaTime);
	ApplyAccumulatedImpulse();
	for (int i = 0; i < 10; ++i) {
		ApplyImpulse(inv_dt);
	}
	IntegrateVelocities(deltaTime);
}

void AIPhysicsSystem::Render() {
	static const float rigidbodyDiffuse[]{ 200.0f / 255.0f, 0.0f, 0.0f, 0.0f };
	static const float rigidbodyAmbient[]{ 200.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f, 0.0f };

	static const float groundDiffuse[]{ 0.0f, 0.0f, 200.0f / 255.0f, 0.0f };
	static const float groundAmbient[]{ 50.0f / 255.0f, 50.0f / 255.0f, 200.0f / 255.0f, 0.0f };

	static const float constraintDiffuse[]{ 0.0f, 200.0f / 255.0f, 0.0f, 0.0f };
	static const float constraintAmbient[]{ 50.0f / 255.0f, 200.0f / 255.0f, 50.0f / 255.0f, 0.0f };

	static const float zero[] = { 0.0f, 0.0f, 0.0f, 0.0f };

	std::vector<const float*> ambient;
	std::vector<const float*> diffuse;
	ambient.push_back(rigidbodyAmbient);
	ambient.push_back(groundAmbient);
	ambient.push_back(constraintAmbient);
	diffuse.push_back(rigidbodyDiffuse);
	diffuse.push_back(groundDiffuse);
	diffuse.push_back(constraintDiffuse);

	for (int i = 0, size = bodies.size(); i < size; ++i) {
		int a_i = i % ambient.size();
		int d_i = i % diffuse.size();
		glColor3f(diffuse[d_i][0], diffuse[d_i][1], diffuse[d_i][2]);
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambient[a_i]);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse[d_i]);
		glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
		if (DebugRender && bodies[i]->type == RIGIDBODY_TYPE_BOX) {
			RigidbodyVolume* mb = (RigidbodyVolume*)bodies[i];
			mb->SynchCollisionVolumes();
			::Render(GetEdges(mb->box));
		}
		else {
			bodies[i]->Render();
		}
	}
}

void AIPhysicsSystem::AddRigidbody(RigidbodyVolume* body) {
	if (body->type == RIGIDBODY_TYPE_BOX) {
		bodies.push_back(body);
	}
	else {
		std::cout << "Accumulated impulse sample only allows boxes!\n";
	}
}

void AIPhysicsSystem::ClearRigidbodys() {
	bodies.clear();
}

void AIPhysicsSystem::UpdateArbiters() { // Done
	for (int i = 0; i < bodies.size(); ++i) {
		RigidbodyVolume* bi = (RigidbodyVolume*)bodies[i];
		for (int j = i + 1; j < bodies.size(); ++j) {
			RigidbodyVolume* bj = (RigidbodyVolume*)bodies[j];
			if (bi->InvMass() == 0.0f && bj->InvMass() == 0.0f) {
				continue;
			}
			CollisionManifold result = FindCollisionFeatures(*bi, *bj);

			AIArbiter newArb(bi, bj, result);
			AIArbiterKey arbKey(bi, bj);

			if (result.colliding && newArb.contacts.size() > 0) {
				AIArbIterator iter = arbiters.find(arbKey);
				if (iter == arbiters.end())
				{
					arbiters.insert(AIArbPair(arbKey, newArb));
				}
				else {
					std::vector<AIContact> oldContacts = iter->second.contacts;
					iter->second.contacts = newArb.contacts;

					for (int k = 0; k < newArb.contacts.size(); ++k) {
						for (int l = 0; l < oldContacts.size(); ++l) {
							float pos = MagnitudeSq(newArb.contacts[k].position - oldContacts[l].position);
							float nor = MagnitudeSq(newArb.contacts[k].normal - oldContacts[l].normal);
							float sep = fabsf(newArb.contacts[k].separation - oldContacts[l].separation);

							if (pos < ContactTolerance && nor < ContactTolerance && sep < ContactTolerance) {
								iter->second.contacts[k].Pn = oldContacts[l].Pn;
								iter->second.contacts[k].Pt = oldContacts[l].Pt;
								iter->second.contacts[k].Pnb = oldContacts[l].Pnb;
							}
						}
					}
				}
			}
			else {
				arbiters.erase(arbKey);
			}
		}
	}
}

void AIPhysicsSystem::IntegrateForces(float deltaTime) { // DONE!
	for (int i = 0; i < bodies.size(); ++i) {
		RigidbodyVolume* b = (RigidbodyVolume*)bodies[i];

		if (b->InvMass() == 0.0f) {
			continue;
		}

		// Apply constant linear forces
		b->forces = GRAVITY_CONST * b->mass;

		// Integrate linear velocity
		vec3 acceleration = b->forces * b->InvMass();
		b->velocity = b->velocity + acceleration * deltaTime;

		// Apply constant forces
		// No constant torque

		// Integrate angular velocity
		vec3 angAccel = MultiplyVector(b->torques, b->InvTensor());
		b->angVel = b->angVel +angAccel * deltaTime;
	}
}

void AIPhysicsSystem::ApplyAccumulatedImpulse() { // DONE!
	return;
	for (AIArbIterator arb = arbiters.begin(); arb != arbiters.end(); ++arb)
	{
		RigidbodyVolume* body1 = arb->second.body1;
		RigidbodyVolume* body2 = arb->second.body2;

		//arb->second.PreStep();
		for (int i = 0; i < arb->second.contacts.size(); ++i)
		{
			AIContact* c = &(arb->second.contacts[i]);

			vec3 r1 = c->position - body1->position;
			vec3 r2 = c->position - body2->position;

			// TODO: This might be reversed!
			//vec3 relativeVel = (arb->second.body1->velocity + Cross(arb->second.body1->angVel, r1)) - (arb->second.body2->velocity + Cross(arb->second.body2->angVel, r2));
			vec3 relativeVel = body2->velocity + Cross(body2->angVel, r2) - body1->velocity - Cross(body1->angVel, r1);
			vec3 tangent = relativeVel - (c->normal * Dot(relativeVel, c->normal));
			if (MagnitudeSq(tangent) != 0.0f) {
				Normalize(tangent);
			}

			// Apply normal + friction impulse
			vec3 P = c->normal * c->Pn + tangent * c->Pt;

			body1->velocity = body1->velocity - (P * body1->InvMass());
			body1->angVel = body1->angVel - MultiplyVector(Cross(r1, P), body1->InvTensor());

			body2->velocity = body2->velocity + (P * body2->InvMass());
			body2->angVel = body2->angVel + MultiplyVector(Cross(r2, P), body2->InvTensor());
		}
	}
}

void AIPhysicsSystem::ApplyImpulse(float inv_dt) {
	for (AIArbIterator arb = arbiters.begin(); arb != arbiters.end(); ++arb)
	{
		RigidbodyVolume* body1 = arb->second.body1;
		RigidbodyVolume* body2 = arb->second.body2;

		float invMass1 = body1->InvMass();
		float invMass2 = body2->InvMass();
		float invMassSum = invMass1 + invMass2;

		mat4 i1 = body1->InvTensor();
		mat4 i2 = body2->InvTensor();

		for (int i = 0; i < arb->second.contacts.size(); ++i)
		{
			AIContact* c = &(arb->second.contacts[i]);
			
			/// Copy pasta start
			vec3 r1 = c->position - arb->second.body1->position;
			vec3 r2 = c->position - arb->second.body2->position;

			// TODO: This might be reversed!
			//vec3 relativeVel = (arb->second.body1->velocity + Cross(arb->second.body1->angVel, r1)) - (arb->second.body2->velocity + Cross(arb->second.body2->angVel, r2));
			vec3 relativeVel = body2->velocity + Cross(body2->angVel, r2) - body1->velocity - Cross(body1->angVel, r1);
			if (ZERO(MagnitudeSq(relativeVel))) {
				relativeVel = vec3();
			}
			vec3 tangent = relativeVel - (c->normal * Dot(relativeVel, c->normal));
			if (MagnitudeSq(tangent) != 0.0f) {
				Normalize(tangent);
			}
			/// Copy pasta end
			
			// Coefficient of restitution: e
			float e = fminf(body1->cor, body2->cor);

			// Mass normal: j
			float numerator = (-(1.0f + e) * Dot(relativeVel, c->normal));
			float d1 = invMassSum;
			vec3 d2 = Cross(MultiplyVector(Cross(r1, c->normal), i1), r1);
			vec3 d3 = Cross(MultiplyVector(Cross(r2, c->normal), i2), r2);
			float denominator = d1 + Dot(c->normal, d2 + d3);
			float massNormal /*j*/ = (ZERO(denominator) || ZERO(numerator)) ? 0.0f : numerator / denominator;

			// Mass tangent: jt
			numerator = -Dot(relativeVel, tangent);
			d1 = invMassSum;
			d2 = Cross(MultiplyVector(Cross(r1, tangent), i1), r1);
			d3 = Cross(MultiplyVector(Cross(r2, tangent), i2), r2);
			denominator = d1 + Dot(tangent, d2 + d3);
			float massTangent /*jt*/ = (ZERO(denominator) || ZERO(numerator)) ? 0.0f : numerator / denominator;

			// Penetration bias: bias
			float bias = -BiasFactor * inv_dt * fminf(0.0f, c->separation + AllowedPenetration);

			vec3 dv = relativeVel;
			float vn = Dot(dv, c->normal);
			float dPn = massNormal * (-vn + bias);

			// Clamp the accumulated impulse
			float Pn0 = c->Pn;
			c->Pn = fmaxf(Pn0 + dPn, 0.0f);
			dPn = c->Pn - Pn0;

			// Apply contact impulse
			vec3 Pn = c->normal * dPn;

			vec3 debugBody1PrevVel = body1->velocity;

			body1->velocity = body1->velocity - (Pn * body1->InvMass());
			body1->angVel =   body1->angVel -   MultiplyVector(Cross(r1, Pn), i1);

			body2->velocity = body2->velocity + (Pn * body2->InvMass());
			body2->angVel =   body2->angVel + MultiplyVector(Cross(r2, Pn), i2);

			// Relative velocity at contact
			dv = body2->velocity + Cross(body2->angVel, r2) - body1->velocity - Cross(body1->angVel, r1);
			if (ZERO(MagnitudeSq(dv))) {
				dv = vec3();
			}

			float vt = Dot(dv, tangent);
			float dPt = massTangent * (-vt);

			// Compute friction impulse
			float friction = sqrtf(body1->friction * body2->friction);
			float maxPt = friction * c->Pn;

			// Clamp friction
			float oldTangentImpulse = c->Pt;
			// Clamp c->pT to -maxPt and maxPt
			//c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
			c->Pt = oldTangentImpulse + dPt;
			if (c->Pt < -maxPt) {
				c->Pt = -maxPt;
			}
			else {
				c->Pt = maxPt;
			}
			dPt = c->Pt - oldTangentImpulse;

			// Apply contact impulse
			vec3 Pt = tangent * dPt;

			body1->velocity = body1->velocity - (Pt * body1->InvMass());
			body1->angVel = body1->angVel - MultiplyVector( Cross(r1, Pt), i1);

			body2->velocity = body2->velocity + (Pt * body2->InvMass());
			body2->angVel = body2->angVel + MultiplyVector(Cross(r2, Pt), i2);
		}
	}
}

void AIPhysicsSystem::IntegrateVelocities(float deltaTime) { // DONE
	// Integrate Velocities
	for (int i = 0; i < bodies.size(); ++i) {
		RigidbodyVolume* b = (RigidbodyVolume*)bodies[i];

		b->position = b->position + b->velocity * deltaTime;
		b->orientation = b->orientation + b->angVel * deltaTime;

		// Reset accumulated force & torque
	}
}