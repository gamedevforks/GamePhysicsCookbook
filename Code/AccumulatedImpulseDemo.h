#ifndef _H_ACCUMULATED_IMPULSE_DEMO_
#define _H_ACCUMULATED_IMPULSE_DEMO_

#include "DemoBase.h"
#include "Geometry3D.h"
#include "AccumulatedImpulse.h"

class AccumulatedImpulseDemo : public DemoBase {
protected:
	AIPhysicsSystem physicsSystem;
	std::vector<RigidbodyVolume> bodies;
	RigidbodyVolume groundBox;

	bool size_imgui_window;
protected:
	void ResetDemo();
public:
	void Initialize(int width, int height);
	void Render();
	void Update(float dt);
	void ImGUI();
};


#endif
