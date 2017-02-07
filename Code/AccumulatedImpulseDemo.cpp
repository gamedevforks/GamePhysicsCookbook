#include "AccumulatedImpulseDemo.h"
#include "FixedFunctionPrimitives.h"
#include "glad/glad.h"
#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <iostream>

void AccumulatedImpulseDemo::Initialize(int width, int height) {
	DemoBase::Initialize(width, height);

	size_imgui_window = true;

	glPointSize(5.0f);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float val[] = { 0.5f, 1.0f, -1.5f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	camera.SetTarget(vec3(3.75622f, 2.98255f, 0.0f));
	camera.SetZoom(12.0f);
	camera.SetRotation(vec2(-67.9312f, 19.8f));

	ResetDemo();
}

void AccumulatedImpulseDemo::ResetDemo() {
	physicsSystem.ClearRigidbodys();

	bodies.clear();
	bodies.resize(2);

	bodies[0].type = RIGIDBODY_TYPE_BOX;
	bodies[0].position = vec3(0.5f, 6, 0);
	bodies[0].orientation = vec3(0.0f, 0.0f, 0.4f);
	bodies[0].name = "bodies[0]";

	bodies[1].type = RIGIDBODY_TYPE_BOX;
	bodies[1].position = vec3(0, 1, 0);
	bodies[1].mass = 5.0f;
	bodies[1].name = "bodies[1]";

	groundBox = RigidbodyVolume(RIGIDBODY_TYPE_BOX);
	groundBox.position = vec3(0, -0.5f, 0) * vec3(1, 0.5f, 1);
	groundBox.box.size = vec3(50, 1, 50) * 0.25f;
	groundBox.mass = 0.0f;
	groundBox.SynchCollisionVolumes();
	groundBox.name = "groundBox";

	for (int i = 0; i < bodies.size(); ++i) {
		bodies[i].SynchCollisionVolumes();
		physicsSystem.AddRigidbody(&bodies[i]);
	}
	physicsSystem.AddRigidbody(&groundBox);
}

void AccumulatedImpulseDemo::ImGUI() {
	DemoBase::ImGUI();

	if (size_imgui_window) {
		size_imgui_window = false;
		ImGui::SetNextWindowPos(ImVec2(400, 10));
		ImGui::SetNextWindowSize(ImVec2(370, 75));
	}

	ImGui::Begin("Accumulated Impulse Demo", 0, ImGuiWindowFlags_NoResize);

	ImGui::PushItemWidth(55);
	ImGui::SliderFloat("Tolerance", &physicsSystem.ContactTolerance, 0.001f, 0.1f);
	ImGui::SameLine();
	ImGui::PushItemWidth(55);
	ImGui::SliderFloat("Penetration", &physicsSystem.AllowedPenetration, 0.01f, 0.1f);
	ImGui::SameLine();
	ImGui::PushItemWidth(55);
	ImGui::SliderFloat("Bias", &physicsSystem.BiasFactor, 0.1f, 1.0f);

	if (ImGui::Button("Reset")) {
		ResetDemo();
	}
	ImGui::SameLine();
	ImGui::Checkbox("Debug Render", &physicsSystem.DebugRender);

	ImGui::End();
}

void AccumulatedImpulseDemo::Render() {
	DemoBase::Render();

	float val[] = { 0.0f, 1.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	physicsSystem.Render();
}

void AccumulatedImpulseDemo::Update(float dt) {
	DemoBase::Update(dt);

	physicsSystem.Update(dt);
}