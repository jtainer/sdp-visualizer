#include <stdlib.h>
#include <raylib.h>
#include <cmath>
#include <raymath.h>
#include "serialib.h"
#include <iostream>
#include <fstream>
#include <string>

struct Joint {
	Vector3 globalpos;	// Position in global coordinate system, not modified manually, only used for drawing
	Vector3 localpos;	// Position relative to parent joint position, typically this remains constant during runtime
	Vector3 axis;		// Orientation relative to parent orientation
	float angle;		// Rotation angle
	Matrix transform;
	
	struct Joint* child;	// Array of child joints
	int numchild;		// Size of array
};

struct HandPose {
	Quaternion finger[5];
	Quaternion palm;
};

static HandPose dict[26];

float QuaternionDiffLenSqr(Quaternion q0, Quaternion q1) {
	Quaternion d = QuaternionSubtract(q0, q1);
	return d.w*d.w + d.x*d.x + d.y*d.y + d.z*d.z;
}

float HandPoseDist(HandPose& p0, HandPose& p1) {
	float sum = 0.f;
	for (int i = 0; i < 5; i++) {
		sum += QuaternionDiffLenSqr(p0.finger[i], p1.finger[i]);
	}
//	sum += QuaternionDiffLenSqr(p0.palm, p1.palm);
	return sum;
}

char PosePrediction(HandPose& p) {
	float minDist = INFINITY;
	int minIdx = 0;
	for (int i = 0; i < 26; i++) {
		float dist = HandPoseDist(p, dict[i]);
		if (dist < minDist) {
			minDist = dist;
			minIdx = i;
		}
	}
	return 'A' + (char) minIdx;
}

Joint LoadJoint(Vector3 offset, Vector3 axis) {
	Joint joint;
	joint.localpos = offset;
	joint.axis = axis;
	joint.globalpos = Vector3Zero();
	joint.angle = 0.f;
	joint.numchild = 0;
	joint.child = NULL;
	return joint;
}

void AttachChildJoints(Joint* base, int numchild) {
	base->numchild = numchild;
	base->child = (Joint*)malloc(sizeof(Joint) * numchild);

	// Initialize child joints to remain at the same position as the parent joint
	for (int i = 0; i < numchild; i++) {
		base->child[i] = LoadJoint(Vector3Zero(), Vector3Zero());
	}
	return;
}

void UnloadSkeleton(Joint* base) {
	if (base != NULL) {
		for (int i = 0; i < base->numchild; i++) {
			UnloadSkeleton(&base->child[i]);
		}
		free(base->child);
	}
	return;
}

void UpdateSkeleton(Joint* base, Matrix matglobal) {
	Matrix matlocal;
	Matrix mattranslate;
	Matrix matrotate;
	if (base != NULL) {
		mattranslate = MatrixTranslate(base->localpos.x, base->localpos.y, base->localpos.z);	// Translation relative to parent
		matrotate = MatrixRotate(base->axis, base->angle);	// Rotation relative to parent
		matlocal = MatrixMultiply(mattranslate, matrotate);	// Translation and rotation relative to parent
		matglobal = MatrixMultiply(matlocal, matglobal);	// Translation and rotation in global coord space
		base->globalpos = Vector3Transform(Vector3Zero(), matglobal);
		
		base->transform = matglobal;

		for (int i = 0; i < base->numchild; i++) {
			UpdateSkeleton(&base->child[i], matglobal);
		}
	}
	return;
}

void DrawSkeleton(Joint* base, Color color) {
	if (base != NULL) {
		Vector3 startpos = base->globalpos, endpos = base->globalpos;
		DrawSphere(startpos, 0.025f, color);
		for (int i = 0; i < base->numchild; i++) {
			endpos = base->child[i].globalpos;
			DrawLine3D(startpos, endpos, color);
			DrawSkeleton(&base->child[i], color);
		}
	}
	return;
}

void DrawSkeletonModel(Joint* base, Model stick, Model ball, Vector3 orientation) {
	if (base != NULL) {
		Vector3 startpos = base->globalpos, endpos = base->globalpos;
		DrawModel(ball, startpos, 1.f, LIGHTGRAY);
		for (int i = 0; i < base->numchild; i++) {
			endpos = base->child[i].globalpos;
			Vector3 pointing = Vector3Subtract(endpos, startpos);
			Vector3 position = Vector3Add(startpos, Vector3Scale(pointing, 0.5));
			Vector3 axis = Vector3CrossProduct(orientation, pointing);
			axis = Vector3Normalize(axis);
			float angle = Vector3Angle(orientation, pointing) * 180 / M_PI;
			float length = Vector3Length(pointing);
			Vector3 scale = { 1, length, 1 };		// Assumes +Y orientation for now

			DrawModelEx(stick, startpos, axis, angle, scale, LIGHTGRAY);

			DrawSkeletonModel(&base->child[i], stick, ball, orientation);
		}
	}
	return;
}


float Sigmoid(float t) {
	return 1.f / (1 + expf(-t));
}

int main(int argc, char** argv) {
	if (argc < 2) {
		printf("No device specified\n");
		return 1;
	}

	serialib serial;

	if (serial.openDevice(argv[1], 1152000) != 1) {
		printf("Failed to open device\n");
		return 1;
	}

	std::ifstream dictFile;
	dictFile.open("dictionary.csv");
	for (int i = 0; i < 26; i++) {
		std::string line;
		getline(dictFile, line);
		sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &dict[i].finger[4].w, &dict[i].finger[4].x, &dict[i].finger[4].y, &dict[i].finger[4].z, &dict[i].finger[3].w, &dict[i].finger[3].x, &dict[i].finger[3].y, &dict[i].finger[3].z, &dict[i].finger[2].w, &dict[i].finger[2].x, &dict[i].finger[2].y, &dict[i].finger[2].z, &dict[i].finger[1].w, &dict[i].finger[1].x, &dict[i].finger[1].y, &dict[i].finger[1].z, &dict[i].finger[0].w, &dict[i].finger[0].x, &dict[i].finger[0].y, &dict[i].finger[0].z, &dict[i].palm.w, &dict[i].palm.x, &dict[i].palm.y, &dict[i].palm.z);
		dict[i].finger[0] = QuaternionNormalize(dict[i].finger[0]);
		dict[i].finger[1] = QuaternionNormalize(dict[i].finger[1]);
		dict[i].finger[2] = QuaternionNormalize(dict[i].finger[2]);
		dict[i].finger[3] = QuaternionNormalize(dict[i].finger[3]);
		dict[i].finger[4] = QuaternionNormalize(dict[i].finger[4]);
		//dict[i].palm = QuaternionNormalize(dict[i].palm);
	}
	dictFile.close();
/*
	std::ofstream headerFile;
	headerFile.open("dictionary.h");
	headerFile << "static HandPose dict[26] = { \n";
	for (int i = 0; i < 26; i++) {
		headerFile << "{ ";
		headerFile << "{";
		for (int j = 0; j < 5; j++) {
			headerFile << "{" << dict[i].finger[j].w << "," << dict[i].finger[j].x << "," << dict[i].finger[j].y << "," << dict[i].finger[j].z << "}";
			if (j < 4) headerFile << ",";
		}
		headerFile << "},";
		headerFile << "{" << dict[i].palm.w << "," << dict[i].palm.x << "," << dict[i].palm.y << "," << dict[i].palm.z << "}";
		headerFile << " },\n";
	}
	headerFile << "};\n";
	headerFile.close();
	return 0;
*/
	Joint root = LoadJoint(Vector3Zero(), (Vector3) { 0, 1, 0 });
/*	AttachChildJoints(&root, 1);
	root.child[0].localpos = (Vector3) { 0, 1, 0 };
	root.child[0].axis = (Vector3) { 0, 1, 0 };
	AttachChildJoints(root.child, 1);
	root.child[0].child[0].localpos = (Vector3) { 0.5, 0, 0 };
	UpdateSkeleton(&root, MatrixIdentity());
*/
	AttachChildJoints(&root, 5);
/*
	root.child[0].localpos = (Vector3) { 0.1, 0, -0.2 };
	root.child[0].axis = (Vector3) { 0, 1, 0 };
	root.child[0].angle = 0;

	root.child[1].localpos = (Vector3) { 0.1, 0, -0.1 };
	root.child[1].axis = (Vector3) { 0, 1, 0 };
	root.child[1].angle = 0;

	root.child[2].localpos = (Vector3) { 0.1, 0, 0.0 };
	root.child[2].axis = (Vector3) { 0, 1, 0 };
	root.child[2].angle = 0;

	root.child[3].localpos = (Vector3) { 0.1, 0, 0.1 };
	root.child[3].axis = (Vector3) { 0, 1, 0 };
	root.child[3].angle = 0;

	root.child[4].localpos = (Vector3) { 0.1, 0, 0.1 };
	root.child[4].axis = (Vector3) { 0, 1, 0 };
	root.child[4].angle = -0.5;
*/

	root.child[0].localpos = (Vector3) { 0.1, 0, -0.2 };
	root.child[0].axis = (Vector3) { 0, 1, 0 };
	root.child[0].angle = 0;

	root.child[1].localpos = (Vector3) { 0.1, 0, -0.1 };
	root.child[1].axis = (Vector3) { 0, 1, 0 };
	root.child[1].angle = 0;

	root.child[2].localpos = (Vector3) { 0.1, 0, 0.0 };
	root.child[2].axis = (Vector3) { 0, 1, 0 };
	root.child[2].angle = 0;

	root.child[3].localpos = (Vector3) { 0.1, 0, 0.1 };
	root.child[3].axis = (Vector3) { 0, 1, 0 };
	root.child[3].angle = 0;

	root.child[4].localpos = (Vector3) { 0.1, 0, 0.1 };
	root.child[4].axis = (Vector3) { 0, 1, 0 };
	root.child[4].angle = -0.5;



	Vector3 metacarpalLocalPos = { 0.2, 0, 0 };
	Vector3 metacarpalAxis = { 0, 0, 1 };

	float length[5] = { 0.15, 0.18, 0.20, 0.18, 0.12 };

	for (int i = 0; i < 5; i++) {
		metacarpalLocalPos.x = length[i];

		AttachChildJoints(&root.child[i], 1);
		root.child[i].child[0].localpos = metacarpalLocalPos;
		root.child[i].child[0].axis = metacarpalAxis;
		root.child[i].child[0].angle = 0;

		AttachChildJoints(&root.child[i].child[0], 1);
		root.child[i].child[0].child[0].localpos = metacarpalLocalPos;
		root.child[i].child[0].child[0].axis = metacarpalAxis;
		root.child[i].child[0].child[0].angle = 0;

		AttachChildJoints(&root.child[i].child[0].child[0], 1);
		root.child[i].child[0].child[0].child[0].localpos = metacarpalLocalPos;
		root.child[i].child[0].child[0].child[0].axis = metacarpalAxis;
		root.child[i].child[0].child[0].child[0].angle = 0;

		AttachChildJoints(&root.child[i].child[0].child[0].child[0], 1);
		root.child[i].child[0].child[0].child[0].child[0].localpos = metacarpalLocalPos;
		root.child[i].child[0].child[0].child[0].child[0].axis = metacarpalAxis;
		root.child[i].child[0].child[0].child[0].child[0].angle = 0;

	}

	root.child[4].child[0].child[0].child[0].child[0].localpos = Vector3Zero();

//	Matrix rotate = MatrixRotate((Vector3) { 1, 0, 0 }, M_PI / 2);
//	Matrix translate = MatrixTranslate(0, 1, 0);
//	Matrix transform = MatrixMultiply(rotate, translate);

//	UpdateSkeleton(&root, transform);

	SetConfigFlags(FLAG_WINDOW_ALWAYS_RUN | FLAG_MSAA_4X_HINT);
	InitWindow(1920, 1080, "bones");
	SetTargetFPS(60);

	Camera camera = { 0 };
	camera.projection = CAMERA_PERSPECTIVE;
	camera.fovy = 90;
	camera.position = (Vector3) { 0.3, 0.3, -1.3 };
	camera.target = (Vector3) { 0.3, 0.2, 0 };
	camera.up = (Vector3) { 0, 1, 0 };

	Shader shader = LoadShader("shaders/lighting.vs", "shaders/lighting.fs");
	Model stick = LoadModelFromMesh(GenMeshCylinder(0.025, 1.f, 10));
	Model ball = LoadModelFromMesh(GenMeshSphere(0.025, 10, 10));
	stick.materials[0].shader = shader;
	ball.materials[0].shader = shader;
	Vector3 orientation = { 0, 1, 0 };

	float time[5] = { 0 };
	float angle[5] = { 0 };
	Vector3 axis[5] = { 0 };
	int key[5] = { KEY_ONE, KEY_TWO, KEY_THREE, KEY_FOUR, KEY_FIVE };

	while (!WindowShouldClose()) {
//		UpdateCamera(&camera, CAMERA_FIRST_PERSON);

		static Quaternion bend[5] = { 0 };
		static Quaternion palm_rot = { 0 };
		static Vector3 palmAxis = { 0 };
		static float palmAngle = 0;
		static char buf[1024] = { 0 };
		if (serial.readString(buf, '\n', 1024, 10) > 0) {
			int n = sscanf(buf, "<%f,%f,%f,%f>,<%f,%f,%f,%f>,<%f,%f,%f,%f>,<%f,%f,%f,%f>,<%f,%f,%f,%f>,<%f,%f,%f,%f>",
					&bend[4].w, &bend[4].x, &bend[4].y, &bend[4].z, &bend[3].w, &bend[3].x, &bend[3].y, &bend[3].z, &bend[2].w, &bend[2].x, &bend[2].y, &bend[2].z, &bend[1].w, &bend[1].x, &bend[1].y, &bend[1].z, &bend[0].w, &bend[0].x, &bend[0].y, &bend[0].z, &palm_rot.w, &palm_rot.x, &palm_rot.y, &palm_rot.z);
			for (int i = 0; i < 5; i++) {
//				angle[i] *= M_PI / 180.f;
//				angle[i] /= 8.f;
//				angle[i] = Vector3Length(bend[i]) / 8.f;
				QuaternionToAxisAngle(bend[i], &axis[i], &angle[i]);
				angle[i] /= 8.f;
			}
		}

		QuaternionToAxisAngle(palm_rot, &palmAxis, &palmAngle);
		root.axis.x = -palmAxis.x;
		root.axis.y = -palmAxis.z;
		root.axis.z = -palmAxis.y;
		root.angle = palmAngle;

		Vector3 embedding = Vector3RotateByQuaternion((Vector3) { 0.f, 1.f, 0.f}, bend[0]);
		//printf("%f\t%f\t%f\n", embedding.x, embedding.y, embedding.z);

		for (int i = 0; i < 5; i++) {
			Vector3 tmpAxis = { -axis[i].x, -axis[i].z, -axis[i].y };
			root.child[i].child[0].axis = Vector3RotateByAxisAngle(metacarpalAxis, tmpAxis, angle[i] * 8.f);
			//root.child[i].child[0].child[0].axis.x = -axis[i].x;
			//root.child[i].child[0].child[0].axis.y = -axis[i].z;
			//root.child[i].child[0].child[0].axis.z = -axis[i].y;
			//root.child[i].child[0].child[0].angle = 8 * angle[i];
			root.child[i].child[0].child[0].angle = 2 * angle[i];
			root.child[i].child[0].child[0].child[0].angle = 3 * angle[i];
			root.child[i].child[0].child[0].child[0].child[0].angle = 4 * angle[i];
		}

		//QuaternionToAxisAngle(palm_rot, &root.axis, &root.angle);

		Vector3 rotateVec = { M_PI, 0.f, 0.f };
		Matrix rotate = MatrixRotate(Vector3Normalize(rotateVec), Vector3Length(rotateVec));
		//rotateVec = (Vector3) { 0.f, -M_PI/2.f, 0.f };
		//rotate = MatrixMultiply(rotate, MatrixRotate(Vector3Normalize(rotateVec), Vector3Length(rotateVec)));
		Matrix translate = MatrixTranslate(0, 0.5, 0);
		Matrix transform = MatrixMultiply(rotate, translate);

		
//		Matrix transform = MatrixIdentity();
		UpdateSkeleton(&root, transform);

		HandPose pose = { bend[0], bend[1], bend[2], bend[3], bend[4], palm_rot };
		char prediction = PosePrediction(pose);

		BeginDrawing();
		ClearBackground(BLACK);
		BeginMode3D(camera);
		DrawGrid(10, 1);
//		DrawSkeleton(&root, WHITE);
		DrawSkeletonModel(&root, stick, ball, orientation);
		//DrawCylinder((Vector3) { 0.25, 0.25, -0.15 }, 0.05, 0.05, 0.5, 10, GREEN);
		EndMode3D();
		DrawFPS(10, 10);
		DrawText(TextFormat("%c", prediction), 50, 50, 80, GREEN);
		EndDrawing();
	}


	UnloadModel(stick);
	UnloadModel(ball);
	UnloadShader(shader);
	UnloadSkeleton(&root);
	CloseWindow();

	serial.closeDevice();
}
