#include "include.h"

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

typedef struct
{
	matrix3 orientation;
	matrix3	world_tensor;
	vec3	velocity;
	vec3	angular_velocity;
	vec3	position;
} cfg_t;

class RigidBody
{
public:
	RigidBody();
	virtual void integrate(float time);
	bool collision_detect(vec3 &v);
	bool collision_detect();
	bool collision_detect(Plane &p);
	bool collision_detect(RigidBody &rigid);
	void trace(RigidBody &body, int index, Plane &plane);
	void impulse(Plane &plane, vec3 &vertex);
	void impulse(RigidBody &rigid, vec3 &point);
	void impulse(RigidBody &rigid, vec3 &point, Plane &plane);
	void save_config(cfg_t &config);
	void load_config(cfg_t &config);
	float *get_matrix(float *matrix);


	float			restitution;
	bool			sleep;
	bool			gravity;
	//Physical
	float			mass;
	matrix3			inverse_tensor;
	matrix3			world_tensor;
	vec3			velocity;
	vec3			angular_velocity;
	vec3			net_force;
	vec3			net_torque;
	vec3			position;
	matrix3			orientation;
	vec3			old_position;	//we restore old positions to prevent "shaking"
	matrix3			old_orientation;
	vec3			aabb[8];
	vec3			center;
};

#endif
