#include "rigidbody.h"

RigidBody::RigidBody()
{
	sleep = false;
	gravity = true;
	mass = 10.0f;
	restitution = 1.0f; // boxes should never rest
	float height = 10.0f;
	float width = 10.0f;
	float depth = 10.0f;
	position = vec3(0.0f, 0.0f, 0.0f);
	velocity = vec3(0.0f, 0.0f, 0.0f);

	orientation.m[0] = 1.0f;
	orientation.m[1] = 0.0f;
	orientation.m[2] = 0.0f;

	orientation.m[3] = 0.0f;
	orientation.m[4] = 1.0f;
	orientation.m[5] = 0.0f;

	orientation.m[6] = 0.0f;
	orientation.m[7] = 0.0f;
	orientation.m[8] = 1.0f;

	world_tensor.m[0] = 0.0f;
	world_tensor.m[1] = 0.0f;
	world_tensor.m[2] = 0.0f;
	world_tensor.m[3] = 0.0f;
	world_tensor.m[4] = 0.0f;
	world_tensor.m[5] = 0.0f;
	world_tensor.m[6] = 0.0f;
	world_tensor.m[7] = 0.0f;
	world_tensor.m[8] = 0.0f;

	inverse_tensor.m[0] = 36.0f / (mass *  (height * height + depth * depth));
	inverse_tensor.m[1] = 0;
	inverse_tensor.m[2] = 0;

	inverse_tensor.m[3] = 0;
	inverse_tensor.m[4] = 36.0f / (mass *  (width * width + depth * depth));
	inverse_tensor.m[5] = 0;
	
	inverse_tensor.m[6] = 0;
	inverse_tensor.m[7] = 0;
	inverse_tensor.m[8] = 36.0f / (mass *  (width * width + height * height));
}

/*
	Integrate physical quantaties over time by a fixed time step

	Writes to angular velocity, velocity, position,	and orientation
*/
void RigidBody::integrate(float time)
{
	matrix3 rotation;
	vec3 acceleration, angular_acceleration;

	if (sleep)
		return;

	//translational
	acceleration = net_force / mass;
	velocity = velocity + acceleration * time;
	old_position = position;
	position = position + velocity * time;

	//rotational
	angular_acceleration = world_tensor * net_torque;
	angular_velocity = angular_velocity + angular_acceleration * time;
	rotation.star(angular_velocity);

	old_orientation = orientation;
	orientation = orientation + orientation * rotation * time;
	orientation.normalize();
	world_tensor = orientation * inverse_tensor * orientation.transpose();
}


float *RigidBody::get_matrix(float *matrix)
{
	matrix[0] = orientation.m[0];
	matrix[1] = orientation.m[1];
	matrix[2] = orientation.m[2];
	matrix[3] = 0.0f;

	matrix[4] = orientation.m[3];
	matrix[5] = orientation.m[4];
	matrix[6] = orientation.m[5];
	matrix[7] = 0.0f;

	matrix[8]  = orientation.m[6];
	matrix[9]  = orientation.m[7];
	matrix[10] = orientation.m[8];
	matrix[11] = 0.0f;

	matrix[12] = position.x;
	matrix[13] = position.y;
	matrix[14] = position.z;
	matrix[15] = 1.0f;
	return matrix;
}


/*
	Detects a collision with a plane and applies physical impulse response
*/
bool RigidBody::collision_detect(Plane &p)
{
	for( int i = 0;	i < 8; i++)
	{
		// make center origin
		vec3 point = center + aabb[i];

		//rotate around origin
		point = orientation * point;

		// rotate center around about true origin
		vec3 offset = orientation * center;

		// translate back to local coordinate origin
		point = point - offset;

		// translate to world coordinates
		point = point + position;

		float d = point * p.normal + p.d;

		if ( d < -0.25f )
		{
			// Simulated too far
			return true;
		}
		else if ( d < 0.0f )
		{
			// Colliding

			// convert rotated point back to local coordinates
			point = point - position;

			// convert point back to radius from center
			point = point + offset;

			// apply impulse to plane and radius vector
			impulse(p, point);
			position = old_position;
			orientation = old_orientation;
		}
	}
	return false;
}

/*
	Applys collision impulse to a vertex
	radius must be in units of meters from CM
*/
void RigidBody::impulse(Plane &plane, vec3 &radius)
{
	float	impulse_numerator;
	float	impulse_denominator;
	vec3	impulse_momentum;

	vec3	vertex_velocity = velocity + vec3::crossproduct(angular_velocity, radius);

	// coefficient of resistution * -relative velocity
	impulse_numerator = -(1.0f + restitution) * (vertex_velocity * plane.normal);

	// 1/mass + N dot [((1/I)(radius cross normal)) cross radius] -- units of momentum p = mv
	impulse_denominator = (1.0f / mass) + plane.normal *
		vec3::crossproduct(world_tensor * vec3::crossproduct(radius, plane.normal), radius);
    
	impulse_momentum = plane.normal * (impulse_numerator/impulse_denominator);

	// apply impulse to primary quantities
	velocity += impulse_momentum / mass;
	angular_velocity = world_tensor * vec3::crossproduct(radius, -impulse_momentum);
}

void RigidBody::impulse(RigidBody &rigid, vec3 &point)
{
	float	impulse_numerator;
	float	impulse_denominator;
	vec3	impulse_force;
	vec3	local_point = (point - position);

	vec3	relative_velocity = (velocity - rigid.velocity) + vec3::crossproduct(angular_velocity, local_point) - vec3::crossproduct(rigid.angular_velocity, local_point);
	vec3	normal = rigid.position - point;
	normal.normalize();

	// coefficient of resistution * -relative velocity
	impulse_numerator = -(1.0f + restitution) * (relative_velocity * normal);

	// 1/mass + N dot [((1/I)(radius cross normal)) cross radius] -- units of momentum p = mv
	impulse_denominator = (1.0f / mass) + (1.0f / rigid.mass)
		+ normal * vec3::crossproduct(world_tensor * vec3::crossproduct(local_point, normal), local_point)
		+ normal * vec3::crossproduct(rigid.world_tensor * vec3::crossproduct(local_point, normal), local_point);
    
	impulse_force = normal * (impulse_numerator/impulse_denominator);

	// apply impulse to primary quantities
	velocity += impulse_force * (1.0f / mass);
	angular_velocity = world_tensor * vec3::crossproduct(local_point, -impulse_force);
	rigid.velocity -= impulse_force * (1.0f / rigid.mass);
	rigid.angular_velocity = rigid.world_tensor * vec3::crossproduct(local_point, impulse_force);
	sleep = false;
	rigid.sleep = false;
	gravity = true;
}

void RigidBody::impulse(RigidBody &rigid, vec3 &point, Plane &plane)
{
	float	impulse_numerator;
	float	impulse_denominator;
	vec3	impulse_force;
	vec3	local_point = (point - position);

	vec3	relative_velocity = (velocity - rigid.velocity) + vec3::crossproduct(angular_velocity, local_point) - vec3::crossproduct(rigid.angular_velocity, local_point);
	vec3	normal = plane.normal;
	normal.normalize();

	// coefficient of resistution * -relative velocity
	impulse_numerator = -(1.0f + restitution) * (relative_velocity * normal);

	// 1/mass + N dot [((1/I)(radius cross normal)) cross radius] -- units of momentum p = mv
	impulse_denominator = (1.0f / mass) + (1.0f / rigid.mass)
		+ normal * vec3::crossproduct(world_tensor * vec3::crossproduct(local_point, normal), local_point)
		+ normal * vec3::crossproduct(rigid.world_tensor * vec3::crossproduct(local_point, normal), local_point);
    
	impulse_force = normal * (impulse_numerator/impulse_denominator);

	// apply impulse to primary quantities
	velocity += impulse_force * (1.0f / mass);
	angular_velocity = world_tensor * vec3::crossproduct(local_point, -impulse_force);
	rigid.velocity -= impulse_force * (1.0f / rigid.mass);
	rigid.angular_velocity = rigid.world_tensor * vec3::crossproduct(local_point, impulse_force);
	sleep = false;
	rigid.sleep = false;
	gravity = true;
}

/*
	Detects a collision between current entity and a point
	We should really ray trace after finding a collision to determine the plane and depth
	This is why boxes stick to each other
*/
bool RigidBody::collision_detect(vec3 &v)
{
	vec3 a, b;

	a = aabb[0] + position + center;
	b = aabb[7] + position + center;

	if ( (v.x > a.x) && (v.x < b.x) )
	{
		if ( (v.y > a.y) && (v.y < b.y) )
		{
			if ( (v.z > a.z) && (v.z < b.z) )
			{
				return true;
			}
		}
	}
	return false;
}

bool RigidBody::collision_detect(RigidBody &body)
{
	Plane plane[6];
	vec3 point;

	// Bounding box planes
	plane[0] = vec4(0.0f, 1.0f, 0.0f, -0.5f); // up
	plane[1] = vec4(0.0f, -1.0f, 0.0f, 0.5f); // down
	plane[2] = vec4(1.0f, 0.0f, 0.0f, -0.5f); // right
	plane[3] = vec4(-1.0f, 0.0f, 0.0f, 0.5f); // left
	plane[4] = vec4(0.0f, 0.0f, 1.0f, -0.5f); // far
	plane[5] = vec4(0.0f, 0.0f, -1.0f, 0.5f); // near


	// Rotate and translate planes to world space
	for(int i = 0; i < 6; i++)
	{
		point = plane[i].normal * 0.5f;						// point on plane
		point = orientation * point + position;				// rotate point
		plane[i].normal = orientation * plane[i].normal;	// rotate normal
		plane[i].normal.normalize();
		plane[i].d = -(plane[i].normal * point);			// recalculate D
	}

	// Check if point of body is inside our bounding box
	for(int i = 0; i < 8; i++)
	{
		const float padding = 0.0125f;
		float depth = 1000.0f;
		int count = 0;
		int closest = 0;

		// oriented body point in world space
		point = body.orientation * body.aabb[i] + body.position;

		for(int j = 0; j < 6; j++)
		{
			float distance = plane[j].normal * point + plane[j].d;
			if ( distance < padding)
			{
				if (depth > abs32(distance))
				{
					depth = abs32(distance);
					closest = j;
				}
				count++;
			}
		}

		// We had a point inside all 6 planes
		if (count == 6)
		{
			// Too deep, divide time step
			if (depth > 0.125f)
				return true;

//			printf("body body impact\n");
			impulse(body, point, plane[closest]);
			position = old_position;
			orientation = old_orientation;
		}
	}
	return false;
}

void RigidBody::save_config(cfg_t &config)
{
	config.orientation = orientation;
	config.world_tensor = world_tensor;
	config.velocity = velocity;
	config.angular_velocity = angular_velocity;
	config.position = position;
}

void RigidBody::load_config(cfg_t &config)
{
	orientation = config.orientation;
	world_tensor = config.world_tensor;
	velocity = config.velocity;
	angular_velocity = config.angular_velocity;
	position = config.position;
}
