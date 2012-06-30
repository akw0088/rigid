#include "include.h"

void Engine::init(void *param1, void *param2)
{
#ifdef WIN32
	hwnd = *((HWND *)param1);
	hdc = *((HDC *)param2);
#endif

#ifdef __linux__
	display = (Display *)param1;
	window = *((Window *)param2);
#endif

#ifdef __MACH__
	window = *((WindowPtr *)param1);
	agl_context = *((AGLContext *)param2);
#endif

	glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
	glClearStencil(0);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CCW);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
}

void draw_box()
{
	//123 243 826 842 678 657 137 175 152 256 347 487
	glBegin(GL_TRIANGLES);
	glVertex3f(-0.5f, -0.5f, -0.5f); //1
	glVertex3f(-0.5f, -0.5f, 0.5f);  //2
	glVertex3f(-0.5f, 0.5f, -0.5f);  //3

	glVertex3f(-0.5f, -0.5f, 0.5f);  //2
	glVertex3f(-0.5f, 0.5f, 0.5f);   //4
	glVertex3f(-0.5f, 0.5f, -0.5f);  //3

	glVertex3f(0.5f, 0.5f, 0.5f);    //8
	glVertex3f(-0.5f, -0.5f, 0.5f);  //2
	glVertex3f(0.5f, -0.5f, 0.5f);   //6

	glVertex3f(0.5f, 0.5f, 0.5f);    //8
	glVertex3f(-0.5f, 0.5f, 0.5f);   //4
	glVertex3f(-0.5f, -0.5f, 0.5f);  //2

	glVertex3f(0.5f, -0.5f, 0.5f);   //6
	glVertex3f(0.5f, 0.5f, -0.5f);   //7
	glVertex3f(0.5f, 0.5f, 0.5f);    //8


	glVertex3f(0.5f, -0.5f, 0.5f);   //6
	glVertex3f(0.5f, -0.5f, -0.5f);  //5
	glVertex3f(0.5f, 0.5f, -0.5f);   //7

	glVertex3f(-0.5f, -0.5f, -0.5f); //1
	glVertex3f(-0.5f, 0.5f, -0.5f);  //3
	glVertex3f(0.5f, 0.5f, -0.5f);   //7

	glVertex3f(-0.5f, -0.5f, -0.5f); //1
	glVertex3f(0.5f, 0.5f, -0.5f);   //7
	glVertex3f(0.5f, -0.5f, -0.5f);  //5

	glVertex3f(-0.5f, -0.5f, -0.5f); //1
	glVertex3f(0.5f, -0.5f, -0.5f);  //5
	glVertex3f(-0.5f, -0.5f, 0.5f);  //2

	glVertex3f(-0.5f, -0.5f, 0.5f);  //2
	glVertex3f(0.5f, -0.5f, -0.5f);  //5
	glVertex3f(0.5f, -0.5f, 0.5f);   //6

	glVertex3f(-0.5f, 0.5f, -0.5f);  //3
	glVertex3f(-0.5f, 0.5f, 0.5f);   //4
	glVertex3f(0.5f, 0.5f, -0.5f);   //7

	glVertex3f(-0.5f, 0.5f, 0.5f);   //4
	glVertex3f(0.5f, 0.5f, 0.5f);    //8
	glVertex3f(0.5f, 0.5f, -0.5f);   //7
	glEnd();
}

void Engine::render()
{
	float matrix[16];

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
	for(int i = 0; i < entity_list.size(); i++)
	{
		if (i % 2 == 0)
			glColor3f(1.0f, 0.0f, 0.0f);
		else
			glColor3f(0.0f, 1.0f, 0.0f);

		if (entity_list[i]->sleep)
			glColor3f(0.0f, 0.0f, 1.0f);

		entity_list[i]->get_matrix(matrix);
		glLoadMatrixf(matrix);
		draw_box();

	}
#ifdef _WIN32
	SwapBuffers(hdc);
#endif
#ifdef __linux__
	glXSwapBuffers(display, window);
#endif
#ifdef __MACH__
	aglSwapBuffers(agl_context);
#endif

}

void Engine::dynamics()
{
	cfg_t	config;
	for(int i = 0; i < entity_list.size(); i++)
	{
		if (entity_list[i]->sleep == true )
			continue;

		RigidBody *body = entity_list[i];
		float delta_time = 0.016f;
		float target_time = delta_time;
		float current_time = 0.0f;
		int divisions = 0;

		if (body->gravity)
			body->net_force = vec3(0.0f, -9.8f * body->mass, 0.0f);

		while (current_time < delta_time)
		{
			body->save_config(config);
			body->integrate(target_time - current_time);
			if ( collision_detect(*body) )
			{
				body->load_config(config);
				target_time = (current_time + target_time) / 2.0f;
				divisions++;

				if (divisions > 10)
				{
//					printf("Entity %d is sleeping\n", i);
					body->sleep = true;
					break;
				}
				continue;
			}
			current_time = target_time;
			target_time = delta_time;
		}
		body->net_force = vec3(0.0f, 0.0f, 0.0f);
	}
}


/*
	Handles all collision detection
	return true if simulated too far.
	return false if collision handled.
*/
bool Engine::collision_detect(RigidBody &body)
{
	Plane floor(vec3(0.0f, 1.0f, 0.0f), 3.0f);
	Plane roof(vec3(0.0f, -1.0f, 0.0f), 10.0f);
	Plane left(vec3(1.0f, 0.0f, 0.0f), 10.0f);
	Plane right(vec3(-1.0f, 0.0f, 0.0f), 10.0f);
	Plane front(vec3(0.0f, 0.0f, 1.0f), 25.0f);
	Plane back(vec3(0.0f, 0.0f, -1.0f), 0.0f);

	if (body_collision(body))
		return true;
	else if (body.collision_detect(floor))
		return true;
	else if (body.collision_detect(roof))
		return true;
	else if (body.collision_detect(left))
		return true;
	else if (body.collision_detect(right))
		return true;
	else if (body.collision_detect(front))
		return true;
	else if (body.collision_detect(back))
		return true;
	else
		return false;
}

//O(N^2)
bool Engine::body_collision(RigidBody &body)
{
	for(int i = 0; i < entity_list.size(); i++)
	{
		if (entity_list[i] == &body)
			continue;

		if ( body.collision_detect(*entity_list[i]) )
			return true;
	}
	return false;
}

void Engine::step()
{
	static int time = 0;
	
	time++;

	if (time % 120 == 0)
	{
		RigidBody	*box = new RigidBody;
		box->position = vec3(0.0f, 3.0f, -10.0f);
		box->aabb[0] = vec3(-0.5f, -0.5f, -0.5f);
		box->aabb[1] = vec3(-0.5f, -0.5f, 0.5f);
		box->aabb[2] = vec3(-0.5f, 0.5f, -0.5f);
		box->aabb[3] = vec3(-0.5f, 0.5f, 0.5f);
		box->aabb[4] = vec3(0.5f, -0.5f, -0.5f);
		box->aabb[5] = vec3(0.5f, -0.5f, 0.5f);
		box->aabb[6] = vec3(0.5f, 0.5f, -0.5f);
		box->aabb[7] = vec3(0.5f, 0.5f, 0.5f);
		box->angular_velocity = vec3(1.0f, 2.0f, 3.0f);
		box->velocity = vec3(-3.0f, 0.0f, -3.0f);
		entity_list.push_back(box);
	}

	dynamics();
}


void Engine::resize(int width, int height)
{
//	gfx.resize(width, height);
	glViewport(0, 0, width, height);
//	projection.perspective( 45.0, (float) width / height, 1.0f, 2001.0f, true );
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (height != 0)
		gluPerspective( 45.0, (float) width / height, 1.0f, 2001.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Engine::destroy()
{
	printf("Shutting down.\n");
//	gfx.destroy();
//	audio.destroy();
}

void Engine::quit()
{
#ifdef _WINDOWS_
	HWND hwnd = *((HWND *)param1);
	PostMessage(hwnd, WM_CLOSE, 0, 0);
#endif
}
