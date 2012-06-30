#include "include.h"

#ifndef ENGINE_H
#define ENGINE_H

class Engine
{
public:
	void init(void *param1, void *param2);
	void destroy();
	void quit();

	void render();
	void resize(int width, int height);

	void step();
	void dynamics();

	bool collision_detect(RigidBody &body);
	bool body_collision(RigidBody &body);

private:
	vector<RigidBody *>	entity_list;
	void	*param1;
	void	*param2;

#ifdef WIN32
	HWND	hwnd;
	HDC	hdc;
#endif

#ifdef __linux__
	Display	*display;
	Window	window;
#endif

#ifdef __MACH__
	WindowPtr		window;
	AGLContext		agl_context;
#endif
};

#endif

