#ifndef INCLUDE_H
#define INCLUDE_H

#ifdef _WIN32
	#define _USE_MATH_DEFINES
	#define WIN32_EXTRA_LEAN
	#define _CRT_SECURE_NO_DEPRECATE
	#define NOMINMAX
	#include <windows.h>
	#include <GL/gl.h>
	#include <GL/glu.h>
#endif

#ifdef __linux__
	#include <GL/gl.h>
	#include <GL/glu.h>
	#include <GL/glx.h>
	#include <X11/Xlib.h>
	#include <X11/Xatom.h>
	#include <X11/keysym.h>
	#include <unistd.h>
	#include <sys/select.h>
	#include <sys/types.h>
	#include <sys/time.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <sys/time.h>

	#define closesocket close

	typedef	int SOCKET;
	#define SOCKET_ERROR	-1
	#define INVALID_SOCKET	-1
#endif

#ifdef __MACH__
	#include <Carbon/Carbon.h>
	#include <agl/agl.h>
	#include <opengl/gl.h>
	#include <opengl/glu.h>
#endif


//std
#include <cstdio>
#include <cstring>
#include <vector>
#include <cmath>

using namespace std;

#include "vector.h"
#include "matrix.h"
#include "plane.h"
#include "rigidbody.h"
#include "engine.h"

float abs32(float val);

#define MY_PI 3.14159265359f
#define MY_HALF_PI 1.5707963268f

#endif
