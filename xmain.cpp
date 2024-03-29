#include "include.h"

int EventProc(Display *display, Window window, GLXContext context);

int main(int argc, char *argv[])
{
	Display			*display;
	XVisualInfo		*vinfo;
	XSetWindowAttributes	winattrs;
	Window			window;
	GLXContext		context;
	int visual[] = {	GLX_RGBA, GLX_DOUBLEBUFFER,
					GLX_RED_SIZE, 8,
					GLX_GREEN_SIZE, 8,
					GLX_BLUE_SIZE, 8,
					GLX_DEPTH_SIZE, 16,
					None	};

	display	= XOpenDisplay(NULL);
	if (!display)
	{
		printf("Cannot connect to X server %s\n", XDisplayName(NULL));
		return 0;
	}
	vinfo = glXChooseVisual(display, DefaultScreen(display), visual);
	if (!vinfo)
	{
		printf("Unable to create suitable opengl visual.\n");
		XCloseDisplay(display);
		return 0;
	}
	context	= glXCreateContext(display, vinfo, 0, True);
	if (!context)
	{
		printf("Unable to create opengl context.\n");
		XCloseDisplay(display);
		return 0;
	}

//	memset(winattrs, 0, sizeof(XSetWindowAttributes));
	winattrs.colormap	= XCreateColormap(display, DefaultRootWindow(display), vinfo->visual, AllocNone);
	winattrs.event_mask	= ExposureMask | StructureNotifyMask | KeyPressMask | KeyReleaseMask | PointerMotionMask | ButtonPressMask | ButtonReleaseMask;
	winattrs.border_pixel	= 0;

	window = XCreateWindow(display, DefaultRootWindow(display), 0, 0, 640, 480, 0, vinfo->depth, InputOutput, vinfo->visual,
		CWBorderPixel | CWColormap | CWEventMask, &winattrs);

	XSetStandardProperties(display, window, "altEngine", "opengl", None, argv, argc, NULL );

	XMapWindow(display, window);
	glXMakeCurrent(display, window, context);
	printf("GL Version: %s\n", glGetString(GL_VERSION));


	while (True)
	{
		if ( XPending(display) )
		{
			if ( EventProc(display, window, context) )
				break;
		}
		else
		{
			EventProc(NULL, window, context);
		}
	}
	printf("Closing...\n");
	glXDestroyContext(display, context);
	XCloseDisplay(display);
	return 0;
}

int EventProc(Display *display, Window window, GLXContext context)
{
	XEvent			event;
	static Engine		altEngine;
	static bool		init = false;
	static int		xcenter, ycenter;
	static Atom		wmDeleteMessage;

	if (init && display == NULL)
	{
		altEngine.step();
		altEngine.render();
		return 0;
	}

	if (display == NULL)
		return 0;

	XNextEvent(display, &event);
	switch(event.type)
	{
	case ClientMessage:
		if (event.xclient.data.l[0] == wmDeleteMessage)
			return 1;
	case CreateNotify:
//		printf("CreateNotify\n");
		break;
	case ConfigureNotify:
//		printf("ConfigureNotify\n");
		xcenter = event.xconfigure.width / 2;
		ycenter = event.xconfigure.height / 2;
		altEngine.resize(event.xconfigure.width, event.xconfigure.height);
		break;
	case MapNotify:
//		printf("MapNotify\n");
		break;
	case ButtonPress:
//		printf("ButtonPress\n");
		break;
	case ButtonRelease:
//		printf("ButtonRelease\n");
		break;
	case MotionNotify:
//		printf("MotionNotify\n");

		if ((event.xmotion.x == xcenter) && (event.xmotion.y == ycenter))
			break;
/*
		if ( altEngine.mousepos(event.xmotion.x, event.xmotion.y, event.xmotion.x - xcenter, event.xmotion.y - ycenter) )
		{
			XWarpPointer(display, None, window, 0, 0, 0, 0, xcenter, ycenter);
		}
*/
		break;
	case KeyPress:
	case KeyRelease:
//		printf("KeyPress\n");
		{
			bool pressed = (event.type == KeyPress) ? true : false;
			KeySym keysym = XKeycodeToKeysym(display, event.xkey.keycode, 0);

	        switch (keysym)
			{
			case XK_Return:
//				altEngine.keystroke("enter", pressed);
				break;
			case XK_Shift_L:
			case XK_Shift_R:
//				altEngine.keystroke("shift", pressed);
				break;
			case XK_Control_L:
			case XK_Control_R:
//				altEngine.keystroke("control", pressed);
				break;
			case XK_Escape:
//				altEngine.keystroke("escape", pressed);
				break;
			case XK_Up:
//				altEngine.keystroke("up", pressed);
				break;
			case XK_Left:
//				altEngine.keystroke("left", pressed);
				break;
			case XK_Down:
//				altEngine.keystroke("down", pressed);
				break;
			case XK_Right:
//				altEngine.keystroke("right", pressed);
				break;
			}
		}
		break;
	case UnmapNotify:
//		printf("UnmapNotify\n");
		break;
	case Expose:
//		printf("Expose %d\n", event.xexpose.count);

		if (!init)
		{
			printf("Initializing\n");

			wmDeleteMessage = XInternAtom(display, "WM_DELETE_WINDOW", False);
			XSetWMProtocols(display, window, &wmDeleteMessage, 1);
			altEngine.init((void *)display, (void *)&window);
		}
		init = true;
		break;
	case DestroyNotify:
//		printf("DestroyNotify\n");
		altEngine.destroy();
		return 1;
	}
	return 0;
}

char *getFile(char *fileName)
{
	FILE	*file;
	char	*buffer;
	int	fSize, bRead;

	file = fopen(fileName, "rb");
	if (file == NULL)
		return 0;
	fseek(file, 0, SEEK_END);
	fSize = ftell(file);
	fseek(file, 0, SEEK_SET);
	buffer = (char *) malloc( fSize * sizeof(char) + 1 );
	bRead = fread(buffer, sizeof(char), fSize, file);
	if (bRead != fSize)
		return 0;
	fclose(file);
	buffer[fSize] = '\0';
	return buffer;
}
