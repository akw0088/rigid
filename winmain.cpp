#include "include.h"
#ifdef WIN32

#include <io.h>
#include <fcntl.h>

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
BOOL setupPixelFormat(HDC);
void RedirectIOToConsole();

#define TICK_TIMER 1
#define WMU_RENDER WM_USER + 1

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR szCmdLine, INT iCmdShow)
{
	HWND		hwnd;
	HACCEL		hAccel;
	MSG		msg;
	WNDCLASS	wndclass;
	char		szAppName[] = TEXT("WinMain");

	wndclass.style		= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wndclass.lpfnWndProc	= WndProc;
	wndclass.cbClsExtra	= 0;
	wndclass.cbWndExtra	= 0;
	wndclass.hInstance	= hInstance;
	wndclass.hIcon		= LoadIcon(hInstance, "altEngine");
	wndclass.hCursor	= LoadCursor(NULL, IDC_ARROW);
	wndclass.hbrBackground	= NULL;
	wndclass.lpszMenuName	= NULL;
	wndclass.lpszClassName	= szAppName;

	if (!RegisterClass(&wndclass))
	{
		MessageBox(NULL, TEXT("Unable to register window class."), szAppName, MB_ICONERROR);
		return 0;
	}
	hwnd = CreateWindow(	szAppName,			// window class name
				TEXT("Rigid Body Dynamics"),		// window caption
				WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS,		// window style
				CW_USEDEFAULT,			// initial x position
				CW_USEDEFAULT,			// initial y position
				CW_USEDEFAULT,			// initial x size
				CW_USEDEFAULT,			// initial y size
				NULL,				// parent window handle
				NULL,				// window menu handle
				hInstance,			// program instance handle
				NULL);				// creation parameters

	hAccel = LoadAccelerators(hInstance, szAppName);
	ShowWindow(hwnd, iCmdShow);
	UpdateWindow(hwnd);
     
	while (TRUE)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE))
		{
			if ( GetMessage(&msg, NULL, 0, 0) > 0 )
			{
				if (!TranslateAccelerator(hwnd, hAccel, &msg))
				{
					TranslateMessage (&msg) ;
					DispatchMessage (&msg) ;
				}
			}
			else
			{
				break;
			}
		}
		else
		{
			SendMessage(hwnd, WMU_RENDER, 0, 0);
		}
	}

	if (!UnregisterClass(szAppName, hInstance))
	{
		MessageBox(NULL, TEXT("Unable to unregister class."), szAppName, MB_ICONERROR);
		return 1;
	}

	return (int)msg.wParam;
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	static HWND		shwnd;
	static HDC		hdc;
	static HGLRC	hglrc, hglrc_legacy;
	static Engine	altEngine;
	static POINT	center;

	switch (message)
	{
	case WM_CREATE:
		AllocConsole();
		RedirectIOToConsole();
		SetTimer ( hwnd, TICK_TIMER, 16, NULL );
		hdc = GetDC(hwnd);
		setupPixelFormat(hdc);
#ifndef DIRECTX
		hglrc_legacy = wglCreateContext(hdc);
		wglMakeCurrent(hdc, hglrc_legacy);

		//opengl 2.0
		hglrc = hglrc_legacy;

#endif
		shwnd = hwnd;
		altEngine.init(&shwnd, &hdc);
		return 0;

	case WMU_RENDER:
		altEngine.render();
		return 0;

	case WM_TIMER:
		switch(wParam)
		{
		case TICK_TIMER:
			altEngine.step();
			break;
		}
		return 0;

	case WM_MOUSEMOVE:
		{

		}
		return 0;
	case WM_SIZE:
		{
			int	width, height;

			width	= LOWORD(lParam);
			height	= HIWORD(lParam);
			center.x = width / 2;
			center.y = height / 2;
			altEngine.resize(width, height);
		}
		return 0;

	case WM_SYSCOMMAND:
		switch(wParam)
		{
		case SC_SCREENSAVE:
		case SC_MONITORPOWER:
			return 0;
		}
		break;

	case WM_DESTROY:
#ifndef DIRECTX
		if (hglrc)
		{
			wglMakeCurrent(NULL, NULL);
			wglDeleteContext(hglrc);
			ReleaseDC(hwnd, hdc);
		}
#endif
		altEngine.destroy();
		PostQuitMessage(0);
		return 0;
	}
	return DefWindowProc (hwnd, message, wParam, lParam);
}

BOOL setupPixelFormat(HDC hdc)
{
	PIXELFORMATDESCRIPTOR pfd;
	INT pixelformat;

	pfd.nSize			= sizeof(PIXELFORMATDESCRIPTOR);
	pfd.nVersion		= 1;
	pfd.dwFlags			= PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_GENERIC_ACCELERATED | PFD_DOUBLEBUFFER;
	pfd.dwLayerMask		= PFD_MAIN_PLANE;
	pfd.iPixelType		= PFD_TYPE_RGBA;
	pfd.cColorBits		= 32;
	pfd.cDepthBits		= 24;
	pfd.cAccumBits		= 32;
	pfd.cStencilBits	= 32;

	if ( (pixelformat = ChoosePixelFormat(hdc, &pfd)) == 0 )
	{
		MessageBox(NULL, "ChoosePixelFormat failed", "Error", MB_OK);
		return FALSE;
	}

	if (SetPixelFormat(hdc, pixelformat, &pfd) == FALSE)
	{
		MessageBox(NULL, "SetPixelFormat failed", "Error", MB_OK);
		return FALSE;
	}
	return TRUE; 
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
	buffer = (char *) new char [fSize * sizeof(char) + 1];
	bRead = (int)fread(buffer, sizeof(char), fSize, file);
	if (bRead != fSize)
		return 0;
	fclose(file);
	buffer[fSize] = '\0';
	return buffer;
}

void RedirectIOToConsole()
{
	int	hConHandle;
	long	lStdHandle;
	FILE	*fp;
	CONSOLE_SCREEN_BUFFER_INFO	coninfo;

	AllocConsole();

	// set the screen buffer to be big enough to let us scroll text
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo);

	coninfo.dwSize.Y = 512;
	SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), coninfo.dwSize);

	// redirect unbuffered STDOUT to the console
	lStdHandle = (long)GetStdHandle(STD_OUTPUT_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);

	fp = _fdopen( hConHandle, "w" );
	*stdout = *fp;
	setvbuf( stdout, NULL, _IONBF, 0 );

	// redirect unbuffered STDIN to the console
	lStdHandle = (long)GetStdHandle(STD_INPUT_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);

	fp = _fdopen( hConHandle, "r" );
	*stdin = *fp;
	setvbuf( stdin, NULL, _IONBF, 0 );

	// redirect unbuffered STDERR to the console
	lStdHandle = (long)GetStdHandle(STD_ERROR_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	fp = _fdopen( hConHandle, "w" );
	*stderr = *fp;
	setvbuf( stderr, NULL, _IONBF, 0 );

	// make cout, wcout, cin, wcin, wcerr, cerr, wclog and clog point to console as well
	//ios::sync_with_stdio();
}
#endif

