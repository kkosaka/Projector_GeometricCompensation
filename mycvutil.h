#ifndef MYCVUTIL_H
#define MYCVUTIL_H

#include "main.h"


//プロジェクタ画面番号
#define PROJECT_MONITOR_NUMBER (1)


using namespace std;

namespace cvutil
{
	typedef struct disp_prop{
		int index;
		int x,y,width,height;
	} Disp_Prop;


	static int dispCount=-1;
	static std::vector<Disp_Prop> Disps_Prop;


	//ディスプレイの情報入手
	inline BOOL CALLBACK DispEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData ) {
		Disp_Prop di;
		di.index = dispCount++;
		di.x = lprcMonitor->left;
		di.y = lprcMonitor->top;
		di.width = lprcMonitor->right - di.x;
		di.height = lprcMonitor->bottom - di.y;
		Disps_Prop.push_back(di);

		return TRUE; // TRUEは探索継続，FALSEで終了
	}

	//ディスプレイ検出
	inline void SearchDisplay(void) {
		// 一度だけ実行する
		if (dispCount == -1) {
			dispCount = 0;
			Disps_Prop = std::vector<Disp_Prop>();
			EnumDisplayMonitors(NULL, NULL, DispEnumProc, 0);
			Sleep(200);
		}
	}

	//ディスプレイ関数
	inline void MySetFullScrean(const int num, const char *windowname){

		HWND windowHandle = ::FindWindowA(NULL, windowname);
		SearchDisplay();

		if (NULL != windowHandle) {

			//-ディスプレイを指定-
			Disp_Prop di = Disps_Prop[num];

			//-ウィンドウスタイル変更（メニューバーなし、最前面）-
			SetWindowLongPtr(windowHandle,  GWL_STYLE, WS_POPUP);
			SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

			//-クライアント領域をディスプレーに合わせる-
			SetWindowPos(windowHandle, NULL, di.x, di.y, di.width, di.height, SWP_NOACTIVATE | SWP_SHOWWINDOW | SWP_FRAMECHANGED);
		}
	}


	/// FPS計算
	inline double calcFPS(){
		static int stepCount = 0;
		static int hoge = 0;
		static double fps = 0;
		stepCount++;
		if (GetTickCount() - hoge >= 1000)
		{
			int elapsed = GetTickCount() - hoge;
			fps = stepCount / (elapsed / 1000.0);
			// 小数第2位で丸める
			fps = (int)(fps * 10) / 10.0;

			//std::cout << "LC:" << fps << "[FPS]" << std::endl;
			hoge = GetTickCount();
			stepCount = 0;
		}
		return fps;
	}
};

#endif
