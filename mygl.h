#ifndef MYGL_H
#define MYGL_H

#include "main.h"
#include "quaternion.h"
#include "Graycode.h"
#include "Calibration.h"
#include "myTexture.h"
#include "mycvutil.h"


class myGL
{
public:
	myGL();
	~myGL();


	typedef struct _Windows {
		int id;
		int positionX;	// ウィンドウ位置のX座標
		int positionY;	// ウィンドウ位置のY座標
		int width;		// ウィンドウの幅
		int height;		// ウィンドウの高さ
		char* title;		// ウィンドウのタイトル
		int flag;			//ウィンドウのon off
	} Windows;

	static const int WindowNum = 2;
	Windows window[WindowNum];
	
	//カメラとプロジェクタの情報
	int CamWidth, CamHeight, CamFovy;
	int ProjWidth, ProjHeight, ProjFovy;
	cv::Mat R, T;
	double ex_inv[16];

	// 初期化で使う関数
	void createWindow(int window_num);
	void initWindow(int window_num);
	void initialize();
	void getPixelCorrespondance(bool projection_image);
	// コールバック関数で呼ばせる関数
	void display_camera_view();
	void display_projector_view();
	void reshape(int w, int h);
	void reshape2(int w, int h);
	void idle();
	void mouseClick(int button, int state, int x, int y);
	void mouseMotion(int x, int y);
	void mouseWheel(int wheel_number, int direction, int x, int y);
	void keyboard(unsigned char key, int x, int y);
	void close();
	void timer(int value);
	void getWorldPoint();
	void getWorldPointforMesh();
	void pointCloudRender();
	void meshRender();
	void normalizeData();
	void writePointData(int y_pixcel);
	void smoothing();

	void polarview();
	void setProjectionMatrix(cv::Mat &perspectiveMatrix, int w, int h);
private:

	//---変数宣言---
	int FormWidth;
	int FormHeight;
	int mButton;
	float twist, elevation, azimuth;
	float cameraDistance,cameraX,cameraY;
	int xBegin, yBegin;
	float gZoom;
	//---マクロ定義---
	double glFovy ;        //視角度
	double glZNear ;        //near面の距離
	double glZFar ;    //far面の距離

	// 対応点の取得
	std::vector<cv::Point2f> imagePoint;
	std::vector<cv::Point2f> projPoint;
	std::vector<cv::Point3i> pointColor;
	std::vector<cv::Point3f> reconstructPoint;
	std::vector<cv::Point2i> proj2camPoint;

	cv::Mat mask;
	cv::Mat surface_image;
	GLuint texture;

	cv::Mat world3DPoint;
	
	cv::Mat rgbImage;
	cv::Mat proj_Pmatrix;
	int y_pixcel;
};



#endif
