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
		int positionX;	// �E�B���h�E�ʒu��X���W
		int positionY;	// �E�B���h�E�ʒu��Y���W
		int width;		// �E�B���h�E�̕�
		int height;		// �E�B���h�E�̍���
		char* title;		// �E�B���h�E�̃^�C�g��
		int flag;			//�E�B���h�E��on off
	} Windows;

	static const int WindowNum = 2;
	Windows window[WindowNum];
	
	//�J�����ƃv���W�F�N�^�̏��
	int CamWidth, CamHeight, CamFovy;
	int ProjWidth, ProjHeight, ProjFovy;
	cv::Mat R, T;
	double ex_inv[16];

	// �������Ŏg���֐�
	void createWindow(int window_num);
	void initWindow(int window_num);
	void initialize();
	void getPixelCorrespondance(bool projection_image);
	// �R�[���o�b�N�֐��ŌĂ΂���֐�
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

	//---�ϐ��錾---
	int FormWidth;
	int FormHeight;
	int mButton;
	float twist, elevation, azimuth;
	float cameraDistance,cameraX,cameraY;
	int xBegin, yBegin;
	float gZoom;
	//---�}�N����`---
	double glFovy ;        //���p�x
	double glZNear ;        //near�ʂ̋���
	double glZFar ;    //far�ʂ̋���

	// �Ή��_�̎擾
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
