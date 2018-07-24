#include "mygl.h"

GRAYCODE gc;
Calibration calib(10, 7, 69.0);
MyTexture myTex;

bool MODE = 1;
bool GETPOINT = 0;
double max = 0.0;

const std::string calib_paramater_file_name("./calibration.xml");
const std::string image_file_name("./pattern.jpg");

#define USE_PS 0
#if USE_PS
std::string excel_file_name("./phaseshift_200.csv");
#else
std::string excel_file_name("./graycode_200.csv");
#endif

cv::Mat image(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

float TEX_M[16] = {
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1
};
double convert[16] = {
	0.5, 0, 0, 0.5,
	0, -0.5, 0, 0.5,
	0, 0, 1, 0,
	0, 0, 0, 1
};

myGL::myGL()
{
	CamWidth = CAMERA_WIDTH;
	CamHeight = CAMERA_HEIGHT;
	ProjWidth = PROJECTOR_WIDTH;
	ProjHeight = PROJECTOR_HEIGHT;

	// Window 0 �̐ݒ�
	window[0].flag = 1;
	window[0].positionX = 0;
	window[0].positionY = 0;
	window[0].width = 600;
	window[0].height = window[0].width * ((double)CamHeight / (double)CamWidth);
	window[0].title = "Camera View";

	// Window1 �̐ݒ�
	window[1].flag = 1;
	window[1].positionX = 0;
	window[1].positionY = 600;
	window[1].width = 600;
	window[1].height = window[1].width * ((double)ProjHeight / (double)ProjWidth);
	window[1].title = "Projector View";

	// Window2 �̐ݒ�
	//window[2].WinFlag = 0;
	//window[2].WindowPositionX = 800;
	//window[2].WindowPositionY = 0;
	//window[2].WindowWidth = 600;
	//window[2].WindowHeight = window[1].WindowWidth * ((double)CamHeight / (double)CamWidth);
	//window[2].WindowTitle = "User View";

	//---�ϐ��錾---
	cameraDistance = 0, cameraX = 0, cameraY = 0;
	xBegin, yBegin;

	glFovy = 42.2;        //���p�x
	CamFovy = 42.2;
	ProjFovy = 42.2;
	glZNear = 12.5*0.001f;        //near�ʂ̋���
	glZFar = 10.0;    //far�ʂ̋���

	world3DPoint = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_32FC3);
	rgbImage = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);

	y_pixcel = 640;

	// �p�����[�^������
	imagePoint.clear();
	projPoint.clear();
	pointColor.clear();
	reconstructPoint.clear();

}
myGL::~myGL()
{
}


/* �E�B���h�E�̐ݒ� */
void myGL::createWindow(int window_num)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);	//�\�����[�h
	glutInitWindowPosition(window[window_num].positionX, window[window_num].positionY); //�E�B���h�E�̈ʒu�̎w��
	glutInitWindowSize(window[window_num].width, window[window_num].height); //�E�B���h�E�T�C�Y�̎w��
	window[window_num].id = glutCreateWindow(window[window_num].title); //�E�B���h�E�̖��O

	//�v���W�F�N�^�Ƀt���X�N���[���ŕ\��
	//cvutil::MySetFullScrean(PROJECT_MONITOR_NUMBER, window[0].WindowTitle);

	//�����ݒ�
	initWindow(window_num);

}


/* �ŏ���1�񂾂��Ă΂��֐� */
void myGL::initWindow(int window_num)
{

	/* Window0 (�J�������_�f��) �̐ݒ� */
	if (window_num == 0)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);

		//�L�����u���[�V�����f�[�^�̓ǂݍ���
		calib.loadCalibParam(calib_paramater_file_name);

		cv::Mat perspectiveMatrix = calib.getCamPerspectiveMat();

		// OpenGL�̎ˉe�s���ݒ肷��
		setProjectionMatrix(perspectiveMatrix, CamWidth, CamHeight);

		//�摜�̓ǂݍ���
		if (myTex.loadImage(image_file_name, &texture) != true)
		{
			exit(0);
		};

	}

	/* Window1 (�v���W�F�N�^���_�f��) �̐ݒ� */
	if (window_num == 1)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);

		//�L�����u���[�V�����f�[�^�̓ǂݍ���
		calib.loadCalibParam(calib_paramater_file_name);

		//��]�E���i�s��̓ǂݍ���
		R = calib.R;
		T = calib.T * 0.001f; // [mm] ���� [m]��
		
		//�O���p�����[�^�̐ݒ�
		cv::Mat extrinsic(4, 4, CV_64F);
		extrinsic.at<double>(0, 0) = R.at<double>(0, 0);
		extrinsic.at<double>(0, 1) = R.at<double>(0, 1);
		extrinsic.at<double>(0, 2) = R.at<double>(0, 2);
		extrinsic.at<double>(1, 0) = R.at<double>(1, 0);
		extrinsic.at<double>(1, 1) = R.at<double>(1, 1);
		extrinsic.at<double>(1, 2) = R.at<double>(1, 2);
		extrinsic.at<double>(2, 0) = R.at<double>(2, 0);
		extrinsic.at<double>(2, 1) = R.at<double>(2, 1);
		extrinsic.at<double>(2, 2) = R.at<double>(2, 2);
		extrinsic.at<double>(0, 3) = T.at<double>(0, 0);
		extrinsic.at<double>(1, 3) = T.at<double>(1, 0);
		extrinsic.at<double>(2, 3) = T.at<double>(2, 0);
		extrinsic.at<double>(3, 0) = 0.0;
		extrinsic.at<double>(3, 1) = 0.0;
		extrinsic.at<double>(3, 2) = 0.0;
		extrinsic.at<double>(3, 3) = 1.0;

		//�����p�����[�^�̐ݒ�
		cv::Mat intrinsic(3, 4, CV_64F);
		intrinsic.at<double>(0, 0) = calib.proj_K.at<double>(0, 0);
		intrinsic.at<double>(0, 1) = calib.proj_K.at<double>(0, 1);
		intrinsic.at<double>(0, 2) = calib.proj_K.at<double>(0, 2);
		intrinsic.at<double>(1, 0) = calib.proj_K.at<double>(1, 0);
		intrinsic.at<double>(1, 1) = calib.proj_K.at<double>(1, 1);
		intrinsic.at<double>(1, 2) = calib.proj_K.at<double>(1, 2);
		intrinsic.at<double>(2, 0) = calib.proj_K.at<double>(2, 0);
		intrinsic.at<double>(2, 1) = calib.proj_K.at<double>(2, 1);
		intrinsic.at<double>(2, 2) = calib.proj_K.at<double>(2, 2);
		intrinsic.at<double>(0, 3) = 0.0;
		intrinsic.at<double>(1, 3) = 0.0;
		intrinsic.at<double>(2, 3) = 0.0;

		//�������e�s��̌v�Z
		cv::Mat perspectiveMatrix = intrinsic * extrinsic;

		// OpenGL�̎ˉe�s���ݒ肷��
		setProjectionMatrix(perspectiveMatrix, ProjWidth, ProjHeight);

		//�摜�̓ǂݍ���
		if (myTex.loadImage(image_file_name, &texture) != true)
		{
			exit(0);
		};
	}

	/* Window2 (���[�U���_�f��) �̐ݒ� */
	if (window_num == 2)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);
	}


}

/*
** 3�����f�[�^�̎擾
*/
void myGL::setProjectionMatrix(cv::Mat &perspectiveMatrix, int w, int h)
{
	//if (perspectiveMatrix.cols != 4 && perspectiveMatrix.row != 3){
	//	std::cout << "�������e�s��̃T�C�Y���Ⴄ" << std::endl;
	//}

	// OpenGL�̎ˉe�s���ݒ肷��
	double CONVERT_TO_OPENGL_WIN[16] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};

	CONVERT_TO_OPENGL_WIN[0] = 1.0 / ( w / 2.0);
	CONVERT_TO_OPENGL_WIN[5] = -1.0 / ( h / 2.0);
	CONVERT_TO_OPENGL_WIN[12] = -1.0;
	CONVERT_TO_OPENGL_WIN[13] = 1.0;

	double projectionMatrix[16];
	for (int i = 0; i < 4; i++){
		projectionMatrix[i * 4] = perspectiveMatrix.at<double>(0, i);
		projectionMatrix[i * 4 + 1] = perspectiveMatrix.at<double>(1, i);
		projectionMatrix[i * 4 + 2] = 0;
		projectionMatrix[i * 4 + 3] = perspectiveMatrix.at<double>(2, i);
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(CONVERT_TO_OPENGL_WIN);
	glMultMatrixd(projectionMatrix);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/*
** 3�����f�[�^�̎擾
*/
void myGL::initialize()
{
	// �L�����u���[�V�����f�[�^�̓ǂݍ���
	calib.loadCalibParam(calib_paramater_file_name);
	// ProCam�Ԃ̑Ή��t�� (true : ���e����, false : ���e���Ȃ�)
	getPixelCorrespondance(false);
	// 3�����v��
	getWorldPoint();

	// GLEW �̏�����
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(1);
	}

	//�摜�̓ǂݍ���
	if ( myTex.loadImage(image_file_name, &texture) != true)
	{
		exit(0);
	};

	std::cout << "r�F���Z�b�g" << std::endl;
	std::cout << "q�F�I��" << std::endl;
	std::cout << "0�F�􉽑Ή��擾" << std::endl;
}


/*
** �R�[�h���e
*/
void myGL::getPixelCorrespondance(bool projection_image)
{
	//�@�O���C�R�[�h�̓��e���B�e���Ή��_�擾
#if USE_PS
	if(projection_image)
		ps.code_projection();
	ps.make_thresh();
	ps.makeCorrespondence();
	surface_image = cv::imread("./PhaseShift/PhaseImage/white.bmp", 1);
#else
	if (projection_image){
		gc.code_projection();
		gc.make_thresh();
	}
	gc.makeCorrespondence();
	surface_image = cv::imread("./cap.jpg", 1);
	gc.reshapeCam2Proj(surface_image, surface_image);
	cv::imwrite("./prj_cap.jpg", surface_image);
	////�摜�̓ǂݍ���
	cv::cvtColor(surface_image, surface_image, CV_BGR2RGB);
#endif

}

//************************
//		�`��
//************************

/* �J�������̕`�揈�� */
void myGL::display_camera_view()
{
	/* �e�N�X�`���s��̐ݒ� */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	static double modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	static double projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	std::cout << "model" << std::endl;
	for (int i = 0; i < 16; i++){
		std::cout << i  << ":" << modelview[i] << std::endl;
	}
	cv::Mat Model(4, 4, CV_64FC1, modelview);
	cv::Mat Proj(4, 4, CV_64FC1, projection);
	cv::Mat C(4, 4, CV_64FC1, convert);
	cv::Mat PM = Model * Proj;
	cv::transpose(PM, PM);
	PM = C * PM;

	int index = 0;
	for (int i = 0; i<4; i++){
		for (int t = 0; t<4; t++){
			TEX_M[index] = cv::saturate_cast<float>(PM.at<double>(t, i));
			index++;
		}
	}

	// UV�̎���������L��������B
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glEnable(GL_TEXTURE_GEN_R);  // ���܂�
	glEnable(GL_TEXTURE_GEN_Q);

	// ���������̌v�Z���ɃI�u�W�F�N�g��Ԃ̒��_���W���g���B
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

	// ���`�������鎮�̌W������ׂĂ����B
	float vs[] = { TEX_M[0], TEX_M[4], TEX_M[8], TEX_M[12] };
	float vt[] = { TEX_M[1], TEX_M[5], TEX_M[9], TEX_M[13] };
	float vr[] = { TEX_M[2], TEX_M[6], TEX_M[10], TEX_M[14] };
	float vq[] = { TEX_M[3], TEX_M[7], TEX_M[11], TEX_M[15] };


	// ���������ϊ��s����I�u�W�F�N�g�̒��_�Ɋ|����Ή�ʂ𕢂��悤��UV���v�Z�����B
	glTexGenfv(GL_S, GL_OBJECT_PLANE, vs);
	glTexGenfv(GL_T, GL_OBJECT_PLANE, vt);
	glTexGenfv(GL_R, GL_OBJECT_PLANE, vr);
	glTexGenfv(GL_Q, GL_OBJECT_PLANE, vq);

	/* ��ʃN���A */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	// �e�N�X�`���}�b�s���O�J�n
	glEnable(GL_TEXTURE_2D);
	pointCloudRender();
	glDisable(GL_TEXTURE_2D);

	//**************************
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	//glEnable(GL_DEPTH_TEST); //�uZ�o�b�t�@�v��L��

	///* ���݂̓����ϊ��s��ƃ��f���r���[�ϊ��s��𓾂� */
	//GLdouble model[16], proj[16];
	//glGetDoublev(GL_MODELVIEW_MATRIX, model);
	//glGetDoublev(GL_PROJECTION_MATRIX, proj);

	//myTex.projectiveTextureMapping(true);
	//// �e�N�X�`���Əd�Ȃ�悤���s�ړ�
	//glMatrixMode(GL_TEXTURE);
	//glMultMatrixd(proj);
	//glMultMatrixd(model);
	//pointCloudRender();
	myTex.projectiveTextureMapping(false);

	glutSwapBuffers();

}

/* �v���W�F�N�^���̕`�揈�� */
void myGL::display_projector_view()
{
	// UV�̎���������L��������B
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glEnable(GL_TEXTURE_GEN_R);  // ���܂�
	glEnable(GL_TEXTURE_GEN_Q);

	// ���������̌v�Z���ɃI�u�W�F�N�g��Ԃ̒��_���W���g���B
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

	// ���`�������鎮�̌W������ׂĂ����B
	float vs[] = { TEX_M[0], TEX_M[4], TEX_M[8], TEX_M[12] };
	float vt[] = { TEX_M[1], TEX_M[5], TEX_M[9], TEX_M[13] };
	float vr[] = { TEX_M[2], TEX_M[6], TEX_M[10], TEX_M[14] };
	float vq[] = { TEX_M[3], TEX_M[7], TEX_M[11], TEX_M[15] };


	// ���������ϊ��s����I�u�W�F�N�g�̒��_�Ɋ|����Ή�ʂ𕢂��悤��UV���v�Z�����B
	glTexGenfv(GL_S, GL_OBJECT_PLANE, vs);
	glTexGenfv(GL_T, GL_OBJECT_PLANE, vt);
	glTexGenfv(GL_R, GL_OBJECT_PLANE, vr);
	glTexGenfv(GL_Q, GL_OBJECT_PLANE, vq);

	/* ���f���r���[�ϊ��s��̐ݒ� */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* ��ʃN���A */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// �e�N�X�`���}�b�s���O�J�n
	glEnable(GL_TEXTURE_2D);
	pointCloudRender();
	glDisable(GL_TEXTURE_2D);

	//***************************

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	//glEnable(GL_DEPTH_TEST); //�uZ�o�b�t�@�v��L��

	//myTex.projectiveTextureMapping(true);
	//pointCloudRender();
	myTex.projectiveTextureMapping(false);

	glutSwapBuffers();

}

/*
** ���e�ʂ�3�������W���i�[����֐�
*/
void myGL::getWorldPoint()
{

#if USE_PS
	//�J�����𑜓x���f�[�^���擾
	//�Ή������Ă��Ȃ��J������f�ɑΉ�����v���W�F�N�^��f�ɂ� -1 ���i�[
	ps.getCorrespondAllPoints2(projPoint, imagePoint, pointColor);
#else
	//�v���W�F�N�^�𑜓x���f�[�^���擾
	//�G���[��f�ɂ� -1 ���i�[
	gc.getCorrespondAllPoints(projPoint, imagePoint, pointColor);
#endif
	// �Ή��_�̘c�ݏ���
	std::vector<cv::Point2f> undistort_imagePoint;
	std::vector<cv::Point2f> undistort_projPoint;
	std::cout << "image num " << imagePoint.size() << std::endl;
	std::cout << "proj num " << projPoint.size() << std::endl;
	cv::undistortPoints(imagePoint, undistort_imagePoint, calib.cam_K, calib.cam_dist);
	cv::undistortPoints(projPoint, undistort_projPoint, calib.proj_K, calib.proj_dist);

	for (int i = 0; i < imagePoint.size(); ++i)
	{
		undistort_imagePoint[i].x = undistort_imagePoint[i].x * calib.cam_K.at<double>(0, 0) + calib.cam_K.at<double>(0, 2);
		undistort_imagePoint[i].y = undistort_imagePoint[i].y * calib.cam_K.at<double>(1, 1) + calib.cam_K.at<double>(1, 2);
		undistort_projPoint[i].x = undistort_projPoint[i].x * calib.proj_K.at<double>(0, 0) + calib.proj_K.at<double>(0, 2);
		undistort_projPoint[i].y = undistort_projPoint[i].y * calib.proj_K.at<double>(1, 1) + calib.proj_K.at<double>(1, 2);
	}

	// 3�������W�̎擾
	calib.reconstruction(reconstructPoint, undistort_projPoint, undistort_imagePoint);
	// �P�ʂ�[mm]����[m]��
	normalizeData();
	// x, y, z ���W��ϊ�
	for (int i = 0; i < imagePoint.size(); i++){
		cv::Point3f data = reconstructPoint[i];
		reconstructPoint[i] = cv::Point3f(data.x, data.y, data.z);
	}
	//����������
	//smoothing();
	//�t�@�C���֏����o��
	writePointData(y_pixcel);

	GETPOINT = true;
}

/*
** 3�������W�̐��K��
** [mm] ���� [m] ��
*/
void myGL::normalizeData()
{

	for (int i = 0; i < reconstructPoint.size(); ++i)
	{
		reconstructPoint[i].x *= 0.001f;
		reconstructPoint[i].y *= 0.001f;
		reconstructPoint[i].z *= 0.001f;
	}

}

/*
** ���e�ʂ̕���������
*/
void myGL::smoothing()
{
	std::cout << reconstructPoint.size() << std::endl;
	std::cout << CAMERA_HEIGHT * CAMERA_WIDTH << std::endl;
	cv::Mat data = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_32FC3, cv::Scalar::all(0));
	int height = CAMERA_HEIGHT;
	int width = CAMERA_WIDTH;
	for (int y = 0; y < height; y++){
		cv::Vec3f* data_p = data.ptr<cv::Vec3f>(y);
		for (int x = 0; x < width; x++){
			int point = y * width + x;
			//�Ή������Ă���Ε`��
			if (projPoint[point].x != -1){
				data_p[x] = cv::Vec3f(reconstructPoint[point].x, reconstructPoint[point].y, reconstructPoint[point].z);
			}
			else
				data_p[x] = cv::Vec3f(0, 0, 0);
		}
	}
	cv::GaussianBlur(data, data, cv::Size(3.0, 3.0), 10);

	for (int y = 0; y < height; y++){
		cv::Vec3f* data_p = data.ptr<cv::Vec3f>(y);
		for (int x = 0; x < width; x++){
			reconstructPoint[y * width + x] = data_p[x];
		}
	}
}

/*
** ���e�ʂ�_�Q�Ƃ��ĕ`�悷��
*/
void myGL::pointCloudRender()
{

	glBegin(GL_POINTS);
	glPointSize(1);
	for (int i = 0; i < reconstructPoint.size(); ++i) {
		//�Ή������Ă���Ε`��
		if (projPoint[i].x != -1){
			//glColor3f(pointColor[i].x / 255.0f, pointColor[i].y / 255.0f, pointColor[i].z / 255.0f); //RGB
			glVertex3f(reconstructPoint[i].x, reconstructPoint[i].y, reconstructPoint[i].z);
		}
	}
	glEnd();

}

/*
**
*/
void myGL::getWorldPointforMesh()
{
#if USE_PS
	ps.getCorrespondAllPoints2(projPoint, imagePoint, pointColor);
#else
	gc.getCorrespondAllPoints(projPoint, imagePoint, pointColor);
#endif
	// �Ή��_�̘c�ݏ���
	std::vector<cv::Point2f> undistort_imagePoint;
	std::vector<cv::Point2f> undistort_projPoint;
	cv::undistortPoints(imagePoint, undistort_imagePoint, calib.cam_K, calib.cam_dist);
	cv::undistortPoints(projPoint, undistort_projPoint, calib.proj_K, calib.proj_dist);

	for (int i = 0; i < imagePoint.size(); ++i)
	{
		undistort_imagePoint[i].x = undistort_imagePoint[i].x * calib.cam_K.at<double>(0, 0) + calib.cam_K.at<double>(0, 2);
		undistort_imagePoint[i].y = undistort_imagePoint[i].y * calib.cam_K.at<double>(1, 1) + calib.cam_K.at<double>(1, 2);
		undistort_projPoint[i].x = undistort_projPoint[i].x * calib.proj_K.at<double>(0, 0) + calib.proj_K.at<double>(0, 2);
		undistort_projPoint[i].y = undistort_projPoint[i].y * calib.proj_K.at<double>(1, 1) + calib.proj_K.at<double>(1, 2);
	}

	// 3�������W�̎擾
	calib.reconstruction(reconstructPoint, undistort_projPoint, undistort_imagePoint);
#if USE_PS
	int count = 0;
	const int CAM_HEIGHT = CAMERA_HEIGHT;
	const int CAM_WIDTH = CAMERA_WIDTH;
	//Mat��
	for (int y = 0; y < CAM_HEIGHT; y++){
		cv::Vec3f* w_p = world3DPoint.ptr<cv::Vec3f>(y);
		for (int x = 0; x < CAM_WIDTH; x++){
			int point = y*CAM_WIDTH + x;
			//�Ή�����肭�Ƃ�Ă��Ȃ��_�����O����
			if (projPoint[point].x != -1 || projPoint[point].y != -1){
				w_p[x] = cv::Vec3f(-reconstructPoint[point].x, -reconstructPoint[point].y, reconstructPoint[point].z);
				reconstructPoint[point] = cv::Point3f(-reconstructPoint[point].x, -reconstructPoint[point].y, reconstructPoint[point].z);
			}
			else{
				reconstructPoint[point] = cv::Point3f(-1.0, -1.0, -1.0);
				w_p[x] = cv::Vec3f(0.0, 0.0, 0.0);
			}

		}
	}
	//std::cout << "reconstructPoint num : " << reconstructPoint[600*CAM_WIDTH +900] << std::endl;
	//std::cout << "reconstructPoint num : " << world3DPoint.at<cv::Vec3f>(600,900) << std::endl;
	GETPOINT = true;
#else
	for (int i = 0; i < imagePoint.size(); i++)
	{
		cv::Point3f data = reconstructPoint[i];
		reconstructPoint[i] = cv::Point3f(-data.x, -data.y, data.z);
	}
#endif
}

/*
** ���e�ʂ����b�V���Ƃ��ĕ`�悷��
*/
void myGL::meshRender()
{
#if USE_PS
	int CAM_HEIGHT = CAMERA_HEIGHT;
	int CAM_WIDTH = CAMERA_WIDTH;
	float THRESHOLD = 100; 

	for (int y= 0; y < CAM_HEIGHT-1; y++){
		cv::Vec3f* w_p = world3DPoint.ptr<cv::Vec3f>(y);
		cv::Vec3f* w_next_p = world3DPoint.ptr<cv::Vec3f>(y+1);
		cv::Vec3b* rgb_p = rgbImage.ptr<cv::Vec3b>(y);
		cv::Vec3b* rgb_next_p = rgbImage.ptr<cv::Vec3b>(y+1);
		for (int x = 0; x < CAM_WIDTH-1; x++){
			//���Ή��̉�f�͏������Ȃ�
			if(w_p[x][0] == 0 && w_p[x][1] == 0 && w_p[x][2] == 0){
				continue;
			}
			if(w_p[x+1][0] == 0 && w_p[x+1][1] == 0 && w_p[x+1][2] == 0){
				continue;
			}
			//�Ίp�̉��s����������΃e�N�X�`����\��Ȃ�
			if(abs(w_p[x][2] - w_next_p[x+1][2]) > THRESHOLD || abs( w_p[x+1][2] - w_next_p[x][2] ) > THRESHOLD) {
				continue;
			}
			//�e�N�X�`����\��
			glBegin(GL_TRIANGLE_STRIP);
			//����
			glTexCoord2f(0, 0);
			glColor3f(pointColor[y*CAM_WIDTH+x].x / 255.0f, pointColor[y*CAM_WIDTH+x].y / 255.0f, pointColor[y*CAM_WIDTH+x].z / 255.0f); //RGB
			//glColor3f(rgb_p[x][2]/ 255.0f, rgb_p[x][1]/ 255.0f, rgb_p[x][0]/ 255.0f);
			glVertex3f(w_p[x][0],w_p[x][1], w_p[x][2]);
			//����
			glTexCoord2f(1, 0);
			glColor3f(pointColor[y*CAM_WIDTH + x].x / 255.0f, pointColor[y*CAM_WIDTH + x].y / 255.0f, pointColor[y*CAM_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_next_p[x][2]/ 255.0f, rgb_next_p[x][1]/ 255.0f, rgb_next_p[x][1]/ 255.0f);
			glVertex3f(w_next_p[x][0], w_next_p[x][1], w_next_p[x][2]);
			//�E��
			glTexCoord2f(0, 1);
			glColor3f(pointColor[y*CAM_WIDTH + x].x / 255.0f, pointColor[y*CAM_WIDTH + x].y / 255.0f, pointColor[y*CAM_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_p[x+1][2]/ 255.0f, rgb_p[x+1][1]/ 255.0f, rgb_p[x+1][0]/ 255.0f);
			glVertex3f(w_p[x + 1][0], w_p[x + 1][1], w_p[x + 1][2]);
			//�E��
			glTexCoord2f(1, 1);
			glColor3f(pointColor[y*CAM_WIDTH + x].x / 255.0f, pointColor[y*CAM_WIDTH + x].y / 255.0f, pointColor[y*CAM_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_next_p[x+1][2]/ 255.0f, rgb_next_p[x+1][1]/ 255.0f, rgb_next_p[x+1][0]/ 255.0f);
			glVertex3f(w_next_p[x + 1][0], w_next_p[x + 1][1], w_next_p[x + 1][2]);

			glEnd();
		}
	}

#else
	int PROJ_HEIGHT = PROJECTOR_HEIGHT;
	int PROJ_WIDTH = PROJECTOR_WIDTH;
	float THRESHOLD = 100;

	for (int y = 0; y < PROJ_HEIGHT - 1; y++){
		for (int x = 0; x < PROJ_WIDTH - 1; x++){
			int point = y * PROJ_WIDTH + x;
			int under_point = (y + 1) * PROJ_WIDTH + x;
			//���Ή��̉�f�͏������Ȃ�
			if (reconstructPoint[point].x == 0 && reconstructPoint[point].y == 0 && reconstructPoint[point].z == 0){
				continue;
			}
			if (reconstructPoint[point + 1].x == 0 && reconstructPoint[point + 1].y == 0 && reconstructPoint[point + 1].z == 0){
				continue;
			}
			//�Ίp�̉��s����������΃e�N�X�`����\��Ȃ�
			if (abs(reconstructPoint[point].z - reconstructPoint[under_point + 1].z) > THRESHOLD || abs(reconstructPoint[point + 1].z - reconstructPoint[under_point].z) > THRESHOLD) {
				continue;
			}
			//�e�N�X�`����\��
			glBegin(GL_TRIANGLE_STRIP);
			//����
			glTexCoord2f(0, 0);
			glColor3f(pointColor[y*PROJ_WIDTH + x].x / 255.0f, pointColor[y*PROJ_WIDTH + x].y / 255.0f, pointColor[y*PROJ_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_p[x][2]/ 255.0f, rgb_p[x][1]/ 255.0f, rgb_p[x][0]/ 255.0f);
			glVertex3f(reconstructPoint[point].x, reconstructPoint[point].y, reconstructPoint[point].z);
			//����
			glTexCoord2f(1, 0);
			glColor3f(pointColor[y*PROJ_WIDTH + x].x / 255.0f, pointColor[y*PROJ_WIDTH + x].y / 255.0f, pointColor[y*PROJ_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_next_p[x][2]/ 255.0f, rgb_next_p[x][1]/ 255.0f, rgb_next_p[x][1]/ 255.0f);
			glVertex3f(reconstructPoint[under_point].x, reconstructPoint[under_point].y, reconstructPoint[under_point].z);
			//�E��
			glTexCoord2f(0, 1);
			glColor3f(pointColor[y*PROJ_WIDTH + x].x / 255.0f, pointColor[y*PROJ_WIDTH + x].y / 255.0f, pointColor[y*PROJ_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_p[x+1][2]/ 255.0f, rgb_p[x+1][1]/ 255.0f, rgb_p[x+1][0]/ 255.0f);
			glVertex3f(reconstructPoint[point + 1].x, reconstructPoint[point + 1].y, reconstructPoint[point + 1].z);
			//�E��
			glTexCoord2f(1, 1);
			glColor3f(pointColor[y*PROJ_WIDTH + x].x / 255.0f, pointColor[y*PROJ_WIDTH + x].y / 255.0f, pointColor[y*PROJ_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_next_p[x+1][2]/ 255.0f, rgb_next_p[x+1][1]/ 255.0f, rgb_next_p[x+1][0]/ 255.0f);
			glVertex3f(reconstructPoint[under_point + 1].x, reconstructPoint[under_point + 1].y, reconstructPoint[under_point + 1].z);

			glEnd();
		}
	}
#endif
}

/*
** 3�����f�[�^��excel�ɏ����o���֐�
*/
void myGL::writePointData(int y_pixcel)
{
#if USE_PS
	std::ofstream ofsRF(excel_file_name);  // �J�����o�͂��L�^����t�@�C���i�����֐�)
	int y = manekin_pixcel;
	//ps.fileWrite(proj2camPoint, y);
	//std::cout << proj2camPoint.size() << std::endl;
	for (int x = 0; x < CAMERA_WIDTH; ++x) {

		int point = y * CAMERA_WIDTH + x;
		if (projPoint[point].x != -1){
			cv::Point3f data = reconstructPoint[point];
			if (x == 0)
				ofsRF << x << "," << y << "," << "," << data.x << "," << data.y << "," << data.z << "," << y << "\n";
			else
				ofsRF << x << "," << y << "," << "," << data.x << "," << data.y << "," << data.z << "\n";
		}
	}
	ofsRF.close();
#else
	std::ofstream ofsRF(excel_file_name);  // �J�����o�͂��L�^����t�@�C���i�����֐�)
	int y = y_pixcel;
	for (int x = 0; x < CAMERA_WIDTH; ++x) {

		int point = y * CAMERA_WIDTH + x;
		if (projPoint[point].x != -1){
			cv::Point3f data = reconstructPoint[point];
			if (x == 0)
				ofsRF << x << "," << y << "," << "," << data.x << "," << data.y << "," << data.z << "," << y << "\n";
			else
				ofsRF << x << "," << y << "," << "," << data.x << "," << data.y << "," << data.z << "\n";
		}
	}

	ofsRF.close();
#endif
}


/* �A�C�h�����̏��� */
void myGL::idle()
{
	for (int loop = 0; loop < WindowNum; ++loop){
		if (window[loop].flag > 0) {
			glutSetWindow(window[loop].id);
			glutPostRedisplay(); //�ĕ`�� (��display()�֐����Ăяo���֐� )
		}
	}
}

/* �}�E�X�N���b�N���̏��� */
void myGL::mouseClick(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN) {
		switch (button) {
		case GLUT_RIGHT_BUTTON:
		case GLUT_MIDDLE_BUTTON:
		case GLUT_LEFT_BUTTON:
			mButton = button;
			break;
		}
		xBegin = x;
		yBegin = y;
	}
}

/* �}�E�X�h���b�O���̏��� */
void myGL::mouseMotion(int x, int y)
{
	int xDisp, yDisp;
	xDisp = x - xBegin;
	yDisp = y - yBegin;
	switch (mButton) {
	case GLUT_LEFT_BUTTON:
		azimuth += (float)xDisp / 10.0;
		elevation -= (float)yDisp / 10.0;
		break;
	case GLUT_MIDDLE_BUTTON:
		cameraX -= (float)xDisp / 100;
		cameraY += (float)yDisp / 100;
		break;
	case GLUT_RIGHT_BUTTON:
		cameraDistance += (float)xDisp / 10.0;
		break;
	}
	xBegin = x;
	yBegin = y;
}

/* �}�E�X�z�C�[�����쎞�̏��� */
void myGL::mouseWheel(int wheel_number, int direction, int x, int y)
{
	//��O direction = -1
	//��   direction = 1

	//cameraDistance -= (float)direction/40;
	glutPostRedisplay();

}

/* ���Ԋu�ŌĂ΂��֐� */
void myGL::timer(int value)
{
}

/* glut�̃��[�v�I�����ɌĂ΂��֐� */
void myGL::close()
{
}

//���_�ύX
void myGL::polarview(){
	glTranslatef(cameraX, cameraY, cameraDistance);
	glRotatef(-twist, 0.0, 0.0, 1.0);
	glRotatef(-elevation, 1.0, 0.0, 0.0);
	glRotatef(-azimuth, 0.0, 1.0, 0.0);
}

/* ���T�C�Y���̏��� */
void myGL::reshape(int w, int h)
{
}

/* �L�[�{�[�h���쎞�̏��� */
void myGL::keyboard(unsigned char key, int x, int y)
{
	switch (key){
	case 'q':	//�I��
		exit(0);
		break;
	case 'i':	// ���_���Z�b�g
		glutFullScreenToggle();
		break;

	case 'r':	// ���_���Z�b�g
		cameraDistance = 0, cameraX = 0, cameraY = 0;
		xBegin = 0, yBegin = 0;
		gluLookAt(
			0.0, 0.0, 0.0,		//�J�����ʒu
			0.0, 0.0, 1.0,		//�����_
			0.0, 1.0, 0.0);		//���_�̌����ݒ�
		break;
	case 'p':
		break;

	case'0':
		MODE = 0;
		MODE = 1;
		GETPOINT = 1;
		break;
	case'1':
		break;
	case'2':
		break;
	case'3':
		break;
	case'4':
		break;
	}
	std::cout << "*******���͑҂�*********" << std::endl;
}


