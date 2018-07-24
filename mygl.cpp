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

	// Window 0 の設定
	window[0].flag = 1;
	window[0].positionX = 0;
	window[0].positionY = 0;
	window[0].width = 600;
	window[0].height = window[0].width * ((double)CamHeight / (double)CamWidth);
	window[0].title = "Camera View";

	// Window1 の設定
	window[1].flag = 1;
	window[1].positionX = 0;
	window[1].positionY = 600;
	window[1].width = 600;
	window[1].height = window[1].width * ((double)ProjHeight / (double)ProjWidth);
	window[1].title = "Projector View";

	// Window2 の設定
	//window[2].WinFlag = 0;
	//window[2].WindowPositionX = 800;
	//window[2].WindowPositionY = 0;
	//window[2].WindowWidth = 600;
	//window[2].WindowHeight = window[1].WindowWidth * ((double)CamHeight / (double)CamWidth);
	//window[2].WindowTitle = "User View";

	//---変数宣言---
	cameraDistance = 0, cameraX = 0, cameraY = 0;
	xBegin, yBegin;

	glFovy = 42.2;        //視角度
	CamFovy = 42.2;
	ProjFovy = 42.2;
	glZNear = 12.5*0.001f;        //near面の距離
	glZFar = 10.0;    //far面の距離

	world3DPoint = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_32FC3);
	rgbImage = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);

	y_pixcel = 640;

	// パラメータ初期化
	imagePoint.clear();
	projPoint.clear();
	pointColor.clear();
	reconstructPoint.clear();

}
myGL::~myGL()
{
}


/* ウィンドウの設定 */
void myGL::createWindow(int window_num)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);	//表示モード
	glutInitWindowPosition(window[window_num].positionX, window[window_num].positionY); //ウィンドウの位置の指定
	glutInitWindowSize(window[window_num].width, window[window_num].height); //ウィンドウサイズの指定
	window[window_num].id = glutCreateWindow(window[window_num].title); //ウィンドウの名前

	//プロジェクタにフルスクリーンで表示
	//cvutil::MySetFullScrean(PROJECT_MONITOR_NUMBER, window[0].WindowTitle);

	//初期設定
	initWindow(window_num);

}


/* 最初に1回だけ呼ばれる関数 */
void myGL::initWindow(int window_num)
{

	/* Window0 (カメラ視点映像) の設定 */
	if (window_num == 0)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);

		//キャリブレーションデータの読み込み
		calib.loadCalibParam(calib_paramater_file_name);

		cv::Mat perspectiveMatrix = calib.getCamPerspectiveMat();

		// OpenGLの射影行列を設定する
		setProjectionMatrix(perspectiveMatrix, CamWidth, CamHeight);

		//画像の読み込み
		if (myTex.loadImage(image_file_name, &texture) != true)
		{
			exit(0);
		};

	}

	/* Window1 (プロジェクタ視点映像) の設定 */
	if (window_num == 1)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);

		//キャリブレーションデータの読み込み
		calib.loadCalibParam(calib_paramater_file_name);

		//回転・並進行列の読み込み
		R = calib.R;
		T = calib.T * 0.001f; // [mm] から [m]へ
		
		//外部パラメータの設定
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

		//内部パラメータの設定
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

		//透視投影行列の計算
		cv::Mat perspectiveMatrix = intrinsic * extrinsic;

		// OpenGLの射影行列を設定する
		setProjectionMatrix(perspectiveMatrix, ProjWidth, ProjHeight);

		//画像の読み込み
		if (myTex.loadImage(image_file_name, &texture) != true)
		{
			exit(0);
		};
	}

	/* Window2 (ユーザ視点映像) の設定 */
	if (window_num == 2)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);
	}


}

/*
** 3次元データの取得
*/
void myGL::setProjectionMatrix(cv::Mat &perspectiveMatrix, int w, int h)
{
	//if (perspectiveMatrix.cols != 4 && perspectiveMatrix.row != 3){
	//	std::cout << "透視投影行列のサイズが違う" << std::endl;
	//}

	// OpenGLの射影行列を設定する
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
** 3次元データの取得
*/
void myGL::initialize()
{
	// キャリブレーションデータの読み込み
	calib.loadCalibParam(calib_paramater_file_name);
	// ProCam間の対応付け (true : 投影する, false : 投影しない)
	getPixelCorrespondance(false);
	// 3次元計測
	getWorldPoint();

	// GLEW の初期化
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(1);
	}

	//画像の読み込み
	if ( myTex.loadImage(image_file_name, &texture) != true)
	{
		exit(0);
	};

	std::cout << "r：リセット" << std::endl;
	std::cout << "q：終了" << std::endl;
	std::cout << "0：幾何対応取得" << std::endl;
}


/*
** コード投影
*/
void myGL::getPixelCorrespondance(bool projection_image)
{
	//　グレイコードの投影＆撮影＆対応点取得
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
	////画像の読み込み
	cv::cvtColor(surface_image, surface_image, CV_BGR2RGB);
#endif

}

//************************
//		描画
//************************

/* カメラ側の描画処理 */
void myGL::display_camera_view()
{
	/* テクスチャ行列の設定 */
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

	// UVの自動生成を有効化する。
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glEnable(GL_TEXTURE_GEN_R);  // おまけ
	glEnable(GL_TEXTURE_GEN_Q);

	// 自動生成の計算式にオブジェクト空間の頂点座標を使う。
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

	// 線形結合する式の係数を並べておく。
	float vs[] = { TEX_M[0], TEX_M[4], TEX_M[8], TEX_M[12] };
	float vt[] = { TEX_M[1], TEX_M[5], TEX_M[9], TEX_M[13] };
	float vr[] = { TEX_M[2], TEX_M[6], TEX_M[10], TEX_M[14] };
	float vq[] = { TEX_M[3], TEX_M[7], TEX_M[11], TEX_M[15] };


	// 合成した変換行列をオブジェクトの頂点に掛ければ画面を覆うようにUVが計算される。
	glTexGenfv(GL_S, GL_OBJECT_PLANE, vs);
	glTexGenfv(GL_T, GL_OBJECT_PLANE, vt);
	glTexGenfv(GL_R, GL_OBJECT_PLANE, vr);
	glTexGenfv(GL_Q, GL_OBJECT_PLANE, vq);

	/* 画面クリア */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	// テクスチャマッピング開始
	glEnable(GL_TEXTURE_2D);
	pointCloudRender();
	glDisable(GL_TEXTURE_2D);

	//**************************
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	//glEnable(GL_DEPTH_TEST); //「Zバッファ」を有効

	///* 現在の透視変換行列とモデルビュー変換行列を得る */
	//GLdouble model[16], proj[16];
	//glGetDoublev(GL_MODELVIEW_MATRIX, model);
	//glGetDoublev(GL_PROJECTION_MATRIX, proj);

	//myTex.projectiveTextureMapping(true);
	//// テクスチャと重なるよう平行移動
	//glMatrixMode(GL_TEXTURE);
	//glMultMatrixd(proj);
	//glMultMatrixd(model);
	//pointCloudRender();
	myTex.projectiveTextureMapping(false);

	glutSwapBuffers();

}

/* プロジェクタ側の描画処理 */
void myGL::display_projector_view()
{
	// UVの自動生成を有効化する。
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glEnable(GL_TEXTURE_GEN_R);  // おまけ
	glEnable(GL_TEXTURE_GEN_Q);

	// 自動生成の計算式にオブジェクト空間の頂点座標を使う。
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

	// 線形結合する式の係数を並べておく。
	float vs[] = { TEX_M[0], TEX_M[4], TEX_M[8], TEX_M[12] };
	float vt[] = { TEX_M[1], TEX_M[5], TEX_M[9], TEX_M[13] };
	float vr[] = { TEX_M[2], TEX_M[6], TEX_M[10], TEX_M[14] };
	float vq[] = { TEX_M[3], TEX_M[7], TEX_M[11], TEX_M[15] };


	// 合成した変換行列をオブジェクトの頂点に掛ければ画面を覆うようにUVが計算される。
	glTexGenfv(GL_S, GL_OBJECT_PLANE, vs);
	glTexGenfv(GL_T, GL_OBJECT_PLANE, vt);
	glTexGenfv(GL_R, GL_OBJECT_PLANE, vr);
	glTexGenfv(GL_Q, GL_OBJECT_PLANE, vq);

	/* モデルビュー変換行列の設定 */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* 画面クリア */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// テクスチャマッピング開始
	glEnable(GL_TEXTURE_2D);
	pointCloudRender();
	glDisable(GL_TEXTURE_2D);

	//***************************

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	//glEnable(GL_DEPTH_TEST); //「Zバッファ」を有効

	//myTex.projectiveTextureMapping(true);
	//pointCloudRender();
	myTex.projectiveTextureMapping(false);

	glutSwapBuffers();

}

/*
** 投影面の3次元座標を格納する関数
*/
void myGL::getWorldPoint()
{

#if USE_PS
	//カメラ解像度分データを取得
	//対応が取れていないカメラ画素に対応するプロジェクタ画素には -1 を格納
	ps.getCorrespondAllPoints2(projPoint, imagePoint, pointColor);
#else
	//プロジェクタ解像度分データを取得
	//エラー画素には -1 を格納
	gc.getCorrespondAllPoints(projPoint, imagePoint, pointColor);
#endif
	// 対応点の歪み除去
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

	// 3次元座標の取得
	calib.reconstruction(reconstructPoint, undistort_projPoint, undistort_imagePoint);
	// 単位を[mm]から[m]へ
	normalizeData();
	// x, y, z 座標を変換
	for (int i = 0; i < imagePoint.size(); i++){
		cv::Point3f data = reconstructPoint[i];
		reconstructPoint[i] = cv::Point3f(data.x, data.y, data.z);
	}
	//平滑化処理
	//smoothing();
	//ファイルへ書き出し
	writePointData(y_pixcel);

	GETPOINT = true;
}

/*
** 3次元座標の正規化
** [mm] から [m] へ
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
** 投影面の平滑化処理
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
			//対応が取れていれば描画
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
** 投影面を点群として描画する
*/
void myGL::pointCloudRender()
{

	glBegin(GL_POINTS);
	glPointSize(1);
	for (int i = 0; i < reconstructPoint.size(); ++i) {
		//対応が取れていれば描画
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
	// 対応点の歪み除去
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

	// 3次元座標の取得
	calib.reconstruction(reconstructPoint, undistort_projPoint, undistort_imagePoint);
#if USE_PS
	int count = 0;
	const int CAM_HEIGHT = CAMERA_HEIGHT;
	const int CAM_WIDTH = CAMERA_WIDTH;
	//Matに
	for (int y = 0; y < CAM_HEIGHT; y++){
		cv::Vec3f* w_p = world3DPoint.ptr<cv::Vec3f>(y);
		for (int x = 0; x < CAM_WIDTH; x++){
			int point = y*CAM_WIDTH + x;
			//対応が上手くとれていない点を除外する
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
** 投影面をメッシュとして描画する
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
			//未対応の画素は処理しない
			if(w_p[x][0] == 0 && w_p[x][1] == 0 && w_p[x][2] == 0){
				continue;
			}
			if(w_p[x+1][0] == 0 && w_p[x+1][1] == 0 && w_p[x+1][2] == 0){
				continue;
			}
			//対角の奥行きが遠ければテクスチャを貼らない
			if(abs(w_p[x][2] - w_next_p[x+1][2]) > THRESHOLD || abs( w_p[x+1][2] - w_next_p[x][2] ) > THRESHOLD) {
				continue;
			}
			//テクスチャを貼る
			glBegin(GL_TRIANGLE_STRIP);
			//左上
			glTexCoord2f(0, 0);
			glColor3f(pointColor[y*CAM_WIDTH+x].x / 255.0f, pointColor[y*CAM_WIDTH+x].y / 255.0f, pointColor[y*CAM_WIDTH+x].z / 255.0f); //RGB
			//glColor3f(rgb_p[x][2]/ 255.0f, rgb_p[x][1]/ 255.0f, rgb_p[x][0]/ 255.0f);
			glVertex3f(w_p[x][0],w_p[x][1], w_p[x][2]);
			//左下
			glTexCoord2f(1, 0);
			glColor3f(pointColor[y*CAM_WIDTH + x].x / 255.0f, pointColor[y*CAM_WIDTH + x].y / 255.0f, pointColor[y*CAM_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_next_p[x][2]/ 255.0f, rgb_next_p[x][1]/ 255.0f, rgb_next_p[x][1]/ 255.0f);
			glVertex3f(w_next_p[x][0], w_next_p[x][1], w_next_p[x][2]);
			//右上
			glTexCoord2f(0, 1);
			glColor3f(pointColor[y*CAM_WIDTH + x].x / 255.0f, pointColor[y*CAM_WIDTH + x].y / 255.0f, pointColor[y*CAM_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_p[x+1][2]/ 255.0f, rgb_p[x+1][1]/ 255.0f, rgb_p[x+1][0]/ 255.0f);
			glVertex3f(w_p[x + 1][0], w_p[x + 1][1], w_p[x + 1][2]);
			//右下
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
			//未対応の画素は処理しない
			if (reconstructPoint[point].x == 0 && reconstructPoint[point].y == 0 && reconstructPoint[point].z == 0){
				continue;
			}
			if (reconstructPoint[point + 1].x == 0 && reconstructPoint[point + 1].y == 0 && reconstructPoint[point + 1].z == 0){
				continue;
			}
			//対角の奥行きが遠ければテクスチャを貼らない
			if (abs(reconstructPoint[point].z - reconstructPoint[under_point + 1].z) > THRESHOLD || abs(reconstructPoint[point + 1].z - reconstructPoint[under_point].z) > THRESHOLD) {
				continue;
			}
			//テクスチャを貼る
			glBegin(GL_TRIANGLE_STRIP);
			//左上
			glTexCoord2f(0, 0);
			glColor3f(pointColor[y*PROJ_WIDTH + x].x / 255.0f, pointColor[y*PROJ_WIDTH + x].y / 255.0f, pointColor[y*PROJ_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_p[x][2]/ 255.0f, rgb_p[x][1]/ 255.0f, rgb_p[x][0]/ 255.0f);
			glVertex3f(reconstructPoint[point].x, reconstructPoint[point].y, reconstructPoint[point].z);
			//左下
			glTexCoord2f(1, 0);
			glColor3f(pointColor[y*PROJ_WIDTH + x].x / 255.0f, pointColor[y*PROJ_WIDTH + x].y / 255.0f, pointColor[y*PROJ_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_next_p[x][2]/ 255.0f, rgb_next_p[x][1]/ 255.0f, rgb_next_p[x][1]/ 255.0f);
			glVertex3f(reconstructPoint[under_point].x, reconstructPoint[under_point].y, reconstructPoint[under_point].z);
			//右上
			glTexCoord2f(0, 1);
			glColor3f(pointColor[y*PROJ_WIDTH + x].x / 255.0f, pointColor[y*PROJ_WIDTH + x].y / 255.0f, pointColor[y*PROJ_WIDTH + x].z / 255.0f); //RGB
			//glColor3f(rgb_p[x+1][2]/ 255.0f, rgb_p[x+1][1]/ 255.0f, rgb_p[x+1][0]/ 255.0f);
			glVertex3f(reconstructPoint[point + 1].x, reconstructPoint[point + 1].y, reconstructPoint[point + 1].z);
			//右下
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
** 3次元データをexcelに書き出す関数
*/
void myGL::writePointData(int y_pixcel)
{
#if USE_PS
	std::ofstream ofsRF(excel_file_name);  // カメラ出力を記録するファイル（応答関数)
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
	std::ofstream ofsRF(excel_file_name);  // カメラ出力を記録するファイル（応答関数)
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


/* アイドル時の処理 */
void myGL::idle()
{
	for (int loop = 0; loop < WindowNum; ++loop){
		if (window[loop].flag > 0) {
			glutSetWindow(window[loop].id);
			glutPostRedisplay(); //再描画 (※display()関数を呼び出す関数 )
		}
	}
}

/* マウスクリック時の処理 */
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

/* マウスドラッグ時の処理 */
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

/* マウスホイール操作時の処理 */
void myGL::mouseWheel(int wheel_number, int direction, int x, int y)
{
	//手前 direction = -1
	//奥   direction = 1

	//cameraDistance -= (float)direction/40;
	glutPostRedisplay();

}

/* 一定間隔で呼ばれる関数 */
void myGL::timer(int value)
{
}

/* glutのループ終了時に呼ばれる関数 */
void myGL::close()
{
}

//視点変更
void myGL::polarview(){
	glTranslatef(cameraX, cameraY, cameraDistance);
	glRotatef(-twist, 0.0, 0.0, 1.0);
	glRotatef(-elevation, 1.0, 0.0, 0.0);
	glRotatef(-azimuth, 0.0, 1.0, 0.0);
}

/* リサイズ時の処理 */
void myGL::reshape(int w, int h)
{
}

/* キーボード操作時の処理 */
void myGL::keyboard(unsigned char key, int x, int y)
{
	switch (key){
	case 'q':	//終了
		exit(0);
		break;
	case 'i':	// 視点リセット
		glutFullScreenToggle();
		break;

	case 'r':	// 視点リセット
		cameraDistance = 0, cameraX = 0, cameraY = 0;
		xBegin = 0, yBegin = 0;
		gluLookAt(
			0.0, 0.0, 0.0,		//カメラ位置
			0.0, 0.0, 1.0,		//注視点
			0.0, 1.0, 0.0);		//視点の向き設定
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
	std::cout << "*******入力待ち*********" << std::endl;
}


