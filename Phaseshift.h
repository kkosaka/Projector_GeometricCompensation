#ifndef PHASESHIFT_H
#define PHASESHIFT_H

// GrayCodeを用いた幾何補正

// グレイコードは，縦，横別に作成し，後で合成
// パターン画像はビット演算を用いて作成（文字列char型は使わない）
// パターン画像は1枚ずつ書き込んで保存
#include <iostream>
#include <sstream>
#include <fstream>
#include <windows.h>
#include <iomanip>  // 文字列ストリーム
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <direct.h>
#include "PGROpenCV.h"
#include "Header.h"
#include "Timer.h"

#define PI 3.14159265358979323846264338327950288
#define N 8
#define USE_BILINEAR 1

/** 
@brief GrayCodeを用いた幾何補正<br>
対応付けられなかった画素には近傍の画素をコピーして補間<br>
最終的なプロジェクタとカメラを対応付けた配列はc->CamProに格納される。
*/
class PHASESHIFT{
public:
	static const int PRJ_HEIGHT = PROJECTOR_HEIGHT;
	static const int PRJ_WIDTH = PROJECTOR_WIDTH;
	static const int CAM_HEIGHT = CAMERA_HEIGHT;
	static const int CAM_WIDTH = CAMERA_WIDTH;
	
	// 正弦波の周期
	static const int Lx = 10; //<! x軸方向の正弦波の波長
	static const int Ly = 20; //<! y軸方向の正弦波の波長
	
	// プロジェクタ解像度
	static const int WaveNumX = PROJECTOR_WIDTH/Lx;		//<! x軸方向の正弦波の波の数
	static const int WaveNumY = PROJECTOR_HEIGHT/Ly;	//<! y軸方向の正弦波の波の数

	int delay;				//<! 遅延量
	int ThresholdValue;		//<! グレイコード2値化時の閾値
	int shatter_ps;			//<! 正弦波投影時のシャッタースピード
	int errorCount;			//<! 対応エラー数

	// グレイコード作成に必要な構造体
	typedef struct _Graycode {
		int GrayCode[WaveNumY][WaveNumX];			//<! 波長に合わせたグレイコード（波の数の解像度[高さ][幅]）
		int PrjSize_GrayCode[PROJECTOR_HEIGHT][PROJECTOR_WIDTH];	//<! グレイコード（プロジェクタ解像度[高さ][幅]）
		unsigned int h_bit, w_bit;							//<! 高さ，幅の必要ビット数
		unsigned int all_bit;								//<! 合計ビット数（h_bit + w_bit）
	} Graycode;

	// 位相シフト作成に必要な構造体
	typedef struct _PhaseShift {
		cv::Point2d Phase_Value[CAMERA_HEIGHT][CAMERA_WIDTH];			//<! 位相値を格納
		cv::Point2d Phase_Value_copy[CAMERA_HEIGHT][CAMERA_WIDTH];		//<! 位相値を格納(画像用)
		cv::Point	Pointdata[CAMERA_HEIGHT][CAMERA_WIDTH];				//<! カメラ画素に対応するプロジェクタ画素の座標を格納
		cv::Point2d SubPixel_Pointdata[CAMERA_HEIGHT][CAMERA_WIDTH];	//<! サブピクセル推定されたプロジェクタ座標を格納
		cv::Point2d GT_Phase_Val[CAMERA_HEIGHT][CAMERA_WIDTH];			//<! Ground Truth
	} PhaseShft;

	// プロジェクタ - カメラ対応に必要な構造体
	typedef struct _correspondence {
		int Decoded_GrayCode[CAMERA_HEIGHT][CAMERA_WIDTH];		//<! グレイコードをデコードしたものをカメラ画素に格納
		int CamPointNum[PROJECTOR_HEIGHT][PROJECTOR_WIDTH];		//<! プロジェクタ1画素を指しているカメラ画素の数を格納
		double Distance[CAMERA_HEIGHT][CAMERA_WIDTH];			//<! 対応するプロジェクタ画素までの距離を格納
		double Sum_Distance[PROJECTOR_HEIGHT][PROJECTOR_WIDTH];	//<! 補完に利用するカメラ画素との距離の総和
		std::map<int, cv::Point> *g_code_map;
		std::multimap<int, cv::Point> *point_map;
		Graycode g;
		PhaseShft p;
	} correspondence;

	correspondence *c;

	PHASESHIFT();
	~PHASESHIFT();

	// 関数
	void code_projection();		//<! グレイコード&正弦波を投影
	void make_thresh();			//<! 2値化処理
	void makeCorrespondence();	//<! 幾何対応付けを行う

	/// 幾何変換する関数
	void reshapeCam2Proj(cv::Mat &cam, cv::Mat &prj);		//<! 幾何変換関数をオーバーライド

	// カメラ座標に対するプロジェクタの対応点を返す(高精度版)
	void getCorrespondSubPixelProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, int size);
	void getCorrespondAllPoints(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<cv::Point3i> &pointColor);
	void ThreeDimentionReconstruction();
	void getCorrespondAllPoints2(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<cv::Point3i> &pointColor);
	void fileWrite(std::vector<cv::Point2i> &cameraPoint, int y);
private:
	// ウィンドウネーム(OpenCV用)
	char* CODE_IMG;

	Graycode *g;
	PhaseShft *p;

	void createDirs();			// ディレクトリの作成

	/// グレイコードの作成関連
	void initCamera();				// カメラの初期化
	void pattern_code_projection();	// パターンコード投影 & 撮影
	void makeGraycodeImage();		// パターンコード画像作成
	void initGraycode();			// 位相シフト用グレイコード作成
	void initOnePixelGraycode();	// 1-bit幅のグレイコード作成

	/// 二値化関連
	void makeMask(cv::Mat &mask);																	// グレイコードの画像を利用してマスクを生成する関数()
	void loadCam(cv::Mat &mat, int div_bin, bool flag, bool pattern);								// カメラ撮影画像を読み込む関数
	void makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue);	// ポジとネガの差分を取ってMASK_THRESH以上の輝度のピクセルを白にする
	void thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value );				// 2値化処理関数 
	void smallMaskRange(cv::Mat &src, cv::Mat &dst);

	/// その他
	void initCorrespondence();						// プロジェクタ - カメラ構造体初期化

	//// 画素補間手法
	void interpolation();							// 画素補間
	cv::Point getInterpolatedPoint2(int x, int y);	// 隣接する画素から持ってくる

	/// 位相シフト
	double	A, B;
	int		xplus;
	int		yplus;
	double	cLx;
	double	cLy;
	int maskcheck;
	cv::Mat mask;
	bool power;

	void initPhaseParameter();														// 変数の初期化
	void setCorrectPhase();															// 正確な位相値
	void assign_pointdata();														// 座標データの割り当て
	void makePhasePattarn();														// 縦・横の正弦波パターン作成
	void readPhaseCam(cv::Mat phase_x_img[N], cv::Mat phase_y_img[N] , bool flag);	//正弦波の読み込み
	void restore_phase_value();														// 位相復元処理
	void reliableMeasuresFunc();													// 信頼性評価関数
	void bilinear(cv::Point2d &src, cv::Point2d &dst);								//座標データを相対的に決定する
	void errorCheck();
	void calcBilinear();
	void calcNearestNeighbor();
	void phaseConnection();															// 位相連結処理
	void checkCorrespondence_for_Nearest(int num);										// 画像で画素対応確認（必要ない）
	void checkCorrespondence();														// 画像で画素対応確認（必要ない）
	void code_restore();															// 2値化コード復元



	cv::Mat xphase_img;
	cv::Mat yphase_img;
};

#endif