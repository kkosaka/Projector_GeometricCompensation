#include "Phaseshift.h"

PHASESHIFT::PHASESHIFT()
{
	CODE_IMG = "CodeImage";
	char buf[128];

	//初期値
	delay = 200;
	ThresholdValue = 17;
	// parameter.iniから読み込み
	GetPrivateProfileStringA("delay", "delay", NULL, buf, sizeof(buf), "./parameter.ini");
	delay = atoi(buf) * 2;
	GetPrivateProfileStringA("graycode", "threshold ", NULL, buf, sizeof(buf), "./parameter.ini");
	ThresholdValue = atoi(buf);


	// 構造体の初期化
	g = new Graycode();
	p = new PhaseShft();
	c = new correspondence();
	c->g_code_map = new std::map<int, cv::Point>();
	c->point_map = new std::multimap< int, cv::Point>();

	// phase shift用 n-bit幅 グレイコード
	c->g.h_bit = (int)ceil( log(WaveNumY+1) / log(2) );
	c->g.w_bit = (int)ceil( log(WaveNumX+1) / log(2) );
	c->g.all_bit = c->g.h_bit + c->g.w_bit;

	// 正弦波初期設定
	xplus = (int)(Lx/2)+1;
	yplus = (int)(Ly/2)+1;
	A = 80.0;	// 振幅
	B = 160.0;	//バイアス成分（底上げ）

	// 正弦波からマスクを作るときの閾値
	maskcheck = 40;

	createDirs();

	//累乗
	power = false;

	//画素対応エラー数の初期化
	errorCount = 0;
}
PHASESHIFT::~PHASESHIFT()
{
}

// ディレクトリの作成
void PHASESHIFT::createDirs()
{
	_mkdir("./PhaseShift");
	// グレイコード用
	_mkdir("./PhaseShift/GrayCodeImage");
	_mkdir("./PhaseShift/GrayCodeImage/CaptureImage");			// グレイコード撮影画像
	_mkdir("./PhaseShift/GrayCodeImage/ProjectionGrayCode");	// グレイコード生画像
	_mkdir("./PhaseShift/GrayCodeImage/ThresholdImage");			// グレイコード撮影画像の二値化した画像
	// 位相シフト用
	_mkdir("./PhaseShift/PhaseImage");
	_mkdir("./PhaseShift/PhaseImage/CaptureImage");				// 正弦波撮影画像
	_mkdir("./PhaseShift/PhaseImage/ProjectionImage");		// 正弦波投影画像
	_mkdir("./PhaseShift/PhaseImage/TestImage");		// 正弦波投影画像
}

// 画像作成 & 投影
void PHASESHIFT::code_projection()
{
	makeGraycodeImage();		//GrayCode作成
	makePhasePattarn();			//正弦波画像作成
	pattern_code_projection();	//GrayCode投影
}

// 初期化
void PHASESHIFT::makeCorrespondence()
{
	std::cout << "幾何対応取得開始" << std::endl;
	mask = cv::imread("./PhaseShift/GrayCode_Mask.bmp",0); // マスク画像の読み込み
	//初期化
	initCorrespondence();
	initPhaseParameter();
	//復元処理
	code_restore();
	//幾何対応マップの作成
	if(USE_BILINEAR){
		std::cout << "Bilinear補完開始" << std::endl;
		calcBilinear();
	}else{
		std::cout << "Nearest補完開始" << std::endl;
		//確認用の画像
		checkCorrespondence_for_Nearest(0); //補完前
		//std::cout << "補完開始" << std::endl;
		interpolation();
		//std::cout << "補完終了" << std::endl;
		checkCorrespondence_for_Nearest(1); //補完後
		calcNearestNeighbor();
	}
	std::cout << "幾何対応取得終了" << std::endl;
	//(Debug用)幾何変換の確認
	checkCorrespondence();
}

/****************************************
** グレイコード & 位相シフトの作成関連 **
****************************************/

// プロジェクタ - カメラ構造体初期化
void PHASESHIFT::initCorrespondence()
{
	// グレイコードの作成
	initGraycode();
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ){
			c->Decoded_GrayCode[y][x] = 0;
		}
	}
}

// 波長に合わせたビット数の計算とグレイコードの作成
void PHASESHIFT::initGraycode()
{
	int bin_code_h[WaveNumY];  // 2進コード（縦）
	int bin_code_w[WaveNumX];   // 2進コード（横）
	int graycode_h[WaveNumY];  // グレイコード（縦）
	int graycode_w[WaveNumX];   // グレイコード（横）
	//int *graycode_h =  new int[c->g.h_bit];  // グレイコード（縦）
	//int *graycode_w =  new int[c->g.w_bit];  // グレイコード（横）

	/***** 2進コード作成 *****/
	// 行について
	for( int y = 0; y < WaveNumY; y++ ){
		bin_code_h[y] = y + 1;
	}
	// 列について
	for( int x = 0; x < WaveNumX; x++ )
		bin_code_w[x] = x + 1;

	/***** グレイコード作成 *****/
	// 行について
	for( int y = 0; y < WaveNumY; y++ )
		graycode_h[y] = bin_code_h[y] ^ ( bin_code_h[y] >> 1 );
	// 列について
	for( int x = 0; x < WaveNumX; x++ )
		graycode_w[x] = bin_code_w[x] ^ ( bin_code_w[x] >> 1 );
	// 行列を合わせる（行 + 列）
	for( int y = 0; y < WaveNumY; y++ ) {
		for( int x = 0; x < WaveNumX; x++ ) {
			c->g.GrayCode[y][x] = ( graycode_h[y] << c->g.w_bit) | graycode_w[x];
			//プロジェクタサイズのグレイコード配列作成
			for( int i = 0; i < Ly; i++ ) 
				for( int j = 0; j < Lx; j++ ) 
					c->g.PrjSize_GrayCode[y*Ly+i][x*Lx+j] = c->g.GrayCode[y][x];
		}
	}
}

// (GrayCode)パターンコード画像作成
void PHASESHIFT::makeGraycodeImage()
{
	std::cout << "投影用グレイコード作成中" << std::endl;
	initGraycode();
	cv::Mat posi_img ( PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	cv::Mat nega_img ( PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	int bit = c->g.all_bit-1;
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit];  // 書式付入出力
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];  // 書式付入出力

	// ポジパターンコード画像作成
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
			for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
				if( ( (c->g.PrjSize_GrayCode[y][x] >> (bit-z)) & 1 ) == 0 ) {  // 最上位ビットから順に抽出し，そのビットが0だった時
					posi_img.at<cv::Vec3b>( y, x )[0] = 0;  // B
					posi_img.at<cv::Vec3b>( y, x )[1] = 0;  // G
					posi_img.at<cv::Vec3b>( y, x )[2] = 0;  // R
				}else if( ( (c->g.PrjSize_GrayCode[y][x] >> (bit-z)) & 1 ) == 1 ) {
					posi_img.at<cv::Vec3b>( y, x )[0] = 255;  // B
					posi_img.at<cv::Vec3b>( y, x )[1] = 255;  // G
					posi_img.at<cv::Vec3b>( y, x )[2] = 255;  // R
				}
			}
		}
		// 連番でファイル名を保存（文字列ストリーム）
		Filename_posi[z] << "./PhaseShift/GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_posi[z].str(), posi_img);
		Filename_posi[z] << std::endl;
	}

	// ネガパターンコード画像作成
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
			for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
				if( ( (c->g.PrjSize_GrayCode[y][x] >> (bit-z)) & 1 ) == 1 ) {
					nega_img.at<cv::Vec3b>( y, x )[0] = 0;  // B
					nega_img.at<cv::Vec3b>( y, x )[1] = 0;  // G
					nega_img.at<cv::Vec3b>( y, x )[2] = 0;  // R
				}else if( ( (c->g.PrjSize_GrayCode[y][x] >> (bit-z)) & 1 ) == 0 ) {
					nega_img.at<cv::Vec3b>( y, x )[0] = 255;  // B
					nega_img.at<cv::Vec3b>( y, x )[1] = 255;  // G
					nega_img.at<cv::Vec3b>( y, x )[2] = 255;  // R
				}
			}
		}
		// 連番でファイル名を保存（文字列ストリーム）
		Filename_nega[z] << "./PhaseShift/GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_nega[z].str(), nega_img);
		Filename_nega[z] << std::endl;
	}

	delete[] Filename_posi;
	delete[] Filename_nega;
}

// 正弦波画像作成
void PHASESHIFT::makePhasePattarn()
{
	initPhaseParameter();

	cv::Mat wave_img(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC1);
	int max=0, min=255,temp=0;
	char buf[256];
	// x軸方向
	for(int i = 0; i < N; i++ ) {
		for(int x=0; x<PROJECTOR_WIDTH; x++){
			for(int y=0; y<PROJECTOR_HEIGHT; y++){
				wave_img.at<uchar>(y,x) = (uchar)( A * sin( 2.0*PI*((double)i/N + (double)(x+xplus)/Lx) ) + B );
				//-最大値・最小値の計算
				temp = (int)wave_img.at<uchar>(y,x);
				if(temp > max)
					max = temp;
				if(temp < min)
					min = temp;
			}
		}
		sprintf_s(buf, "./PhaseShift/PhaseImage/ProjectionImage/x_patarn%02d.bmp", i);
		cv::imwrite(buf,wave_img);
	}
	// y軸方向
	for(int i = 0; i < N; i++ ) {
		for(int y=0; y<PROJECTOR_HEIGHT; y++){
			for(int x=0; x<PROJECTOR_WIDTH; x++){
				wave_img.at<uchar>(y,x) = (uchar)( A * sin( 2.0*PI*((double)i/N + (double)(y+yplus)/Ly) ) + B );
			}
		}
		sprintf_s(buf, "./PhaseShift/PhaseImage/ProjectionImage/y_patarn%02d.bmp", i);
		cv::imwrite(buf,wave_img);
	}

	std::cout << "max : " << max << std::endl;
	std::cout << "min : " << min << std::endl;
	wave_img.release();
}

// パターンコード投影 & 撮影
void PHASESHIFT::pattern_code_projection()
{
	// 定数
	typedef enum flag{
		POSI = true,
		NEGA = false,
		VERTICAL = true,
		HORIZONTAL = false,
	} flag;

	Graycode *g = new Graycode();

	TPGROpenCV	pgrOpenCV;
	char buf[256];

	//初期設定
	if(pgrOpenCV.init( FlyCapture2::PIXEL_FORMAT_BGR, FlyCapture2::HQ_LINEAR) == -1){
		exit(0);
	}
	pgrOpenCV.setShutterSpeed(pgrOpenCV.getShutter_h());
	pgrOpenCV.start();

	// 全画面表示用ウィンドウの作成 
	cv::namedWindow(CODE_IMG, 0);
	Projection::MySetFullScrean(DISPLAY_NUMBER, CODE_IMG);

	/******* GrayCode読込 *********/

	//cv::Mat white = cv::imread("./UseImage/whitebar.bmp", 1);
	//cv::imshow(CODE_IMG, white);
	//cv::waitKey(0);

	cv::Mat *posi_img = new cv::Mat[c->g.all_bit];  // ポジパターン用
	cv::Mat *nega_img = new cv::Mat[c->g.all_bit];  // ネガパターン用

	// 書式付入出力（グレイコード読み込み用）
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];
	// 書式付入出力（撮影画像書き込み用）
	std::stringstream *Filename_posi_cam = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega_cam = new std::stringstream[c->g.all_bit];

	// 連番でファイル名を読み込む（文字列ストリーム）
	std::cout << "投影用グレイコード画像読み込み中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		Filename_posi[i] << "./PhaseShift/GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		Filename_nega[i] << "./PhaseShift/GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		//-読み込み
		posi_img[i] = cv::imread(Filename_posi[i].str(), 1);
		nega_img[i] = cv::imread(Filename_nega[i].str(), 1);
		Filename_posi[i] << std::endl;
		Filename_nega[i] << std::endl;
		//-読み込む枚数が足りなかったらグレイコード画像を作り直す
		if(posi_img[i].empty() || nega_img[i].empty()){
			std::cout << "ERROR(1)：投影用のグレイコード画像が不足しています。" << std::endl;
			std::cout << "ERROR(2)：グレイコード画像を作成します。" << std::endl;
			makeGraycodeImage();
			pattern_code_projection();
			return;
		}
	}

	/******* sin波読込 *********/

	cv::Mat *phase_x_img = new cv::Mat[N];
	cv::Mat *phase_y_img = new cv::Mat[N];
	readPhaseCam(phase_x_img, phase_y_img, 0);


	/***** グレイコード投影 & 撮影 *****/

	// ポジパターン投影 & 撮影
	std::cout << "ポジパターン撮影中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// 投影
		cv::imshow(CODE_IMG, posi_img[i]);
		// カメラ撮影
		cv::waitKey(delay);
		pgrOpenCV.queryFrame();
		cv::Mat cap = pgrOpenCV.getVideo();
		pgrOpenCV.showCapImg(cap);
		// ポジパターン撮影結果を保存
		// 横縞
		if(i < c->g.h_bit)
			Filename_posi_cam[i] << "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << POSI << ".bmp"; 
		// 縦縞
		else
			Filename_posi_cam[i] << "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << POSI << ".bmp"; 
		//Filename_posi_cam[i] << "./output/Camera_posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		cv::imwrite(Filename_posi_cam[i].str(), cap);
		Filename_posi_cam[i] << std::endl;
	}

	// ネガパターン投影 & 撮影
	std::cout << "ネガパターン撮影中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// 投影
		cv::imshow(CODE_IMG, nega_img[i]);
		// カメラ撮影
		cv::waitKey(delay);
		pgrOpenCV.queryFrame();
		cv::Mat cap = pgrOpenCV.getVideo();
		pgrOpenCV.showCapImg(cap);

		// ポジパターン撮影結果を保持
		// 横縞
		if(i < c->g.h_bit)
			Filename_nega_cam[i] << "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << NEGA << ".bmp"; 
		// 縦縞
		else
			Filename_nega_cam[i] << "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << NEGA << ".bmp"; 
		//Filename_nega_cam[i] << "./output/Camera_nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		cv::imwrite(Filename_nega_cam[i].str(), cap);
		Filename_nega_cam[i] << std::endl;
	}

	/***** 正弦波 投影 & 撮影 *****/
	pgrOpenCV.setShutterSpeed((float)pgrOpenCV.getShutter_h());
	cv::waitKey(delay);

	std::cout << "正弦波パターン撮影中" << std::endl;
	//横方向
	for(int i = 0; i < N; i++ ) {
		// 投影
		cv::imshow(CODE_IMG, phase_x_img[i]);
		// カメラ撮影
		cv::waitKey(delay);
		pgrOpenCV.queryFrame();
		cv::Mat cap = pgrOpenCV.getVideo();
		pgrOpenCV.showCapImg(cap);
		// パターン撮影結果を保存
		sprintf_s(buf, "./PhaseShift/PhaseImage/CaptureImage/x_patarn%02d.bmp", i);
		cv::imwrite(buf,cap);
	}

	//縦方向
	for(int i = 0; i < N; i++ ) {
		// 投影
		cv::imshow(CODE_IMG, phase_y_img[i]);

		// カメラ撮影 & 保存
		cv::waitKey(delay);
		pgrOpenCV.queryFrame();
		cv::Mat cap = pgrOpenCV.getVideo();
		pgrOpenCV.showCapImg(cap);
		sprintf_s(buf, "./PhaseShift/PhaseImage/CaptureImage/y_patarn%02d.bmp", i);
		cv::imwrite(buf,cap);
	}
	std::cout << "正弦波パターン撮影終了" << std::endl;
	//pgrOpenCV.stop();

	/*  確認用の画像投影  */

	// カメラ設定
	pgrOpenCV.setPixelFormat( FlyCapture2::PIXEL_FORMAT_BGR );
	pgrOpenCV.setColorProcessingAlgorithm( FlyCapture2::HQ_LINEAR );
	pgrOpenCV.setShutterSpeed(pgrOpenCV.getShutter_h());
	//pgrOpenCV.start();
	Sleep(delay);
	//cv::waitKey(delay);

	// 画像の読み込み
	//cv::Mat image = cv::imread( "./PhaseShift/PhaseImage/TestImage/pattern.jpg", -1); // -1 そのままのチャネル数で読み込み
	cv::Mat image(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar::all(255));

	//cv::Mat color_image(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3); // -1 そのままのチャネル数で読み込み
	//// -動画がない場合、新たに作成する．
	//if(image.empty()){
	//	std::cout << "./TestImage 内に pattern.jpg がありません。作成します。" << std::endl;
	//	cv::Mat pattern(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar::all(255));
	//	cv::imwrite("./PhaseShift/PhaseImage/TestImage/pattern.jpg",pattern);
	//	image = pattern;
	//}

	// 投影
	cv::imshow(CODE_IMG, image);

	// カメラ撮影 & 保存
	cv::waitKey(delay);
	pgrOpenCV.queryFrame();
	cv::Mat cap = pgrOpenCV.getVideo();
	pgrOpenCV.showCapImg(cap);
	//cv::cvtColor(pgrOpenCV.getVideo(),color_image,CV_BayerBG2BGR);	//
	sprintf_s(buf, "./PhaseShift/PhaseImage/white.bmp");	//-パターン撮影結果を保存
	cv::imwrite(buf,cap);

	/***** 投影 & 撮影終了 *****/

	pgrOpenCV.stop();
	pgrOpenCV.release();

	/***** 終了 *****/

	cv::destroyWindow(CODE_IMG);
	delete[] posi_img;
	delete[] nega_img;
	delete[] Filename_posi;
	delete[] Filename_nega;
	delete[] Filename_posi_cam;
	delete[] Filename_nega_cam;
	delete[] phase_x_img;
	delete[] phase_y_img;
}

/***************
** 二値化関連 **
****************/

// カメラ撮影画像を読み込む関数
void PHASESHIFT::loadCam(cv::Mat &mat, int div_bin, bool vh, bool pn)
{
	char buf[256];
	sprintf_s(buf, "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg%d_%02d_%d.bmp", vh, div_bin, pn);
	mat = cv::imread(buf, 0);
}

// マスクを作成するインタフェース
void PHASESHIFT::makeMask(cv::Mat &mask)
{
	cv::Mat posi_img;
	cv::Mat nega_img;

	// マスク画像生成
	cv::Mat mask_vert, mask_hor;
	static int useImageNumber = 5;
	// y方向のグレイコード画像読み込み
	loadCam(posi_img, useImageNumber, 0, 1);
	loadCam(nega_img, useImageNumber, 0, 0);

	// 仮のマスク画像Y生成
	makeMaskFromCam(posi_img, nega_img, mask_vert, ThresholdValue);

	// x方向のグレイコード画像読み込み
	loadCam(posi_img, useImageNumber, 1, 1);
	loadCam(nega_img, useImageNumber, 1, 0);

	// 仮のマスク画像X生成
	makeMaskFromCam(posi_img, nega_img, mask_hor, ThresholdValue);

	// XとYのORを取る
	// マスク外はどちらも黒なので黒
	// マスク内は（理論的には）必ず一方が白でもう一方が黒なので、白になる
	// 実際はごま塩ノイズが残ってしまう
	cv::bitwise_or(mask_vert, mask_hor, mask);

	// 残ったごま塩ノイズを除去（白ゴマか黒ゴマかで適用順が逆になる）
	dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
	erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

	for (int i = 0; i < 2; i++)
		smallMaskRange(mask, mask);

	cv::imwrite("./PhaseShift/GrayCode_Mask.bmp", mask);


}

// グレイコードの画像を利用してマスクを生成する関数
// ポジとネガの差分を取ってthresholdValue以上の輝度のピクセルを白にする
void PHASESHIFT::makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue)
{
	result = cv::Mat::zeros(cv::Size(CAMERA_WIDTH,CAMERA_HEIGHT), CV_8UC1);

	for(int j=0; j<CAMERA_HEIGHT; j++){
		for(int i=0; i<CAMERA_WIDTH; i++){
			int posi_i = posi.at<uchar>(j, i);
			int nega_i = nega.at<uchar>(j, i);

			if (abs(posi_i - nega_i) > thresholdValue){
				result.at<uchar>(j, i) = 255;
			}else{
				result.at<uchar>(j, i) = 0;
			}
		}
	}
}

// 撮影画像の2値化をするインタフェース
void PHASESHIFT::make_thresh()
{
	cv::Mat posi_img;
	cv::Mat nega_img;
	cv::Mat Geometric_thresh_img;  // 2値化された画像
	cv::Mat mask;

	// マスクを生成
	makeMask(mask);

	int h_bit = (int)ceil( log(WaveNumY+1) / log(2) ); //小数点切り上げ
	int w_bit = (int)ceil( log(WaveNumX+1) / log(2) );
	int all_bit = h_bit + w_bit;

	std::cout << "二値化開始" << std::endl;
	// 連番でファイル名を読み込む
	for( int i = 0; i < h_bit; i++ ) {
		// 読み込み
		char buf[256];
		// ポジパターン読み込み
		loadCam(posi_img, i+1, 0, 1);
		// ネガパターン読み込み
		loadCam(nega_img, i+1, 0, 0);

		// 2値化
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// マスクを適用して2値化
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./PhaseShift/GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::imwrite(buf, masked_img);

		std::cout << i << ", ";
	}
	for( int i = 0; i < w_bit; i++ ) {
		// 読み込み
		char buf[256];
		// ポジパターン読み込み
		loadCam(posi_img, i+1, 1, 1);
		// ネガパターン読み込み
		loadCam(nega_img, i+1, 1, 0);

		// 2値化
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// マスクを適用して2値化
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./PhaseShift/GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i+h_bit);
		cv::imwrite(buf, masked_img);

		std::cout << i+h_bit << ", ";
	}
	std::cout << std::endl;
	std::cout << "二値化終了" << std::endl;
}

// 実際の2値化処理 
void PHASESHIFT::thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value )
{
	thresh_img = cv::Mat(posi.rows, posi.cols, CV_8UC1);
	for( int y = 0; y < posi.rows; y++ ) {
		for(int x = 0; x < posi.cols; x++ ) {
			int posi_pixel = posi.at<uchar>( y, x );
			int nega_pixel = nega.at<uchar>( y, x );

			// thresh_valueより大きいかどうかで二値化
			if( posi_pixel - nega_pixel >= thresh_value )
				thresh_img.at<uchar>( y, x ) = 255;
			else
				thresh_img.at<uchar>( y, x ) = 0;
		}
	}
}

/*	
マスク領域を1pixel小さくする関数 
投影領域の縁は信頼性が低いため，縁は幾何対応付けさせない．
*/
void PHASESHIFT::smallMaskRange(cv::Mat &src, cv::Mat &dst) 
{
	cv::Mat newMask(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1);

	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
			if( (x > 0 && src.at<uchar>(y, x-1) == 0 && src.at<uchar>(y, x) == 255) 
				|| (y > 0 && src.at<uchar>(y-1, x) == 0 && src.at<uchar>(y, x) == 255) )
				newMask.at<uchar>(y, x) = 0;
			else if( (x < CAMERA_WIDTH-1 && src.at<uchar>(y, x) == 255 && src.at<uchar>(y, x+1) == 0) 
				|| (y < CAMERA_HEIGHT-1 && src.at<uchar>(y, x) == 255 && src.at<uchar>(y+1, x) == 0) )
				newMask.at<uchar>(y, x) = 0;
			else if(src.at<uchar>(y, x) == 0)
				newMask.at<uchar>(y, x) = 0;
			else if(src.at<uchar>(y, x) == 255)
				newMask.at<uchar>(y, x) = 255;
		}
	}

	dst = newMask.clone(); 
}

/***********************************
** プロジェクタとカメラの対応付け **
************************************/

// 復元処理
void PHASESHIFT::code_restore()
{
	std::cout << "コード復元開始 " << std::endl;
	Timer tm;
	// グレイコードをデコード
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		char buf[256];
		sprintf_s(buf, "./PhaseShift/GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::Mat a = cv::imread(buf, 0);
		for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
			for( int x = 0; x < CAMERA_WIDTH; x++ ) {
				//- 255ならば1，それ以外は0として2進コードを復元
				if( a.at<uchar>( y, x ) == 255)
					//- 左にシフトさせる
						c->Decoded_GrayCode[y][x] = ( 1 << (c->g.all_bit-i-1) ) | c->Decoded_GrayCode[y][x]; 
			}
		}
	}

	//コードマップの初期化
	c->g_code_map->clear();
	c->point_map->clear();

	//コードマップを作成( Key:グレイコード，value:波の座標データ(何周期目か) )
	for( int y = 0; y < WaveNumY; y++ ) {
		for( int x = 0; x < WaveNumX; x++ ) {
			int a = c->g.GrayCode[y][x];
			if(a != 0)
				(*c->g_code_map)[a] = cv::Point(x,y);
		}
	}

	// 0番目は使わない
	(*c->g_code_map)[0] = cv::Point(-1, -1);

	// 位相復元
	restore_phase_value();
	// 連結処理
	phaseConnection();
	// 信頼性評価関数で位相値のエラー除去
	//reliableMeasuresFunc();
	// 対応エラーを修正する
	errorCheck();

	std::cout << "コード復元終了 : " ;
	tm.elapsed();

	//ファイルの書き込み
	{

		std::ofstream ofsRF("./PhaseShift/PhaseImage/位相データ.csv");  // カメラ出力を記録するファイル（応答関数）
		// エラー処理
		//if(ofsRF.fail()){
		//	std::cout << "ERROR:応答関数.csvが開けません。" << std::endl;
		//}
		//for(int y=0; y<CAMERA_HEIGHT; y++){
		int y = 649;
		//for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
			//マスク外は何もしない
			if(mask.at<uchar>(y, x) == 0){
				continue;
			}
			//if( 1300 < x ){
			//double a = c->p.phase_val_copy[y][x].x;correct_phase_val
			double a = c->p.Phase_Value_copy[y][x].x;
			double b = c->p.Phase_Value[y][x].x;
			//double d = c->p.subpixel_pointdata[y][x].x;
			//double e = c->p.subpixel_pointdata[y][x].y;
			ofsRF << x << ","  << a << "," << b << ","  << "," << "," << c->p.Pointdata[y][x].x << "," << c->p.Pointdata[y][x].y << "\n" ;
			//}
		}
		//}
		ofsRF.close();
	}

}

/***********************************
**         位相シフト関連         **
************************************/

void PHASESHIFT::initPhaseParameter()
{

	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ){
			c->p.Pointdata[y][x] = cv::Point(-1, -1);
			c->p.SubPixel_Pointdata[y][x] = cv::Point(-1, -1);
			c->p.Phase_Value[y][x] = cv::Point2d(NULL, NULL);
			c->p.Phase_Value_copy[y][x] = cv::Point2d(NULL, NULL);
			c->Distance[y][x] = 0.0;
		}
	}
	for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
		for( int x = 0; x < PROJECTOR_WIDTH; x++ ){
			c->CamPointNum[y][x] = 0;
			c->Sum_Distance[y][x] = 0.0;
		}
	}
	// 位相値を画像で確認する用
	xphase_img = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar::all(0));
	yphase_img = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar::all(0));

	// (Debug用)正解データを計算する
	setCorrectPhase();
}

// （Debug用）理想値の代入
void PHASESHIFT::setCorrectPhase()
{

	for(int y=0; y<PROJECTOR_HEIGHT; y++){
		for(int x=0; x<PROJECTOR_WIDTH; x++){
			c->p.GT_Phase_Val[y][x].x = (x+xplus)*2.0*PI/Lx;
			c->p.GT_Phase_Val[y][x].y = (y+yplus)*2.0*PI/Ly;
		}
	}
	//ファイルの書き込み
	//{
	//	std::ofstream ofsRF("./PhaseShift/PhaseImage/正解データ.csv");  // カメラ出力を記録するファイル（応答関数）
	//	//エラー処理
	//	if(ofsRF.fail()){
	//		std::cout << "ERROR:応答関数.csvが開けません。" << std::endl;
	//	}
	//	//for(int y=0; y<PROJECTOR_HEIGHT; y++){
	//	int y = 400;
	//	for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
	//		double a = c->p.GT_Phase_Val[y][x].x;
	//		double b = c->p.Phase_Value[y][x].x;
	//		ofsRF << a << "," << (double)(a*Lx) / (2.0*PI) - (double)xplus<< "\n" ;
	//	}
	//	//}
	//	ofsRF.close();
	//}
}

// 正弦波画像の読み込み
void PHASESHIFT::readPhaseCam(cv::Mat phase_x_img[N], cv::Mat phase_y_img[N], bool flag)
{
	char buf[256];
	char* str;
	if(flag)
		str = "Capture";
	else
		str = "Projection";

	//読み込み
	for( unsigned int i = 0; i < N; i++){
		sprintf_s(buf, "./PhaseShift/PhaseImage/%sImage/x_patarn%02d.bmp", str, i);
		phase_x_img[i] = cv::imread(buf,0);
		sprintf_s(buf, "./PhaseShift/PhaseImage/%sImage/y_patarn%02d.bmp",str ,i);
		phase_y_img[i] = cv::imread(buf,0);

		//error
		if(phase_x_img[i].empty() && phase_y_img[i].empty()){
			printf("Cannot open image\n");
			exit(0);
		}
	}
}

/* 位相復元 */
void PHASESHIFT::restore_phase_value()
{

	cv::Point2d lumi[N];
	cv::Mat *phase_x_img = new cv::Mat[N];
	cv::Mat *phase_y_img = new cv::Mat[N];
	readPhaseCam(phase_x_img, phase_y_img, 1); // flag 1:capture , 0:projection

	int height = (*phase_x_img).rows;
	int width = (*phase_x_img).cols;
	//復元処理

	//（Debug用）画像
	//cv::Mat phasemask((*phase_x_img).rows, (*phase_x_img).cols, CV_8UC1, cv::Scalar::all(0));

	for(int y=0; y<height; y++){
		for(int x=0; x<width; x++){

			//マスク外は何もしない
			if(mask.at<uchar>(y, x) == 0){
				continue;
			}

			cv::Point2d cosValue = cv::Point2d(0.0, 0.0);
			cv::Point2d sinValue = cv::Point2d(0.0, 0.0);

			//分子について
			for(int i = 0; i < N; i++ ){
				lumi[i] = cv::Point2d( (double)phase_x_img[i].at<uchar>(y, x), (double)phase_y_img[i].at<uchar>(y, x) );
				lumi[i] *= cos( 2.0*PI*i / N );
				cosValue += lumi[i];
			}

			//分母について
			for(int i = 0; i < N; i++ ){
				lumi[i] = cv::Point2d( (double)phase_x_img[i].at<uchar>(y, x), (double)phase_y_img[i].at<uchar>(y, x) );
				lumi[i] *= sin( 2.0*PI*i / N );
				sinValue += lumi[i];
			}

			//0割防止処理
			if(  ( (0 <= abs(sinValue.x)) && (abs(sinValue.x) <= 1) ) && cosValue.x > 0 )
				sinValue.x = 1;
			if( ( (0 <= abs(sinValue.x)) && (abs(sinValue.x) <= 1) ) && cosValue.x < 0 )
				sinValue.x = -1;
			if( ( (0 <= abs(sinValue.y)) && (abs(sinValue.y) <= 1) ) && cosValue.x > 0)
				sinValue.y = 1;
			if( ( (0 <= abs(sinValue.y)) && (abs(sinValue.y) <= 1) ) && cosValue.x < 0 )
				sinValue.y = -1;

			//位相復元
			cv::Point2d phase_value = cv::Point2d(atan2(cosValue.x, sinValue.x),atan2(cosValue.y, sinValue.y));

			//格納
			c->p.Phase_Value[y][x] = phase_value;

			//位相情報からマスクを作成する
			//phasemask.at<uchar>(y,x) = 255;
			//if( fabs(cosValue.x) < maskcheck &&  fabs(cosValue.y) < maskcheck){
			//	if( fabs(sinValue.x) < maskcheck &&  fabs(sinValue.y) < maskcheck)
			//		phasemask.at<uchar>(y,x) = 255;
			//}


			// (Debug用) 正規化するために値を 0< value <2π にする
			xphase_img.at<double>(y,x) = phase_value.x + (PI);
			yphase_img.at<double>(y,x) = phase_value.y + (PI);


		}
	}

	/* マスク画像を二周り小さくする*/
	//for (int i = 0; i < 2; i++)
	//	smallMaskRange(phasemask, phasemask);

	// (Debug用)鋸刃状の画像を作成&保存
	cv::Mat temp;
	xphase_img.convertTo(temp, CV_8U, 255/(2.0*PI));
	cv::imwrite("./PhaseShift/PhaseImage/xPhase_val.bmp", temp);
	yphase_img.convertTo(temp, CV_8U, 255/(2.0*PI));
	cv::imwrite("./PhaseShift/PhaseImage/yPhase_val.bmp", temp);

	// マスク画像を変更
	//cv::imwrite("./PhaseShift/PhaseImage/Phase_Mask.bmp", phasemask);
	//cv::imwrite("./CalculationData/mask.bmp", phasemask);
	//cv::imwrite("./CalculationData/Phase_Mask.bmp", phasemask);
	//mask = phasemask.clone();

	delete[] phase_x_img;
	delete[] phase_y_img;
	temp.release();

}


/* 連結処理 */
void PHASESHIFT::phaseConnection()
{
	//連結処理(位相値を連続にする処理)を行う
	for(int y=0; y<CAMERA_HEIGHT; y++){
		for(int x=0; x<CAMERA_WIDTH; x++){

			//マスク外は何もしない
			if(mask.at<uchar>(y, x) == 0){
				continue;
			}

			int a = c->Decoded_GrayCode[y][x];

			//コードが見つからない場合エラー処理
			if( (*c->g_code_map).find(a) == (*c->g_code_map).end() ){
				c->p.Pointdata[y][x].x = -1;
				c->p.Pointdata[y][x].y = -1;
				c->p.Phase_Value[y][x].x = NULL;
				c->p.Phase_Value[y][x].y = NULL;
			}else{
				// 波の座標(+1)を格納
				int nx = ((*c->g_code_map)[a]).x + 1;
				int ny = ((*c->g_code_map)[a]).y + 1;

				// (Debug用)
				c->p.Phase_Value_copy[y][x] = c->p.Phase_Value[y][x];

				// 2nπ加算
				c->p.Phase_Value[y][x].x += ( nx*2.0*PI );
				c->p.Phase_Value[y][x].y += ( ny*2.0*PI );

				//対応するプロジェクタ座標(サブピクセル精度)を算出
				c->p.SubPixel_Pointdata[y][x].x = ( (c->p.Phase_Value[y][x].x * Lx)/(2*PI) - (double)xplus );
				c->p.SubPixel_Pointdata[y][x].y = ( (c->p.Phase_Value[y][x].y * Ly)/(2*PI) - (double)yplus );

				// 対応する座標を近傍の画素とする(Nearest Neighbor補完)
				c->p.Pointdata[y][x].x = (int)( c->p.SubPixel_Pointdata[y][x].x + 0.5);
				c->p.Pointdata[y][x].y = (int)( c->p.SubPixel_Pointdata[y][x].y + 0.5);

			}
		}
	}
}


void PHASESHIFT::errorCheck()
{

	// 位相画像を作成
	cv::Mat xphase_img(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar::all(0));
	cv::Mat yphase_img(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar::all(0));
	int error;
	for(int y=1; y<CAMERA_HEIGHT; y++){
		for(int x=1; x<CAMERA_WIDTH; x++){

			//マスク・エラー値の場合何もしない
			if(mask.at<uchar>(y, x) == 0 || c->p.Phase_Value[y][x].x == NULL || c->p.Phase_Value[y][x].y == NULL
				|| c->p.Phase_Value[y][x-1].x == NULL || c->p.Phase_Value[y-1][x].y == NULL){
					continue;
			}
			if( c->p.Pointdata[y][x].x  == -1 || c->p.Pointdata[y][x-1].x  == -1 
				|| c->p.Pointdata[y][x].y  == -1 || c->p.Pointdata[y-1][x].y  == -1 ){
					continue;
			}

			// x について
			error = c->p.Pointdata[y][x].x - c->p.Pointdata[y][x-1].x;
			// 座標の推移がリニアではない場合,着目画素の位相値に2π足して座標を算出し，
			// そのときの座標値が妥当なものかを判断する
			if(error < 0){
				double value = c->p.Phase_Value[y][x].x + 2.0*PI;
				double sub_p = (value * Lx)/(2*PI) - (double)xplus;
				int p =  (int)(sub_p + 0.5);
				error = p - c->p.Pointdata[y][x-1].x;
				if( 0 <= error  ){
					//-グレイコードの境界誤差
					if( error < 2 ){ 
						c->p.SubPixel_Pointdata[y][x].x = sub_p;
						c->p.Pointdata[y][x].x = p;
						c->p.Phase_Value[y][x].x = value;
					}
					//-位相の誤差
					else{
						//c->p.pointdata[y][x].x = c->p.pointdata[y][x-1].x;
					}
				}
				else{
					c->p.Phase_Value[y][x].x = NULL;
					c->p.Pointdata[y][x].x = -1;
					continue;
					//std::cout << "(x)ズレが大きすぎる：" << x  << ", " << y  <<std::endl;
				}
			}
			//-四捨五入による誤差
			else if(1 < error && error < 3)
				c->p.Pointdata[y][x].x -= 1; 

			// y について 
			error = c->p.Pointdata[y][x].y - c->p.Pointdata[y-1][x].y;
			//-座標の推移がリニアではない場合
			if(error < 0){
				double value = c->p.Phase_Value[y][x].y + 2.0*PI;
				double sub_p = (value * Ly)/(2*PI) - (double)yplus;
				int p = (int)(sub_p + 0.5);
				error = p - c->p.Pointdata[y-1][x].y;
				if( 0 <= error  ){
					//-グレイコードの境界誤差
					if( error < 2 ){ 
						c->p.SubPixel_Pointdata[y][x].y = sub_p;
						c->p.Pointdata[y][x].y = p;
						c->p.Phase_Value[y][x].y = value;
					}
					//-位相の誤差
					else{
						//c->p.pointdata[y][x].y = c->p.pointdata[y-1][x].y;
					}
				}
				else{
					c->p.Phase_Value[y][x].y = NULL;
					c->p.Pointdata[y][x].y = -1;
					continue;
					//	std::cout << "(y)ズレが大きすぎる：" << x  << ", " << y  <<std::endl;
				}
			}
			//-四捨五入による誤差
			else if(1 < error && error < 3)
				c->p.Pointdata[y][x].y -= 1; 

			// error処理
			if(c->p.Pointdata[y][x].x < 0 || c->p.Pointdata[y][x].x > PROJECTOR_WIDTH-1 )
				c->p.Pointdata[y][x].x = -1;
			if(c->p.Pointdata[y][x].y < 0 || c->p.Pointdata[y][x].y > PROJECTOR_HEIGHT-1 )
				c->p.Pointdata[y][x].y = -1;

			// (Debug用)
			xphase_img.at<double>(y,x) = c->p.Phase_Value[y][x].x;
			yphase_img.at<double>(y,x) = c->p.Phase_Value[y][x].y;

		}
	}

	// （確認用）画像で確認
	double nx = (int)( 2.0*PI*(PROJECTOR_WIDTH/Lx)+PI - 2.0*PI*1*(0+xplus)/Lx );	//xの位相値の取り得る範囲(max - min)
	double ny = (int)( 2.0*PI*(PROJECTOR_HEIGHT/Ly)+PI - 2.0*PI*1*(0+yplus)/Ly  );	//yの位相値の取り得る範囲(max - min)
	//std::cout << nx << std::endl;
	//std::cout << ny << std::endl;
	xphase_img.convertTo(xphase_img, CV_8U, 255/nx);
	yphase_img.convertTo(yphase_img, CV_8U, 255/ny);
	cv::imwrite("./PhaseShift/PhaseImage/xPhase_Restore.bmp",xphase_img);
	cv::imwrite("./PhaseShift/PhaseImage/yPhase_Restore.bmp",yphase_img);

	//（確認用）
	xphase_img.release();
	yphase_img.release();
}

void PHASESHIFT::calcBilinear()
{
	Timer tm;
	cv::Point2i prj_point;	//連想配列用
	int key;				//連想配列用

	//OpenMP用
	//int x,y;
	//#pragma omp parallel for private(x)

	// 連想配列に近傍カメラ画素の座標値を格納
	for(int y=1; y<CAMERA_HEIGHT; y++){
		for(int x=1; x<CAMERA_WIDTH; x++){

			//マスク・エラー値の場合何もしない
			if( mask.at<uchar>(y, x) == 0 || c->p.Pointdata[y][x].x  == -1 || c->p.Pointdata[y][x-1].x  == -1 
				|| c->p.Pointdata[y][x].y  == -1 || c->p.Pointdata[y-1][x].y  == -1 ){
					continue;
			}
			//multimapに格納（key:プロジェクタ座標、data:カメラ座標）

			//xを切り捨て，yを切り捨てた座標
			prj_point = cv::Point2i((int)floor(c->p.SubPixel_Pointdata[y][x].x), (int)floor(c->p.SubPixel_Pointdata[y][x].y) );
			key = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));

			// xを切り捨て，yを繰り上げた座標
			prj_point = cv::Point2i((int)floor(c->p.SubPixel_Pointdata[y][x].x), (int)ceil(c->p.SubPixel_Pointdata[y][x].y) );
			key = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));

			// xを繰上げ，yを切り捨てた座標
			prj_point = cv::Point2i((int)ceil(c->p.SubPixel_Pointdata[y][x].x), (int)floor(c->p.SubPixel_Pointdata[y][x].y) );
			key = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));

			// xを繰上げ，yを繰上げた座標
			prj_point = cv::Point2i((int)ceil(c->p.SubPixel_Pointdata[y][x].x), (int)ceil(c->p.SubPixel_Pointdata[y][x].y) );
			key = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));

		}
	}
	std::cout << "連想配列に格納 : " ;
	tm.elapsed();

	// Sub_Distanceの計算
	for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
		for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
			int key = y * PROJECTOR_WIDTH + x;
			//存在する場合
			if( (*c->point_map).find(key) != (*c->point_map).end() ){
				auto range = (*c->point_map).equal_range(key);
				// 距離の合計を計算
				double sum_dist = 0.0;
				for(auto iterator = range.first; iterator != range.second; iterator++){
					auto target = *iterator;
					cv::Point p = target.second;

					// 距離を計算
					double d_x = fabs(c->p.SubPixel_Pointdata[p.y][p.x].x - x);
					double d_y = fabs(c->p.SubPixel_Pointdata[p.y][p.x].y - y);
					double d = (d_x * d_x) + (d_y * d_y);
					//double area = d_x * d_y;
					if(!power)
						d = sqrt(d); //距離

					double Reciprocal_d = 1.0/d;
					//double Reciprocal_d = 1.0/area;

					sum_dist += Reciprocal_d;
				}
				c->Sum_Distance[y][x] = sum_dist;
			}
		}
	}

}

//二アレストネイバー補完の場合
void PHASESHIFT::calcNearestNeighbor()
{
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {

			//マスク外は何もしない
			if( mask.at<uchar>(y, x) == 0 ){
				continue;
			}

			// 距離を計算
			double d_x = abs(c->p.SubPixel_Pointdata[y][x].x - c->p.Pointdata[y][x].x);
			double d_y = abs(c->p.SubPixel_Pointdata[y][x].y - c->p.Pointdata[y][x].y);
			double d = pow(d_x, 2.0) + pow(d_y, 2.0);
			d = sqrt(d);
			if(d > 0){
				//c->Distance[y][x] = pow(1.0/d, 2.0);
				c->Distance[y][x] = 1.0/d;
			}else{
				c->Distance[y][x] = 100; //100は適当
			}

			//multimapに格納（key:プロジェクタ座標、data:カメラ座標）
			int key = c->p.Pointdata[y][x].y * PROJECTOR_WIDTH + c->p.Pointdata[y][x].x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));
		}
	}
	// 距離の総和を計算
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
			cv::Point p = c->p.Pointdata[y][x];
			if( (p.x != -1) && (p.y != -1) ) 
				c->Sum_Distance[p.y][p.x] += c->Distance[y][x];
		}
	}
	assign_pointdata();
}

// (Nearest用)撮影画像を幾何変換
void PHASESHIFT::checkCorrespondence_for_Nearest(int num)
{
	// （確認用）プロジェクタとカメラの対応付け(位相連結処理）
	cv::Mat test = cv::imread("./PhaseShift/PhaseImage/white.bmp",1);
	cv::Mat dst(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3);
	char buf[256];
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {

			//マスク外は何もしない
			if(mask.at<uchar>(y, x) == 0){
				continue;
			}

			cv::Point p = c->p.Pointdata[y][x];

			if( (p.x != -1) && (p.y != -1) ) {
				for( int c = 0; c < dst.channels(); c++)
					dst.data[p.y * dst.step + p.x * dst.elemSize() + c] = test.data[y * test.step + x * test.elemSize() + c];
			}
		}
	}

	std::cout << "保存" << std::endl;
	if(num == 0)
		sprintf_s(buf, "./PhaseShift/PhaseImage/NN_interpolation(before).bmp");
	if(num == 1)
		sprintf_s(buf, "./PhaseShift/PhaseImage/NN_interpolation(after).bmp");
	cv::imwrite(buf,dst);

	test.release();
	dst.release();
}

// (Debug用)補完後に幾何変換
void PHASESHIFT::checkCorrespondence()
{
	cv::Mat test = cv::imread("./PhaseShift/PhaseImage/white.bmp",1);
	cv::Mat dst(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3);
	char buf[256];

	// 幾何変換
	reshapeCam2Proj(test, dst);

	//std::cout << "保存" << std::endl;
#if USE_BILINEAR
	if(power)
		sprintf_s(buf, "./PhaseShift/PhaseImage/BL_Convertion(2乗).bmp");
	else
		sprintf_s(buf, "./PhaseShift/PhaseImage/BL_Convertion.bmp");

#else
	sprintf_s(buf, "./PhaseShift/PhaseImage/NN_Convertion.bmp");
#endif
	cv::imwrite(buf,dst);

	test.release();
	dst.release();
}


/***********************************
**  (Nearest Neighbor用)画素補間  **
************************************/

/*
画素補完を行うための関数．
未対応プロジェクタ画素に隣接するプロジェクタ画素を走査し，
その中で2つ以上対応するカメラ画素があった場合，それらのカメラ画素の中で
本来未対応部分に入るべき位相値に尤も近いカメラ画素を対応させる
*/
void PHASESHIFT::interpolation()
{
	errorCount=0;
	// 投影領域の枠は対応が取れないので，走査しない
	for( int y = 1; y < PROJECTOR_HEIGHT-3; y++ ) {
		for( int x = 1; x < PROJECTOR_WIDTH-3; x++ ) {	
			int a = y * PROJECTOR_WIDTH + x;

			//if((*c->code_map).count(a) == 1){
			//	count++;
			//}
			//if( (*c->code_map).count(a) > 10 ){
			//	std::cout << (*c->code_map).count(a) << std::endl;
			//	std::cout << x << "," << y << std::endl;
			//	auto range = (*c->code_map).equal_range(a);
			//	for( auto it = range.first; it != range.second; it++ ){
			//		auto target = *it;
			//		std::cout << target.first << " : " << target.second << std::endl;
			//		cv::waitKey(0);
			//	}
			//}

			//-未対応のプロジェクタ画素があった場合
			if( (*c->point_map).find(a) == (*c->point_map).end() ){
				errorCount++;
				//-近傍画素から持ってくる関数
				cv::Point point = getInterpolatedPoint2(x,y);
				if( point.x != -1 && point.y != -1)
					//-更新
						c->p.Pointdata[point.y][point.x] = cv::Point(x, y);

			}
		}
	}

	std::cout << "対応エラー数：" << errorCount << std::endl;
	//std::cout << "all" << (*c->code_map).size() << std::endl;
}

/*
実際に近傍カメラ画素を走査する関数
カメラの座標値が返ってくる
*/
cv::Point PHASESHIFT::getInterpolatedPoint2(int x, int y)
{
	// 未対応画素がとりうる位相値を計算
	double x_ideal = (x+xplus)*2.0*PI/Lx;
	double y_ideal = (y+yplus)*2.0*PI/Ly;

	const int radius = 1;							//走査する半径
	double xtemp = 0.0, ytemp = 0.0, temp = 0.0;	//位相値を入れておくための変数
	double ave = 100.0;								//位相値の平均を入れるための変数
	cv::Point nearest_point = cv::Point(-1, -1);
	cv::Point prj_point = cv::Point(-1, -1);

	// radius分走査する
	for (int j = -radius; j <= radius; j++) {
		for (int i = -radius; i <= radius; i++) {
			int yj = j + y;
			int xi = i + x;

			if( j == 0 && i == 0 )
				continue;
			if ( ( 1 <= yj && yj < PROJECTOR_HEIGHT-1 ) && ( 1 <= xi && xi < PROJECTOR_WIDTH-1 ) ){
				int a = yj* PROJECTOR_WIDTH + xi;
				auto range = (*c->point_map).equal_range(a);
				//-近傍画素が未対応画素の場合
				if( (*c->point_map).find(a) ==  (*c->point_map).end() )
					continue;

				//-見つかった場合 && 複数存在する場合
				else if( (*c->point_map).count(a) > 1 ){
					for (auto iterator = range.first; iterator != range.second; iterator++){
						auto target = *iterator;
						//-理想的な位相値と着目画素の位相値の差分の平均を計算する
						xtemp = fabs(x_ideal - c->p.Phase_Value[target.second.y][target.second.x].x);
						ytemp = fabs(y_ideal - c->p.Phase_Value[target.second.y][target.second.x].y);
						temp = (xtemp + ytemp)/2.0;
						//-平均値が一番小さいなら更新し，その座標を保存
						if( temp < ave ){
							ave = temp;
							nearest_point = target.second;
							prj_point = cv::Point(xi, yj);
						}
					}
				}
			}
		}
	}

	//連想配列の更新(削除&挿入)
	if( nearest_point.x != -1  && nearest_point.y != -1 ){
		int code = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
		auto range = (*c->point_map).equal_range(code);
		auto iterator = range.first;

		//for (auto iterator = range.first; iterator != range.second; iterator++){
		//	auto target = *iterator;
		//	std::cout << "削除前の要素 : " << target.second << std::endl;
		//}

		//連想配列から削除
		while(iterator != range.second){
			if(iterator->second == nearest_point){
				(*c->point_map).erase(iterator++);
			}else ++iterator;

		}

		//auto range1 = (*c->code_map).equal_range(code);
		//for (auto iterator = range1.first; iterator != range1.second; iterator++){
		//	auto target = *iterator;
		//	std::cout << "削除後の要素 : " << target.second << std::endl;
		//}
		//cv::waitKey(0);

		//連想配列に挿入
		int a = y * PROJECTOR_WIDTH + x;
		(*c->point_map).insert( std::pair<int, cv::Point>(a, nearest_point) );
		return nearest_point;

	}else{
		//周囲に2画素以上対応する画素がなかった場合，補完しない
		return cv::Point(-1, -1);
	}
}


// (Nearest用)カメラ座標の数を計算(輝度値の平均用)
void PHASESHIFT::assign_pointdata()
{
	for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
		for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
			int code = y * PROJECTOR_WIDTH + x;
			c->CamPointNum[y][x] = (*c->point_map).count(code);
		}
	}

}


/***********************************
**           幾何変換             **
************************************/

#if USE_BILINEAR
void PHASESHIFT::reshapeCam2Proj(cv::Mat &cam, cv::Mat &prj)
{
	Timer tm;
	cv::Mat temp(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_32FC3);
	cv::Mat dst(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3);
	int x,y;
#pragma omp parallel for private(x)
	for(y = 0; y < PROJECTOR_HEIGHT; y++ ) {
		cv::Vec3f* temp_p = temp.ptr<cv::Vec3f>(y);
		cv::Vec3b* dst_p = dst.ptr<cv::Vec3b>(y);
		for(x = 0; x < PROJECTOR_WIDTH; x++ ) {
			int key = y * PROJECTOR_WIDTH + x;
			//存在する場合
			if( (*c->point_map).find(key) != (*c->point_map).end() ){
				auto range = (*c->point_map).equal_range(key);
				//距離の逆数による重み付け
				for(auto iterator = range.first; iterator != range.second; iterator++){
					auto target = *iterator;
					cv::Point p = target.second;
					cv::Vec3b* cam_p = cam.ptr<cv::Vec3b>(p.y);
					// 距離を計算
					double d_x = fabs(c->p.SubPixel_Pointdata[p.y][p.x].x - x);
					double d_y = fabs(c->p.SubPixel_Pointdata[p.y][p.x].y - y);
					double d = (d_x*d_x) + (d_y*d_y);
					//double area = d_x * d_y;

					if(!power)
						d = sqrt(d);

					double Reciprocal_d = 1.0/d;
					//double Reciprocal_d = 1.0/area;
					//std::cout << "p : " << p << std::endl;
					//std::cout << test_p[p.x] << std::endl;
					if( (p.x != -1) && (p.y != -1) ) {
						temp_p[x] += ( Reciprocal_d * (cv::Vec3f)cam_p[p.x] / c->Sum_Distance[y][x] );
					}
				}
				dst_p[x] = (cv::Vec3b)temp_p[x];
			}

		}
	}
	//std::cout << "PhaseShift 幾何補正:";
	//tm.elapsed();
	prj = dst.clone();
	temp.release();
	dst.release();

}
#else

//void PHASESHIFT::reshapeCam2Proj(cv::Mat &cam, cv::Mat &prj)
//{
//	cv::Mat temp_f(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_32FC3, cv::Scalar(0, 0, 0));
//	cv::Mat temp_c(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
//	cv::Mat normal_mask = cv::imread("./CalculationData/mask.bmp",0);
//	//	int x,y;
//	//	#pragma omp parallel for private(x)
//	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
//		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
//
//			//マスク外は何もしない
//			if(normal_mask.at<uchar>(y, x) == 0){
//				continue;
//			}
//			cv::Point p = c->p.Pointdata[y][x];
//			cv::Vec3b* cam_p = cam.ptr<cv::Vec3b>(y);
//			cv::Vec3f* temp_f_p = temp_f.ptr<cv::Vec3f>(p.y);
//			cv::Vec3b* temp_c_p = temp_c.ptr<cv::Vec3b>(y);
//			if( (p.x != -1) && (p.y != -1) ) {
//				temp_f_p[p.x] += c->Distance[y][x] * (cv::Vec3f)cam_p[x] / c->Sum_Distance[p.y][p.x];
//				//temp.at<uchar>( p.y, 3*p.x ) = src.at<uchar>( y, 3*x );      // B
//				//temp.at<uchar>( p.y, 3*p.x+1 ) = src.at<uchar>( y, 3*x+1 );  // G
//				//temp.at<uchar>( p.y, 3*p.x+2 ) = src.at<uchar>( y, 3*x+2 );  // R
//			}
//		}
//	}
//
//	for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
//		cv::Vec3f* temp_f_p = temp_f.ptr<cv::Vec3f>(y);
//		cv::Vec3b* temp_c_p = temp_c.ptr<cv::Vec3b>(y);
//		for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
//			temp_c_p[x] = (cv::Vec3b)temp_f_p[x];
//			//		for( int i = 0; i < temp.channels(); i++)
//			//			dst.data[y * dst.step + x * dst.elemSize() + i] = (uchar)temp.data[y * temp.step + x * temp.elemSize() + i];
//
//		}
//	}
//
//	prj = temp_c.clone();
//}

//対応画素の輝度値の平均を用いて幾何変換
void PHASESHIFT::reshapeCam2Proj(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat temp_f(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_32FC3, cv::Scalar(0, 0, 0));
	cv::Mat temp_c(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat normal_mask = cv::imread("./PhaseShift/GrayCode_Mask.bmp",0);

	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {

			//マスク外は何もしない
			if(normal_mask.at<uchar>(y, x) == 0){
				continue;
			}
			cv::Point p = c->p.Pointdata[y][x];
			cv::Vec3b* src_p = src.ptr<cv::Vec3b>(y);
			cv::Vec3f* temp_f_p = temp_f.ptr<cv::Vec3f>(p.y);
			if( (p.x != -1) && (p.y != -1) ) {
				temp_f_p[p.x] += (cv::Vec3f)src_p[x];
				//temp.at<uchar>( p.y, 3*p.x ) = src.at<uchar>( y, 3*x );      // B
				//temp.at<uchar>( p.y, 3*p.x+1 ) = src.at<uchar>( y, 3*x+1 );  // G
				//temp.at<uchar>( p.y, 3*p.x+2 ) = src.at<uchar>( y, 3*x+2 );  // R
			}
		}
	}

	for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
		cv::Vec3f* temp_f_p = temp_f.ptr<cv::Vec3f>(y);
		cv::Vec3b* temp_c_p = temp_c.ptr<cv::Vec3b>(y);
		for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {

			if( c->CamPointNum[y][x] > 0 ){
				temp_f_p[x] = temp_f_p[x] / (float)c->CamPointNum[y][x];
			}

			temp_c_p[x] = (cv::Vec3b)temp_f_p[x];

			//		for( int i = 0; i < temp.channels(); i++)
			//			dst.data[y * dst.step + x * dst.elemSize() + i] = (uchar)temp.data[y * temp.step + x * temp.elemSize() + i];
		}
	}
	dst = temp_c.clone();

}


#endif

/**********************************************/
//	3次元再構成用　追加
/**********************************************/

// カメラ座標に対するプロジェクタの対応点を返す(高精度版)
void PHASESHIFT::getCorrespondSubPixelProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, int size)
{
	for (int i=0; i < imagePoint.size(); ++i)
	{
		std::vector<cv::Point2f> iPoints, pPoints;
		if(imagePoint[i].x > size && imagePoint[i].x+size < CAMERA_WIDTH && imagePoint[i].y > size && imagePoint[i].y+size < CAMERA_HEIGHT)
		{
			// 領域毎の対応点
			for( float h = imagePoint[i].y-size; h < imagePoint[i].y+size; h+=1.0f){
				for( float w = imagePoint[i].x-size; w < imagePoint[i].x+size; w+=1.0f){
					cv::Point2f point = c->p.SubPixel_Pointdata[int(h+0.5f)][int(w+0.5f)];
					if( point.x != -1.0f) {
						iPoints.emplace_back(cv::Point2f(w, h));
						pPoints.emplace_back(point);
					}
				}
			}

			// 対応点同士でHomographyの計算
			cv::Mat H = cv::findHomography(iPoints, pPoints, CV_RANSAC, 2.0);
			// Homographyを使ってチェッカーパターンの交点を射影
			cv::Point3d Q = cv::Point3d(cv::Mat(H * cv::Mat(cv::Point3d(imagePoint[i].x,imagePoint[i].y,1.0))));
			projPoint.emplace_back(cv::Point2f(Q.x/Q.z, Q.y/Q.z));
		}
		else
		{
			cv::Point2f point = c->p.SubPixel_Pointdata[int(imagePoint[i].y+0.5f)][int(imagePoint[i].x+0.5f)];

			if( point.x != -1.0f) {
				projPoint.emplace_back(point);
			}
		}
	}
}

// 対応のとれた点を全て返す
void PHASESHIFT::getCorrespondAllPoints(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<cv::Point3i> &pointColor)
{
	cv::Mat surface_image = cv::imread("./PhaseShift/PhaseImage/white.bmp",1);
	cv::cvtColor(surface_image, surface_image,CV_BGR2RGB);
	for( int y = 0; y < CAM_HEIGHT; y++ ) {
		for( int x = 0; x < CAM_WIDTH; x++ ) {
			cv::Point p = c->p.SubPixel_Pointdata[y][x];
			if( p.x != -1 ) {
				cv::Point3i color = (cv::Point3i)surface_image.at<cv::Vec3b>(y,x);
				projPoint.emplace_back(cv::Point2f(p.x, p.y));
				imagePoint.emplace_back(cv::Point2f(x, y));
				pointColor.emplace_back(color);
			}
		}
	}
}

// 対応のとれた点を全て返す
void PHASESHIFT::getCorrespondAllPoints2(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<cv::Point3i> &pointColor)
{
	cv::Mat surface_image = cv::imread("./PhaseShift/PhaseImage/white.bmp",1);
	cv::cvtColor(surface_image, surface_image,CV_BGR2RGB);
	for( int y = 0; y < CAM_HEIGHT; y++ ) {
		for( int x = 0; x < CAM_WIDTH; x++ ) {
			cv::Point p = c->p.SubPixel_Pointdata[y][x];
			//全ての座標値を格納
			imagePoint.emplace_back(cv::Point2f(x, y));
			cv::Point3i color = (cv::Point3i)surface_image.at<cv::Vec3b>(y,x);
			pointColor.emplace_back(color);
			if( p.x != -1 ) {
				projPoint.emplace_back(cv::Point2f(p.x, p.y));
			}else{
				projPoint.emplace_back(cv::Point2f(-1,-1));
			}
		}
	}
}

void PHASESHIFT::fileWrite(std::vector<cv::Point2i> &cameraPoint, int y)
{
	cameraPoint.clear();
	for(int x = 0; x < PROJECTOR_WIDTH; x++ ) {
		int key = y * PROJECTOR_WIDTH + x;
		//存在する場合
		if( (*c->point_map).find(key) != (*c->point_map).end() ){
			auto range = (*c->point_map).equal_range(key);
			for(auto iterator = range.first; iterator != range.second; iterator++){
				auto target = *iterator;
				//pはプロジェクタ座標に対応するカメラ座標
				cv::Point2i p = target.second;
				cameraPoint.emplace_back(p);
			}
		}

	}
	
}
