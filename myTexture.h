#ifndef TEXTURE_H
#define TEXTURE_H

#pragma once
#include <GL/freeglut.h>
#include <opencv2/opencv.hpp>

class MyTexture{

public:
	MyTexture();
	~MyTexture();

	// 解像度を格納しておく
	int imgWidth, imgHeight;
	int movWidth, movHeight;
	// 画像と動画のアドレス
	std::string imgUrl, movUrl;
	// ウィンドウのフレームサイズ
	int frameWidth, frameHeight;
	// 最後にテクスチャに使った画像の解像度
	int nowTextureWidth, nowTextureHeight;

	bool loadImage(std::string imgFilename, GLuint* textureImg);
	// Mat型の画像データだけ取得する関数（テクスチャ化する前に何かしら処理加えたいときに使用）
	cv::Mat getImageMat(std::string imgFilename);
	void projectiveTextureMapping(bool toggle);

private:
	// 射影テクスチャマッピングのテクスチャ座標自動生成用配列
	double genfunc[4][4];

	bool loadImage(std::string imgFilename, GLuint* textureImg, bool mipmaps);
	void setTexture(cv::Mat &src, GLuint &textureID, bool mipmaps = 0);
	void surroundPixel(cv::Mat &src, cv::Mat &dst);
};


//***************************
//	以下各メソッドの処理
//***************************

// クラス作成時にここが最初に呼ばれる
inline MyTexture::MyTexture()
{
	//コンストラクタ
}

inline MyTexture::~MyTexture()
{
	// デストラクタ
}

/*
** 画像を読み込む
*/
inline bool MyTexture::loadImage(std::string imgFilename, GLuint* textureImg)
{
	return loadImage(imgFilename, textureImg, 0);
}

inline bool MyTexture::loadImage(std::string imgFilename, GLuint* textureImg, bool mipmaps)
{
	//画像を読み込む
	cv::Mat image = cv::imread(imgFilename, 1);
	if (image.data == NULL)
	{
		std::cerr << "ERROR：画像が読み込めません" << std::endl;
		image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));
		return false;
	}
	// ファイル名と解像度を保管しておく
	imgUrl = imgFilename;
	imgWidth = image.cols;
	imgHeight = image.rows;

	//画像周辺を白く塗りつぶす
	surroundPixel(image, image);

	cv::imwrite("./surround.jpg", image);

	//テクスチャを作成する
	setTexture(image, *textureImg, mipmaps);

	return true;
}

/*
** 画像の周辺を白くする
*/
inline void MyTexture::surroundPixel(cv::Mat &src, cv::Mat &dst)
{
	for (int y = 0; y < imgHeight; y++){
		for (int x = 0; x < imgWidth; x++){
			if (y == 0 || y == (imgHeight-1) || x == 0 || x == (imgWidth-1) ){
				src.at<uchar>(y, 3*x) = 0;
				src.at<uchar>(y, 3*x+1) = 0;
				src.at<uchar>(y, 3*x+2) = 0;
			}
		}
	}
	dst = src.clone();

}

/*
** テクスチャを生成する
*/
inline 	void MyTexture::setTexture(cv::Mat &src, GLuint &textureID, bool mipmaps)
{
	nowTextureWidth = src.cols;
	nowTextureHeight = src.rows;
	int format;
	// 色と向きをOpenGL用に修正
	if (src.channels() == 1)
		format = GL_LUMINANCE;
	else if (src.channels() == 4){
		format = GL_RGBA;
		cv::cvtColor(src, src, CV_BGRA2RGBA);
	}
	else{
		format = GL_RGB;
		cv::cvtColor(src, src, CV_BGR2RGB);
	}
	cv::flip(src, src, 0);
	// ポインタで受け取った変数にテクスチャ情報を入れる
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID); //指定した名前のテクスチャを有効化
	glTexImage2D(GL_TEXTURE_2D, 0, format, nowTextureWidth, nowTextureHeight, 0, format, GL_UNSIGNED_BYTE, src.data);
	if (mipmaps == 1)
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, nowTextureWidth, nowTextureHeight, format, GL_UNSIGNED_BYTE, src.data);

	/* テクスチャを拡大・縮小する方法の指定 */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	/* テクスチャの繰り返しの指定 */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	/* テクスチャ環境 */
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);	//初期値:GL_MODULATE：テクスチャ材質をポリゴンと合わせるか分離するかを決定する

	////バインドを解除
	//glBindTexture(GL_TEXTURE_2D, 0);
}

/*
** Mat型の画像データを取得する関数
** imgUrl, imgWidth, imgHeightにそれぞれアドレス, 幅, 高さが入っている
** テクスチャ化したいときはsetTextureExに渡す
*/
inline cv::Mat MyTexture::getImageMat(std::string imgFilename){
	cv::Mat image;

	image = cv::imread(imgFilename, -1);
	if (image.data == NULL)	{
		std::cerr << "ERROR：画像が読み込めません" << std::endl;
		image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));
	}
	// ファイル名と解像度を保管しておく
	imgUrl = imgFilename;
	imgWidth = image.cols;
	imgHeight = image.rows;
	return image;
}

/*
** 投影テクスチャマッピングを行うようにする関数
** 投影テクスチャマッピング行うオブジェクトを描画する関数をこの関数のtrueとfalseで挟む
*/
inline void MyTexture::projectiveTextureMapping(bool toggle){
	if (toggle){
		// テクスチャ座標自動生成用の行列を作成
		static double _genfunc[][4] = {
			{ 1.0, 0.0, 0.0, 0.0 },
			{ 0.0, 1.0, 0.0, 0.0 },
			{ 0.0, 0.0, 1.0, 0.0 },
			{ 0.0, 0.0, 0.0, 1.0 },
		};
		for (int i = 0; i<4; i++){
			for (int j = 0; j<4; j++){
				genfunc[i][j] = _genfunc[i][j];
			}
		}

		// 頂点のオブジェクト空間における座標値を使ってマッピングする
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

		// テクスチャ座標生成関数の設定
		glTexGendv(GL_S, GL_OBJECT_PLANE, genfunc[0]);
		glTexGendv(GL_T, GL_OBJECT_PLANE, genfunc[1]);
		glTexGendv(GL_R, GL_OBJECT_PLANE, genfunc[2]);
		glTexGendv(GL_Q, GL_OBJECT_PLANE, genfunc[3]);

		// テクスチャと重なるよう平行移動
		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glTranslated(0.5, 0.5, 0.0);
		//glScaled(0.5, 0.5, 1.0);

		/* モデルビュー変換行列の設定に戻す */
		glMatrixMode(GL_MODELVIEW);

		// テクスチャマッピング開始
		glEnable(GL_TEXTURE_2D);
		// テクスチャ座標の自動生成を有効にする
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);
		glEnable(GL_TEXTURE_GEN_Q);
	}
	else{
		// テクスチャ座標の自動生成を無効にする
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		glDisable(GL_TEXTURE_GEN_R);
		glDisable(GL_TEXTURE_GEN_Q);
		// テクスチャマッピング終了
		glDisable(GL_TEXTURE_2D);
	}
}



#endif