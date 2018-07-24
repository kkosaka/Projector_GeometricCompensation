/***************************************************************************
** マウスドラッグでクォータニオンを使って回転できるようにするライブラリ
** 
** 【使い方】
** 1.インスタンス生成
** 2.マウスのクリック時とドラッグ時に呼ばれるコールバック関数でmouseClickとmouseMotionを呼ぶようにしておく
** 3.glutIdleFuncでアイドル状態時にglutPostRedisplayを実行するようにしておく
** 4.Display関数の適当なところで行列rtをglMultMatrixdで掛ける
** 5.起動してマウスでドラッグすると回転する
**
** 基本的には、以下のページのコードを汎用的に使い回せるようにしたものです。
** natural science VisualC++ を使った OpenGL 入門
** http://www.natural-science.or.jp/article/20091124233406.php
**
** @小針 千春
**
** 更新履歴
**  - 2014/06/27
**		・平行移動追加
**		・全体的に綺麗にした
**		　一部関数名や制限を変更したので注意
****************************************************************************/

#ifndef QUATERNION_H
#define QUATERNION_H

// freeglutを入れて無い場合はここをglutに書き換える
// freeglutだとホイールの回転が認識できる
#include <GL/freeglut.h>
#include <math.h>
#include <iostream>


class Quaternion
{
public:
	Quaternion();
	~Quaternion();

	// 初期化（最初に必ず呼ぶ）
	void initialize();
	// 回転を実行する関数（引数にtrueで回転中心に点を描画）
	void multMatrix(bool drawPoint = false);
	// 回転の変換行列（glMultMatrixdで掛ける行列）
	double rt[16];


	/********************
	setter
	********************/
	// 半径
	template<typename T1>
	void setRadius(T1 _Radius);
	// マウスホイール回転時の拡縮の間隔
	template<typename T1>
	void setDR(T1 _dR);
	// 回転の中心座標
	template<typename T1>
	void setTranslate(T1 translate[3]);
	template<typename T1, typename T2, typename T3>
	void setTranslate(T1 _x, T2 _y, T3 _z);
	// 1px移動に対する平行移動の移動量
	template<typename T1>
	void setTranslateVelocity(T1 _velocity);
	// クォータニオンの初期値
	template<typename T1>
	void setQuaternion(T1 _Quaternion[4]);


	/********************
	getter
	********************/
	double getRadius();
	double getDR();
	template<typename T1>
	void getTranslate(T1 vec[3]);
	double getTranslateVelocity();
	template<typename T1>
	void getQuaternion(T1 quaternion[4]);


	/********************
	マウスの各コールバック関数内で呼ぶ関数
	********************/
	void mouseMotion(int x, int y);
	void mouseClick(int button, int state, int x, int y);
	void mouseWheel(int wheel_number, int direction, int x, int y);
	void mouseMotionTranslate(int x, int y);
	void mouseClickTranslate(int button, int state, int x, int y);


	/********************
	その他の関数
	********************/
	// クォータニオンをコンソールに表示する関数
	void showQuaternion();
	// ウィンドウサイズ変更時に呼ぶ関数
	void reshapeWindow();
	// 単位クォータニオンに戻す関数
	void resetQuaternion();
	// setQuaternionで読み込んだクォータニオンに戻す関数
	void resetQuaternionEx();
	// クォータニオンをコピーする関数
	void copy(Quaternion src);
	// クォータニオンを保存する関数
	bool saveQuaternion(FILE *fp);
	// クォータニオンを読み込む関数
	bool loadQuaternion(FILE *fp);

private:
	/********************
	定数
	********************/
	static const long PI = (long)3.1415926535;
	static const long SCALE = (long)(2.0 * PI);	// マウスの相対位置→回転角の換算係数
	double sx, sy;	// マウスの絶対位置→ウィンドウ内での相対位置の換算係数

	/********************
	回転用
	********************/
	double Radius;	// 原点からの距離
	double dR;		// ホイールを回したときに動く距離
	bool onRotate;

	int cx, cy;		// ドラッグ開始位置
	double cq[4];	// 回転開始の初期値 (クォータニオン)
	double tq[4];	// ドラッグ中の回転 (クォータニオン)
	double initQuaternion[4];	// setQuaternionで設定したクォータニオン

	void qmul(double r[], const double p[], const double q[]);
	void qrot(double r[], double q[]);

	/********************
	平行移動用
	********************/
	bool onTranslate;
	int velocity;
	int tcx, tcy;		// ドラッグ開始位置
	double translate[3];	// 移動距離
	double initTranslate[3];	// setQuaternionで設定したクォータニオン
};

/*
** インスタンス生成時にウィンドウのサイズを与える
*/
inline Quaternion::Quaternion(){
	Radius = 10;
	dR = 0.5;
	velocity = 10;
	cq[0] = initQuaternion[0] = 1.0;
	cq[1] = initQuaternion[1] = 0.0;
	cq[2] = initQuaternion[2] = 0.0;
	cq[3] = initQuaternion[3] = 0.0;
	initTranslate[0] = initTranslate[1] = initTranslate[2] = 0.0;
}

// デストラクタ
inline Quaternion::~Quaternion(){
}


/*
** 初期化をする関数
** glutMainLoopを呼ぶ前に呼んでおく
*/
inline void Quaternion::initialize(){
	// デプスバッファを使用：glutInitDisplayMode() で GLUT_DEPTH を指定する
	glEnable(GL_DEPTH_TEST);

	// マウスポインタ位置のウィンドウ内の相対的位置への換算用
	sx = 1.0 / (double)glutGet(GLUT_INIT_WINDOW_WIDTH);
	sy = 1.0 / (double)glutGet(GLUT_INIT_WINDOW_HEIGHT);

	// 回転行列の初期化
	qrot(rt, cq);
}

/*
** 行列の掛け算をして回転を実行する関数
*/
inline void Quaternion::multMatrix(bool drawPoint){
	if(drawPoint){
		glPushMatrix();
		glTranslated(translate[0], translate[1], translate[2]);
		glPointSize(10);
		glColor3d(0.3, 0.3, 0.3);
		glBegin(GL_POINTS);
		glVertex3d(0, 0, 0);
		glEnd();
		glColor3d(1.0, 1.0, 1.0);
		glPopMatrix();
	}

	glTranslated(translate[0], translate[1], translate[2]);
	glMultMatrixd(rt);
	glTranslated(-translate[0], -translate[1], -translate[2]);
}


/*
** setter 
*/
template<typename T1>
inline void Quaternion::setRadius(T1 _Radius){
	Radius = _Radius;
}
template<typename T1>
inline void Quaternion::setDR(T1 _dR){
	if(_dR <= 0){
		dR = 1.0;
		std::cout << "dRは正の値を入れて下さい" << std::endl;
		return;
	}
	dR = _dR;
}
template<typename T1>
inline void Quaternion::setTranslate(T1 _translate[3]){
	for(int i=0; i<3; i++){
		translate[i] = _translate[i];
		initTranslate[i] = _translate[i];
	}
}
template<typename T1, typename T2, typename T3>
inline void Quaternion::setTranslate(T1 _x, T2 _y, T3 _z){
	translate[0] = _x;
	translate[1] = _y;
	translate[2] = _z;
}
template<typename T1>
inline void Quaternion::setTranslateVelocity(T1 _velocity){
	if(_velocity <= 0){
		velocity = 1.0;
		std::cout << "velocityは正の値を入れて下さい" << std::endl;
		return;
	}
	velocity = _velocity;
}
template<typename T1>
inline void Quaternion::setQuaternion(T1 _Quaternion[4]){
	for(int i=0; i<4; i++){
		cq[i] = _Quaternion[i];
		initQuaternion[i] = _Quaternion[i];
	}
	qrot(rt, cq);
}


/*
** getter 
*/
inline double Quaternion::getRadius(){
	return Radius;
}
inline double Quaternion::getDR(){
	return dR;
}
inline double Quaternion::getTranslateVelocity(){
	return velocity;
}
template<typename T1>
inline void Quaternion::getTranslate(T1 vec[3]){
	vec[0] = translate[0];
	vec[1] = translate[1];
	vec[2] = translate[2];
}
template<typename T1>
inline void Quaternion::getQuaternion(T1 quaternion[4]){
	quaternion[0] = cq[0];
	quaternion[1] = cq[1];
	quaternion[2] = cq[2];
	quaternion[2] = cq[3];
}


/*
** マウスドラッグ時にクォータニオンを求める
*/
inline void Quaternion::mouseMotion(int x, int y){
	if(!onRotate)
		return;

	double dx, dy, a;

	// マウスポインタの位置のドラッグ開始位置からの変位
	dx = (x - cx) * sx;
	dy = (y - cy) * sy;

	// マウスポインタの位置のドラッグ開始位置からの距離
	a = sqrt(dx * dx + dy * dy);

	if( a != 0.0 )
	{
		// マウスのドラッグに伴う回転のクォータニオン dq を求める
		double ar = a * SCALE * 0.5;
		double as = sin(ar) / a;
		double dq[4] = { cos(ar), dy * as, dx * as, 0.0 };

		// 回転の初期値 cq に dq を掛けて回転を合成
		qmul(tq, dq, cq);

		// クォータニオンから回転の変換行列を求める
		qrot(rt, tq);
	}
}

/*
** マウスクリック時の座標とクォータニオンを取得
*/
inline void Quaternion::mouseClick(int button, int state, int x, int y){
	switch (state) {
	case 0:
		onRotate = true;
		// ドラッグ開始点を記録
		cx = x;
		cy = y;
		break;
	case 1:
		onRotate = false;
		// 回転の保存
		cq[0] = tq[0];
		cq[1] = tq[1];
		cq[2] = tq[2];
		cq[3] = tq[3];
		break;
	default:
		break;
	}
}

/*
** マウスホイールで拡大縮小
*/
inline void Quaternion::mouseWheel(int wheel_number, int direction, int x, int y){
	if(!onTranslate){
		switch(direction)
		{
		case +1:
			Radius -= dR;
			break;
		case -1:
			if(Radius <= 0){
				break;
			}
			Radius += dR;
			break;
		}
	}

	if(onTranslate){
		switch(direction)
		{
		case +1:
			translate[2] += dR;
			break;
		case -1:
			translate[2] -= dR;
			break;
		}
	}
}

/*
** マウスドラッグ時に平行移動を行う
*/
inline void Quaternion::mouseMotionTranslate(int x, int y){
	if(!onTranslate)
		return;

	double dx, dy, a;

	// マウスポインタの位置のドラッグ開始位置からの変位
	dx = velocity * (x - tcx) * sx;
	dy = velocity * (y - tcy) * sy;

	// 移動後の座標を取得（常に現在のマウスポインタ位置を開始位置に使う）
	tcx = x;
	tcy = y;

	// マウスポインタの位置のドラッグ開始位置からの距離
	a = sqrt(dx * dx + dy * dy);

	if( a != 0.0 )
	{
		translate[0] += dx;
		translate[1] -= dy;
	}
}

/*
** マウスクリック時の座標を取得
*/
inline void Quaternion::mouseClickTranslate(int button, int state, int x, int y){
	switch (state) {
	case 0:
		onTranslate = true;
		// ドラッグ開始点を記録
		tcx = x;
		tcy = y;
		break;
	case 1:
		onTranslate = false;
		break;
	default:
		break;
	}
}

/*
** コンソール上にクォータニオンを表示する関数
*/
inline void Quaternion::showQuaternion(){
	std::cout << "Quaternion: " << cq[0] << ", " << cq[1] << ", " << cq[2] << ", " << cq[3] << std::endl;
}

/*
** ウィンドウ内の相対的位置を計算しなおす関数
** ウィンドウサイズ変更時に呼ぶ
*/
inline void Quaternion::reshapeWindow(){
	sx = 1.0 / (double)glutGet(GLUT_WINDOW_WIDTH);
	sy = 1.0 / (double)glutGet(GLUT_WINDOW_HEIGHT);
}


/*
** クォータニオンを初期化する関数
** これを呼ぶと最初の向きに戻る
*/
inline void Quaternion::resetQuaternion(){
	/* 単位クォーターニオン */
	cq[0] = 1.0;
	cq[1] = 0.0;
	cq[2] = 0.0;
	cq[3] = 0.0;

	for(int i=0; i<3; i++)
		translate[i] = 0.0;

	// trueのまま戻らないことがあるので念のため戻る
	onTranslate = false;
	onRotate = false;

	/* 回転行列の初期化 */
	qrot(rt, cq);
}

/*
** setQuaternionで設定したクォータニオンに戻す関数
*/
inline void Quaternion::resetQuaternionEx(){
	for(int i=0; i<4; i++)
		cq[i] = initQuaternion[i];

	for(int i=0; i<3; i++)
		translate[i] = initTranslate[i];

	qrot(rt, cq);
}

/*
** クォータニオンの中身をコピーする関数
** A.copy(B)でBの中身がAにコピーされる
*/
inline void Quaternion::copy(Quaternion src){
	for(int i=0; i<4; i++)
		cq[i] = src.cq[i];
	for(int i=0; i<16; i++)
		rt[i] = src.rt[i];
}

/*
** クォータニオンの値を保存する関数
** 保存した値を使うときは、loadMatrixFromQuaternionを使うか
** 値を配列に入れてsetInitQuaternionに渡す
*/
inline bool Quaternion::saveQuaternion(FILE *fp){
	if(fp == NULL){
		std::cout << "SAVE ERROR：ファイルが作成できませんでした" << std::endl;
		return false;
	}

	// クォータニオンの保存
	for(int i=0; i<4; i++){
		fprintf(fp, "%.5f", cq[i]);
		if(i == 3)
			fprintf(fp, "\n");
		else
			fprintf(fp, "\t");
	}
	// 回転中心の保存
	for(int i=0; i<3; i++){
		fprintf(fp, "%.5f", translate[i]);
		if(i == 2)
			fprintf(fp, "\n");
		else
			fprintf(fp, "\t");
	}
	return true;
}

/*
** クォータニオンをファイルから読み込んで変換する
*/
inline bool Quaternion::loadQuaternion(FILE *fp){
	if(fp == NULL){
		std::cout << "LOAD ERROR：ファイルが読み込めませんでした" << std::endl;
		return false;
	}

	float _cq[4];
	float _tr[3];
	bool check = true;
	char buf[64];

	fgets(buf, sizeof(buf), fp );
	// コメントと空行は無視する
	while( strncmp(buf, "//", 2) == 0 || strcmp(buf, "\n") == 0 )
		fgets( buf, sizeof(buf), fp );

	// クォータニオンを取得
	fgets(buf, sizeof(buf), fp );
	if(sscanf_s(buf, "%f %f %f %f", &_cq[0], &_cq[1], &_cq[2], &_cq[3] ) != 4){
		check = false;
	};

	// 回転中心を取得
	fgets(buf, sizeof(buf), fp );
	if(sscanf_s(buf, "%f %f %f", &_tr[0], &_tr[1], &_tr[2] ) != 3){
		check = false;
	};

	// エラー処理
	if(!check){
		std::cout << "LOAD ERROR：書式が間違っています" << std::endl;
		std::cout << "LOAD ERROR：1行目に半径、2行目にクォータニオン（4つの値）を記述して下さい" << std::endl;
		return false;
	}

	setQuaternion(_cq);
	setTranslate(_tr);
	return true;
}


//////////////////////////////////////////////////////////////////////////
// 実際にクォータニオンを求める関数
//////////////////////////////////////////////////////////////////////////

// クォータニオンの積 r <- p x q
inline void Quaternion::qmul(double r[], const double p[], const double q[])
{
	r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
	r[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
	r[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
	r[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

// 回転の変換行列 r <- クォータニオン q
inline void Quaternion::qrot(double r[], double q[]){
	double x2 = q[1] * q[1] * 2.0;
	double y2 = q[2] * q[2] * 2.0;
	double z2 = q[3] * q[3] * 2.0;
	double xy = q[1] * q[2] * 2.0;
	double yz = q[2] * q[3] * 2.0;
	double zx = q[3] * q[1] * 2.0;
	double xw = q[1] * q[0] * 2.0;
	double yw = q[2] * q[0] * 2.0;
	double zw = q[3] * q[0] * 2.0;

	r[ 0] = 1.0 - y2 - z2;
	r[ 1] = xy + zw;
	r[ 2] = zx - yw;
	r[ 4] = xy - zw;
	r[ 5] = 1.0 - z2 - x2;
	r[ 6] = yz + xw;
	r[ 8] = zx + yw;
	r[ 9] = yz - xw;
	r[10] = 1.0 - x2 - y2;
	r[ 3] = r[ 7] = r[11] = r[12] = r[13] = r[14] = 0.0;
	r[15] = 1.0;
}

#endif