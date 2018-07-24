/***************************************************************************
** �}�E�X�h���b�O�ŃN�H�[�^�j�I�����g���ĉ�]�ł���悤�ɂ��郉�C�u����
** 
** �y�g�����z
** 1.�C���X�^���X����
** 2.�}�E�X�̃N���b�N���ƃh���b�O���ɌĂ΂��R�[���o�b�N�֐���mouseClick��mouseMotion���ĂԂ悤�ɂ��Ă���
** 3.glutIdleFunc�ŃA�C�h����Ԏ���glutPostRedisplay�����s����悤�ɂ��Ă���
** 4.Display�֐��̓K���ȂƂ���ōs��rt��glMultMatrixd�Ŋ|����
** 5.�N�����ă}�E�X�Ńh���b�O����Ɖ�]����
**
** ��{�I�ɂ́A�ȉ��̃y�[�W�̃R�[�h��ėp�I�Ɏg���񂹂�悤�ɂ������̂ł��B
** natural science VisualC++ ���g���� OpenGL ����
** http://www.natural-science.or.jp/article/20091124233406.php
**
** @���j ��t
**
** �X�V����
**  - 2014/06/27
**		�E���s�ړ��ǉ�
**		�E�S�̓I���Y��ɂ���
**		�@�ꕔ�֐����␧����ύX�����̂Œ���
****************************************************************************/

#ifndef QUATERNION_H
#define QUATERNION_H

// freeglut�����Ė����ꍇ�͂�����glut�ɏ���������
// freeglut���ƃz�C�[���̉�]���F���ł���
#include <GL/freeglut.h>
#include <math.h>
#include <iostream>


class Quaternion
{
public:
	Quaternion();
	~Quaternion();

	// �������i�ŏ��ɕK���Ăԁj
	void initialize();
	// ��]�����s����֐��i������true�ŉ�]���S�ɓ_��`��j
	void multMatrix(bool drawPoint = false);
	// ��]�̕ϊ��s��iglMultMatrixd�Ŋ|����s��j
	double rt[16];


	/********************
	setter
	********************/
	// ���a
	template<typename T1>
	void setRadius(T1 _Radius);
	// �}�E�X�z�C�[����]���̊g�k�̊Ԋu
	template<typename T1>
	void setDR(T1 _dR);
	// ��]�̒��S���W
	template<typename T1>
	void setTranslate(T1 translate[3]);
	template<typename T1, typename T2, typename T3>
	void setTranslate(T1 _x, T2 _y, T3 _z);
	// 1px�ړ��ɑ΂��镽�s�ړ��̈ړ���
	template<typename T1>
	void setTranslateVelocity(T1 _velocity);
	// �N�H�[�^�j�I���̏����l
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
	�}�E�X�̊e�R�[���o�b�N�֐����ŌĂԊ֐�
	********************/
	void mouseMotion(int x, int y);
	void mouseClick(int button, int state, int x, int y);
	void mouseWheel(int wheel_number, int direction, int x, int y);
	void mouseMotionTranslate(int x, int y);
	void mouseClickTranslate(int button, int state, int x, int y);


	/********************
	���̑��̊֐�
	********************/
	// �N�H�[�^�j�I�����R���\�[���ɕ\������֐�
	void showQuaternion();
	// �E�B���h�E�T�C�Y�ύX���ɌĂԊ֐�
	void reshapeWindow();
	// �P�ʃN�H�[�^�j�I���ɖ߂��֐�
	void resetQuaternion();
	// setQuaternion�œǂݍ��񂾃N�H�[�^�j�I���ɖ߂��֐�
	void resetQuaternionEx();
	// �N�H�[�^�j�I�����R�s�[����֐�
	void copy(Quaternion src);
	// �N�H�[�^�j�I����ۑ�����֐�
	bool saveQuaternion(FILE *fp);
	// �N�H�[�^�j�I����ǂݍ��ފ֐�
	bool loadQuaternion(FILE *fp);

private:
	/********************
	�萔
	********************/
	static const long PI = (long)3.1415926535;
	static const long SCALE = (long)(2.0 * PI);	// �}�E�X�̑��Έʒu����]�p�̊��Z�W��
	double sx, sy;	// �}�E�X�̐�Έʒu���E�B���h�E���ł̑��Έʒu�̊��Z�W��

	/********************
	��]�p
	********************/
	double Radius;	// ���_����̋���
	double dR;		// �z�C�[�����񂵂��Ƃ��ɓ�������
	bool onRotate;

	int cx, cy;		// �h���b�O�J�n�ʒu
	double cq[4];	// ��]�J�n�̏����l (�N�H�[�^�j�I��)
	double tq[4];	// �h���b�O���̉�] (�N�H�[�^�j�I��)
	double initQuaternion[4];	// setQuaternion�Őݒ肵���N�H�[�^�j�I��

	void qmul(double r[], const double p[], const double q[]);
	void qrot(double r[], double q[]);

	/********************
	���s�ړ��p
	********************/
	bool onTranslate;
	int velocity;
	int tcx, tcy;		// �h���b�O�J�n�ʒu
	double translate[3];	// �ړ�����
	double initTranslate[3];	// setQuaternion�Őݒ肵���N�H�[�^�j�I��
};

/*
** �C���X�^���X�������ɃE�B���h�E�̃T�C�Y��^����
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

// �f�X�g���N�^
inline Quaternion::~Quaternion(){
}


/*
** ������������֐�
** glutMainLoop���ĂԑO�ɌĂ�ł���
*/
inline void Quaternion::initialize(){
	// �f�v�X�o�b�t�@���g�p�FglutInitDisplayMode() �� GLUT_DEPTH ���w�肷��
	glEnable(GL_DEPTH_TEST);

	// �}�E�X�|�C���^�ʒu�̃E�B���h�E���̑��ΓI�ʒu�ւ̊��Z�p
	sx = 1.0 / (double)glutGet(GLUT_INIT_WINDOW_WIDTH);
	sy = 1.0 / (double)glutGet(GLUT_INIT_WINDOW_HEIGHT);

	// ��]�s��̏�����
	qrot(rt, cq);
}

/*
** �s��̊|���Z�����ĉ�]�����s����֐�
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
		std::cout << "dR�͐��̒l�����ĉ�����" << std::endl;
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
		std::cout << "velocity�͐��̒l�����ĉ�����" << std::endl;
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
** �}�E�X�h���b�O���ɃN�H�[�^�j�I�������߂�
*/
inline void Quaternion::mouseMotion(int x, int y){
	if(!onRotate)
		return;

	double dx, dy, a;

	// �}�E�X�|�C���^�̈ʒu�̃h���b�O�J�n�ʒu����̕ψ�
	dx = (x - cx) * sx;
	dy = (y - cy) * sy;

	// �}�E�X�|�C���^�̈ʒu�̃h���b�O�J�n�ʒu����̋���
	a = sqrt(dx * dx + dy * dy);

	if( a != 0.0 )
	{
		// �}�E�X�̃h���b�O�ɔ�����]�̃N�H�[�^�j�I�� dq �����߂�
		double ar = a * SCALE * 0.5;
		double as = sin(ar) / a;
		double dq[4] = { cos(ar), dy * as, dx * as, 0.0 };

		// ��]�̏����l cq �� dq ���|���ĉ�]������
		qmul(tq, dq, cq);

		// �N�H�[�^�j�I�������]�̕ϊ��s������߂�
		qrot(rt, tq);
	}
}

/*
** �}�E�X�N���b�N���̍��W�ƃN�H�[�^�j�I�����擾
*/
inline void Quaternion::mouseClick(int button, int state, int x, int y){
	switch (state) {
	case 0:
		onRotate = true;
		// �h���b�O�J�n�_���L�^
		cx = x;
		cy = y;
		break;
	case 1:
		onRotate = false;
		// ��]�̕ۑ�
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
** �}�E�X�z�C�[���Ŋg��k��
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
** �}�E�X�h���b�O���ɕ��s�ړ����s��
*/
inline void Quaternion::mouseMotionTranslate(int x, int y){
	if(!onTranslate)
		return;

	double dx, dy, a;

	// �}�E�X�|�C���^�̈ʒu�̃h���b�O�J�n�ʒu����̕ψ�
	dx = velocity * (x - tcx) * sx;
	dy = velocity * (y - tcy) * sy;

	// �ړ���̍��W���擾�i��Ɍ��݂̃}�E�X�|�C���^�ʒu���J�n�ʒu�Ɏg���j
	tcx = x;
	tcy = y;

	// �}�E�X�|�C���^�̈ʒu�̃h���b�O�J�n�ʒu����̋���
	a = sqrt(dx * dx + dy * dy);

	if( a != 0.0 )
	{
		translate[0] += dx;
		translate[1] -= dy;
	}
}

/*
** �}�E�X�N���b�N���̍��W���擾
*/
inline void Quaternion::mouseClickTranslate(int button, int state, int x, int y){
	switch (state) {
	case 0:
		onTranslate = true;
		// �h���b�O�J�n�_���L�^
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
** �R���\�[����ɃN�H�[�^�j�I����\������֐�
*/
inline void Quaternion::showQuaternion(){
	std::cout << "Quaternion: " << cq[0] << ", " << cq[1] << ", " << cq[2] << ", " << cq[3] << std::endl;
}

/*
** �E�B���h�E���̑��ΓI�ʒu���v�Z���Ȃ����֐�
** �E�B���h�E�T�C�Y�ύX���ɌĂ�
*/
inline void Quaternion::reshapeWindow(){
	sx = 1.0 / (double)glutGet(GLUT_WINDOW_WIDTH);
	sy = 1.0 / (double)glutGet(GLUT_WINDOW_HEIGHT);
}


/*
** �N�H�[�^�j�I��������������֐�
** ������ĂԂƍŏ��̌����ɖ߂�
*/
inline void Quaternion::resetQuaternion(){
	/* �P�ʃN�H�[�^�[�j�I�� */
	cq[0] = 1.0;
	cq[1] = 0.0;
	cq[2] = 0.0;
	cq[3] = 0.0;

	for(int i=0; i<3; i++)
		translate[i] = 0.0;

	// true�̂܂ܖ߂�Ȃ����Ƃ�����̂ŔO�̂��ߖ߂�
	onTranslate = false;
	onRotate = false;

	/* ��]�s��̏����� */
	qrot(rt, cq);
}

/*
** setQuaternion�Őݒ肵���N�H�[�^�j�I���ɖ߂��֐�
*/
inline void Quaternion::resetQuaternionEx(){
	for(int i=0; i<4; i++)
		cq[i] = initQuaternion[i];

	for(int i=0; i<3; i++)
		translate[i] = initTranslate[i];

	qrot(rt, cq);
}

/*
** �N�H�[�^�j�I���̒��g���R�s�[����֐�
** A.copy(B)��B�̒��g��A�ɃR�s�[�����
*/
inline void Quaternion::copy(Quaternion src){
	for(int i=0; i<4; i++)
		cq[i] = src.cq[i];
	for(int i=0; i<16; i++)
		rt[i] = src.rt[i];
}

/*
** �N�H�[�^�j�I���̒l��ۑ�����֐�
** �ۑ������l���g���Ƃ��́AloadMatrixFromQuaternion���g����
** �l��z��ɓ����setInitQuaternion�ɓn��
*/
inline bool Quaternion::saveQuaternion(FILE *fp){
	if(fp == NULL){
		std::cout << "SAVE ERROR�F�t�@�C�����쐬�ł��܂���ł���" << std::endl;
		return false;
	}

	// �N�H�[�^�j�I���̕ۑ�
	for(int i=0; i<4; i++){
		fprintf(fp, "%.5f", cq[i]);
		if(i == 3)
			fprintf(fp, "\n");
		else
			fprintf(fp, "\t");
	}
	// ��]���S�̕ۑ�
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
** �N�H�[�^�j�I�����t�@�C������ǂݍ���ŕϊ�����
*/
inline bool Quaternion::loadQuaternion(FILE *fp){
	if(fp == NULL){
		std::cout << "LOAD ERROR�F�t�@�C�����ǂݍ��߂܂���ł���" << std::endl;
		return false;
	}

	float _cq[4];
	float _tr[3];
	bool check = true;
	char buf[64];

	fgets(buf, sizeof(buf), fp );
	// �R�����g�Ƌ�s�͖�������
	while( strncmp(buf, "//", 2) == 0 || strcmp(buf, "\n") == 0 )
		fgets( buf, sizeof(buf), fp );

	// �N�H�[�^�j�I�����擾
	fgets(buf, sizeof(buf), fp );
	if(sscanf_s(buf, "%f %f %f %f", &_cq[0], &_cq[1], &_cq[2], &_cq[3] ) != 4){
		check = false;
	};

	// ��]���S���擾
	fgets(buf, sizeof(buf), fp );
	if(sscanf_s(buf, "%f %f %f", &_tr[0], &_tr[1], &_tr[2] ) != 3){
		check = false;
	};

	// �G���[����
	if(!check){
		std::cout << "LOAD ERROR�F�������Ԉ���Ă��܂�" << std::endl;
		std::cout << "LOAD ERROR�F1�s�ڂɔ��a�A2�s�ڂɃN�H�[�^�j�I���i4�̒l�j���L�q���ĉ�����" << std::endl;
		return false;
	}

	setQuaternion(_cq);
	setTranslate(_tr);
	return true;
}


//////////////////////////////////////////////////////////////////////////
// ���ۂɃN�H�[�^�j�I�������߂�֐�
//////////////////////////////////////////////////////////////////////////

// �N�H�[�^�j�I���̐� r <- p x q
inline void Quaternion::qmul(double r[], const double p[], const double q[])
{
	r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
	r[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
	r[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
	r[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

// ��]�̕ϊ��s�� r <- �N�H�[�^�j�I�� q
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