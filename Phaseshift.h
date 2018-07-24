#ifndef PHASESHIFT_H
#define PHASESHIFT_H

// GrayCode��p�����􉽕␳

// �O���C�R�[�h�́C�c�C���ʂɍ쐬���C��ō���
// �p�^�[���摜�̓r�b�g���Z��p���č쐬�i������char�^�͎g��Ȃ��j
// �p�^�[���摜��1������������ŕۑ�
#include <iostream>
#include <sstream>
#include <fstream>
#include <windows.h>
#include <iomanip>  // ������X�g���[��
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
@brief GrayCode��p�����􉽕␳<br>
�Ή��t�����Ȃ�������f�ɂ͋ߖT�̉�f���R�s�[���ĕ��<br>
�ŏI�I�ȃv���W�F�N�^�ƃJ������Ή��t�����z���c->CamPro�Ɋi�[�����B
*/
class PHASESHIFT{
public:
	static const int PRJ_HEIGHT = PROJECTOR_HEIGHT;
	static const int PRJ_WIDTH = PROJECTOR_WIDTH;
	static const int CAM_HEIGHT = CAMERA_HEIGHT;
	static const int CAM_WIDTH = CAMERA_WIDTH;
	
	// �����g�̎���
	static const int Lx = 10; //<! x�������̐����g�̔g��
	static const int Ly = 20; //<! y�������̐����g�̔g��
	
	// �v���W�F�N�^�𑜓x
	static const int WaveNumX = PROJECTOR_WIDTH/Lx;		//<! x�������̐����g�̔g�̐�
	static const int WaveNumY = PROJECTOR_HEIGHT/Ly;	//<! y�������̐����g�̔g�̐�

	int delay;				//<! �x����
	int ThresholdValue;		//<! �O���C�R�[�h2�l������臒l
	int shatter_ps;			//<! �����g���e���̃V���b�^�[�X�s�[�h
	int errorCount;			//<! �Ή��G���[��

	// �O���C�R�[�h�쐬�ɕK�v�ȍ\����
	typedef struct _Graycode {
		int GrayCode[WaveNumY][WaveNumX];			//<! �g���ɍ��킹���O���C�R�[�h�i�g�̐��̉𑜓x[����][��]�j
		int PrjSize_GrayCode[PROJECTOR_HEIGHT][PROJECTOR_WIDTH];	//<! �O���C�R�[�h�i�v���W�F�N�^�𑜓x[����][��]�j
		unsigned int h_bit, w_bit;							//<! �����C���̕K�v�r�b�g��
		unsigned int all_bit;								//<! ���v�r�b�g���ih_bit + w_bit�j
	} Graycode;

	// �ʑ��V�t�g�쐬�ɕK�v�ȍ\����
	typedef struct _PhaseShift {
		cv::Point2d Phase_Value[CAMERA_HEIGHT][CAMERA_WIDTH];			//<! �ʑ��l���i�[
		cv::Point2d Phase_Value_copy[CAMERA_HEIGHT][CAMERA_WIDTH];		//<! �ʑ��l���i�[(�摜�p)
		cv::Point	Pointdata[CAMERA_HEIGHT][CAMERA_WIDTH];				//<! �J������f�ɑΉ�����v���W�F�N�^��f�̍��W���i�[
		cv::Point2d SubPixel_Pointdata[CAMERA_HEIGHT][CAMERA_WIDTH];	//<! �T�u�s�N�Z�����肳�ꂽ�v���W�F�N�^���W���i�[
		cv::Point2d GT_Phase_Val[CAMERA_HEIGHT][CAMERA_WIDTH];			//<! Ground Truth
	} PhaseShft;

	// �v���W�F�N�^ - �J�����Ή��ɕK�v�ȍ\����
	typedef struct _correspondence {
		int Decoded_GrayCode[CAMERA_HEIGHT][CAMERA_WIDTH];		//<! �O���C�R�[�h���f�R�[�h�������̂��J������f�Ɋi�[
		int CamPointNum[PROJECTOR_HEIGHT][PROJECTOR_WIDTH];		//<! �v���W�F�N�^1��f���w���Ă���J������f�̐����i�[
		double Distance[CAMERA_HEIGHT][CAMERA_WIDTH];			//<! �Ή�����v���W�F�N�^��f�܂ł̋������i�[
		double Sum_Distance[PROJECTOR_HEIGHT][PROJECTOR_WIDTH];	//<! �⊮�ɗ��p����J������f�Ƃ̋����̑��a
		std::map<int, cv::Point> *g_code_map;
		std::multimap<int, cv::Point> *point_map;
		Graycode g;
		PhaseShft p;
	} correspondence;

	correspondence *c;

	PHASESHIFT();
	~PHASESHIFT();

	// �֐�
	void code_projection();		//<! �O���C�R�[�h&�����g�𓊉e
	void make_thresh();			//<! 2�l������
	void makeCorrespondence();	//<! �􉽑Ή��t�����s��

	/// �􉽕ϊ�����֐�
	void reshapeCam2Proj(cv::Mat &cam, cv::Mat &prj);		//<! �􉽕ϊ��֐����I�[�o�[���C�h

	// �J�������W�ɑ΂���v���W�F�N�^�̑Ή��_��Ԃ�(�����x��)
	void getCorrespondSubPixelProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, int size);
	void getCorrespondAllPoints(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<cv::Point3i> &pointColor);
	void ThreeDimentionReconstruction();
	void getCorrespondAllPoints2(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<cv::Point3i> &pointColor);
	void fileWrite(std::vector<cv::Point2i> &cameraPoint, int y);
private:
	// �E�B���h�E�l�[��(OpenCV�p)
	char* CODE_IMG;

	Graycode *g;
	PhaseShft *p;

	void createDirs();			// �f�B���N�g���̍쐬

	/// �O���C�R�[�h�̍쐬�֘A
	void initCamera();				// �J�����̏�����
	void pattern_code_projection();	// �p�^�[���R�[�h���e & �B�e
	void makeGraycodeImage();		// �p�^�[���R�[�h�摜�쐬
	void initGraycode();			// �ʑ��V�t�g�p�O���C�R�[�h�쐬
	void initOnePixelGraycode();	// 1-bit���̃O���C�R�[�h�쐬

	/// ��l���֘A
	void makeMask(cv::Mat &mask);																	// �O���C�R�[�h�̉摜�𗘗p���ă}�X�N�𐶐�����֐�()
	void loadCam(cv::Mat &mat, int div_bin, bool flag, bool pattern);								// �J�����B�e�摜��ǂݍ��ފ֐�
	void makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue);	// �|�W�ƃl�K�̍����������MASK_THRESH�ȏ�̋P�x�̃s�N�Z���𔒂ɂ���
	void thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value );				// 2�l�������֐� 
	void smallMaskRange(cv::Mat &src, cv::Mat &dst);

	/// ���̑�
	void initCorrespondence();						// �v���W�F�N�^ - �J�����\���̏�����

	//// ��f��Ԏ�@
	void interpolation();							// ��f���
	cv::Point getInterpolatedPoint2(int x, int y);	// �אڂ����f���玝���Ă���

	/// �ʑ��V�t�g
	double	A, B;
	int		xplus;
	int		yplus;
	double	cLx;
	double	cLy;
	int maskcheck;
	cv::Mat mask;
	bool power;

	void initPhaseParameter();														// �ϐ��̏�����
	void setCorrectPhase();															// ���m�Ȉʑ��l
	void assign_pointdata();														// ���W�f�[�^�̊��蓖��
	void makePhasePattarn();														// �c�E���̐����g�p�^�[���쐬
	void readPhaseCam(cv::Mat phase_x_img[N], cv::Mat phase_y_img[N] , bool flag);	//�����g�̓ǂݍ���
	void restore_phase_value();														// �ʑ���������
	void reliableMeasuresFunc();													// �M�����]���֐�
	void bilinear(cv::Point2d &src, cv::Point2d &dst);								//���W�f�[�^�𑊑ΓI�Ɍ��肷��
	void errorCheck();
	void calcBilinear();
	void calcNearestNeighbor();
	void phaseConnection();															// �ʑ��A������
	void checkCorrespondence_for_Nearest(int num);										// �摜�ŉ�f�Ή��m�F�i�K�v�Ȃ��j
	void checkCorrespondence();														// �摜�ŉ�f�Ή��m�F�i�K�v�Ȃ��j
	void code_restore();															// 2�l���R�[�h����



	cv::Mat xphase_img;
	cv::Mat yphase_img;
};

#endif