#include "Phaseshift.h"

PHASESHIFT::PHASESHIFT()
{
	CODE_IMG = "CodeImage";
	char buf[128];

	//�����l
	delay = 200;
	ThresholdValue = 17;
	// parameter.ini����ǂݍ���
	GetPrivateProfileStringA("delay", "delay", NULL, buf, sizeof(buf), "./parameter.ini");
	delay = atoi(buf) * 2;
	GetPrivateProfileStringA("graycode", "threshold ", NULL, buf, sizeof(buf), "./parameter.ini");
	ThresholdValue = atoi(buf);


	// �\���̂̏�����
	g = new Graycode();
	p = new PhaseShft();
	c = new correspondence();
	c->g_code_map = new std::map<int, cv::Point>();
	c->point_map = new std::multimap< int, cv::Point>();

	// phase shift�p n-bit�� �O���C�R�[�h
	c->g.h_bit = (int)ceil( log(WaveNumY+1) / log(2) );
	c->g.w_bit = (int)ceil( log(WaveNumX+1) / log(2) );
	c->g.all_bit = c->g.h_bit + c->g.w_bit;

	// �����g�����ݒ�
	xplus = (int)(Lx/2)+1;
	yplus = (int)(Ly/2)+1;
	A = 80.0;	// �U��
	B = 160.0;	//�o�C�A�X�����i��グ�j

	// �����g����}�X�N�����Ƃ���臒l
	maskcheck = 40;

	createDirs();

	//�ݏ�
	power = false;

	//��f�Ή��G���[���̏�����
	errorCount = 0;
}
PHASESHIFT::~PHASESHIFT()
{
}

// �f�B���N�g���̍쐬
void PHASESHIFT::createDirs()
{
	_mkdir("./PhaseShift");
	// �O���C�R�[�h�p
	_mkdir("./PhaseShift/GrayCodeImage");
	_mkdir("./PhaseShift/GrayCodeImage/CaptureImage");			// �O���C�R�[�h�B�e�摜
	_mkdir("./PhaseShift/GrayCodeImage/ProjectionGrayCode");	// �O���C�R�[�h���摜
	_mkdir("./PhaseShift/GrayCodeImage/ThresholdImage");			// �O���C�R�[�h�B�e�摜�̓�l�������摜
	// �ʑ��V�t�g�p
	_mkdir("./PhaseShift/PhaseImage");
	_mkdir("./PhaseShift/PhaseImage/CaptureImage");				// �����g�B�e�摜
	_mkdir("./PhaseShift/PhaseImage/ProjectionImage");		// �����g���e�摜
	_mkdir("./PhaseShift/PhaseImage/TestImage");		// �����g���e�摜
}

// �摜�쐬 & ���e
void PHASESHIFT::code_projection()
{
	makeGraycodeImage();		//GrayCode�쐬
	makePhasePattarn();			//�����g�摜�쐬
	pattern_code_projection();	//GrayCode���e
}

// ������
void PHASESHIFT::makeCorrespondence()
{
	std::cout << "�􉽑Ή��擾�J�n" << std::endl;
	mask = cv::imread("./PhaseShift/GrayCode_Mask.bmp",0); // �}�X�N�摜�̓ǂݍ���
	//������
	initCorrespondence();
	initPhaseParameter();
	//��������
	code_restore();
	//�􉽑Ή��}�b�v�̍쐬
	if(USE_BILINEAR){
		std::cout << "Bilinear�⊮�J�n" << std::endl;
		calcBilinear();
	}else{
		std::cout << "Nearest�⊮�J�n" << std::endl;
		//�m�F�p�̉摜
		checkCorrespondence_for_Nearest(0); //�⊮�O
		//std::cout << "�⊮�J�n" << std::endl;
		interpolation();
		//std::cout << "�⊮�I��" << std::endl;
		checkCorrespondence_for_Nearest(1); //�⊮��
		calcNearestNeighbor();
	}
	std::cout << "�􉽑Ή��擾�I��" << std::endl;
	//(Debug�p)�􉽕ϊ��̊m�F
	checkCorrespondence();
}

/****************************************
** �O���C�R�[�h & �ʑ��V�t�g�̍쐬�֘A **
****************************************/

// �v���W�F�N�^ - �J�����\���̏�����
void PHASESHIFT::initCorrespondence()
{
	// �O���C�R�[�h�̍쐬
	initGraycode();
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ){
			c->Decoded_GrayCode[y][x] = 0;
		}
	}
}

// �g���ɍ��킹���r�b�g���̌v�Z�ƃO���C�R�[�h�̍쐬
void PHASESHIFT::initGraycode()
{
	int bin_code_h[WaveNumY];  // 2�i�R�[�h�i�c�j
	int bin_code_w[WaveNumX];   // 2�i�R�[�h�i���j
	int graycode_h[WaveNumY];  // �O���C�R�[�h�i�c�j
	int graycode_w[WaveNumX];   // �O���C�R�[�h�i���j
	//int *graycode_h =  new int[c->g.h_bit];  // �O���C�R�[�h�i�c�j
	//int *graycode_w =  new int[c->g.w_bit];  // �O���C�R�[�h�i���j

	/***** 2�i�R�[�h�쐬 *****/
	// �s�ɂ���
	for( int y = 0; y < WaveNumY; y++ ){
		bin_code_h[y] = y + 1;
	}
	// ��ɂ���
	for( int x = 0; x < WaveNumX; x++ )
		bin_code_w[x] = x + 1;

	/***** �O���C�R�[�h�쐬 *****/
	// �s�ɂ���
	for( int y = 0; y < WaveNumY; y++ )
		graycode_h[y] = bin_code_h[y] ^ ( bin_code_h[y] >> 1 );
	// ��ɂ���
	for( int x = 0; x < WaveNumX; x++ )
		graycode_w[x] = bin_code_w[x] ^ ( bin_code_w[x] >> 1 );
	// �s������킹��i�s + ��j
	for( int y = 0; y < WaveNumY; y++ ) {
		for( int x = 0; x < WaveNumX; x++ ) {
			c->g.GrayCode[y][x] = ( graycode_h[y] << c->g.w_bit) | graycode_w[x];
			//�v���W�F�N�^�T�C�Y�̃O���C�R�[�h�z��쐬
			for( int i = 0; i < Ly; i++ ) 
				for( int j = 0; j < Lx; j++ ) 
					c->g.PrjSize_GrayCode[y*Ly+i][x*Lx+j] = c->g.GrayCode[y][x];
		}
	}
}

// (GrayCode)�p�^�[���R�[�h�摜�쐬
void PHASESHIFT::makeGraycodeImage()
{
	std::cout << "���e�p�O���C�R�[�h�쐬��" << std::endl;
	initGraycode();
	cv::Mat posi_img ( PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	cv::Mat nega_img ( PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	int bit = c->g.all_bit-1;
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit];  // �����t���o��
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];  // �����t���o��

	// �|�W�p�^�[���R�[�h�摜�쐬
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
			for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
				if( ( (c->g.PrjSize_GrayCode[y][x] >> (bit-z)) & 1 ) == 0 ) {  // �ŏ�ʃr�b�g���珇�ɒ��o���C���̃r�b�g��0��������
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
		// �A�ԂŃt�@�C������ۑ��i������X�g���[���j
		Filename_posi[z] << "./PhaseShift/GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_posi[z].str(), posi_img);
		Filename_posi[z] << std::endl;
	}

	// �l�K�p�^�[���R�[�h�摜�쐬
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
		// �A�ԂŃt�@�C������ۑ��i������X�g���[���j
		Filename_nega[z] << "./PhaseShift/GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_nega[z].str(), nega_img);
		Filename_nega[z] << std::endl;
	}

	delete[] Filename_posi;
	delete[] Filename_nega;
}

// �����g�摜�쐬
void PHASESHIFT::makePhasePattarn()
{
	initPhaseParameter();

	cv::Mat wave_img(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC1);
	int max=0, min=255,temp=0;
	char buf[256];
	// x������
	for(int i = 0; i < N; i++ ) {
		for(int x=0; x<PROJECTOR_WIDTH; x++){
			for(int y=0; y<PROJECTOR_HEIGHT; y++){
				wave_img.at<uchar>(y,x) = (uchar)( A * sin( 2.0*PI*((double)i/N + (double)(x+xplus)/Lx) ) + B );
				//-�ő�l�E�ŏ��l�̌v�Z
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
	// y������
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

// �p�^�[���R�[�h���e & �B�e
void PHASESHIFT::pattern_code_projection()
{
	// �萔
	typedef enum flag{
		POSI = true,
		NEGA = false,
		VERTICAL = true,
		HORIZONTAL = false,
	} flag;

	Graycode *g = new Graycode();

	TPGROpenCV	pgrOpenCV;
	char buf[256];

	//�����ݒ�
	if(pgrOpenCV.init( FlyCapture2::PIXEL_FORMAT_BGR, FlyCapture2::HQ_LINEAR) == -1){
		exit(0);
	}
	pgrOpenCV.setShutterSpeed(pgrOpenCV.getShutter_h());
	pgrOpenCV.start();

	// �S��ʕ\���p�E�B���h�E�̍쐬 
	cv::namedWindow(CODE_IMG, 0);
	Projection::MySetFullScrean(DISPLAY_NUMBER, CODE_IMG);

	/******* GrayCode�Ǎ� *********/

	//cv::Mat white = cv::imread("./UseImage/whitebar.bmp", 1);
	//cv::imshow(CODE_IMG, white);
	//cv::waitKey(0);

	cv::Mat *posi_img = new cv::Mat[c->g.all_bit];  // �|�W�p�^�[���p
	cv::Mat *nega_img = new cv::Mat[c->g.all_bit];  // �l�K�p�^�[���p

	// �����t���o�́i�O���C�R�[�h�ǂݍ��ݗp�j
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];
	// �����t���o�́i�B�e�摜�������ݗp�j
	std::stringstream *Filename_posi_cam = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega_cam = new std::stringstream[c->g.all_bit];

	// �A�ԂŃt�@�C������ǂݍ��ށi������X�g���[���j
	std::cout << "���e�p�O���C�R�[�h�摜�ǂݍ��ݒ�" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		Filename_posi[i] << "./PhaseShift/GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		Filename_nega[i] << "./PhaseShift/GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		//-�ǂݍ���
		posi_img[i] = cv::imread(Filename_posi[i].str(), 1);
		nega_img[i] = cv::imread(Filename_nega[i].str(), 1);
		Filename_posi[i] << std::endl;
		Filename_nega[i] << std::endl;
		//-�ǂݍ��ޖ���������Ȃ�������O���C�R�[�h�摜����蒼��
		if(posi_img[i].empty() || nega_img[i].empty()){
			std::cout << "ERROR(1)�F���e�p�̃O���C�R�[�h�摜���s�����Ă��܂��B" << std::endl;
			std::cout << "ERROR(2)�F�O���C�R�[�h�摜���쐬���܂��B" << std::endl;
			makeGraycodeImage();
			pattern_code_projection();
			return;
		}
	}

	/******* sin�g�Ǎ� *********/

	cv::Mat *phase_x_img = new cv::Mat[N];
	cv::Mat *phase_y_img = new cv::Mat[N];
	readPhaseCam(phase_x_img, phase_y_img, 0);


	/***** �O���C�R�[�h���e & �B�e *****/

	// �|�W�p�^�[�����e & �B�e
	std::cout << "�|�W�p�^�[���B�e��" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// ���e
		cv::imshow(CODE_IMG, posi_img[i]);
		// �J�����B�e
		cv::waitKey(delay);
		pgrOpenCV.queryFrame();
		cv::Mat cap = pgrOpenCV.getVideo();
		pgrOpenCV.showCapImg(cap);
		// �|�W�p�^�[���B�e���ʂ�ۑ�
		// ����
		if(i < c->g.h_bit)
			Filename_posi_cam[i] << "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << POSI << ".bmp"; 
		// �c��
		else
			Filename_posi_cam[i] << "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << POSI << ".bmp"; 
		//Filename_posi_cam[i] << "./output/Camera_posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		cv::imwrite(Filename_posi_cam[i].str(), cap);
		Filename_posi_cam[i] << std::endl;
	}

	// �l�K�p�^�[�����e & �B�e
	std::cout << "�l�K�p�^�[���B�e��" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// ���e
		cv::imshow(CODE_IMG, nega_img[i]);
		// �J�����B�e
		cv::waitKey(delay);
		pgrOpenCV.queryFrame();
		cv::Mat cap = pgrOpenCV.getVideo();
		pgrOpenCV.showCapImg(cap);

		// �|�W�p�^�[���B�e���ʂ�ێ�
		// ����
		if(i < c->g.h_bit)
			Filename_nega_cam[i] << "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << NEGA << ".bmp"; 
		// �c��
		else
			Filename_nega_cam[i] << "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << NEGA << ".bmp"; 
		//Filename_nega_cam[i] << "./output/Camera_nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		cv::imwrite(Filename_nega_cam[i].str(), cap);
		Filename_nega_cam[i] << std::endl;
	}

	/***** �����g ���e & �B�e *****/
	pgrOpenCV.setShutterSpeed((float)pgrOpenCV.getShutter_h());
	cv::waitKey(delay);

	std::cout << "�����g�p�^�[���B�e��" << std::endl;
	//������
	for(int i = 0; i < N; i++ ) {
		// ���e
		cv::imshow(CODE_IMG, phase_x_img[i]);
		// �J�����B�e
		cv::waitKey(delay);
		pgrOpenCV.queryFrame();
		cv::Mat cap = pgrOpenCV.getVideo();
		pgrOpenCV.showCapImg(cap);
		// �p�^�[���B�e���ʂ�ۑ�
		sprintf_s(buf, "./PhaseShift/PhaseImage/CaptureImage/x_patarn%02d.bmp", i);
		cv::imwrite(buf,cap);
	}

	//�c����
	for(int i = 0; i < N; i++ ) {
		// ���e
		cv::imshow(CODE_IMG, phase_y_img[i]);

		// �J�����B�e & �ۑ�
		cv::waitKey(delay);
		pgrOpenCV.queryFrame();
		cv::Mat cap = pgrOpenCV.getVideo();
		pgrOpenCV.showCapImg(cap);
		sprintf_s(buf, "./PhaseShift/PhaseImage/CaptureImage/y_patarn%02d.bmp", i);
		cv::imwrite(buf,cap);
	}
	std::cout << "�����g�p�^�[���B�e�I��" << std::endl;
	//pgrOpenCV.stop();

	/*  �m�F�p�̉摜���e  */

	// �J�����ݒ�
	pgrOpenCV.setPixelFormat( FlyCapture2::PIXEL_FORMAT_BGR );
	pgrOpenCV.setColorProcessingAlgorithm( FlyCapture2::HQ_LINEAR );
	pgrOpenCV.setShutterSpeed(pgrOpenCV.getShutter_h());
	//pgrOpenCV.start();
	Sleep(delay);
	//cv::waitKey(delay);

	// �摜�̓ǂݍ���
	//cv::Mat image = cv::imread( "./PhaseShift/PhaseImage/TestImage/pattern.jpg", -1); // -1 ���̂܂܂̃`���l�����œǂݍ���
	cv::Mat image(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar::all(255));

	//cv::Mat color_image(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3); // -1 ���̂܂܂̃`���l�����œǂݍ���
	//// -���悪�Ȃ��ꍇ�A�V���ɍ쐬����D
	//if(image.empty()){
	//	std::cout << "./TestImage ���� pattern.jpg ������܂���B�쐬���܂��B" << std::endl;
	//	cv::Mat pattern(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar::all(255));
	//	cv::imwrite("./PhaseShift/PhaseImage/TestImage/pattern.jpg",pattern);
	//	image = pattern;
	//}

	// ���e
	cv::imshow(CODE_IMG, image);

	// �J�����B�e & �ۑ�
	cv::waitKey(delay);
	pgrOpenCV.queryFrame();
	cv::Mat cap = pgrOpenCV.getVideo();
	pgrOpenCV.showCapImg(cap);
	//cv::cvtColor(pgrOpenCV.getVideo(),color_image,CV_BayerBG2BGR);	//
	sprintf_s(buf, "./PhaseShift/PhaseImage/white.bmp");	//-�p�^�[���B�e���ʂ�ۑ�
	cv::imwrite(buf,cap);

	/***** ���e & �B�e�I�� *****/

	pgrOpenCV.stop();
	pgrOpenCV.release();

	/***** �I�� *****/

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
** ��l���֘A **
****************/

// �J�����B�e�摜��ǂݍ��ފ֐�
void PHASESHIFT::loadCam(cv::Mat &mat, int div_bin, bool vh, bool pn)
{
	char buf[256];
	sprintf_s(buf, "./PhaseShift/GrayCodeImage/CaptureImage/CameraImg%d_%02d_%d.bmp", vh, div_bin, pn);
	mat = cv::imread(buf, 0);
}

// �}�X�N���쐬����C���^�t�F�[�X
void PHASESHIFT::makeMask(cv::Mat &mask)
{
	cv::Mat posi_img;
	cv::Mat nega_img;

	// �}�X�N�摜����
	cv::Mat mask_vert, mask_hor;
	static int useImageNumber = 5;
	// y�����̃O���C�R�[�h�摜�ǂݍ���
	loadCam(posi_img, useImageNumber, 0, 1);
	loadCam(nega_img, useImageNumber, 0, 0);

	// ���̃}�X�N�摜Y����
	makeMaskFromCam(posi_img, nega_img, mask_vert, ThresholdValue);

	// x�����̃O���C�R�[�h�摜�ǂݍ���
	loadCam(posi_img, useImageNumber, 1, 1);
	loadCam(nega_img, useImageNumber, 1, 0);

	// ���̃}�X�N�摜X����
	makeMaskFromCam(posi_img, nega_img, mask_hor, ThresholdValue);

	// X��Y��OR�����
	// �}�X�N�O�͂ǂ�������Ȃ̂ō�
	// �}�X�N���́i���_�I�ɂ́j�K����������ł�����������Ȃ̂ŁA���ɂȂ�
	// ���ۂ͂��܉��m�C�Y���c���Ă��܂�
	cv::bitwise_or(mask_vert, mask_hor, mask);

	// �c�������܉��m�C�Y�������i���S�}�����S�}���œK�p�����t�ɂȂ�j
	dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
	erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

	for (int i = 0; i < 2; i++)
		smallMaskRange(mask, mask);

	cv::imwrite("./PhaseShift/GrayCode_Mask.bmp", mask);


}

// �O���C�R�[�h�̉摜�𗘗p���ă}�X�N�𐶐�����֐�
// �|�W�ƃl�K�̍����������thresholdValue�ȏ�̋P�x�̃s�N�Z���𔒂ɂ���
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

// �B�e�摜��2�l��������C���^�t�F�[�X
void PHASESHIFT::make_thresh()
{
	cv::Mat posi_img;
	cv::Mat nega_img;
	cv::Mat Geometric_thresh_img;  // 2�l�����ꂽ�摜
	cv::Mat mask;

	// �}�X�N�𐶐�
	makeMask(mask);

	int h_bit = (int)ceil( log(WaveNumY+1) / log(2) ); //�����_�؂�グ
	int w_bit = (int)ceil( log(WaveNumX+1) / log(2) );
	int all_bit = h_bit + w_bit;

	std::cout << "��l���J�n" << std::endl;
	// �A�ԂŃt�@�C������ǂݍ���
	for( int i = 0; i < h_bit; i++ ) {
		// �ǂݍ���
		char buf[256];
		// �|�W�p�^�[���ǂݍ���
		loadCam(posi_img, i+1, 0, 1);
		// �l�K�p�^�[���ǂݍ���
		loadCam(nega_img, i+1, 0, 0);

		// 2�l��
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// �}�X�N��K�p����2�l��
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./PhaseShift/GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::imwrite(buf, masked_img);

		std::cout << i << ", ";
	}
	for( int i = 0; i < w_bit; i++ ) {
		// �ǂݍ���
		char buf[256];
		// �|�W�p�^�[���ǂݍ���
		loadCam(posi_img, i+1, 1, 1);
		// �l�K�p�^�[���ǂݍ���
		loadCam(nega_img, i+1, 1, 0);

		// 2�l��
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// �}�X�N��K�p����2�l��
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./PhaseShift/GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i+h_bit);
		cv::imwrite(buf, masked_img);

		std::cout << i+h_bit << ", ";
	}
	std::cout << std::endl;
	std::cout << "��l���I��" << std::endl;
}

// ���ۂ�2�l������ 
void PHASESHIFT::thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value )
{
	thresh_img = cv::Mat(posi.rows, posi.cols, CV_8UC1);
	for( int y = 0; y < posi.rows; y++ ) {
		for(int x = 0; x < posi.cols; x++ ) {
			int posi_pixel = posi.at<uchar>( y, x );
			int nega_pixel = nega.at<uchar>( y, x );

			// thresh_value���傫�����ǂ����œ�l��
			if( posi_pixel - nega_pixel >= thresh_value )
				thresh_img.at<uchar>( y, x ) = 255;
			else
				thresh_img.at<uchar>( y, x ) = 0;
		}
	}
}

/*	
�}�X�N�̈��1pixel����������֐� 
���e�̈�̉��͐M�������Ⴂ���߁C���͊􉽑Ή��t�������Ȃ��D
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
** �v���W�F�N�^�ƃJ�����̑Ή��t�� **
************************************/

// ��������
void PHASESHIFT::code_restore()
{
	std::cout << "�R�[�h�����J�n " << std::endl;
	Timer tm;
	// �O���C�R�[�h���f�R�[�h
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		char buf[256];
		sprintf_s(buf, "./PhaseShift/GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::Mat a = cv::imread(buf, 0);
		for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
			for( int x = 0; x < CAMERA_WIDTH; x++ ) {
				//- 255�Ȃ��1�C����ȊO��0�Ƃ���2�i�R�[�h�𕜌�
				if( a.at<uchar>( y, x ) == 255)
					//- ���ɃV�t�g������
						c->Decoded_GrayCode[y][x] = ( 1 << (c->g.all_bit-i-1) ) | c->Decoded_GrayCode[y][x]; 
			}
		}
	}

	//�R�[�h�}�b�v�̏�����
	c->g_code_map->clear();
	c->point_map->clear();

	//�R�[�h�}�b�v���쐬( Key:�O���C�R�[�h�Cvalue:�g�̍��W�f�[�^(�������ڂ�) )
	for( int y = 0; y < WaveNumY; y++ ) {
		for( int x = 0; x < WaveNumX; x++ ) {
			int a = c->g.GrayCode[y][x];
			if(a != 0)
				(*c->g_code_map)[a] = cv::Point(x,y);
		}
	}

	// 0�Ԗڂ͎g��Ȃ�
	(*c->g_code_map)[0] = cv::Point(-1, -1);

	// �ʑ�����
	restore_phase_value();
	// �A������
	phaseConnection();
	// �M�����]���֐��ňʑ��l�̃G���[����
	//reliableMeasuresFunc();
	// �Ή��G���[���C������
	errorCheck();

	std::cout << "�R�[�h�����I�� : " ;
	tm.elapsed();

	//�t�@�C���̏�������
	{

		std::ofstream ofsRF("./PhaseShift/PhaseImage/�ʑ��f�[�^.csv");  // �J�����o�͂��L�^����t�@�C���i�����֐��j
		// �G���[����
		//if(ofsRF.fail()){
		//	std::cout << "ERROR:�����֐�.csv���J���܂���B" << std::endl;
		//}
		//for(int y=0; y<CAMERA_HEIGHT; y++){
		int y = 649;
		//for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
			//�}�X�N�O�͉������Ȃ�
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
**         �ʑ��V�t�g�֘A         **
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
	// �ʑ��l���摜�Ŋm�F����p
	xphase_img = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar::all(0));
	yphase_img = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar::all(0));

	// (Debug�p)�����f�[�^���v�Z����
	setCorrectPhase();
}

// �iDebug�p�j���z�l�̑��
void PHASESHIFT::setCorrectPhase()
{

	for(int y=0; y<PROJECTOR_HEIGHT; y++){
		for(int x=0; x<PROJECTOR_WIDTH; x++){
			c->p.GT_Phase_Val[y][x].x = (x+xplus)*2.0*PI/Lx;
			c->p.GT_Phase_Val[y][x].y = (y+yplus)*2.0*PI/Ly;
		}
	}
	//�t�@�C���̏�������
	//{
	//	std::ofstream ofsRF("./PhaseShift/PhaseImage/�����f�[�^.csv");  // �J�����o�͂��L�^����t�@�C���i�����֐��j
	//	//�G���[����
	//	if(ofsRF.fail()){
	//		std::cout << "ERROR:�����֐�.csv���J���܂���B" << std::endl;
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

// �����g�摜�̓ǂݍ���
void PHASESHIFT::readPhaseCam(cv::Mat phase_x_img[N], cv::Mat phase_y_img[N], bool flag)
{
	char buf[256];
	char* str;
	if(flag)
		str = "Capture";
	else
		str = "Projection";

	//�ǂݍ���
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

/* �ʑ����� */
void PHASESHIFT::restore_phase_value()
{

	cv::Point2d lumi[N];
	cv::Mat *phase_x_img = new cv::Mat[N];
	cv::Mat *phase_y_img = new cv::Mat[N];
	readPhaseCam(phase_x_img, phase_y_img, 1); // flag 1:capture , 0:projection

	int height = (*phase_x_img).rows;
	int width = (*phase_x_img).cols;
	//��������

	//�iDebug�p�j�摜
	//cv::Mat phasemask((*phase_x_img).rows, (*phase_x_img).cols, CV_8UC1, cv::Scalar::all(0));

	for(int y=0; y<height; y++){
		for(int x=0; x<width; x++){

			//�}�X�N�O�͉������Ȃ�
			if(mask.at<uchar>(y, x) == 0){
				continue;
			}

			cv::Point2d cosValue = cv::Point2d(0.0, 0.0);
			cv::Point2d sinValue = cv::Point2d(0.0, 0.0);

			//���q�ɂ���
			for(int i = 0; i < N; i++ ){
				lumi[i] = cv::Point2d( (double)phase_x_img[i].at<uchar>(y, x), (double)phase_y_img[i].at<uchar>(y, x) );
				lumi[i] *= cos( 2.0*PI*i / N );
				cosValue += lumi[i];
			}

			//����ɂ���
			for(int i = 0; i < N; i++ ){
				lumi[i] = cv::Point2d( (double)phase_x_img[i].at<uchar>(y, x), (double)phase_y_img[i].at<uchar>(y, x) );
				lumi[i] *= sin( 2.0*PI*i / N );
				sinValue += lumi[i];
			}

			//0���h�~����
			if(  ( (0 <= abs(sinValue.x)) && (abs(sinValue.x) <= 1) ) && cosValue.x > 0 )
				sinValue.x = 1;
			if( ( (0 <= abs(sinValue.x)) && (abs(sinValue.x) <= 1) ) && cosValue.x < 0 )
				sinValue.x = -1;
			if( ( (0 <= abs(sinValue.y)) && (abs(sinValue.y) <= 1) ) && cosValue.x > 0)
				sinValue.y = 1;
			if( ( (0 <= abs(sinValue.y)) && (abs(sinValue.y) <= 1) ) && cosValue.x < 0 )
				sinValue.y = -1;

			//�ʑ�����
			cv::Point2d phase_value = cv::Point2d(atan2(cosValue.x, sinValue.x),atan2(cosValue.y, sinValue.y));

			//�i�[
			c->p.Phase_Value[y][x] = phase_value;

			//�ʑ���񂩂�}�X�N���쐬����
			//phasemask.at<uchar>(y,x) = 255;
			//if( fabs(cosValue.x) < maskcheck &&  fabs(cosValue.y) < maskcheck){
			//	if( fabs(sinValue.x) < maskcheck &&  fabs(sinValue.y) < maskcheck)
			//		phasemask.at<uchar>(y,x) = 255;
			//}


			// (Debug�p) ���K�����邽�߂ɒl�� 0< value <2�� �ɂ���
			xphase_img.at<double>(y,x) = phase_value.x + (PI);
			yphase_img.at<double>(y,x) = phase_value.y + (PI);


		}
	}

	/* �}�X�N�摜�����菬��������*/
	//for (int i = 0; i < 2; i++)
	//	smallMaskRange(phasemask, phasemask);

	// (Debug�p)���n��̉摜���쐬&�ۑ�
	cv::Mat temp;
	xphase_img.convertTo(temp, CV_8U, 255/(2.0*PI));
	cv::imwrite("./PhaseShift/PhaseImage/xPhase_val.bmp", temp);
	yphase_img.convertTo(temp, CV_8U, 255/(2.0*PI));
	cv::imwrite("./PhaseShift/PhaseImage/yPhase_val.bmp", temp);

	// �}�X�N�摜��ύX
	//cv::imwrite("./PhaseShift/PhaseImage/Phase_Mask.bmp", phasemask);
	//cv::imwrite("./CalculationData/mask.bmp", phasemask);
	//cv::imwrite("./CalculationData/Phase_Mask.bmp", phasemask);
	//mask = phasemask.clone();

	delete[] phase_x_img;
	delete[] phase_y_img;
	temp.release();

}


/* �A������ */
void PHASESHIFT::phaseConnection()
{
	//�A������(�ʑ��l��A���ɂ��鏈��)���s��
	for(int y=0; y<CAMERA_HEIGHT; y++){
		for(int x=0; x<CAMERA_WIDTH; x++){

			//�}�X�N�O�͉������Ȃ�
			if(mask.at<uchar>(y, x) == 0){
				continue;
			}

			int a = c->Decoded_GrayCode[y][x];

			//�R�[�h��������Ȃ��ꍇ�G���[����
			if( (*c->g_code_map).find(a) == (*c->g_code_map).end() ){
				c->p.Pointdata[y][x].x = -1;
				c->p.Pointdata[y][x].y = -1;
				c->p.Phase_Value[y][x].x = NULL;
				c->p.Phase_Value[y][x].y = NULL;
			}else{
				// �g�̍��W(+1)���i�[
				int nx = ((*c->g_code_map)[a]).x + 1;
				int ny = ((*c->g_code_map)[a]).y + 1;

				// (Debug�p)
				c->p.Phase_Value_copy[y][x] = c->p.Phase_Value[y][x];

				// 2n�Ή��Z
				c->p.Phase_Value[y][x].x += ( nx*2.0*PI );
				c->p.Phase_Value[y][x].y += ( ny*2.0*PI );

				//�Ή�����v���W�F�N�^���W(�T�u�s�N�Z�����x)���Z�o
				c->p.SubPixel_Pointdata[y][x].x = ( (c->p.Phase_Value[y][x].x * Lx)/(2*PI) - (double)xplus );
				c->p.SubPixel_Pointdata[y][x].y = ( (c->p.Phase_Value[y][x].y * Ly)/(2*PI) - (double)yplus );

				// �Ή�������W���ߖT�̉�f�Ƃ���(Nearest Neighbor�⊮)
				c->p.Pointdata[y][x].x = (int)( c->p.SubPixel_Pointdata[y][x].x + 0.5);
				c->p.Pointdata[y][x].y = (int)( c->p.SubPixel_Pointdata[y][x].y + 0.5);

			}
		}
	}
}


void PHASESHIFT::errorCheck()
{

	// �ʑ��摜���쐬
	cv::Mat xphase_img(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar::all(0));
	cv::Mat yphase_img(CAMERA_HEIGHT, CAMERA_WIDTH, CV_64F, cv::Scalar::all(0));
	int error;
	for(int y=1; y<CAMERA_HEIGHT; y++){
		for(int x=1; x<CAMERA_WIDTH; x++){

			//�}�X�N�E�G���[�l�̏ꍇ�������Ȃ�
			if(mask.at<uchar>(y, x) == 0 || c->p.Phase_Value[y][x].x == NULL || c->p.Phase_Value[y][x].y == NULL
				|| c->p.Phase_Value[y][x-1].x == NULL || c->p.Phase_Value[y-1][x].y == NULL){
					continue;
			}
			if( c->p.Pointdata[y][x].x  == -1 || c->p.Pointdata[y][x-1].x  == -1 
				|| c->p.Pointdata[y][x].y  == -1 || c->p.Pointdata[y-1][x].y  == -1 ){
					continue;
			}

			// x �ɂ���
			error = c->p.Pointdata[y][x].x - c->p.Pointdata[y][x-1].x;
			// ���W�̐��ڂ����j�A�ł͂Ȃ��ꍇ,���ډ�f�̈ʑ��l��2�Α����č��W���Z�o���C
			// ���̂Ƃ��̍��W�l���Ó��Ȃ��̂��𔻒f����
			if(error < 0){
				double value = c->p.Phase_Value[y][x].x + 2.0*PI;
				double sub_p = (value * Lx)/(2*PI) - (double)xplus;
				int p =  (int)(sub_p + 0.5);
				error = p - c->p.Pointdata[y][x-1].x;
				if( 0 <= error  ){
					//-�O���C�R�[�h�̋��E�덷
					if( error < 2 ){ 
						c->p.SubPixel_Pointdata[y][x].x = sub_p;
						c->p.Pointdata[y][x].x = p;
						c->p.Phase_Value[y][x].x = value;
					}
					//-�ʑ��̌덷
					else{
						//c->p.pointdata[y][x].x = c->p.pointdata[y][x-1].x;
					}
				}
				else{
					c->p.Phase_Value[y][x].x = NULL;
					c->p.Pointdata[y][x].x = -1;
					continue;
					//std::cout << "(x)�Y�����傫������F" << x  << ", " << y  <<std::endl;
				}
			}
			//-�l�̌ܓ��ɂ��덷
			else if(1 < error && error < 3)
				c->p.Pointdata[y][x].x -= 1; 

			// y �ɂ��� 
			error = c->p.Pointdata[y][x].y - c->p.Pointdata[y-1][x].y;
			//-���W�̐��ڂ����j�A�ł͂Ȃ��ꍇ
			if(error < 0){
				double value = c->p.Phase_Value[y][x].y + 2.0*PI;
				double sub_p = (value * Ly)/(2*PI) - (double)yplus;
				int p = (int)(sub_p + 0.5);
				error = p - c->p.Pointdata[y-1][x].y;
				if( 0 <= error  ){
					//-�O���C�R�[�h�̋��E�덷
					if( error < 2 ){ 
						c->p.SubPixel_Pointdata[y][x].y = sub_p;
						c->p.Pointdata[y][x].y = p;
						c->p.Phase_Value[y][x].y = value;
					}
					//-�ʑ��̌덷
					else{
						//c->p.pointdata[y][x].y = c->p.pointdata[y-1][x].y;
					}
				}
				else{
					c->p.Phase_Value[y][x].y = NULL;
					c->p.Pointdata[y][x].y = -1;
					continue;
					//	std::cout << "(y)�Y�����傫������F" << x  << ", " << y  <<std::endl;
				}
			}
			//-�l�̌ܓ��ɂ��덷
			else if(1 < error && error < 3)
				c->p.Pointdata[y][x].y -= 1; 

			// error����
			if(c->p.Pointdata[y][x].x < 0 || c->p.Pointdata[y][x].x > PROJECTOR_WIDTH-1 )
				c->p.Pointdata[y][x].x = -1;
			if(c->p.Pointdata[y][x].y < 0 || c->p.Pointdata[y][x].y > PROJECTOR_HEIGHT-1 )
				c->p.Pointdata[y][x].y = -1;

			// (Debug�p)
			xphase_img.at<double>(y,x) = c->p.Phase_Value[y][x].x;
			yphase_img.at<double>(y,x) = c->p.Phase_Value[y][x].y;

		}
	}

	// �i�m�F�p�j�摜�Ŋm�F
	double nx = (int)( 2.0*PI*(PROJECTOR_WIDTH/Lx)+PI - 2.0*PI*1*(0+xplus)/Lx );	//x�̈ʑ��l�̎�蓾��͈�(max - min)
	double ny = (int)( 2.0*PI*(PROJECTOR_HEIGHT/Ly)+PI - 2.0*PI*1*(0+yplus)/Ly  );	//y�̈ʑ��l�̎�蓾��͈�(max - min)
	//std::cout << nx << std::endl;
	//std::cout << ny << std::endl;
	xphase_img.convertTo(xphase_img, CV_8U, 255/nx);
	yphase_img.convertTo(yphase_img, CV_8U, 255/ny);
	cv::imwrite("./PhaseShift/PhaseImage/xPhase_Restore.bmp",xphase_img);
	cv::imwrite("./PhaseShift/PhaseImage/yPhase_Restore.bmp",yphase_img);

	//�i�m�F�p�j
	xphase_img.release();
	yphase_img.release();
}

void PHASESHIFT::calcBilinear()
{
	Timer tm;
	cv::Point2i prj_point;	//�A�z�z��p
	int key;				//�A�z�z��p

	//OpenMP�p
	//int x,y;
	//#pragma omp parallel for private(x)

	// �A�z�z��ɋߖT�J������f�̍��W�l���i�[
	for(int y=1; y<CAMERA_HEIGHT; y++){
		for(int x=1; x<CAMERA_WIDTH; x++){

			//�}�X�N�E�G���[�l�̏ꍇ�������Ȃ�
			if( mask.at<uchar>(y, x) == 0 || c->p.Pointdata[y][x].x  == -1 || c->p.Pointdata[y][x-1].x  == -1 
				|| c->p.Pointdata[y][x].y  == -1 || c->p.Pointdata[y-1][x].y  == -1 ){
					continue;
			}
			//multimap�Ɋi�[�ikey:�v���W�F�N�^���W�Adata:�J�������W�j

			//x��؂�̂āCy��؂�̂Ă����W
			prj_point = cv::Point2i((int)floor(c->p.SubPixel_Pointdata[y][x].x), (int)floor(c->p.SubPixel_Pointdata[y][x].y) );
			key = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));

			// x��؂�̂āCy���J��グ�����W
			prj_point = cv::Point2i((int)floor(c->p.SubPixel_Pointdata[y][x].x), (int)ceil(c->p.SubPixel_Pointdata[y][x].y) );
			key = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));

			// x���J�グ�Cy��؂�̂Ă����W
			prj_point = cv::Point2i((int)ceil(c->p.SubPixel_Pointdata[y][x].x), (int)floor(c->p.SubPixel_Pointdata[y][x].y) );
			key = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));

			// x���J�グ�Cy���J�グ�����W
			prj_point = cv::Point2i((int)ceil(c->p.SubPixel_Pointdata[y][x].x), (int)ceil(c->p.SubPixel_Pointdata[y][x].y) );
			key = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));

		}
	}
	std::cout << "�A�z�z��Ɋi�[ : " ;
	tm.elapsed();

	// Sub_Distance�̌v�Z
	for( int y = 0; y < PROJECTOR_HEIGHT; y++ ) {
		for( int x = 0; x < PROJECTOR_WIDTH; x++ ) {
			int key = y * PROJECTOR_WIDTH + x;
			//���݂���ꍇ
			if( (*c->point_map).find(key) != (*c->point_map).end() ){
				auto range = (*c->point_map).equal_range(key);
				// �����̍��v���v�Z
				double sum_dist = 0.0;
				for(auto iterator = range.first; iterator != range.second; iterator++){
					auto target = *iterator;
					cv::Point p = target.second;

					// �������v�Z
					double d_x = fabs(c->p.SubPixel_Pointdata[p.y][p.x].x - x);
					double d_y = fabs(c->p.SubPixel_Pointdata[p.y][p.x].y - y);
					double d = (d_x * d_x) + (d_y * d_y);
					//double area = d_x * d_y;
					if(!power)
						d = sqrt(d); //����

					double Reciprocal_d = 1.0/d;
					//double Reciprocal_d = 1.0/area;

					sum_dist += Reciprocal_d;
				}
				c->Sum_Distance[y][x] = sum_dist;
			}
		}
	}

}

//��A���X�g�l�C�o�[�⊮�̏ꍇ
void PHASESHIFT::calcNearestNeighbor()
{
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {

			//�}�X�N�O�͉������Ȃ�
			if( mask.at<uchar>(y, x) == 0 ){
				continue;
			}

			// �������v�Z
			double d_x = abs(c->p.SubPixel_Pointdata[y][x].x - c->p.Pointdata[y][x].x);
			double d_y = abs(c->p.SubPixel_Pointdata[y][x].y - c->p.Pointdata[y][x].y);
			double d = pow(d_x, 2.0) + pow(d_y, 2.0);
			d = sqrt(d);
			if(d > 0){
				//c->Distance[y][x] = pow(1.0/d, 2.0);
				c->Distance[y][x] = 1.0/d;
			}else{
				c->Distance[y][x] = 100; //100�͓K��
			}

			//multimap�Ɋi�[�ikey:�v���W�F�N�^���W�Adata:�J�������W�j
			int key = c->p.Pointdata[y][x].y * PROJECTOR_WIDTH + c->p.Pointdata[y][x].x;
			(*c->point_map).insert(std::pair<int, cv::Point>(key, cv::Point(x,y)));
		}
	}
	// �����̑��a���v�Z
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
			cv::Point p = c->p.Pointdata[y][x];
			if( (p.x != -1) && (p.y != -1) ) 
				c->Sum_Distance[p.y][p.x] += c->Distance[y][x];
		}
	}
	assign_pointdata();
}

// (Nearest�p)�B�e�摜���􉽕ϊ�
void PHASESHIFT::checkCorrespondence_for_Nearest(int num)
{
	// �i�m�F�p�j�v���W�F�N�^�ƃJ�����̑Ή��t��(�ʑ��A�������j
	cv::Mat test = cv::imread("./PhaseShift/PhaseImage/white.bmp",1);
	cv::Mat dst(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3);
	char buf[256];
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {

			//�}�X�N�O�͉������Ȃ�
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

	std::cout << "�ۑ�" << std::endl;
	if(num == 0)
		sprintf_s(buf, "./PhaseShift/PhaseImage/NN_interpolation(before).bmp");
	if(num == 1)
		sprintf_s(buf, "./PhaseShift/PhaseImage/NN_interpolation(after).bmp");
	cv::imwrite(buf,dst);

	test.release();
	dst.release();
}

// (Debug�p)�⊮��Ɋ􉽕ϊ�
void PHASESHIFT::checkCorrespondence()
{
	cv::Mat test = cv::imread("./PhaseShift/PhaseImage/white.bmp",1);
	cv::Mat dst(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3);
	char buf[256];

	// �􉽕ϊ�
	reshapeCam2Proj(test, dst);

	//std::cout << "�ۑ�" << std::endl;
#if USE_BILINEAR
	if(power)
		sprintf_s(buf, "./PhaseShift/PhaseImage/BL_Convertion(2��).bmp");
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
**  (Nearest Neighbor�p)��f���  **
************************************/

/*
��f�⊮���s�����߂̊֐��D
���Ή��v���W�F�N�^��f�ɗאڂ���v���W�F�N�^��f�𑖍����C
���̒���2�ȏ�Ή�����J������f���������ꍇ�C�����̃J������f�̒���
�{�����Ή������ɓ���ׂ��ʑ��l�ɖނ��߂��J������f��Ή�������
*/
void PHASESHIFT::interpolation()
{
	errorCount=0;
	// ���e�̈�̘g�͑Ή������Ȃ��̂ŁC�������Ȃ�
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

			//-���Ή��̃v���W�F�N�^��f���������ꍇ
			if( (*c->point_map).find(a) == (*c->point_map).end() ){
				errorCount++;
				//-�ߖT��f���玝���Ă���֐�
				cv::Point point = getInterpolatedPoint2(x,y);
				if( point.x != -1 && point.y != -1)
					//-�X�V
						c->p.Pointdata[point.y][point.x] = cv::Point(x, y);

			}
		}
	}

	std::cout << "�Ή��G���[���F" << errorCount << std::endl;
	//std::cout << "all" << (*c->code_map).size() << std::endl;
}

/*
���ۂɋߖT�J������f�𑖍�����֐�
�J�����̍��W�l���Ԃ��Ă���
*/
cv::Point PHASESHIFT::getInterpolatedPoint2(int x, int y)
{
	// ���Ή���f���Ƃ肤��ʑ��l���v�Z
	double x_ideal = (x+xplus)*2.0*PI/Lx;
	double y_ideal = (y+yplus)*2.0*PI/Ly;

	const int radius = 1;							//�������锼�a
	double xtemp = 0.0, ytemp = 0.0, temp = 0.0;	//�ʑ��l�����Ă������߂̕ϐ�
	double ave = 100.0;								//�ʑ��l�̕��ς����邽�߂̕ϐ�
	cv::Point nearest_point = cv::Point(-1, -1);
	cv::Point prj_point = cv::Point(-1, -1);

	// radius����������
	for (int j = -radius; j <= radius; j++) {
		for (int i = -radius; i <= radius; i++) {
			int yj = j + y;
			int xi = i + x;

			if( j == 0 && i == 0 )
				continue;
			if ( ( 1 <= yj && yj < PROJECTOR_HEIGHT-1 ) && ( 1 <= xi && xi < PROJECTOR_WIDTH-1 ) ){
				int a = yj* PROJECTOR_WIDTH + xi;
				auto range = (*c->point_map).equal_range(a);
				//-�ߖT��f�����Ή���f�̏ꍇ
				if( (*c->point_map).find(a) ==  (*c->point_map).end() )
					continue;

				//-���������ꍇ && �������݂���ꍇ
				else if( (*c->point_map).count(a) > 1 ){
					for (auto iterator = range.first; iterator != range.second; iterator++){
						auto target = *iterator;
						//-���z�I�Ȉʑ��l�ƒ��ډ�f�̈ʑ��l�̍����̕��ς��v�Z����
						xtemp = fabs(x_ideal - c->p.Phase_Value[target.second.y][target.second.x].x);
						ytemp = fabs(y_ideal - c->p.Phase_Value[target.second.y][target.second.x].y);
						temp = (xtemp + ytemp)/2.0;
						//-���ϒl����ԏ������Ȃ�X�V���C���̍��W��ۑ�
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

	//�A�z�z��̍X�V(�폜&�}��)
	if( nearest_point.x != -1  && nearest_point.y != -1 ){
		int code = prj_point.y * PROJECTOR_WIDTH + prj_point.x;
		auto range = (*c->point_map).equal_range(code);
		auto iterator = range.first;

		//for (auto iterator = range.first; iterator != range.second; iterator++){
		//	auto target = *iterator;
		//	std::cout << "�폜�O�̗v�f : " << target.second << std::endl;
		//}

		//�A�z�z�񂩂�폜
		while(iterator != range.second){
			if(iterator->second == nearest_point){
				(*c->point_map).erase(iterator++);
			}else ++iterator;

		}

		//auto range1 = (*c->code_map).equal_range(code);
		//for (auto iterator = range1.first; iterator != range1.second; iterator++){
		//	auto target = *iterator;
		//	std::cout << "�폜��̗v�f : " << target.second << std::endl;
		//}
		//cv::waitKey(0);

		//�A�z�z��ɑ}��
		int a = y * PROJECTOR_WIDTH + x;
		(*c->point_map).insert( std::pair<int, cv::Point>(a, nearest_point) );
		return nearest_point;

	}else{
		//���͂�2��f�ȏ�Ή������f���Ȃ������ꍇ�C�⊮���Ȃ�
		return cv::Point(-1, -1);
	}
}


// (Nearest�p)�J�������W�̐����v�Z(�P�x�l�̕��ϗp)
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
**           �􉽕ϊ�             **
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
			//���݂���ꍇ
			if( (*c->point_map).find(key) != (*c->point_map).end() ){
				auto range = (*c->point_map).equal_range(key);
				//�����̋t���ɂ��d�ݕt��
				for(auto iterator = range.first; iterator != range.second; iterator++){
					auto target = *iterator;
					cv::Point p = target.second;
					cv::Vec3b* cam_p = cam.ptr<cv::Vec3b>(p.y);
					// �������v�Z
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
	//std::cout << "PhaseShift �􉽕␳:";
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
//			//�}�X�N�O�͉������Ȃ�
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

//�Ή���f�̋P�x�l�̕��ς�p���Ċ􉽕ϊ�
void PHASESHIFT::reshapeCam2Proj(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat temp_f(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_32FC3, cv::Scalar(0, 0, 0));
	cv::Mat temp_c(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat normal_mask = cv::imread("./PhaseShift/GrayCode_Mask.bmp",0);

	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {

			//�}�X�N�O�͉������Ȃ�
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
//	3�����č\���p�@�ǉ�
/**********************************************/

// �J�������W�ɑ΂���v���W�F�N�^�̑Ή��_��Ԃ�(�����x��)
void PHASESHIFT::getCorrespondSubPixelProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, int size)
{
	for (int i=0; i < imagePoint.size(); ++i)
	{
		std::vector<cv::Point2f> iPoints, pPoints;
		if(imagePoint[i].x > size && imagePoint[i].x+size < CAMERA_WIDTH && imagePoint[i].y > size && imagePoint[i].y+size < CAMERA_HEIGHT)
		{
			// �̈斈�̑Ή��_
			for( float h = imagePoint[i].y-size; h < imagePoint[i].y+size; h+=1.0f){
				for( float w = imagePoint[i].x-size; w < imagePoint[i].x+size; w+=1.0f){
					cv::Point2f point = c->p.SubPixel_Pointdata[int(h+0.5f)][int(w+0.5f)];
					if( point.x != -1.0f) {
						iPoints.emplace_back(cv::Point2f(w, h));
						pPoints.emplace_back(point);
					}
				}
			}

			// �Ή��_���m��Homography�̌v�Z
			cv::Mat H = cv::findHomography(iPoints, pPoints, CV_RANSAC, 2.0);
			// Homography���g���ă`�F�b�J�[�p�^�[���̌�_���ˉe
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

// �Ή��̂Ƃꂽ�_��S�ĕԂ�
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

// �Ή��̂Ƃꂽ�_��S�ĕԂ�
void PHASESHIFT::getCorrespondAllPoints2(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<cv::Point3i> &pointColor)
{
	cv::Mat surface_image = cv::imread("./PhaseShift/PhaseImage/white.bmp",1);
	cv::cvtColor(surface_image, surface_image,CV_BGR2RGB);
	for( int y = 0; y < CAM_HEIGHT; y++ ) {
		for( int x = 0; x < CAM_WIDTH; x++ ) {
			cv::Point p = c->p.SubPixel_Pointdata[y][x];
			//�S�Ă̍��W�l���i�[
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
		//���݂���ꍇ
		if( (*c->point_map).find(key) != (*c->point_map).end() ){
			auto range = (*c->point_map).equal_range(key);
			for(auto iterator = range.first; iterator != range.second; iterator++){
				auto target = *iterator;
				//p�̓v���W�F�N�^���W�ɑΉ�����J�������W
				cv::Point2i p = target.second;
				cameraPoint.emplace_back(p);
			}
		}

	}
	
}
