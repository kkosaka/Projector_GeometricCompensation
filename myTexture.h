#ifndef TEXTURE_H
#define TEXTURE_H

#pragma once
#include <GL/freeglut.h>
#include <opencv2/opencv.hpp>

class MyTexture{

public:
	MyTexture();
	~MyTexture();

	// �𑜓x���i�[���Ă���
	int imgWidth, imgHeight;
	int movWidth, movHeight;
	// �摜�Ɠ���̃A�h���X
	std::string imgUrl, movUrl;
	// �E�B���h�E�̃t���[���T�C�Y
	int frameWidth, frameHeight;
	// �Ō�Ƀe�N�X�`���Ɏg�����摜�̉𑜓x
	int nowTextureWidth, nowTextureHeight;

	bool loadImage(std::string imgFilename, GLuint* textureImg);
	// Mat�^�̉摜�f�[�^�����擾����֐��i�e�N�X�`��������O�ɉ������珈�����������Ƃ��Ɏg�p�j
	cv::Mat getImageMat(std::string imgFilename);
	void projectiveTextureMapping(bool toggle);

private:
	// �ˉe�e�N�X�`���}�b�s���O�̃e�N�X�`�����W���������p�z��
	double genfunc[4][4];

	bool loadImage(std::string imgFilename, GLuint* textureImg, bool mipmaps);
	void setTexture(cv::Mat &src, GLuint &textureID, bool mipmaps = 0);
	void surroundPixel(cv::Mat &src, cv::Mat &dst);
};


//***************************
//	�ȉ��e���\�b�h�̏���
//***************************

// �N���X�쐬���ɂ������ŏ��ɌĂ΂��
inline MyTexture::MyTexture()
{
	//�R���X�g���N�^
}

inline MyTexture::~MyTexture()
{
	// �f�X�g���N�^
}

/*
** �摜��ǂݍ���
*/
inline bool MyTexture::loadImage(std::string imgFilename, GLuint* textureImg)
{
	return loadImage(imgFilename, textureImg, 0);
}

inline bool MyTexture::loadImage(std::string imgFilename, GLuint* textureImg, bool mipmaps)
{
	//�摜��ǂݍ���
	cv::Mat image = cv::imread(imgFilename, 1);
	if (image.data == NULL)
	{
		std::cerr << "ERROR�F�摜���ǂݍ��߂܂���" << std::endl;
		image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));
		return false;
	}
	// �t�@�C�����Ɖ𑜓x��ۊǂ��Ă���
	imgUrl = imgFilename;
	imgWidth = image.cols;
	imgHeight = image.rows;

	//�摜���ӂ𔒂��h��Ԃ�
	surroundPixel(image, image);

	cv::imwrite("./surround.jpg", image);

	//�e�N�X�`�����쐬����
	setTexture(image, *textureImg, mipmaps);

	return true;
}

/*
** �摜�̎��ӂ𔒂�����
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
** �e�N�X�`���𐶐�����
*/
inline 	void MyTexture::setTexture(cv::Mat &src, GLuint &textureID, bool mipmaps)
{
	nowTextureWidth = src.cols;
	nowTextureHeight = src.rows;
	int format;
	// �F�ƌ�����OpenGL�p�ɏC��
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
	// �|�C���^�Ŏ󂯎�����ϐ��Ƀe�N�X�`����������
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID); //�w�肵�����O�̃e�N�X�`����L����
	glTexImage2D(GL_TEXTURE_2D, 0, format, nowTextureWidth, nowTextureHeight, 0, format, GL_UNSIGNED_BYTE, src.data);
	if (mipmaps == 1)
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, nowTextureWidth, nowTextureHeight, format, GL_UNSIGNED_BYTE, src.data);

	/* �e�N�X�`�����g��E�k��������@�̎w�� */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	/* �e�N�X�`���̌J��Ԃ��̎w�� */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	/* �e�N�X�`���� */
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);	//�����l:GL_MODULATE�F�e�N�X�`���ގ����|���S���ƍ��킹�邩�������邩�����肷��

	////�o�C���h������
	//glBindTexture(GL_TEXTURE_2D, 0);
}

/*
** Mat�^�̉摜�f�[�^���擾����֐�
** imgUrl, imgWidth, imgHeight�ɂ��ꂼ��A�h���X, ��, �����������Ă���
** �e�N�X�`�����������Ƃ���setTextureEx�ɓn��
*/
inline cv::Mat MyTexture::getImageMat(std::string imgFilename){
	cv::Mat image;

	image = cv::imread(imgFilename, -1);
	if (image.data == NULL)	{
		std::cerr << "ERROR�F�摜���ǂݍ��߂܂���" << std::endl;
		image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));
	}
	// �t�@�C�����Ɖ𑜓x��ۊǂ��Ă���
	imgUrl = imgFilename;
	imgWidth = image.cols;
	imgHeight = image.rows;
	return image;
}

/*
** ���e�e�N�X�`���}�b�s���O���s���悤�ɂ���֐�
** ���e�e�N�X�`���}�b�s���O�s���I�u�W�F�N�g��`�悷��֐������̊֐���true��false�ŋ���
*/
inline void MyTexture::projectiveTextureMapping(bool toggle){
	if (toggle){
		// �e�N�X�`�����W���������p�̍s����쐬
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

		// ���_�̃I�u�W�F�N�g��Ԃɂ�������W�l���g���ă}�b�s���O����
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

		// �e�N�X�`�����W�����֐��̐ݒ�
		glTexGendv(GL_S, GL_OBJECT_PLANE, genfunc[0]);
		glTexGendv(GL_T, GL_OBJECT_PLANE, genfunc[1]);
		glTexGendv(GL_R, GL_OBJECT_PLANE, genfunc[2]);
		glTexGendv(GL_Q, GL_OBJECT_PLANE, genfunc[3]);

		// �e�N�X�`���Əd�Ȃ�悤���s�ړ�
		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glTranslated(0.5, 0.5, 0.0);
		//glScaled(0.5, 0.5, 1.0);

		/* ���f���r���[�ϊ��s��̐ݒ�ɖ߂� */
		glMatrixMode(GL_MODELVIEW);

		// �e�N�X�`���}�b�s���O�J�n
		glEnable(GL_TEXTURE_2D);
		// �e�N�X�`�����W�̎���������L���ɂ���
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);
		glEnable(GL_TEXTURE_GEN_Q);
	}
	else{
		// �e�N�X�`�����W�̎��������𖳌��ɂ���
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		glDisable(GL_TEXTURE_GEN_R);
		glDisable(GL_TEXTURE_GEN_Q);
		// �e�N�X�`���}�b�s���O�I��
		glDisable(GL_TEXTURE_2D);
	}
}



#endif