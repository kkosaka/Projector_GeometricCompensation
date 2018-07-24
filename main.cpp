#include "main.h"
#include "mygl.h"

myGL gl;

// �v���g�^�C�v�錾
void initCallFunc();
void setCallBackFunction();
void display0();
void display1();
void reshape0(int, int);
void reshape1(int, int);
void idle();
void mouseClick(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void mouseWheel(int wheel_number, int direction, int x, int y);
void keyboard(unsigned char key, int x, int y);
void timer(int value);
void close();


int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	initCallFunc();
	return 0;
}

// �ŏ������Ă΂��֐��i�T�u�X���b�h��run�̒��ŌĂԁj
void initCallFunc()
{
	// �e�R�[���o�b�N�֐��̐ݒ�
	setCallBackFunction();

	gl.initialize();

	// ���[�v�J�n
	glutMainLoop();
}

// �R�[���o�b�N�֐��̐ݒ�
void setCallBackFunction()
{
	// Window1
	gl.createWindow(0);
	glutDisplayFunc(display0);
	glutReshapeFunc(reshape0);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMotion);
	glutMouseWheelFunc(mouseWheel);
	glutTimerFunc(unsigned int(0.01*1000), timer, 0);
	glutCloseFunc(close);
	gl.initWindow(0);

	// Window2
	gl.createWindow(1);
	glutDisplayFunc(display1);
	glutReshapeFunc(reshape1);
	glutIdleFunc(idle);
	gl.initWindow(1);

	// Window3
	//gl.createWindow(2);
	//glutDisplayFunc(display);
	//glutReshapeFunc(reshape);
	//glutIdleFunc(idle);

}

// �N���X���̊֐��͒��ڃR�[���o�b�N�Ɏg���Ȃ��̂ŕ��ʂ̊֐������
void display0()
{
	gl.display_camera_view();
}
void display1()
{
	gl.display_projector_view();
}

void idle()
{
	gl.idle();
}

void reshape0(int w, int h)
{
	gl.reshape(w, h);
}
void reshape1(int w, int h)
{
	gl.reshape(w, h);
}

void mouseClick(int button, int state, int x, int y )
{
	gl.mouseClick(button, state, x, y);
}

void mouseMotion(int x, int y)
{
	gl.mouseMotion(x, y);
}

void mouseWheel(int wheel_number, int direction, int x, int y)
{
	gl.mouseWheel(wheel_number, direction, x, y);
}

void keyboard(unsigned char key, int x, int y)
{
	gl.keyboard(key, x, y);
}

void timer(int value){
	gl.timer(value);
	glutTimerFunc(unsigned int(0.01*1000), timer, 0);
}

void close(){
	gl.close();
}