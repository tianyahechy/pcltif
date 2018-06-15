#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <vector>
#include "common.h"

#ifdef __APPLE__
#include <opencl/cl.h>
#else
#include <CL/cl.h>
#endif

class myOpenCL
{
public:
	myOpenCL(std::string strOpenCLFileName,
		std::string strOpenCLKernalEntry,
		int sizeOfInputType,
		int sizeOfInputObject,
		int sizeOfEachInputUnit,
		std::vector<std::vector<float>> inputVec2,
		int sizeOfOutputType,
		int sizeOfOutputObject,
		int sizeOfEachOutputUnit,
		std::vector<std::vector<float>> outputVec2);
	~myOpenCL();

public:
	//����ȫ����
	void process();
	//���ؽ��
	std::vector<std::vector<float>> getResult();
	//Ϊcpuƽ̨����������
	cl_context createContext();
	//ѡ���һ�������豸��������һ���������
	cl_command_queue createCommandQueue(cl_context context, cl_device_id & device);
	//�Ӵ��̼����ں�Դ�ļ������͹���һ���������
	cl_program createProgram( const char* fileName);
	//�����ں˲���
	cl_int setKernelParameter(int id, cl_mem theData);
	//ʹ���������ʹ�����豸��ִ�е��ں��Ŷ�
	cl_int setKernalQueue(size_t* globalWorkSize, size_t* localWorkSize);
	//���ں˶��ؽ��
	cl_int readResult(cl_mem memObject, float * result);

public:
	//�����豸������ 
	cl_context getContext();
private:
	std::string _strOpenCLFileName; //opencl������ļ�����
	std::string _strOpenCLKernalEntry;//opencl�������
	cl_context _theContext;		//�豸������
	cl_command_queue _commandQueue;//�������
	cl_device_id  _device;	//�豸ID
	cl_program _theProgram; //�������
	cl_kernel _theKernel;//����opencl�ں�	
	int _sizeOfInputType;		//�����������Ŀ
	int _sizeOfInputObject;	//ÿ���������庬����ֵ
	int _sizeOfEachInputUnit;	//ÿ������ֵ�Ĵ�С
	std::vector<std::vector<float>> _inputVec2;//�����vector
	int _sizeOfOutputType;		//����������Ŀ
	int _sizeOfOutputObject;	//ÿ��������庬����ֵ
	int _sizeOfEachOutputUnit;	//ÿ�������Ԫ�Ĵ�С
	std::vector<std::vector<float>> _outputVec2;//�����vector
	
};

