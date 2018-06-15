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
	//处理全过程
	void process();
	//返回结果
	std::vector<std::vector<float>> getResult();
	//为cpu平台创建上下文
	cl_context createContext();
	//选择第一个可用设备，并创建一个命令队列
	cl_command_queue createCommandQueue(cl_context context, cl_device_id & device);
	//从磁盘加载内核源文件创建和构建一个程序对象
	cl_program createProgram( const char* fileName);
	//建立内核参数
	cl_int setKernelParameter(int id, cl_mem theData);
	//使用命令队列使将在设备上执行的内核排队
	cl_int setKernalQueue(size_t* globalWorkSize, size_t* localWorkSize);
	//从内核读回结果
	cl_int readResult(cl_mem memObject, float * result);

public:
	//返回设备上下文 
	cl_context getContext();
private:
	std::string _strOpenCLFileName; //opencl处理的文件名称
	std::string _strOpenCLKernalEntry;//opencl入口名称
	cl_context _theContext;		//设备上下文
	cl_command_queue _commandQueue;//命令队列
	cl_device_id  _device;	//设备ID
	cl_program _theProgram; //程序对象
	cl_kernel _theKernel;//创建opencl内核	
	int _sizeOfInputType;		//输入物体的数目
	int _sizeOfInputObject;	//每个输入物体含多少值
	int _sizeOfEachInputUnit;	//每个输入值的大小
	std::vector<std::vector<float>> _inputVec2;//输入的vector
	int _sizeOfOutputType;		//输出物体的数目
	int _sizeOfOutputObject;	//每个输出物体含多少值
	int _sizeOfEachOutputUnit;	//每个输出单元的大小
	std::vector<std::vector<float>> _outputVec2;//输出的vector
	
};

