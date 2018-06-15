#include "myOpenCL.h"
myOpenCL::myOpenCL(std::string strOpenCLFileName,
	std::string strOpenCLKernalEntry,
	int sizeOfInputType,
	int sizeOfInputObject,
	int sizeOfEachInputUnit,
	std::vector<std::vector<float>> inputVec2,
	int sizeOfOutputType,
	int sizeOfOutputObject,
	int sizeOfEachOutputUnit,
	std::vector<std::vector<float>> outputVec2)
{
	_strOpenCLFileName = strOpenCLFileName;
	_strOpenCLKernalEntry = strOpenCLKernalEntry;
	_sizeOfInputType = sizeOfInputType;
	_sizeOfInputObject = sizeOfInputObject;
	_sizeOfEachInputUnit = sizeOfEachInputUnit;
	_inputVec2 = inputVec2;
	_sizeOfOutputType = sizeOfOutputType;
	_sizeOfOutputObject = sizeOfOutputObject;
	_sizeOfEachOutputUnit = sizeOfEachOutputUnit;
	_outputVec2 = outputVec2;
	_theContext = NULL;
	_commandQueue = NULL;
	_theProgram = NULL;
	_theKernel = NULL;

}

myOpenCL::~myOpenCL()
{
	_inputVec2.clear();
	_outputVec2.clear();
	if (_commandQueue != 0)
	{
		clReleaseCommandQueue(_commandQueue);
	}

	if (_theKernel != 0)
	{
		clReleaseKernel(_theKernel);
	}
	if (_theProgram != 0)
	{
		clReleaseProgram(_theProgram);
	}
	if (_theContext != 0)
	{
		clReleaseContext(_theContext);
	}
}

//为cpu平台创建上下文
cl_context myOpenCL::createContext()
{
	cl_platform_id firstPlatformId = 0;
	cl_uint numPlatforms = 0;
	//这里选择第一个平台
	cl_int errNum = clGetPlatformIDs(1, &firstPlatformId, &numPlatforms);
	//创建平台的一个上下文，先试图创建一个gpu的，如果没有的话，就创建cpu的
	cl_context_properties contextProperties[] =
	{
		CL_CONTEXT_PLATFORM,
		(cl_context_properties)firstPlatformId,
		0
	};
	cl_context context = clCreateContextFromType(contextProperties, CL_DEVICE_TYPE_GPU, NULL, NULL, &errNum);
	if (errNum != CL_SUCCESS)
	{
		context = clCreateContextFromType(contextProperties, CL_DEVICE_TYPE_CPU, NULL, NULL, &errNum);
	}
	return context;
}
//选择第一个可用设备，并创建一个命令队列
cl_command_queue myOpenCL::createCommandQueue(cl_context context, cl_device_id & device)
{
	size_t deviceBufferSize = -1;
	clGetContextInfo(context, CL_CONTEXT_DEVICES, 0, NULL, &deviceBufferSize);
	//为设备缓存分配空间
	cl_device_id * devices = new cl_device_id[deviceBufferSize / sizeof(cl_device_id)];
	clGetContextInfo(context, CL_CONTEXT_DEVICES, deviceBufferSize, devices, NULL);
	//这里只选择第一个可用的设备，在该设备创建一个命令队列.这个命令队列用于将程序中要执行的内核排队，并读回结果
	cl_command_queue commandQueue = clCreateCommandQueue(context, devices[0], 0, NULL);
	
	device = devices[0];
	delete[] devices;
	return commandQueue;
}

//从磁盘加载内核源文件创建和构建一个程序对象
cl_program myOpenCL::createProgram( const char* fileName)
{
	std::ifstream kernelFile(fileName, std::ios::in);
	if (!kernelFile.is_open())
	{
		std::cerr << "不能打开文件" << fileName << std::endl;
		return NULL;
	}

	std::ostringstream oss;
	oss << kernelFile.rdbuf();
	std::string srcStdStr = oss.str();
	const char * srcStr = srcStdStr.c_str();
	//创建程序对象 
	cl_program program = clCreateProgramWithSource(_theContext, 1, (const char**)&srcStr, NULL, NULL);
	//编译内核源码
	clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
	return program;
}

//返回设备上下文 
cl_context myOpenCL::getContext()
{
	return _theContext;
}

//建立内核参数
cl_int myOpenCL::setKernelParameter( int id, cl_mem theData)
{
	cl_int errNum = clSetKernelArg(_theKernel, id, sizeof(cl_mem), &theData);
	return errNum;
}

//使用命令队列使将在设备上执行的内核排队
cl_int myOpenCL::setKernalQueue(size_t* globalWorkSize, size_t* localWorkSize)
{
	cl_int errNum = clEnqueueNDRangeKernel(_commandQueue, _theKernel, 1, NULL, globalWorkSize, localWorkSize, 0, NULL, NULL);
	return errNum;
}
//从内核读回结果
cl_int myOpenCL::readResult(cl_mem memObject, float * result)
{
	cl_int errNum = clEnqueueReadBuffer(_commandQueue, memObject, CL_TRUE, 0, 
		_sizeOfOutputObject * _sizeOfEachOutputUnit, result, 0, NULL, NULL);
	return errNum;
}

//处理全过程
void myOpenCL::process()
{
	_theContext = this->createContext();
	if ( _theContext == NULL )
	{
		std::cout << "_theContext = NULL" << std::endl;
	}
	else
	{
		std::cout << "_theContext OK" << std::endl;
	}
	_commandQueue = this->createCommandQueue(_theContext, _device);

	if (_commandQueue == NULL)
	{
		std::cout << "_commandQueue = NULL" << std::endl;
	}
	else
	{
		std::cout << "_commandQueue OK" << std::endl;
	}
	_theProgram = this->createProgram(_strOpenCLFileName.c_str());
	if (_theProgram == NULL)
	{
		std::cout << "_theProgram = NULL" << std::endl;
	}
	else
	{
		std::cout << "_theProgram OK" << std::endl;
	}
	//创建opencl内核
	_theKernel = clCreateKernel(_theProgram, _strOpenCLKernalEntry.c_str(), NULL);	
	if (_theKernel == NULL)
	{
		std::cout << "_theKernel = NULL" << std::endl;
	}
	else
	{
		std::cout << "_theKernel OK" << std::endl;
	}

	std::vector<cl_mem> memInputVector;
	memInputVector.clear();
	memInputVector.resize(_sizeOfInputType);
	for (size_t i = 0; i < _sizeOfInputType; i++)
	{
		memInputVector[i] = 0;
	}
	//先读后写分配内存
	for (size_t i = 0; i < _sizeOfInputType; i++)
	{
		memInputVector[i] = clCreateBuffer(_theContext, 
			CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
			_sizeOfEachInputUnit * _sizeOfInputObject, 
			&_inputVec2[i][0], NULL);
		if (memInputVector[i] == NULL)
		{
			std::cout << "memInputVector["<<i<<"] = NULL" << std::endl;
		}
		else
		{
			std::cout << "memInputVector[" << i << "] OK" << std::endl;
		}
	}
	std::vector<cl_mem> memOutputVector;
	memOutputVector.clear();
	memOutputVector.resize(_sizeOfOutputType);
	for (size_t i = 0; i < _sizeOfOutputType; i++)
	{
		memOutputVector[i] = 0;
	}
	//先读后写分配内存
	for (size_t i = 0; i < _sizeOfOutputType; i++)
	{
		memOutputVector[i] = clCreateBuffer(_theContext, 
			CL_MEM_READ_WRITE, 
			_sizeOfEachOutputUnit * _sizeOfOutputObject, 
			NULL, NULL);
		if (memOutputVector[i] == NULL)
		{
			std::cout << "memOutputVector[" << i << "] = NULL" << std::endl;
		}
		else
		{
			std::cout << "memOutputVector[" << i << "] OK" << std::endl;
		}
	}
	
	//建立内核参数
	for (size_t i = 0; i < _sizeOfInputType; i++)
	{
		//this->setKernelParameter(i, memInputVector[i]);
		if (this->setKernelParameter(i, memInputVector[i]) == NULL)
		{
			std::cout << "setKernelParameter memInputVector[" << i << "] = NULL" << std::endl;
		}
		else
		{
			std::cout << "setKernelParameter memInputVector[" << i << "] OK" << std::endl;
		}
	}
	for (size_t i = 0; i < _sizeOfOutputType; i++)
	{
		if (this->setKernelParameter(i + _sizeOfInputType, memOutputVector[i]) == NULL)
		{
			std::cout << "setKernelParameter memOutputVector[" << i << "] = NULL" << std::endl;
		}
		else
		{
			std::cout << "setKernelParameter memOutputVector[" << i << "] OK" << std::endl;
		}
	}
	//使用命令队列使将在设备上执行的内核排队
	size_t globalWorkSize[1] = { _sizeOfInputObject };
	size_t localWorkSize[1] = { 1 };
	if (this->setKernalQueue(globalWorkSize, localWorkSize) == NULL)
	{
		std::cout << "setKernalQueue fail" << std::endl;
	}
	else
	{
		std::cout << "setKernalQueue ok" << std::endl;
	}
	//从内核读回结果
	if (this->readResult(memInputVector[_sizeOfInputType - 1], &_outputVec2[_sizeOfOutputType - 1][0]) == NULL)
	{
		std::cout << "readResult fail" << std::endl;
	}
	else
	{
		std::cout << "readResult ok" << std::endl;
	}
	memInputVector.clear();
}

//返回结果
std::vector<std::vector<float>> myOpenCL::getResult()
{
	return _outputVec2;
}