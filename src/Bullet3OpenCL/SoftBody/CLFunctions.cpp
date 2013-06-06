#include <cstdio>
#include <cstring>
#include "CLFunctions.h"


cl_kernel CLFunctions::compileCLKernelFromString( const char* kernelSource, const char* kernelName, const char* additionalMacros )
{
	printf("compiling kernelName: %s ",kernelName);
	cl_kernel kernel=0;
	cl_int ciErrNum;
	size_t program_length = strlen(kernelSource);

	cl_program m_cpProgram = clCreateProgramWithSource(m_cxMainContext, 1, (const char**)&kernelSource, &program_length, &ciErrNum);
//	oclCHECKERROR(ciErrNum, CL_SUCCESS);
		
    // Build the program with 'mad' Optimization option

	
#ifdef MAC
	char* flags = "-cl-mad-enable -DMAC -DGUID_ARG";
#else
	//const char* flags = "-DGUID_ARG= -fno-alias";
	const char* flags = "-DGUID_ARG= ";
#endif

	char* compileFlags = new char[strlen(additionalMacros) + strlen(flags) + 5];
	sprintf(compileFlags, "%s %s", flags, additionalMacros);
    ciErrNum = clBuildProgram(m_cpProgram, 0, NULL, compileFlags, NULL, NULL);

    if (ciErrNum != CL_SUCCESS)
    {
		size_t numDevices;
		clGetProgramInfo( m_cpProgram, CL_PROGRAM_DEVICES, 0, 0, &numDevices );
		cl_device_id *devices = new cl_device_id[numDevices];
		clGetProgramInfo( m_cpProgram, CL_PROGRAM_DEVICES, numDevices, devices, &numDevices );
        for( int i = 0; i < 2; ++i )
		{
			char *build_log;
			size_t ret_val_size;
			clGetProgramBuildInfo(m_cpProgram, devices[i], CL_PROGRAM_BUILD_LOG, 0, NULL, &ret_val_size);
			build_log = new char[ret_val_size+1];
			clGetProgramBuildInfo(m_cpProgram, devices[i], CL_PROGRAM_BUILD_LOG, ret_val_size, build_log, NULL);
    
			// to be carefully, terminate with \0
			// there's no information in the reference whether the string is 0 terminated or not
			build_log[ret_val_size] = '\0';
        

			printf("Error in clBuildProgram, Line %u in file %s, Log: \n%s\n !!!\n\n", __LINE__, __FILE__, build_log);
			delete[] build_log;
		}

		m_kernelCompilationFailures++;
		return 0;
    }	
	
    // Create the kernel
    kernel = clCreateKernel(m_cpProgram, kernelName, &ciErrNum);

    if (ciErrNum != CL_SUCCESS)
    {
		const char* msg = "";
        switch(ciErrNum)
        {
        case CL_INVALID_PROGRAM:
            msg = "Program is not a valid program object.";
            break;
        case CL_INVALID_PROGRAM_EXECUTABLE:
            msg = "There is no successfully built executable for program.";
            break;
        case CL_INVALID_KERNEL_NAME:
            msg = "kernel_name is not found in program.";
            break;
        case CL_INVALID_KERNEL_DEFINITION:
            msg = "the function definition for __kernel function given by kernel_name such as the number of arguments, the argument types are not the same for all devices for which the program executable has been built.";
            break;
        case CL_INVALID_VALUE:
            msg = "kernel_name is NULL.";
            break;
        case CL_OUT_OF_HOST_MEMORY:
            msg = "Failure to allocate resources required by the OpenCL implementation on the host.";
            break;
		default:
			{
			}
        }

        printf("Error in clCreateKernel for kernel '%s', error is \"%s\", Line %u in file %s !!!\n\n", kernelName, msg, __LINE__, __FILE__);		
		m_kernelCompilationFailures++;
		return 0;
    }

	printf("ready. \n");

	delete [] compileFlags;

	if (!kernel)
		m_kernelCompilationFailures++;

	//clReleaseProgram(m_cpProgram);

	return kernel;
}