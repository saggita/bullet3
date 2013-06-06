#pragma once

#include <CL\cl.h>

class CLFunctions
{
public:
	cl_command_queue	m_cqCommandQue;
	cl_context			m_cxMainContext;

	int	m_kernelCompilationFailures;

public:
	CLFunctions() : m_kernelCompilationFailures(0)
	{ }

	CLFunctions(cl_command_queue cqCommandQue, cl_context cxMainContext) :
		m_cqCommandQue( cqCommandQue ),
		m_cxMainContext( cxMainContext ),
		m_kernelCompilationFailures(0)
	{ }

	int getKernelCompilationFailures() const
	{
		return m_kernelCompilationFailures;
	}

	cl_kernel compileCLKernelFromString( const char* kernelSource, const char* kernelName, const char* additionalMacros = "" );

	void	clearKernelCompilationFailures()
	{
		m_kernelCompilationFailures=0;
	}
};