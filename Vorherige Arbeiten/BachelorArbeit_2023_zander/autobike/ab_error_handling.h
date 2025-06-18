#ifndef __ERROR_HANDLING_H__
#define __ERROR_HANDLING_H__


#define DEBUG 1

#include <stdio.h>
#include <stdlib.h>

#define AB_ERR_ABORT    1
#define AB_ERR_CONTINUE 0

#define AB_ERROR(err,msg,abort)     ab_error(err,msg,__FILE__, __LINE__, abort)
#define AB_WARN(warn,msg,abort)     ab_warning(warn,msg,__FILE__, __LINE__, abort)


int ab_error(int err, char * msg, char * filename, int iLineNo, int abort);
int ab_warning(int warn, char * msg, char * filename, int iLineNo, int abort);

#endif // __ERROR_HANDLING_H__
