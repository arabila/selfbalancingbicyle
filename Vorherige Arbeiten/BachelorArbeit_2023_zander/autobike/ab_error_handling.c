#include "ab_error_handling.h"


int ab_error(int err, char * msg, char * filename, int iLineNo, int abort)
{
    fprintf(stderr, "ERROR  (%s, Line: %d): %s\n", filename, iLineNo, msg); 
    if(abort) 
    {
        exit(err);
    }
    return err;
}

int ab_warning(int warn, char * msg, char * filename, int iLineNo, int abort)
{
    fprintf(stderr, "WARNING(%s, Line: %d): %s\n", filename, iLineNo, msg); 

    return warn;
}

