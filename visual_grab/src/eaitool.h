#ifndef EAITOOL_H
#define EAITOOL_H

#ifndef _cplusplus
#define _cplusplus

#endif


#ifdef _cplusplus
extern "C"
{
#endif
    bool bEAIHadAuthed(void);
    char * chEAIGetRegCode(const char* chSN);

#ifdef _cplusplus
}
#endif


#endif // EAITOOL_H
