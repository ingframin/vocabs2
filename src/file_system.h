#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H
#include <stdio.h>
#define FILE_SIZE(fp, size)  fseek(fp,0L,SEEK_END);\
    size = ftell(fp);\
    fseek(fp,0L,SEEK_SET);
#include <stdint.h>
#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),(mode)))==NULL
#define strcat_s(dest,size,source) strcat((dest),(source))
#endif

typedef struct content{
    char* text;
    uint32_t size;
}Text;

//Allocates an array on the heap containing the text file content
//!!Remember to free the returned array when done!!!
Text read_text_file(const char* filename);
void write_text_file(const char* filename, const Text* text);
#endif
