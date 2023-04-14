#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H
#define FILE_SIZE(fp, size)  fseek(fp,0L,SEEK_END);\
    size = ftell(fp);\
    fseek(fp,0L,SEEK_SET);
#include <stdint.h>

typedef struct content{
    char* text;
    uint32_t size;
}Text;

//Allocates an array on the heap containing the text file content
//!!Remember to free the returned array when done!!!
Text read_text_file(const char* filename);

#endif
