#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H
#define FILE_SIZE(fp, size)  fseeko(fp,0L,SEEK_END);\
    size = ftello(fp);\
    fseeko(fp,0L,SEEK_SET);
#include <stdint.h>

typedef struct content{
    char* text;
    uint32_t size;
}Text;

//Allocates an array on the heap containing the text file content
//!!Remember to free the returned array when done!!!
Text read_text_file(const char* filename);
void write_text_file(const char* filename, const Text* text);
#endif
