#include "file_system.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Text read_text_file(const char* filename){
    //Open file and create a buffer of the corresponding size
    FILE* fp;
    fopen_s(&fp,filename,"r");
    int size;
    FILE_SIZE(fp,size);
    printf("%d\n",size);
    //size+1 to be able to add a '\0' at the end
    Text cnt;
    cnt.size = size;
    cnt.text = calloc(sizeof(char)*(size+1),size+1);
    char buffer[1024];
    //Read content
    while(fgets(buffer, 1024, fp)!=NULL){
        strcat_s(cnt.text,size,buffer);
    };
    
    cnt.text[size] = '\0';
    fclose(fp);

    return cnt;
}

void write_text_file(const char* filename, const Text* text) {
    FILE* fp; 
    fopen_s(&fp,filename, "w");
    if (fp == NULL) {
        fprintf(stderr, "Error: could not open file %s for writing\n", filename);
        exit(1);
    }

    size_t len = text->size;
    size_t bytes_written = fwrite(text->text, sizeof(char), len, fp);
    if (bytes_written != len) {
        fprintf(stderr, "Error: could not write entire file %s\n", filename);
        exit(1);
    }

    fclose(fp);
}