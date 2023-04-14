#include "file_system.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Content read_text_file(const char* filename){
    //Open file and create a buffer of the corresponding size
    FILE* fp = fopen(filename,"r");
    long size;
    FILE_SIZE(fp,size);
    printf("%d\n",size);
    //size+1 to be able to add a '\0' at the end
    Content cnt;
    cnt.size = size;
    cnt.text = calloc(sizeof(char)*(size+1),size+1);
    char buffer[1024];
    //Read content
    while(fgets(buffer, 1024, fp)!=NULL){
        strcat(cnt.text,buffer);
    };
    
    cnt.text[size] = '\0';
    fclose(fp);

    return cnt;
}