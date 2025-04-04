#include "zf_driver_file.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>


// 辅助函数：执行文件读写操作
int file_io_operation(const char *path, int flags, uint8 *buf, size_t size) 
{
    if (path == NULL || buf == NULL) {
        fprintf(stderr, "Invalid input parameters\n");
        return -1;
    }

    int fd = open(path, flags);
    if (fd == -1) {
        perror("Failed to open file");
        return -1;
    }

    ssize_t result;
    if (flags & O_WRONLY) {
        result = write(fd, buf, size);
    } else {
        result = read(fd, buf, size);
    }

    if (result == -1) {
        perror("File read/write error");
        close(fd);
        return -1;
    }

    if (close(fd) == -1) {
        perror("Failed to close file");
        return -1;
    }

    return 0;
}

int8 file_read_string(const char *path, char *str)
{
    int ret = 0;
	FILE *fp;

    fp = fopen(path, "r"); /* 只读打开 */
    if(fp == NULL) 
    {
		printf("can not open file %s\r\n", path);
		return -1;
	}

	ret = fscanf(fp, "%s", str);
    if(!ret)
    {
        printf("file read error!\r\n");
    } 
    else if(ret == EOF) 
    {
        fseek(fp, 0, SEEK_SET);  
    }

	fclose(fp);		
	return 0;
}

