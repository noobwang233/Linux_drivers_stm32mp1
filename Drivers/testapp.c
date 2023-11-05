#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"

int main(int argc, char *argv[])
{
    int fd, retvalue;
    char *filename;
    unsigned char cnt = 0;
    char writebuf[1];

    if(argc != 3)
    {
        printf("Error Usage!\r\n");
        return -1;
    }

    filename = argv[1];

    /* 打开驱动文件 */
    fd = open(filename, O_RDWR);
    if(fd < 0)
    {
        printf("Can't open file %s\r\n", filename);
        return -1;
    }
    //传入cmd
    writebuf[0] = (char)atoi(argv[2]);
    retvalue = write(fd, writebuf, 1);
    if(retvalue < 0){
        printf("write file %s failed!\r\n", filename);
    }

    /* 模拟占用 25S LED */
    while(1) {
        sleep(5);
        cnt++;
        printf("App running times:%d\r\n", cnt);
        if(cnt >= 5)
            break;
    }

    /* 关闭设备 */
    retvalue = close(fd);
    if(retvalue < 0){
        printf("Can't close file %s\r\n", filename);
        return -1;
    }

    return 0;
}
