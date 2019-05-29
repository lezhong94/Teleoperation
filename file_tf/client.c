#include <stdio.h> 
#include <string.h> 
#include <unistd.h> 
#include <ctype.h> 
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <arpa/inet.h> 
#include <stdlib.h>
#include <sys/types.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <sys/mman.h> 

#define MAXLEN 4096 

int     port = 5678;
int     to_port = 9012;
char    to_ip[] = "127.0.0.1"; 
char    startcmd[13] = "StartTransfer";
char    quitcmd[12] = "QuitTransfer";

void main(void)
{
    struct sockaddr_in local_sin; 
    struct sockaddr_in to_sin;
    int    mysock; 
        
    FILE *fp;
    
    char buf[MAXLEN];
    
    int len;
    int i;
    int    recvlen = sizeof(struct sockaddr_in),sendlen = sizeof(struct sockaddr_in);

    bzero( &local_sin, sizeof(local_sin)); 
    local_sin.sin_family = AF_INET; 
    local_sin.sin_addr.s_addr = inet_addr("127.0.0.1");
    local_sin.sin_port = htons( port ); 
        
    bzero( &to_sin, sizeof(to_sin)); 
    to_sin.sin_family = AF_INET; 
    to_sin.sin_addr.s_addr = inet_addr("127.0.0.1");
    to_sin.sin_port = htons(to_port); 

    mysock = socket( AF_INET, SOCK_DGRAM, 0 ); // 创建套接字
    bind( mysock, ( struct sockaddr * )&local_sin, sizeof(local_sin) );   // 绑定

    i = sendto(mysock,startcmd,13,0,(struct sockaddr *)&to_sin,sendlen);   // 开始传输文件
    if(i == -1){
      perror("sendto error");
    exit(-1);
    }
    // 打开文件
    if((fp=fopen("myrecv.txt","w"))==NULL)
    {
        printf("myrecv.txt open failure!!\n");
 
    }

    while(1)
    {
    bzero(buf,MAXLEN);
        len=recvfrom(mysock,buf,MAXLEN,0,(struct sockaddr *)&to_sin,&recvlen);
        if(strncmp(buf,quitcmd,12)==0)
        {
            fclose(fp);
            break;
        }
        fwrite(buf,len,1,fp);
    }
    close(mysock);
    printf("Quit\n");
}
