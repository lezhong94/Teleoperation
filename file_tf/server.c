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

int     port = 9012;
int     to_port = 5678;
char    to_ip[] = "127.0.0.1"; 
char    startcmd[13] = "StartTransfer";
char    quitcmd[12] = "QuitTransfer";

int main(void)
{
    struct sockaddr_in local_sin; 
    struct sockaddr_in to_sin;
    int    mysock; 
        
    FILE *fp;
    int filelen;
    
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
    to_sin.sin_addr.s_addr = inet_addr(to_ip); 
    to_sin.sin_port = htons(to_port); 

    mysock = socket( AF_INET, SOCK_DGRAM, 0 ); // 创建套接字
    int bind_stats = bind( mysock, ( struct sockaddr * )&local_sin, sizeof(local_sin) );   // 绑定
    if(bind_stats == -1){
    perror("bind error");
    exit(-1);
    }
    len=recvfrom(mysock,buf,MAXLEN,0,(struct sockaddr *)&to_sin,&recvlen);
    if(len == -1){
      perror("recvfrom error");
    exit(-1);
    }
    if(strncmp(buf,startcmd,13)==0)  // 开始传输
    {
        // 打开文件
    if((fp=fopen("mysend.txt","r"))==NULL)
        {
            printf("mysend.txt open failure!!\n");
            return -1;
        }
        fseek(fp,0,SEEK_END);
        filelen = ftell(fp);
        fseek(fp,0,SEEK_SET);
           printf( "filelen = %d\n",filelen); 

           while ( 1 ) 
        { 
            bzero(buf,MAXLEN);
            fread(buf,MAXLEN,1,fp);
            if(filelen>=MAXLEN)
            {
            len = sendto(mysock,buf,MAXLEN,0,(struct sockaddr *)&to_sin,sendlen);
            if(len < 0)
            {
                printf("send failure!!\n");
                break;
            }
            filelen-=MAXLEN;
            }
            else
            {
            sendto(mysock,buf,filelen,0,(struct sockaddr *)&to_sin,sendlen);
            if(len < 0)
            {
                printf("send failure!!\n");
                break;
            }
            sendto(mysock,quitcmd,strlen(quitcmd),0,(struct sockaddr *)&to_sin,sendlen);
            break;
            }    
        }
    } 
    printf("Quit\n");
    return 0;
}
