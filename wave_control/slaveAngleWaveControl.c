//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
//  Version 3.6.0


#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "drdc.h"

#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define N 3
#define ENC 20300 // encoder reading corresponding to origin

#define WAVEIMPEDANCE 0.08 // from 0.01 to 1.0; 0.05 is the min value to stablize the system at no-delay case
#define DEFAULT_K 20.0  
#define ALPHA 1.0e-1 // velocity filter parameter
#define BETA 1.0e-4 // wave velocity filter parameter
#define D 1.0 // wave filter parameter

#define LOCAL_PORT 39775
#define CLIE_PORT 39775
#define MAXLINE 128
#define MASTER_IP "155.33.198.234"

struct parameters
{
	double s0;
	double s1;
	double s2;
};

struct parameters *split(char str[],char delims[], struct parameters *p)
{
	char *result = NULL;
	double x;
	double pa[10];
	result = strtok( str, delims);
	for(int i=1; i<= 3;i++)
	{
		x = atof(result);
		pa[i] = x;
		result = strtok(NULL, delims);
	}
	struct parameters z;
	z.s0 = pa[1];
	z.s1 = pa[2];
	z.s2 = pa[3];
	p = &z;
	return p;
}

int create_socket(void)
{
	int socketfd;
	socketfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketfd == -1)
	{
		perror("create socket error:");
		exit (1);
	}
	printf("create socket successfully!\n");
	return socketfd;
}



int main(int  argc, char **argv)
{
	int    done = 0;

	double Kp = DEFAULT_K; // 1e3;
	double Kv = Kp / 200;
	double b = WAVEIMPEDANCE;

	double sp[N] = { 0.0 };
	double sv[N] = { 0.0 };
	double sj[N] = { 0.0 };

	double us[N] = { 0.0 };
	double vs[N] = { 0.0 };
	double um[N] = { 0.0 };

	double umDelay[N] = { 0.0 };

	double sjback[N];
	double svbackv[N] = { 0.0 };

	double sjd[N];
	double svd[N] = { 0.0 };

	double usv[N] = { 0.0 };
	double usvback[N] = { 0.0 };

	double usback[N] = { 0.0 };

	double st[N];
	double sq[N];

	double sTempTime;
	double sLastTempTime;
	double sCycleTime;

	double delayTime = 0.0001;
	
	//variables about udp 
	int slavesock;
	int mc,sc;
	char buff[MAXLINE];
	char delims[10] = ",";
	struct parameters *p;

	slavesock = create_socket();
	
	struct sockaddr_in cliaddr,seraddr;
	
	seraddr.sin_family = AF_INET; 
	seraddr.sin_port   = htons(LOCAL_PORT);
	seraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	bind(slavesock, (struct sockaddr*)&seraddr, sizeof(seraddr));

	bzero(&cliaddr, sizeof(cliaddr));
	socklen_t cliaddr_len;
	cliaddr.sin_family = AF_INET;
	cliaddr.sin_port = htons(CLIE_PORT);
	inet_pton(AF_INET, MASTER_IP , &cliaddr.sin_addr);
	cliaddr_len = sizeof(cliaddr);

	// open device
	if (drdOpen() < 0) 
	{
		printf("error: not enough devices found\n");
	}

	// start robot control loop
	if (drdStart() < 0) 
	{
		printf("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr());
	}

	dhdEnableExpertMode();

	// center both devices
	drdMoveToEnc(ENC, ENC, ENC, true);

	// stop regulation on master, stop motion filters on slave
	drdStop(true);
	dhdSetForce(0.0, 0.0, 0.0);

	// initialize lists of backstep joint angle	
	dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2]);
	for (int i = 0; i < N; i++) 
	{
		sjback[i] = sj[i];
		sjd[i] = sj[i];
	}

	// function for preparation


	sLastTempTime = dhdGetTime();

	while (!done) 
	{

		// get position and gravity compensation torque
		dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2]);
		dhdDeltaGravityJointTorques(sj[0], sj[1], sj[2], &sq[0], &sq[1], &sq[2]);

		// get step time
		sTempTime = dhdGetTime();
		sCycleTime = sTempTime - sLastTempTime;
		sLastTempTime = sTempTime;
		
		// calculate velocity
		for (int i = 0; i < N; i++) 
		{
			sv[i] = (sj[i] - sjback[i]) / sCycleTime * ALPHA + (1 - ALPHA)*svbackv[i];
			sjback[i] = sj[i];
			svbackv[i] = sv[i];
		}

		// receive um[N]
		mc = recvfrom(slavesock,buff, MAXLINE , 0, (struct sockaddr*)&cliaddr, &cliaddr_len);
		p = split(buff,delims,p);
		um[0] = p->s0;
		um[1] = p->s1;
		um[2] = p->s2;
		
		
		
		// caculate feedback torque from wave variables
		for (int i = 0; i < N; i++) {
			usv[i] = ((um[i] - usback[i]) / sCycleTime * BETA + (1 - BETA)*usvback[i]) / (1 + D * delayTime / sCycleTime * BETA);
			usvback[i] = usv[i];
			us[i] = um[i] - usv[i] * D*delayTime;
			usback[i] = us[i];
			sjd[i] += svd[i] * sCycleTime;
			svd[i] = (sqrt(2 * b)*us[i] + Kv * sv[i] + Kp * (sj[i] - sjd[i])) / (Kv + b);
			st[i] = Kv * (svd[i] - sv[i]) + Kp * (sjd[i] - sj[i]);
			vs[i] = us[i] - sqrt(2 / b)*st[i];
		}

		// send vs[N]
		struct parameters s;
		s.s0 = vs[0];
		s.s1 = vs[1];
		s.s2 = vs[2];

		void*buf;
		char str[1024];
		buf = &str;
		sprintf(str , "%f,%f,%f" , s.s0, s.s1,s.s2);
		sc = sendto(slavesock, buf, MAXLINE, 0, (struct sockaddr*)&cliaddr, cliaddr_len);
		if(sc <0)
		{
			perror("sendto error:\n");
			break;
		}
		
		




		dhdSetDeltaJointTorques(st[0] + sq[0], st[1] + sq[1], st[2] + sq[2]);

		dhdGetPosition(&sp[0], &sp[1], &sp[2]);
		printf("sf = %0.03f [kHz] \r", dhdGetComFreq());

		
		
	}

	// function to stop link


	// report exit cause
	printf("                                                                           \r");
	if (done == -1) printf("\nregulation finished abnormally on slave device\n");
	else            printf("\nexiting on user request\n");

	// close the connection
	drdClose();

	// exit
	printf("\ndone.\n");
	return 0;

}

