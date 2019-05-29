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
#define SERV_PORT 39775
#define MAXLINE 128
#define SLAVE_IP "155.33.198.174"

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

	double mp[N] = { 0.0 };
	double mv[N] = { 0.0 };
	double mj[N] = { 0.0 };

	double um[N] = { 0.0 };
	double vm[N] = { 0.0 };
	double vs[N] = { 0.0 };

	double vsDelay[N] = { 0.0 };

	double mjback[N];

	double mvbackv[N] = { 0.0 };

	double vmv[N] = { 0.0 };
	double vmvback[N] = { 0.0 };

	double vmback[N] = { 0.0 };

	double mt[N];
	double mq[N];

	double mTempTime;
	double mLastTempTime;
	double mCycleTime;

	double delayTime = 0.0001;

	//variables about udp 
	int mastersock;
	int mc,sc;
	char buff[MAXLINE];
	char delims[10] = ",";
	struct parameters *p;

	mastersock = create_socket();
	
	struct sockaddr_in cliaddr,seraddr;
	
	cliaddr.sin_family = AF_INET; 
	cliaddr.sin_port   = htons(LOCAL_PORT);
	cliaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	bind(mastersock, (struct sockaddr*)&cliaddr, sizeof(cliaddr));

	bzero(&seraddr, sizeof(seraddr));
	socklen_t seraddr_len;
	seraddr.sin_family = AF_INET;
	seraddr.sin_port = htons(SERV_PORT);
	inet_pton(AF_INET, SLAVE_IP , &seraddr.sin_addr);
	seraddr_len = sizeof(seraddr);


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
	dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2]);
	for (int i = 0; i < N; i++) 
	{
		mjback[i] = mj[i];
	}

	// function for preparation


	mLastTempTime = dhdGetTime();

	while (!done) 
	{

		// get position and gravity compensation torque
		dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2]);
		dhdDeltaGravityJointTorques(mj[0], mj[1], mj[2], &mq[0], &mq[1], &mq[2]);

		// get step time
		mTempTime = dhdGetTime();
		mCycleTime = mTempTime - mLastTempTime;
		mLastTempTime = mTempTime;

		// calculate velocity
		for (int i = 0; i < N; i++) {
			mv[i] = (mj[i] - mjback[i]) / mCycleTime * ALPHA + (1 - ALPHA)*mvbackv[i];
			mjback[i] = mj[i];
			mvbackv[i] = mv[i];
		}
		
		
		
		
		// caculate feedback torque from wave variables
		for (int i = 0; i < N; i++) {
			vmv[i] = ((vs[i] - vmback[i]) / mCycleTime * BETA + (1 - BETA)*vmvback[i]) / (1 + D * delayTime / mCycleTime * BETA);
			vmvback[i] = vmv[i];
			vm[i] = vs[i] - vmv[i] * D* delayTime;
			vmback[i] = vm[i];
			mt[i] = -(b * mv[i] - sqrt(2 * b)*vm[i]);
			um[i] = sqrt(2 * b)*mv[i] - vm[i];
		}
		
		// send um[N]
		struct parameters m;
		m.s0 = um[0];
		m.s1 = um[1];
		m.s2 = um[2];

		void*buf;
		char str[1024];
		buf = &str;
		sprintf(str , "%f,%f,%f" , m.s0, m.s1,m.s2);
		
		mc = sendto(mastersock, buf, MAXLINE, 0, (struct sockaddr*)&seraddr, seraddr_len);
		if(mc <0)
		{
			perror("sendto error:\n");
			break;
		}								
		
		// receive vs[N]
		sc = recvfrom(mastersock,buff,MAXLINE , 0, (struct sockaddr*)&seraddr, &seraddr_len);
		p = split(buff,delims,p);
		vs[0] = p->s0;
		vs[1] = p->s1;
		vs[2] = p->s2;

		


		dhdSetDeltaJointTorques(mt[0] + mq[0], mt[1] + mq[1], mt[2] + mq[2]);

		dhdGetPosition(&mp[0], &mp[1], &mp[2]);
		printf("mf = %0.03f [kHz] \r", dhdGetComFreq());

		if (dhdGetButtonMask()) done = 1;
		if (dhdKbHit()) 
		{
			switch (dhdKbGet()) 
			{
				case 'q': done = 1;   break;
			}
		}
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

