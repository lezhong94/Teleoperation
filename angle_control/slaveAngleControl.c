//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
//  Version 3.6.0

#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "drdc.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define LOCAL_PORT 8888
#define CLIE_PORT 39775
#define MAXLINE 128
#define MASTER_IP ""



#define DEFAULT_K 1.0  // 1.0 12.0 14.0 16.0
#define N 3
#define DELAYCONST 101 // max 2001 141 121 101
#define ALPHA 0.08 // 0.08
#define ENC 20300 // encoder reading corresponding to origin

struct parameters
{
	float j0;
	float j1;
	float j2;
	float v1;
	float v2;
	float v3;
}

struct parameters *split(char str[],char delims[], struct parameters *p)
{
	char *result = NULL;
	float x;
	float pa[10];
	result = strtok( str, delims);
	for(int i=1; i<= 6;i++)
	{
		x = atoi(result);
		co[i] = x;
		result = strtok(NULL, delims);
	}
	struct parameters z;
	z.j0 = co[1];
	z.j1 = co[2];
	z.j2 = co[3];
	z.v1 = co[4];
	z.v2 = co[5];
	z.v3 = co[6];
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

int main(int  argc,char **argv)
{
	int    done = 0;

	double Kp, Kv, Kd;

	double sp[N] = { 0.0 }; // end-effector position
	double sj[N] = { 0.0 }; // joint angle
	double sv[N] = { 0.0 }; // angular velocity

	double mj[N] = { 0.0 };
	double mv[N] = { 0.0 };

	double svback[N] = { 0.0 }; // last step angular velocity
	double sjback[N] = { 0.0 }; // list of backstep joint angle to simulate delay

	double st[N]; // output torque
	double sq[N]; // output torque for gravity compensation

	double backTime; // list of time for each backstep
	double refTime = dhdGetTime();
	double tempTime;

	//variables about udp 
	int slavesock;
	int addr_len;
	int sjv,mjv;
	char buff[MAXLINE];
	
	slavesock = create_socket();
	
	seraddr.sin_family = AF_INET; 
	seraddr.sin_port   = htons(LOCAL_PORT);
	seraddr.sin_addr.s_addr = htonl(INADDR_ADY);
	bind(matersock, (struct sockaddr*)&seraddr, sizeof(seraddr));

	bezero(&cliaddr, sizeof(cliaddr));
	cliaddr.sin_famliy = AF_INET;
	cliaddr.sin_port = htons(SERV_PORT);
	inet_pton(AF_INET, MASTER_IP , &cliaddr.sin_addr);
	addr_len = sizeof(cliaddr);
	

	Kp = DEFAULT_K; // start kgain with a quarter of default k
	Kv = Kp / 800;
	Kd = 0.25e-3*(DELAYCONST - 1)*Kp;

	drdOpen();
	drdStart();

	dhdEnableExpertMode();

	// center both devices
	drdMoveToEnc(ENC, ENC, ENC, true);

	// stop regulation on master, stop motion filters on slave
	drdStop(true);
	dhdSetForce(0.0, 0.0, 0.0);

	dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2]);

	// initialize lists of backstep joint angle
	for (int i = 0; i < N; i++) {
		sjback[i] = sj[i];
	}

	// function for preparation


	// loop while the button is not pushed
	while (!done) 
	{	
		struct parameters s;
		s.j0 = sj[0];
		s.j1 = sj[1];
		s.j2 = sj[2];
		s.v0 = sv[0];
		s.v1 = sv[1];
		s.v2 = sv[2];

		void*buf; 
		char str[1024];
		buf = &str;
		sprintf(str, "%f,%f,%f,%f,%f,%f", s.j0, s.j1, s.j2, s.v0,s.v1,s.v2);

		// get slave's position and velocity
		dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2]);
		dhdDeltaGravityJointTorques(sj[0], sj[1], sj[2], &sq[0], &sq[1], &sq[2]);

		// calculate backstep time 
		tempTime = dhdGetTime();
		backTime = tempTime - refTime;
		refTime = tempTime;

		// calculate slave's velocity
		for (int i = 0; i < N; i++) {
			sv[i] = (sj[i] - sjback[i]) / backTime * ALPHA + (1 - ALPHA)*svback[i];
			sjback[i] = sj[i];
			svback[i] = sv[i];
		}

		
		//send sj sv to slave
		sjv = sendto(slavesock, buf, MAXLINE, (struct sockaddr*)&cliaddr, addr_len);
		if(sjv < 0)
		{
		perror("sendto error:\n");
		break;
		}
		
		// funtion to get mj and mv
		mjv = recvfrom(slavesock,buff,MAXLINE,0,(struct sockaddr*)&cliaddr,&addr_len);
		p = split(buff,delims,p);
		mj[0] = p->j0;
		mj[1] = p->j1;
		mj[2] = p->j2;
		mv[0] = p->v0;
		mv[1] = p->v1;
		mv[2] = p->v2;		
		// calculate slave's joint torque 
		for (int i = 0; i < N; i++) 
		{
			st[i] = Kv * (mv[i] - sv[i]) - Kd * sv[i] + Kp * (mj[i] - sj[i]) + sq[i];
		}

		dhdSetDeltaJointTorques(st[0], st[1], st[2]);
		dhdGetPosition(&sp[0], &sp[1], &sp[2]);

		printf("sf = %0.03f [kHz] \r", dhdGetComFreq());

		// function to receive stop singal

	}

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
