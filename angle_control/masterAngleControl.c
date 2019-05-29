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

#define LOCAL_PORT 39775
#define SERV_PORT 8888
#define MAXLINE 128
#define SLAVE_IP ""



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



int  main(int  argc,char **argv)
{
	int    done = 0;
         
	double Kp, Kv, Kd;

	double mp[N] = { 0.0 }; // end-effector position
	double mj[N] = { 0.0 }; // joint angle
	double mv[N] = { 0.0 }; // angular velocity

	double sj[N] = { 0.0 };
	double sv[N] = { 0.0 };

	double mvback[N] = { 0.0 }; // last step angular velocity
	double mjback[N] = { 0.0 }; // list of backstep joint angle to simulate delay

	double mt[N]; // output torque
	double mq[N]; // output torque for gravity compensation

	double backTime; // list of time for each backstep
	double refTime = dhdGetTime();
	double tempTime;

	//variables about udp 
	int mastersock;
	int addr_len;
	int mjv,slv;
	char buff[MAXLINE];
	

	mastersock = create_socket();
	
	struct sockaddr_in clieaddr,seraddr;
	
	cliaddr.sin_family = AF_INET; 
	cliaddr.sin_port   = htons(LOCAL_PORT);
	cliaddr.sin_addr.s_addr = htonl(INADDR_ADY);
	bind(matersock, (struct sockaddr*)&cliaddr, sizeof(cliaddr));

	bezero(&seraddr, sizeof(seraddr));
	seraddr.sin_famliy = AF_INET;
	seraddr.sin_port = htons(SERV_PORT);
	inet_pton(AF_INET, SLAVE_IP , &seraddr.sin_addr);
	addr_len = sizeof(seraddr);
	
	
	

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

	dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2]);

	// initialize lists of backstep joint angle
	for (int i = 0; i < N; i++) {
		mjback[i] = mj[i];
	}

	// function for preparation
	


	// loop while the button is not pushed
	while (!done)
       	{

        struct parameters m;
	m.j0 = mj[0];
	m.j1 = mj[1];
	m.j2 = mj[2];
	m.v0 = mv[0];
	m.v1 = mv[1];
	m.v2 = mv[2];

	void*buf; 
	char str[1024];
	buf = &str;
	sprintf(str, "%f,%f,%f,%f,%f,%f", m.j0, m.j1, m.j2, m.v0,m.v1,m.v2);


	// get master's position and velocity
	dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2]);
	dhdDeltaGravityJointTorques(mj[0], mj[1], mj[2], &mq[0], &mq[1], &mq[2]);

	// calculate backstep time 
	tempTime = dhdGetTime();
	backTime = tempTime - refTime;
	refTime = tempTime;

	// calculate master's velocity
	for (int i = 0; i < N; i++)
       	{

		mv[i] = (mj[i] - mjback[i]) / backTime * ALPHA + (1 - ALPHA)*mvback[i];
		mjback[i] = mj[i];
		mvback[i] = mv[i];
	}

	//send mj mv to slave
	mjv = sendto(mastersock, buf, MAXLINE, (struct sockaddr*)&seraddr, addr_len);
	if(mjv < 0)
	{
		perror("sendto error:\n");
		break;
	}
	//echo sjv from slave 
	
	
	// funtion to get sj and sv
	sjv = recvfrom(mastersock,buff,MAXLINE,0,(struct sockaddr*)&seraddr,&addr_len);
	p = split(buff,delims,p);
	sj[0] = p->j0;
	sj[1] = p->j1;
	sj[2] = p->j2;
	sv[0] = p->v0;
	sv[1] = p->v1;
	sv[2] = p->v2;


	// calculate master's joint torque
	for (int i = 0; i < N; i++)
       	{
		mt[i] = Kv * (sv[i] - mv[i]) - Kd * mv[i] + Kp * (sj[i] - mj[i]) + mq[0];
	}

	dhdSetDeltaJointTorques(mt[0], mt[1], mt[2]);
	dhdGetPosition(&mp[0], &mp[1], &mp[2]);

	printf("mf = %0.03f [kHz] \r", dhdGetComFreq());

	if (dhdGetButtonMask()) done = 1;
	if (dhdKbHit())
       	{
		switch (dhdKbGet())
	       	{
	
			case 'q':
			done = 1;  
	                break;
			
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
