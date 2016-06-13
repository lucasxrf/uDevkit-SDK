/**
 * @file asserv_dspic.c
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2016
 *
 * @date avril 28, 2016, 15:10  
 * 
 * @brief Support for motor control with positionning
 */

void setup_timer1()
{
	T1CON  = 0b1000000000100000;	// FCY / 64
	PR1 = 625;			// 1kHz
	IEC0bits.T1IE = 1;
}

#include <math.h>
#define M_PI		3.14159265358979323846

unsigned short i=0;
int tabc2[1000] __attribute__ ((far));;

long int c1, c2, cons;
long int v1, v2;
long int ancv1=0, ancv2=0;
long int ancc1=0, ancc2=0;
float dt,ds,tand;
float x=1500, y=1000, t=0;
#define ENTRAX 195.0
//#define TICK1  0.006731785714	// 60mm wheel
#define TICK1  0.00849123461395001864975755315181	// 72.5mm wheel

int xi, yi;

long int err, preverr=0;
long int pid;

void locTask()
{
	c1 = getCoder1();
	c2 = getCoder2();
	
	// loc
	v1=c1-ancc1;
	v2=-c2+ancc2;
	
	dt = atan((v2-v1)*TICK1/ENTRAX);
	ds = (v1 + v2)*(TICK1/2);
	x += ds * cos(t+dt*.5);
	y -= ds * sin(t+dt*.5);
	t -= dt;
	if(t>M_PI) t-=2*M_PI;
	if(t<-M_PI) t+=2*M_PI;
	
	xi = x;
	yi = y;

	ancc1=c1;
	ancc2=c2;
	ancv1=v1;
	ancv2=v2;
}

unsigned short k = 8, p = 4;
void __attribute__ ((interrupt,no_auto_psv)) _T1Interrupt(void)
{
	locTask();
	
	// log
	if(i<1000)
	{
		tabc2[i]=c2>>4;
		i++;
	}
	else
	{
		err=0;
	}
	
	cons = c1;
	
	err = c2 - cons;
	pid = err * k + (err - preverr) * p;
	
	if(pid<1000 && pid>-1000) pid=0;
	setMotor1(pid/4);
	
	preverr = err;
	IFS0bits.T1IF = 0;
}

int getXPos()
{
	return (int)x;
}

int getYPos()
{
	return (int)y;
}

int getTPos()
{
	return (int)(180.0*t/3.1415);
}