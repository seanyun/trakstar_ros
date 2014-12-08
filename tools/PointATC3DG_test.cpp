/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

/** 
 * Edited by: Sean Seungkook Yun <seungkook.yun@sri.com> 
*/

#include <cstdio>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <ctime>
#include <time.h>
#include <unistd.h>
#include "trakstar/PointATC3DG.hpp"

using namespace trakstar;

struct termios oldterm, newterm;
void set_unbuffered ( void ) {
  tcgetattr( STDIN_FILENO, &oldterm );
  newterm = oldterm;
  newterm.c_lflag &= ~( ICANON | ECHO );
  tcsetattr( STDIN_FILENO, TCSANOW, &newterm );
}
void set_buffered ( void ) {
  tcsetattr( STDIN_FILENO, TCSANOW, &oldterm );
}

// kbhit() function
int kbhit ( void ) {
    int result;
    fd_set  set;
    struct timeval tv;

    FD_ZERO(&set);
    FD_SET(STDIN_FILENO,&set);  /* watch stdin */
    tv.tv_sec = 0;
    tv.tv_usec = 0;             /* don't wait */

    /* quick peek at the input, to see if anything is there */
    set_unbuffered();
    result = select( STDIN_FILENO+1,&set,NULL,NULL,&tv);
    set_buffered();

    return result == 1;
}
int main( int argc, char** argv )
{
	PointATC3DG bird;

	if( !bird ) return -1;	

	double dX, dY, dZ, dAzimuth, dElevation, dRoll;
	double* quat=new double[4];
	
	int numsen=bird.getNumberOfSensors();
	printf( "nSensors: %d\n", numsen );

	for (int i=0; i<numsen; i++) {
		bird.setSuddenOutputChangeLock(i);
		bird.setSensorQuaternion(i);
	}

	bird.setMaximumRange(true);

	
	printf( "    X       Y       Z	   Qw    Qx    Qy    Qz\n" );
	int rec_count=0;	 
	time_t ta=time(NULL);
	while (!kbhit()) 
	{	
		rec_count++;
	//	usleep( 100000 );
		for( int i = 0; i <numsen  ; ++i ) 
		{
			bird.getCoordinatesQuaternion(i, dX, dY, dZ, quat);
			if (i==0) printf( "%i %+6.4f  %+6.4f  %+6.4f  %+4.3f  %+4.3f  %+4.3f   %+4.3f\r",i,
				dX, dY, dZ, quat[0], quat[1], quat[2], quat[3] );
		}
	}

	delete [] quat;

	time_t tb=time(NULL);
	printf("\n%i samples collected\n",rec_count);

	return 0;
}


