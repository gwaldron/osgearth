/******************************************************************************
 * Fast DXT - a realtime DXT compression tool
 *
 * Author : Luc Renambot
 *
 * Copyright (C) 2007 Electronic Visualization Laboratory,
 * University of Illinois at Chicago
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either Version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser Public License along
 * with this library; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *****************************************************************************/

#include "libdxt.h"

#if defined(__APPLE__)
#define memalign(x,y) malloc((y))
#else
#include <malloc.h>
#endif

#include <pthread.h>

typedef struct _work_t {
	int width, height;
	int nbb;
	byte *in, *out;
} work_t;



void *slave1(void *arg)
{
	work_t *param = (work_t*) arg;
	int nbbytes = 0;
	CompressImageDXT1( param->in, param->out, param->width, param->height, nbbytes);
	param->nbb = nbbytes;
	return NULL;
}

void *slave5(void *arg)
{
	work_t *param = (work_t*) arg;
	int nbbytes = 0;
	CompressImageDXT5( param->in, param->out, param->width, param->height, nbbytes);
	param->nbb = nbbytes;	
	return NULL;
}

void *slave5ycocg(void *arg)
{
	work_t *param = (work_t*) arg;
	int nbbytes = 0;
	CompressImageDXT5YCoCg( param->in, param->out, param->width, param->height, nbbytes);
	param->nbb = nbbytes;
	return NULL;
}

int CompressDXT(const byte *in, byte *out, int width, int height, int format, int numthreads)
{
  pthread_t *pid;
  work_t    *job;
  int        nbbytes;

  if ( (numthreads!=1) && (numthreads!=2) && (numthreads!=4) ) {
	fprintf(stderr, "DXT> Errror, number of thread should be 1, 2 or 4\n");
	return 0;
  }

  job = new work_t[numthreads];
  pid = new pthread_t[numthreads];

  for (int k=0;k<numthreads;k++)
    {
      job[k].width = width;
      job[k].height = height/numthreads;
      job[k].nbb = 0;
      job[k].in =  (byte*)in  + (k*width*4*height/numthreads);
      if (format == FORMAT_DXT1)
	job[k].out = out + (k*width*4*height/(numthreads*8));
      else
	job[k].out = out + (k*width*4*height/(numthreads*4));

      switch (format) {
      case FORMAT_DXT1:
	pthread_create(&pid[k], NULL, slave1, &job[k]);
	break;
      case FORMAT_DXT5:
	pthread_create(&pid[k], NULL, slave5, &job[k]);
	break;
      case FORMAT_DXT5YCOCG:
	pthread_create(&pid[k], NULL, slave5ycocg, &job[k]);
	break;
      }
    }

  // Join all the threads
  nbbytes = 0;
  for (int k=0;k<numthreads;k++)
  {
    pthread_join(pid[k], NULL);
    nbbytes += job[k].nbb;
  }
  return nbbytes;
}
