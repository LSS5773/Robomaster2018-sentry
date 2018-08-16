#ifndef __FILTER_H
#define __FILTER_H	 

typedef struct{
	double raw_value;
	double xbuf[5];
	double ybuf[5];
	double filtered_value;
}Filter_t;

extern Filter_t MPUz50Hz,MPUy50Hz,MPUx50Hz;
extern Filter_t PIDOUTPUT50Hz;
void Chebyshev50HzLPF(Filter_t *F);
#endif
