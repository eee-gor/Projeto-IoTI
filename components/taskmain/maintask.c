#include <stdio.h>

#include "maintask.h"

float calcRMS(float *Temperatura, int cont)
{
	float rms, soma = 0;
	int i;

	for (i = 0; i < cont; i++)
	{
		soma = (powf(Temperatura[i], 2)) + soma;
	}

	rms = sqrtf(soma / cont);

	return rms;
}