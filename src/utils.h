#pragma once

#include <math.h>

float softplus(float x) { return log(1 + exp(x)); }

float softlin(float x) { return (tanh(x - 2) + 1) / 2; }
