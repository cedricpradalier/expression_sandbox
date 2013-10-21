
#include "rotation_errors.h"

double cerise::AccelerometerErrorQuat::G[3] = {0,0,-9.81};
double cerise::AccelerometerErrorQuat::Kdepth = -2.951575e-03;

// Extracted with igrf, from the matlab side, for the beach

double cerise::MagnetometerErrorQuat::S[3] = { 1.607130e-03, 1.683643e-03, 1.862850e-03};
