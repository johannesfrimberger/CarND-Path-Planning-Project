#include "Traffic.h"

Traffic::Traffic(const json input):
world(input[1], input[2]),
speed(input[3], input[4]),
frenet(input[5], input[6]),
id(input[0])
{
    
}
