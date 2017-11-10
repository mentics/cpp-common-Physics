#pragma once

#include "MenticsCommon.h"
#include "MenticsMath.h"

namespace MenticsGame {

struct PosVel {
	vect3 pos;
	vect3 vel;
};
PTRS(PosVel)

struct PosVelAcc {
	vect3 pos;
	vect3 vel;
	vect3 acc;
};
PTRS(PosVelAcc)

}