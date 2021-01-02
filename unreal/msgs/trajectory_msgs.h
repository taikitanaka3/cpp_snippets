#pragma once
#include "../define/core_define.h"
#include "../define/trajectory_define.h"
#include "../design/SingletonPattern.h"

class TrajectoryMsgs : public Singleton<TrajectoryMsgs>
{
    friend class Singleton<TrajectoryMsgs>;
    TrajectoryMsgs(){};
    TrajectoryMsgs(TrajectoryMsgs const &){};
    ~TrajectoryMsgs(){};

public:
    Loci loci = {};
};