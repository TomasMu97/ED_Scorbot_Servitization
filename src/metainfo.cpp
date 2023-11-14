#include "definitions.cpp"
#include "conversion-functions.cpp"

/**
The global metainfo of this arm.
**/
MetaInfoObject initial_metainfo()
{
    MetaInfoObject result = MetaInfoObject();
    /**
    TODO: ask Antonio about the exact robot model. Datasheets dont match and robot gets stuck at certain points.
    **/
    JointInfo j1 = JointInfo();
    j1.minimum = 155; //limit minus 10
    j1.maximum = -155; //limit minus 10
    j1.name = "J1";
    result.mi_joints.push_back(j1);

    JointInfo j2 = JointInfo();
    j2.minimum = -60; //limit
    j2.maximum = 110; //Limit
    j2.name = "J2";
    result.mi_joints.push_back(j2);

    JointInfo j3 = JointInfo();
    j3.minimum = -145; //limit minus 5
    j3.maximum = 80; //Limit minus 10
    j3.name = "J3";
    result.mi_joints.push_back(j3);
    
    JointInfo j4 = JointInfo();
    j4.minimum = -90; //Limit minus 10
    j4.maximum = 90; //Limit minus 10
    j4.name = "J4";
    result.mi_joints.push_back(j4);

    //Las ultimas 2 joints no se usan

    JointInfo j5 = JointInfo();
    j5.minimum = -180; //Limit
    j5.maximum = 180; //Limit
    j5.name = "J5";
    result.mi_joints.push_back(j5);

    JointInfo j6 = JointInfo();
    j6.minimum = -180;
    j6.maximum = 180;
    j6.name = "J6";
    result.mi_joints.push_back(j6); 

    return result;
}