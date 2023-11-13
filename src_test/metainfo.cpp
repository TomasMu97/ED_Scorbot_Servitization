#include "definitions.cpp"
#include "conversion-functions.cpp"

/**
 * The global metainfo of this arm.
 *
 * TODO: Adjust only the reference values (the second argument of ref_to_angle)
 * for each motor
 *
 **/
MetaInfoObject initial_metainfo()
{
    MetaInfoObject result = MetaInfoObject();

    //TODO: ask Antonio about the exact robot model. datasheets dont match and robot gets stuck at certain points.

    JointInfo j1 = JointInfo();
    j1.minimum = 160;
    j1.maximum = -160;
    j1.name = "J1";
    result.mi_joints.push_back(j1);

    JointInfo j2 = JointInfo();
    j2.minimum = 90;
    j2.maximum = -120;
    j2.name = "J2";
    result.mi_joints.push_back(j2);

    JointInfo j3 = JointInfo();
    j3.minimum = 170;
    j3.maximum = -60;
    j3.name = "J3";
    result.mi_joints.push_back(j3);
    
    JointInfo j4 = JointInfo();
    j4.minimum = 170;
    j4.maximum = -170;
    j4.name = "J4";
    result.mi_joints.push_back(j4);
/*
    JointInfo j5 = JointInfo();
    j5.minimum = 20;
    j5.maximum = -200;
    j5.name = "J5";
    result.mi_joints.push_back(j5);

    JointInfo j6 = JointInfo();
    j6.minimum = 360;
    j6.maximum = -360;
    j6.name = "J6";
    result.mi_joints.push_back(j6);
*/    
    return result;
}