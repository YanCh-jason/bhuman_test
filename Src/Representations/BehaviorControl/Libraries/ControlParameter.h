/**
 * @file ControlParameter.h
 */

#pragma once
#include "Tools/Math/Eigen.h"
#include "Tools/Function.h"
#include "Tools/Math/Pose2f.h"
//#include "Representations/Perception/PlayersPercepts/PlayersFieldPercept.h"
#include "Tools/Modeling/Obstacle.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Communication/TeamData.h"
#include "Modules/BehaviorControl/BehaviorControl/ParameterDefine.h"
//#include "Representations/Perception/PlayersPercepts/PlayersFieldPercept.h"








STREAMABLE(ControlParameter,
{
    //FUNCTION(bool(float value, float min, float max)) between,
	//FUNCTION(void(void)) Obstacle_deal,
    FUNCTION(Vector2f(int kind,const Vector2f& pos,const Vector2f& Ball_pose)) PosChange,
    (Vector2f)(Vector2f::Zero())Ball_GlobalPoseEs,       //the best Pose from our team
    (Vector2f)(Vector2f::Zero())Ball_GlobalPoseEs_last,
    (Vector2f)(Vector2f::Zero())Ball_RobotPose,

    (unsigned int)(0)time_now_temp,
    (bool)(false) Ball_Moved_flag,     /*球移动标志*/
    (int)(0) SinceBallWasOut_time,
    (int)(0) Ball_Seen_flag,
    (int)(0)      Other_Robot_Seen,

    (float)(0.f) theRobotToGoal_dis,
    (float)(0.f) otherRobotToGoal_dis,

    (float)(0.f) defender1_dis,
    (float)(0.f) defender2_dis,
    (float)(0.f) df_x1,
    (float)(0.f) df_x2,

    (Teammate) striker1,/**定义的角色，用于通信分角色的策略，详情看Representations/Communication/TeamData.h，分角色调用TeamdData的变量*/
    (Teammate) striker2,				/**同上，类似*/
    (Teammate) defender1,				/**类似*/
    (Teammate) defender2,				/**类似*/
    (Teammate) keeper,				/**类似*/
    
    // (bool)(false) Primary_defense,    //初级防御
    // (bool)(false) Two_level_defense,  //二级防御
    // (bool)(false) Three_level_defense,  //二级防御
    // (bool)(false) Primary_attack,    //初级防御
    // (bool)(false) Two_level_attack,  //二级防御
    // (bool)(false) Three_level_attack,  //二级防御


    (bool)(false) Obstacle_Arm,
    (bool)(false) Obstacle_Front,
    (int)(1) Obstacle_Ball,
    (bool)(false) Obstacle_Front_Near,
    (Vector2f)(Vector2f(4700,0))    Pose_Goal,
    (bool)(false)  ball_goal,
	
});
