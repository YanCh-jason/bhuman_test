option(Striker5){
    bool temp;
    static bool flag=false;
    common_transition{
        if(flag==false){
        Vector2f temp_pose;
        temp_pose.x()=2500-theRobotPose.translation.x();
        temp_pose.y()=-1200-theRobotPose.translation.y();
        
        if(temp_pose.norm()<50)
            goto turn;
        }
        if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber&&option_time>10000)
            goto SecondKick;

    }
    initial_state(start)
    {
        transition
        {       if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
                    flag=false;
                else
                    flag=true;
                if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
                        goto walk;  
                else if(theControlParameter.Ball_Moved_flag || state_time >13000 || theBehaviorStatus.activity == 18)        
					goto SecondKick;
        }
        action{
            OUTPUT_TEXT("start");
            LookForward();
            Stand();
            }
    }
        state(walk){
            transition{
                if(theControlParameter.Obstacle_Arm)
                    goto planWalk;
            }
            action{
                 HeadScan(20_deg);
                WalkToTargetDes(Vector2f(1200,-2500));
            }
        }
        state(planWalk){
            transition{
                if(!theControlParameter.Obstacle_Arm)
                    goto walk;
            }
            action{
                 HeadScan(30_deg);
              theMotionRequest=thePathPlanner.plan(Pose2f(0,Vector2f(1200,-2500)),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);
            }
        }
        state(turn){
            transition{
                if(theControlParameter.Ball_RobotPose.angle()<10_deg){
                    flag=true;
                    goto SecondKick;
                }
            }
            action{
                WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theControlParameter.Ball_RobotPose.angle(),0,0));
            }
        }
        state(SecondKick){
            action{
                Striker5_attack();
            }
        }
}