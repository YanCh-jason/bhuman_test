option(Striker2){
    initial_state(start)
    {
    transition
    {
            if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
                    goto wait;  
            else if(theControlParameter.Ball_Moved_flag|| state_time >13000 || theBehaviorStatus.activity == 18)        
                    goto SecondKick;
    }
    action{
        OUTPUT_TEXT("start");
        LookForward();
        Stand();
    }
    }
        state(wait){
            transition{
                HeadScan(30_deg);
                if(state_time>2500)
                    goto FirstKick;
            }
        }
        state(FirstKick){
            transition{
                if(action_done)
                    goto Kick;
            }
            action{
                 HeadPanTurn(theControlParameter.Ball_RobotPose.angle()) ;
                WalkToBall(Vector2f(750,-750));
            }
        }
        state(Kick){
        transition{
            if(action_done){
                goto searchForOnBall;
                }
        }
        action{
            OUTPUT_TEXT("kick");
            kick(2);
        }
        }
        state(searchForOnBall){
            transition{
               if((state_time>1000&&theControlParameter.Ball_Seen_flag)||theBallModel.estimate.position.norm()<100){
                   goto SecondKick;}
            }
            action{
                OUTPUT_TEXT("Striker_attack::searchForOnBall");
                HeadScan(15_deg);
                Vector2f temp_pose=Transformation::robotToField(theRobotPose,theBallModel.estimate.position);
                    if(theControlParameter.Obstacle_Arm)
                        theMotionRequest=thePathPlanner.plan(Pose2f(theBallModel.estimate.position.angle(),temp_pose),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);
                    else
                        WalkToTargetDes(temp_pose);
            }
    }
        state(SecondKick){
            action{
                Striker2_attack();
            }
        }
}