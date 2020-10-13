option(Striker2_attack){
   static bool judge_Goal1=false,judge_Goal2=false,judge_Goal3=false;
   static Vector2f Goal_pose_t;
   static bool judge_kick=false;
   static int time,flag=0;
   static int time_now;
    static int test_flag1=0;
   static float tilt_angle=0;
   common_transition{
        if(theArmContactModel.status[Arms::left].contact == true || theArmContactModel.status[Arms::right].contact == true||theControlParameter.Obstacle_Arm) 
        {          
            time = theFrameInfo.time;
            theArmMotionRequest.armMotion[Arms::left] = theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
            theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;
        }
        if((theFrameInfo.time - time) > 5000)
        {
       
            theArmMotionRequest.armMotion[Arms::left] = theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;
            theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::useDefault;
        
        }
        if(!judge_kick){
            if(!theControlParameter.Ball_Seen_flag){
                flag=0;
                goto SearchForBallNoBall;
            }
            if(theControlParameter.Ball_GlobalPoseEs.x()<10&&theControlParameter.Ball_RobotPose.norm()>1000){
                if(flag!=3){
                    flag=3;
                goto stayForBall;}
            }
            else if(theControlParameter.theRobotToGoal_dis<1000){
                if(flag!=1){
                    flag=1;
                     OUTPUT_TEXT("1000");
                    goto walkToBall;}
            }
            else if(std::abs(theControlParameter.theRobotToGoal_dis-theControlParameter.otherRobotToGoal_dis)<200){
                Vector2f temp_pose1,temp_pose2,temp_pose3;
                if(theFrameInfo.getTimeSince(theControlParameter.striker2.timeWhenLastPacketReceived)>7000){
                    if(flag!=1){
                        flag=1;
                        goto walkToBall;}
                }
                else{
                    temp_pose1.x()=theControlParameter.Ball_GlobalPoseEs.x()-4700;
                    temp_pose1.y()=theControlParameter.Ball_GlobalPoseEs.y()-0;
                    temp_pose2.x()=theRobotPose.translation.x()-theControlParameter.Ball_GlobalPoseEs.x();
                    temp_pose2.y()=theRobotPose.translation.y()-theControlParameter.Ball_GlobalPoseEs.y();
                    temp_pose3.x()=theControlParameter.striker2.theRobotPose.translation.x()-theControlParameter.Ball_GlobalPoseEs.x();
                    temp_pose3.y()=theControlParameter.striker2.theRobotPose.translation.y()-theControlParameter.Ball_GlobalPoseEs.y();
                    if(std::abs(temp_pose1.angle()-temp_pose2.angle())<std::abs(temp_pose1.angle()-temp_pose3.angle())){
                        if(flag!=1){
                            flag=1;
                            goto walkToBall;}
                    }
                    else{
                        if(flag!=2){
                            flag=2;
                        test_flag1=1;
                        goto searchForBallStay;
                        }
                    }
                }
            }
            else if(theControlParameter.theRobotToGoal_dis<theControlParameter.otherRobotToGoal_dis){
                if(flag!=1){
                    flag=1;
                    goto walkToBall;}
            }
            else{
                if(flag!=2){
                    flag=2;
                    test_flag1=2;
                    goto searchForBallStay;
                    }
                }
        }
   }
    initial_state(start){
        transition{
            judge_kick=false;
            flag=0;
           OUTPUT_TEXT("ok");
        }
    }


    state(SearchForBallNoBall){
        action{
            OUTPUT_TEXT("Striker_attack::SearchForBallNoBall");
            SearchForBallNoBall(2);
        }
    }


    state(searchForBallStay){
        action{
            OUTPUT_TEXT("Striker_attack::searchForBallStay");
            OUTPUT_TEXT(test_flag1);
            searchForBallStay(2);
        }
    }
    state(walkToBall){
        transition{
            if(action_done){
                judge_kick=true;
                judge_Goal2=false;
                judge_Goal1=false;
                judge_Goal3=false;
                goto Kick;
            }
        }
        action{
            if(theControlParameter.Ball_RobotPose.norm()>=2000)
                tilt_angle=0_deg;    
            else 
                tilt_angle=19_deg;    

                
            if(std::abs(theControlParameter.Ball_RobotPose.angle())<10_deg&&theControlParameter.Ball_RobotPose.norm()>600)
                HeadScan(10_deg,tilt_angle);
            else
                HeadPanTurn(theControlParameter.Ball_RobotPose.angle(),tilt_angle) ;          
            if(theBallModel.estimate.position.norm()<700&&!judge_Goal1){
                OUTPUT_TEXT("judge_Goal");
                Goal_pose_t=theControlParameter.PosChange(1,Vector2f(0,0),Vector2f(0,0));
                judge_Goal1=true;
            }
            if(theBallModel.estimate.position.norm()<500&&!judge_Goal2){
                OUTPUT_TEXT("judge_Goal");
                Goal_pose_t=theControlParameter.PosChange(1,Vector2f(0,0),Vector2f(0,0));
                judge_Goal2=true;
            }
            if(theBallModel.estimate.position.norm()<300&&!judge_Goal3){
                OUTPUT_TEXT("judge_Goal");
                Goal_pose_t=theControlParameter.PosChange(1,Vector2f(0,0),Vector2f(0,0));
                judge_Goal3=true;
            }
            if(judge_Goal1||judge_Goal2||judge_Goal3)
                  WalkToBall(Goal_pose_t);  
            else{    
                if(state_time<800){
                    Goal_pose_t=theControlParameter.PosChange(1,Vector2f(0,0),Vector2f(0,0));
                }
                if(state_time>800){
                    WalkToBall(Goal_pose_t);  
                }
                else  
                    WalkToBall(Vector2f(4700,0));
            }
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
            kick(theControlParameter.Obstacle_Ball);
        }
    }
    state(searchForOnBall){
        transition{
            if((state_time>1000&&theControlParameter.Ball_Seen_flag)||theBallModel.estimate.position.norm()<300){
                 judge_kick=false;
                 flag=0;
                goto start;}
        }
        action{
            OUTPUT_TEXT("Striker_attack::searchForOnBall");
             HeadScan(20_deg,18_deg);
             Vector2f temp_pose=Transformation::robotToField(theRobotPose,theBallModel.estimate.position);
                if(theControlParameter.Obstacle_Front)
                    theMotionRequest=thePathPlanner.plan(Pose2f(theBallModel.estimate.position.angle(),temp_pose),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);
                else
                    WalkToTargetDes(temp_pose);
        }
    }
    state(stayForBall1){
        action{
            Vector2f temp_pose_stay=Transformation::fieldToRobot(theRobotPose,Vector2f(800,100));
            if(std::abs(theControlParameter.Ball_RobotPose.angle())>10_deg){
                WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theControlParameter.Ball_RobotPose.angle(),0,0));
                HeadPanTurn(theControlParameter.Ball_RobotPose.angle(),10_deg);
            }
            else
            {
                    HeadScan(20_deg,10_deg);
                    Stand();
            }
                
        }
    }
    state(stayForBall){
        transition{
            if(action_done)
                goto stayForBall1;
        }
        action{
            WalkToTargetDes(Vector2f(800,100)); 
            HeadPanTurn(0,10_deg);
        }
    }
}
