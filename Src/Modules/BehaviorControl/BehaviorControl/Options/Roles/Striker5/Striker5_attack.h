option(Striker5_attack){
   static bool judge_Goal1=false,judge_Goal2=false,judge_Goal3=false;
   static Vector2f Goal_pose_t;
   static bool judge_kick=false;
   static int time,flag=0;
   static int time_now;

   static float tilt_angle=0;
   static int test_flag1=0;
   static bool df_flag1,df_flag2;
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
            df_flag1=theControlParameter.defender1_dis!=99999.f&&theControlParameter.defender1_dis==std::min(theControlParameter.defender1_dis,theControlParameter.defender2_dis)&&theRobotPose.translation.x()<theControlParameter.df_x1;
            df_flag2=theControlParameter.defender2_dis!=99999.f&&theControlParameter.defender2_dis==std::min(theControlParameter.defender1_dis,theControlParameter.defender2_dis)&&theRobotPose.translation.x()<theControlParameter.df_x2;
            if(!theControlParameter.Ball_Seen_flag){
                flag=0;
                goto SearchForBallNoBall;
            }
            if(theControlParameter.Ball_GlobalPoseEs.x()<10){
                if(theControlParameter.theRobotToGoal_dis>std::min(theControlParameter.defender1_dis,theControlParameter.defender2_dis)||df_flag2||df_flag1)
               { if(flag!=3){
                    flag=3;
                    goto stayForBall1;
                }}
                else{
                if(flag!=1){
                        flag=1;
                        goto walkToBall;}}
            }
            else if(theControlParameter.otherRobotToGoal_dis<1000){
                if(flag!=2){
                    flag=2;
                    test_flag1=1;
                    goto searchForBallStay;
                }
            }
            else if(std::abs(theControlParameter.theRobotToGoal_dis-theControlParameter.otherRobotToGoal_dis)<60){
                Vector2f temp_pose1,temp_pose2,temp_pose3;
                if(theFrameInfo.getTimeSince(theControlParameter.striker1.timeWhenLastPacketReceived)>10000){
                    if(flag!=1){
                        flag=1;
                        goto walkToBall;}
                }
                else{
                    temp_pose1.x()=theControlParameter.Ball_GlobalPoseEs.x()-4700;
                    temp_pose1.y()=theControlParameter.Ball_GlobalPoseEs.y()-0;
                    temp_pose2.x()=theRobotPose.translation.x()-theControlParameter.Ball_GlobalPoseEs.x();
                    temp_pose2.y()=theRobotPose.translation.y()-theControlParameter.Ball_GlobalPoseEs.y();
                    temp_pose3.x()=theControlParameter.striker1.theRobotPose.translation.x()-theControlParameter.Ball_GlobalPoseEs.x();
                    temp_pose3.y()=theControlParameter.striker1.theRobotPose.translation.y()-theControlParameter.Ball_GlobalPoseEs.y();
                    if(std::abs(temp_pose1.angle()-temp_pose2.angle())<std::abs(temp_pose1.angle()-temp_pose3.angle())){
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
            else if(theControlParameter.theRobotToGoal_dis<theControlParameter.otherRobotToGoal_dis){
                if(flag!=1){
                            flag=1;
                            goto walkToBall;}
            }
            else{
                if(flag!=2){
                         flag=2;
                          test_flag1=3;
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
            SearchForBallNoBall(5);
        }
    }


    state(searchForBallStay){
        action{
            OUTPUT_TEXT("Striker_attack::searchForBallStay");
            OUTPUT_TEXT(test_flag1);
            searchForBallStay(5);
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
                if(state_time<1000){
                    Goal_pose_t=theControlParameter.PosChange(1,Vector2f(0,0),Vector2f(0,0));
                }
                if(state_time>1000){
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
            if((state_time>1000&&theControlParameter.Ball_Seen_flag)||theControlParameter.Ball_GlobalPoseEs.norm()<300){
                 judge_kick=false;
                 flag=0;
                goto start;}
        }
        action{
            OUTPUT_TEXT("Striker_attack::searchForOnBall");
             HeadScan(20_deg,18_deg);
             Vector2f temp_pose=Transformation::robotToField(theRobotPose,theControlParameter.Ball_GlobalPoseEs);
                if(theControlParameter.Obstacle_Front)
                    theMotionRequest=thePathPlanner.plan(Pose2f(theControlParameter.Ball_GlobalPoseEs.angle(),temp_pose),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);
                else
                    WalkToTargetDes(temp_pose);
        }
    }
     state(stayForBall1){
        action{
            if(df_flag1||df_flag2)
                {
                        if(theRobotPose.translation.x()<0)
                        WalkToTargetDes(Vector2f(-800,-300));
                        else
                        WalkToTargetDes(Vector2f(-800,300));
                }
            else if(std::abs(theControlParameter.Ball_RobotPose.angle())>10_deg){
                WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theControlParameter.Ball_RobotPose.angle(),0,0));
                HeadPanTurn(theControlParameter.Ball_RobotPose.angle(),10_deg);
            }
            else if(theControlParameter.Ball_RobotPose.norm()<1100){
                WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theControlParameter.Ball_RobotPose.angle(),-1000,0));
                HeadPanTurn(theControlParameter.Ball_RobotPose.angle(),10_deg);
            }
            else
            {
                    HeadScan(20_deg,10_deg);
                    Stand();
            }
                
        }
    }
}
