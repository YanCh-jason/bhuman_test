#define POSE_DEFINE_Y 400
#define POSE_DEFINE_X -150

option(WalkToBall,(const Vector2f& )temp_goal){
    static Vector2f temp_pose_G,temp_pose_R,temp_goal_R;
    static float temp_angle;
    static Vector2f temp_pose_MV;
    static Vector2f temp_pose;
    static int flag=0;
    static int times=0;
    common_transition{
        temp_pose_R.x()=theRobotPose.translation.x()-theControlParameter.Ball_GlobalPoseEs.x();
        temp_pose_R.y()=theRobotPose.translation.y()-theControlParameter.Ball_GlobalPoseEs.y();
        temp_pose_G.x()=temp_goal.x()-theControlParameter.Ball_GlobalPoseEs.x();
        temp_pose_G.y()=temp_goal.y()-theControlParameter.Ball_GlobalPoseEs.y();
        temp_goal_R=Transformation::fieldToRobot(theRobotPose,temp_goal);
        temp_angle=temp_pose_R.angle()-temp_pose_G.angle();
        if(temp_angle<0&&std::abs(temp_angle)>pi)
            temp_angle+=2*pi;
        else if(temp_angle>0&&std::abs(temp_angle)>pi)
            temp_angle-=2*pi;
        if(std::abs(temp_pose_R.angle()-temp_pose_G.angle())<105_deg&&flag!=4){
            flag=1;
        }
        else if(std::abs(temp_pose_R.angle()-temp_pose_G.angle())>=105_deg&&flag!=2&&flag!=4){
            flag=2;
        }
        if(flag==1){
                if(temp_angle<=0||std::abs(temp_angle)<2_deg){
                    temp_pose_MV=theControlParameter.PosChange(0,Vector2f(POSE_DEFINE_X,-POSE_DEFINE_Y),temp_goal);
                    }
                else{
                    temp_pose_MV=theControlParameter.PosChange(0,Vector2f(POSE_DEFINE_X,POSE_DEFINE_Y),temp_goal);
                }
                if(theControlParameter.Obstacle_Front_Near){
                    temp_pose.x()=theControlParameter.Ball_GlobalPoseEs.x()-theRobotPose.translation.x();
                    temp_pose.y()=theControlParameter.Ball_GlobalPoseEs.y()-theRobotPose.translation.y();
                    theMotionRequest=thePathPlanner.plan(Pose2f(temp_pose.angle(),temp_pose_MV),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);  
                    goto start;
                }
                else{
                if(theControlParameter.Ball_RobotPose.norm()<650){
                        goto WalkToKickArea_2;	
                }        
                else{
                    goto WalkToKickArea_1; 
                }
            }
        }
        if(flag==2){            
            if(std::abs(temp_angle)>130_deg){
                temp_pose_MV=theControlParameter.PosChange(0,Vector2f(-170,50),temp_goal);
                temp_pose=Transformation::fieldToRobot(theRobotPose,temp_pose_MV);
            }
            else{
                temp_pose_MV=theControlParameter.PosChange(0,Vector2f(-400,0),temp_goal);
                temp_pose=Transformation::fieldToRobot(theRobotPose,temp_pose_MV);
            }    
            if(temp_pose.norm()>15||std::abs(temp_goal_R.angle()>=3_deg)){
                    if(theControlParameter.Obstacle_Front_Near){
                        temp_pose_MV=theControlParameter.PosChange(0,Vector2f(-200,30),temp_goal);
                        temp_pose.x()=temp_goal.x()-theRobotPose.translation.x();
                        temp_pose.y()=temp_goal.y()-theRobotPose.translation.y();
                        theMotionRequest=thePathPlanner.plan(Pose2f(temp_pose.angle(),temp_pose_MV),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);   
                    }
                    else
                        goto WalkAhead; 
                }  
            else{
                flag=4;
                goto end;
            }
        }
       
    }
    initial_state(start){
        action{
            flag=0;
            OUTPUT_TEXT("loop");
        }
    }
    state(WalkToKickArea_1){
        transition{
            if(action_done){
                flag=0;
                goto start;
            }    
        }
        action{
            WalkToTargetDes(temp_pose_MV);
        }
        OUTPUT_TEXT("WalkToKickArea_1");
    }
    state(WalkToKickArea_2){
        transition{
            temp_pose=Transformation::fieldToRobot(theRobotPose,temp_pose_MV);
            if(temp_pose.norm()<DIS_NEAR_TARGET&&std::abs(theControlParameter.Ball_RobotPose.angle())<5_deg){
                flag=0;
                goto start;
            }    
        }
        action{
            temp_pose=Transformation::fieldToRobot(theRobotPose,temp_pose_MV);
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theControlParameter.Ball_RobotPose.angle(),temp_pose));
            OUTPUT_TEXT("WalkToKickArea_2");
        }
    }
    state(WalkAhead){
        action{
                if(temp_pose.norm()>500)
                   WalkToTargetDes(temp_pose_MV); 
                else   
                    WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(temp_goal_R.angle(),temp_pose));  
            }
            OUTPUT_TEXT("WalkAhead");
    }
//    state(end){
//        transition{
//            if(times==1){
//                times=0;
//                goto endok;
//            }   
//            else if(state_time>200&&times==0){
//                times++;
//                flag=0;
//                goto start;}
//             
//        }
//        action{
//            OUTPUT_TEXT("times");
//        }
//    }
    target_state(end){
        transition{
            if(state_time>20){
                flag=0;
                goto start;
            }
            action{
                OUTPUT_TEXT("walkToBall::end");
            }
        }
    } 
}   