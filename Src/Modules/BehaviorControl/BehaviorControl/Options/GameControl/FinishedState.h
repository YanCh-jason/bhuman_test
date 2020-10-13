option(finishedState,(int)(2)number){
    static Vector2f temp_pose;
    static bool flag=false;
    common_transition{
        Vector2f temp_pose_t;
        float angle_temp;
         if(theRobotPose.translation.x()>400){
            HeadScan(60_deg)  ;
            }
        else{
            SetHeadPanTilt(0, 0.38f, HEAD_PAN_SPEED);
        }
        if(number==2){
            temp_pose=Vector2f(-1000,0);
        }
        else if(number==5){
            temp_pose=Vector2f(-600,-1200);
        }
        temp_pose_t.x()=theRobotPose.translation.x()-temp_pose.x();
        temp_pose_t.y()=theRobotPose.translation.y()-temp_pose.y();
        if(theControlParameter.Obstacle_Arm&&!flag)
                goto planner;
    }
    initial_state(start){
        transition{
            flag=false;
            if(number==2){
                temp_pose=Vector2f(-1000,0);
                goto striker2S;
            }
            else{
                temp_pose=Vector2f(-600,-1200);
                goto striker2S;
            }
        }
        action{
            OUTPUT_TEXT("finishedState::start");
        }
    }
    state(planner){
        transition{
            if(!theControlParameter.Obstacle_Arm)
                goto striker2S;
        }
        action{
             OUTPUT_TEXT("planner");
            theMotionRequest=thePathPlanner.plan(Pose2f(0,temp_pose),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);
        }
    }
    state(striker2S){
        transition{

        
        if(action_done){
           // flag=true;
            OUTPUT_TEXT("striker2Sendd");
            goto end;   
        }
        }
       action{
           OUTPUT_TEXT("striker2S");
           WalkToTargetDes(temp_pose);
       }
    }
    state(end){
        transition{
            flag=true;
            if(state_time>5000)
                goto endok;
        }
        action{
            OUTPUT_TEXT("end");
            theMotionRequest=thePathPlanner.plan(Pose2f(0,temp_pose),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);
        }
    }
    target_state(endok){
        action{
            OUTPUT_TEXT("end1");
            Stand();
        }
    }



}