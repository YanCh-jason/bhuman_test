option(Ready,(int)(2)number){
     static Vector2f temp_pose,temp_pose_t;
     static float angle_temp;
     static int time_now1=0,time_now2=0;
     static int flag=0;
    common_transition{
        float angle_t;
        angle_temp=Transformation::fieldToRobot(theRobotPose,Vector2f(0,0.f)).angle();
        if(theFrameInfo.time-time_now1>5000&&time_now2==0){
            angle_t=angle_temp;
            time_now1=theFrameInfo.time;
            time_now2=1;
            }
        else if(theFrameInfo.time-time_now1>5000&&time_now2==1){
            angle_t=0;
            time_now1=theFrameInfo.time;
            time_now2=0;}
        if(angle_temp>70_deg)
                HeadScan(30_deg,10_deg);
        else        
                SetHeadPanTilt(angle_temp, 0.38f, HEAD_PAN_SPEED);   
        if(number==2){
                if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
                    temp_pose=Vector2f(-500,0);
                else
                    temp_pose=Vector2f(-1000,0);
            }
            else if(number==5){
                temp_pose=Vector2f(-600,-1200);
            } 
        temp_pose_t.x()=temp_pose.x()-theRobotPose.translation.x();
        temp_pose_t.y()=temp_pose.y()-theRobotPose.translation.y(); 
    }
    initial_state(start){
        transition{
            if(number==2){
                if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
                    temp_pose=Vector2f(-500,0);
                else
                    temp_pose=Vector2f(-1000,0);
            }
            else if(number==5){
                temp_pose=Vector2f(-600,-1200);
            }
            goto striker2S;
        }
        action{
            OUTPUT_TEXT("finishedState::start");
        }
    }

    state(striker2S){
        transition{
            if(temp_pose_t.norm()<1000)
                goto walk_target;
        }
       action{
           OUTPUT_TEXT("striker2S");
           if(!theControlParameter.Obstacle_Front)
                WalkToTargetDes(temp_pose);
            else    
                theMotionRequest=thePathPlanner.plan(Pose2f(0,temp_pose),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false);    
       }
    }
    state(walk_target){
        transition{
            if(temp_pose_t.norm()<=40)
                goto turn;
        }
        action{
            if(!theControlParameter.Obstacle_Front)
                WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(-theRobotPose.rotation,Transformation::fieldToRobot(theRobotPose,temp_pose)));
            else
                theMotionRequest=thePathPlanner.plan(Pose2f(0,temp_pose),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),false); 

        }
    }
    state(turn){
        transition{
            if(std::abs(angle_temp)<3_deg&&number==2)
                goto endok;
            if(std::abs(theRobotPose.rotation)<5_deg&&number==5)   
				goto endok; 
        }
        action{
            if(number==2)
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(angle_temp,0,0));
            else
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(-theRobotPose.rotation,0,0));
        }
    }
    target_state(endok){
        action{
            flag=2;
            OUTPUT_TEXT("end1");
            Stand();
        }
    }


}