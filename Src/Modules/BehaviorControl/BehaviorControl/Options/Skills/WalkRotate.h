option(walkRotate,(bool)(false)LR,(float)(MOVE_ROTATION_SPEED)speed){
    static Vector2f temp_pose;
    float temp_angle;
    static Vector2f temp_pose1;
    initial_state(start){
        transition{
            OUTPUT_TEXT("walkRotate::start");
            temp_pose.x()=1000*std::cos(theRobotPose.rotation);
            temp_pose.y()=1000*std::sin(theRobotPose.rotation);
            temp_pose.x()=theRobotPose.translation.x()-temp_pose.x();
            temp_pose.y()=theRobotPose.translation.y()-temp_pose.y();
            if(!LR){
				goto left;
			}
            else{
                goto right;
			}
        }
    }
    state(left){
        transition{
            temp_pose1=Transformation::fieldToRobot(theRobotPose,temp_pose);
            if(std::abs(temp_pose1.angle())<25_deg||state_time>7000)
                goto adjust; 		
        }
        action{
            OUTPUT_TEXT("walkRotate::left");
            if(state_time<1200)
                WalkToTarget(Pose2f(speed,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(170,0,0));
            else 
                WalkToTarget(Pose2f(speed,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(179,0,0));
        }
    }
    state(right){
        transition{
            temp_pose1=Transformation::fieldToRobot(theRobotPose,temp_pose);
            if(std::abs(temp_pose1.angle())<25_deg||state_time>7000)
                goto adjust; 	
        }
        action{
            OUTPUT_TEXT("walkRotate::right");
            if(state_time<1200)
                WalkToTarget(Pose2f(speed,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(-170,0,0));
            else 
                WalkToTarget(Pose2f(speed,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(-179,0,0));
        }
    }
    state(adjust){
        transition{
            temp_angle=Transformation::fieldToRobot(theRobotPose,temp_pose).angle();
            if(std::abs(temp_angle)<3_deg){
                goto end;
            }
        }
        action{
            OUTPUT_TEXT("walkRotate::adjust");
            temp_angle=Transformation::fieldToRobot(theRobotPose,temp_pose).angle();
            WalkToTarget(Pose2f(speed,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(temp_angle,0,0)); 
        }
    }
    target_state(end){
        transition{
            OUTPUT_TEXT("walkRotate::end");
                goto start;  
        }
    }
}