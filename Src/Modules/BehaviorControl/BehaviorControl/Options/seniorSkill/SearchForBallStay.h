//**原地找球考虑到障碍物和接球**/
option(searchForBallStay,(int)(0)number){

    static Vector2f temp_pose,temp_pose_MV;
    static float tilt_angle=0;
    initial_state(start){
		transition{
            if(state_time>100){
			if(theControlParameter.Ball_GlobalPoseEs.x()>theRobotPose.translation.x()+1000||theControlParameter.Ball_GlobalPoseEs.x()>theRobotPose.translation.x()&&theControlParameter.Ball_RobotPose.norm()>1500||theControlParameter.Ball_RobotPose.norm()>3000)
                goto Near;
            else if(theControlParameter.Ball_RobotPose.norm()<700)
                goto FarBall;    
            else
                goto Wait;
            }
        }
		action{
            HeadPanTurn(theControlParameter.Ball_RobotPose.angle());
			Stand();
            OUTPUT_TEXT("SearchForBallStay::start");
		}
	}
    state(Near){
        transition{
            if(theControlParameter.Ball_GlobalPoseEs.x()<theRobotPose.translation.x()+1100||theControlParameter.Ball_RobotPose.norm()<1200)
                goto Wait;
        }
        action{
        if(theRobotPose.translation.y()>=theControlParameter.Ball_GlobalPoseEs.y())
                temp_pose_MV=Vector2f(theControlParameter.Ball_GlobalPoseEs.x()-1000,theControlParameter.Ball_GlobalPoseEs.y()+1000);
        else
                temp_pose_MV=Vector2f(theControlParameter.Ball_GlobalPoseEs.x()-1000,theControlParameter.Ball_GlobalPoseEs.y()-1000);
        if(theControlParameter.Ball_RobotPose.norm()>=2000)
            tilt_angle=0_deg;    
        else 
            tilt_angle=19_deg;         
        HeadScan(10_deg,tilt_angle);   
         
        temp_pose.x()=theControlParameter.Ball_GlobalPoseEs.x()-theRobotPose.translation.x();
        temp_pose.y()=theControlParameter.Ball_GlobalPoseEs.y()-theRobotPose.translation.y();
        if(theControlParameter.Obstacle_Arm)
            theMotionRequest=thePathPlanner.plan(Pose2f(theControlParameter.Ball_RobotPose.angle(),temp_pose_MV),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED-5,MOVE_Y_SPEED-5),false);   
        else
            WalkToTargetDes(temp_pose_MV);
        }
    }
    state(Wait){
        transition{
			if(theControlParameter.Ball_GlobalPoseEs.x()>theRobotPose.translation.x()+1300||theControlParameter.Ball_GlobalPoseEs.x()>theRobotPose.translation.x()&&theControlParameter.Ball_RobotPose.norm()>1500||theControlParameter.Ball_RobotPose.norm()>3000)            
                goto start;
            if(theControlParameter.Ball_RobotPose.angle()>50_deg){
                goto turn;
            }
        }
        action{
            if(theControlParameter.Ball_RobotPose.norm()>=2000)
                tilt_angle=0_deg;    
            else 
                tilt_angle=19_deg;   
            if(theControlParameter.Ball_RobotPose.angle()<10_deg)         
                HeadScan(20_deg,tilt_angle);   
            else
                HeadPanTurn(theControlParameter.Ball_RobotPose.angle(),tilt_angle) ;   
            Stand();    
    }
    }
    state(turn){
        transition{
            if(theControlParameter.Ball_RobotPose.angle()<10_deg){
                goto Wait;
            }
        }
        action{
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theControlParameter.Ball_RobotPose.angle(),0,0));
        }
    }
    state(FarBall){
        transition{
            if(action_done||theControlParameter.Ball_RobotPose.norm()>800)
                goto start;
        }

        action{
            pushingfreeKick(1);
        }
    }

}