// //
option(SearchForBallNoBall,(int)number){
  static bool  flag=false;
    static float temp_angle_R,tilt_angle;
    static bool LR=false;
    static int turn_times=0;
    static Vector2f temp_pose,temp_pose_MV;

	common_transition{
		if(number==2){
                if(theRobotPose.translation.x()>2000)
                    temp_pose_MV=Vector2f(2100,500);
                else
                    temp_pose_MV=Vector2f(500,1000);
        }
        else{
            if(theRobotPose.translation.x()>2000)
                    temp_pose_MV=Vector2f(2100,-500);
                else
                    temp_pose_MV=Vector2f(500,-1000);
        }
        if(theControlParameter.Ball_Seen_flag&&!flag){
                flag=true;
				goto found_ok;
		}
	}
	
    initial_state(start){
		transition{
			flag=false;
            if(theBallModel.lastPerception.angle()<0)
                LR=true;
            else
                LR=false;  
            turn_times=0;                       
            if(number==2){
                if(theRobotPose.translation.x()>2200)
                    temp_pose_MV=Vector2f(2000,500);
                else
                    temp_pose_MV=Vector2f(500,1000);
            }
            else{
                if(theRobotPose.translation.x()>2200)
                    temp_pose_MV=Vector2f(2000,-500);
                else
                    temp_pose_MV=Vector2f(500,-1000);
            }
            	goto turn_ball;
		}
		action{
            HeadPanTurn(0);
			Stand();
            OUTPUT_TEXT("SearchForBallStay::start");
		}
	}

    state(turn_ball){
        transition{
            if(std::abs(theBallModel.lastPerception.angle())<10_deg){
             goto wait0;
            }
        }
        action{
            OUTPUT_TEXT("SearchForBallStay::turn_ball");
            HeadPanTurn(0);
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theBallModel.lastPerception.angle(),0,0));
        }
    }
    state(wait0){
        transition{
            if(state_time>3000){
                HeadPanTurn(0);
                goto turn1;}
        }
        action{
            if(theControlParameter.Ball_RobotPose.norm()>=2500)
                tilt_angle=-6_deg;
            else if(theControlParameter.Ball_RobotPose.norm()>=1500)
                tilt_angle=0_deg;    
            else 
                tilt_angle=19_deg;    
            HeadScan(60_deg,tilt_angle,LR);
            Stand();
        }
    }


    state(turn1){
        transition{
            if(action_done)
                goto turn2;
        }
        action{
            if(turn_times==1)
                HeadPanTurn(0,0);
            walkRotate(LR);
            OUTPUT_TEXT("SearchForBallStay::turn1");         
        }
    }



    state(turn2){
        transition{
            if(action_done){
                turn_times++;
                if(turn_times>=2){
                    OUTPUT_TEXT(turn_times);
                    turn_times=0;
                    goto wait1;
                    }
                else
                    goto turn1;
            }
        }
        action{     
            OUTPUT_TEXT("SearchForBallStay::turn2");    
            walkRotate(LR);
        }
    }

    state(wait1){
        transition{
            if(state_time>5500){
                HeadPanTurn(0,0);
                goto turn3;
            }
        }
        action{
            Stand();
            OUTPUT_TEXT("SearchForBallStay::wait1");
            HeadScan(60_deg);
        }
    }



    state(turn3){
        transition{
            if(action_done)
                goto wait2;
        }
        action{
            walkRotate(LR);
            // HeadPanTurn(0,HEAD_PAN_SPEED);
            OUTPUT_TEXT("SearchForBallStay::turn3");
        }
    }

    state(wait2){
        transition{
            if(state_time>5500){
                HeadPanTurn(0,0);
                goto turn4;
            }
        }
        action{
            Stand();
            HeadScan(60_deg);
            OUTPUT_TEXT("SearchForBallStay::wait2");
        }
    }

    state(turn4){
        transition{
            Vector2f temp_pose_t;
            temp_pose_t=Transformation::fieldToRobot(theRobotPose,Vector2f(0,0));
            if(std::abs(temp_pose_t.angle())<5_deg)
                goto wait3;
        }
        action{
            Vector2f temp_pose_t=Transformation::fieldToRobot(theRobotPose,Vector2f(0,0));
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(temp_pose_t.angle(),0,0));
            OUTPUT_TEXT("SearchForBallStay::turn4");
            HeadPanTurn(0);
        }
    }
    state(wait3){
        transition{
            if(state_time>4000)
                goto move;

        }{
            Stand();
            HeadScan(30_deg);
            OUTPUT_TEXT("SearchForBallStay::wait3");
        }
    }

    state(move){
        transition{
            temp_pose.x()=theControlParameter.Ball_GlobalPoseEs.x()-temp_pose_MV.x();
            temp_pose.y()=theControlParameter.Ball_GlobalPoseEs.y()-temp_pose_MV.y();
            if(state_time>10000||temp_pose.norm()<80){
                goto turn_ball;
            }
        }
        action{
            OUTPUT_TEXT("SearchForBallStay::move");
            HeadScan(20_deg);
            theMotionRequest=thePathPlanner.plan(Pose2f(temp_pose.angle(),temp_pose_MV),Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED-5,MOVE_Y_SPEED-5),false);
        }
    }




    state(found_ok){
        transition{
            if(std::abs(theBallModel.lastPerception.angle())<5_deg)
                goto end;
        }
        action{
            OUTPUT_TEXT("SearchForBallStay::found_ok");
            
            if(theControlParameter.Ball_RobotPose.norm()>=2500)
                tilt_angle=-6_deg;
            else if(theControlParameter.Ball_RobotPose.norm()>=1500)
                tilt_angle=0_deg;    
            else 
                tilt_angle=19_deg;  
            HeadPanTurn(0,tilt_angle);
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theBallModel.lastPerception.angle(),0,0));
        }
    }
    target_state(end){
        transition{
            if(state_time>20){
                goto start;
            }
        }
        action{
            Stand();
            OUTPUT_TEXT("SearchForBallStay::end");
        }
    } 
    }