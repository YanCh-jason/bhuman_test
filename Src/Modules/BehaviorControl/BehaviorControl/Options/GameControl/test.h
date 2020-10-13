option(test1){
	static float angle=180_deg,tilt_angle;
	static int time_now;
    initial_state(WalkToBall){
        transition{
//           if(action_done)
//			goto Kick;
        }
        action{
			//searchForBallStay(2);
//        HeadPanTurn(theBallModel.estimate.position.angle());
			//Striker1Ready();
			//SearchForBallNoBall(2);
		//OUTPUT_TEXT(theControlParameter.Ball_Moved_flag);
			//HeadTurnMiddle(0,1.5,-1.5,0,3);
			//OUTPUT_TEXT("test1::start");
			//OUTPUT_TEXT(theControlParameter.Ball_GlobalPoseEs.x());
			//OUTPUT_TEXT(theControlParameter.Ball_GlobalPoseEs.y());
			 //HeadPanTurn(theControlParameter.Ball_RobotPose.angle(),HEAD_PAN_SPEED);
			// Vector2f _test(4500.0,0.0);
			 //theControlParameter.Obstacle_deal(Vector2f(4500,0));
			//SearchForBallStay(3,2);
			//SearchForBallStay(2);
			//Ready(5);
			//WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(180_deg,0,0));
			//walkRotate(false);
//			WalkToBall(Vector2f(4700,0));
			//FallDownState
			Vector2f temp;
			temp=theControlParameter.PosChange(1,Vector2f(0,0),Vector2f(0,0));
			// if(theFrameInfo.time-time_now>1000){
			// temp=theControlParameter.PosChange(1,Vector2f(0,0),Vector2f(0,0));
			// time_now=theFrameInfo.time;
			// }
			//   if(std::abs(theControlParameter.Ball_RobotPose.angle())<20_deg&&theControlParameter.Ball_RobotPose.norm()>1000)
            //     HeadScan(30_deg);
            // else
            //     HeadPanTurn(theControlParameter.Ball_RobotPose.angle()) ;       
		    //theControlParameter.PosChange(1,Vector2f(0,0),Vector2f(0,0));
			//OUTPUT_TEXT(theControlParameter.Pose_Goal.x());
			//OUTPUT_TEXT(theControlParameter.Pose_Goal.y());
			// OUTPUT_TEXT("theControlParameter.Obstacle_Between_B_G:");
			// OTPUT_TEXT(theControlParameter.Obstacle_Between_B_G);
			// OUTPUT_TEXT("//");
//			OUTPUT_TEXT("theControlParameter.Obstacle_Between_B_G_front:");
//			OUTPUT_TEXT(theControlParameter.Obstacle_Between_B_G_front);
//			OUTPUT_TEXT("//");
//			OUTPUT_TEXT("theControlParameter.Obstacle_Arm:");
//			OUTPUT_TEXT(theControlParameter.Obstacle_Arm);
//			OUTPUT_TEXT("//");
			// bool test=false;
			// WalkToBall(temp,test);
        }
	}
	
	state(SearchForBallNoBall){
		transition{
			if(theControlParameter.Ball_Seen_flag){
                goto WalkToBall;
            }
		}
		action{
			SearchForBallNoBall(2);
		}
	}
	state(Kick){
		transition{
			if(action_done)
				goto WalkToBall;
		}
		action{
			kick(1);
		}
	}
	state(goBall){
		transition{
			if(action_done)
				goto WalkToBall;
		}
		action{
			WalkToTargetDes(theControlParameter.Ball_GlobalPoseEs);
		}
	}
	
    // state(loop){
    //    transition{
	// 	 if(state_time>800)
	// 	 	goto start;
	//    }
	//    action{
	// 	   OUTPUT_TEXT("loop");
	// 	   //kick();
	//    }
    // }
	
	
	
	
	
}
option(test2,(bool&)xc){
	initial_state(start){
		action{
		xc=true;
		}
	}
}