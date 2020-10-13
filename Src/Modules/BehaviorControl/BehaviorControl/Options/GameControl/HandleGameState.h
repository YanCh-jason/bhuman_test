/** Triggers the options for the different game states.
 *  This option also invokes the get up behavior after a fall, as this is needed in most game states.
 */
option(HandleGameState)
{
  /** As game state changes are discrete external events and all states are independent of each other,
      a common transition can be used here. */

	  float angle_temp=0;
	static int flag=0;
  common_transition
  {
    //OUTPUT_TEXT("HandleGameState_common_transition");
    if(theGameInfo.state == STATE_INITIAL)
    {
        //OUTPUT_TEXT("HandleGameState_STATE_INITIAL");
        goto initial;
    }
      
    if(theGameInfo.state == STATE_FINISHED)
    {
        //OUTPUT_TEXT("HandleGameState_STATE_FINISHED");
        goto finished;
    }
      

    if(theFallDownState.state == FallDownState::fallen)
    {
        //OUTPUT_TEXT("HandleGameState_FallDownState::fallen");
				flag=1;
        goto getUp;
    }
      

    if(theGameInfo.state == STATE_READY)
    {
        //OUTPUT_TEXT("HandleGameState_STATE_READY");
				flag=0;
        goto ready;
    }
      
    if(theGameInfo.state == STATE_SET)
    {
        //OUTPUT_TEXT("HandleGameState_STATE_SET");
				flag=0;
        goto set;
    }
      
    if(theGameInfo.state == STATE_PLAYING)
    {
       if(theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK||theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK)
      {
				flag=1;
		  if(theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber)
			goto pushingfree;
		  else{
			  goto secondPlaying;
		  }	
			
      }
		
	  else if(flag==0)
		goto playing;
		else if(flag==1)
		goto  secondPlaying;
    }
	if(theGameInfo.gamePhase==GAME_PHASE_PENALTYSHOOT){
		flag=0;
		goto penltyKick;
	}
  }

  /** Stand still and wait. */
  initial_state(initial)
  {
    action
    {
     // OUTPUT_TEXT("HandleGameState_initial_action");
      SetHeadPanTilt(0.f, 0.f, 150_deg);
	  HeadControlMode(HeadControl::none);
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }

  /** Stand still and wait. If somebody wants to implement cheering moves => This is the place. ;-) */
  state(finished)
  {
    action
    {
      if(theRobotInfo.number == 2)     
			finishedState(2);
		if(theRobotInfo.number == 5)
			finishedState(5);
    }
  }

  state(afterFinished)
  {
      transition
      {
          if(state_time>100)
              goto initial;
      }
      action
      {
		LookForward();
        Stand();
      }
  }
  
  /** Get up from the carpet. */
  state(getUp)
  {
    action
    {
			flag=1;
      GetUp();
    }
  }

  /** Walk to kickoff position. */
  state(ready)
  {
    action
    {
        if(theRobotInfo.number == 2)     
			Ready(2);
		if(theRobotInfo.number == 5)
			Ready(5);
    }
  }
  
  /** Stand and look around. */
  state(set)
  {
    action
    {
		if(theControlParameter.Ball_Seen_flag)
			HeadPanTurn(theControlParameter.Ball_RobotPose.angle(),12_deg);
		else	
			HeadScan(45_deg,10_deg);
		OUTPUT_TEXT("set");
		Stand();
    }
  }

  /** Play soccer! */
  state(playing)
  {
    action
    {	
	if(theRobotInfo.number == 2) Striker2();  
	if(theRobotInfo.number == 5)Striker5();
  //test1();
  //penltyKick();
   //pushingfreeKick();
	}

}
state(secondPlaying){
	action{
		if(theRobotInfo.number == 2)  Striker2_attack(); 
		     if(theRobotInfo.number == 5)Striker5_attack();
	}

}
  

 state(pushingfree){
	action{
		pushingfreeKick();
	}
}
state(penltyKick){
	action{
		penltyKick();
	}
}
}
option(pushingfreeKick,(int)(0)kind){
	static Vector2f temp_pose_aw,temp_pose_df;
	common_transition{
		float temp_dis;
		Vector2f temp_pose1;
		Vector2f temp_pose2;
		if(option_time>=40000&&kind==0)
			goto end;
			if(kind==0){
			if(theControlParameter.Ball_Seen_flag)
					temp_pose2=theControlParameter.Ball_GlobalPoseEs;
			else if(theRobotPose.translation.y()>0)
		      temp_pose2=Vector2f(3200,1100);
			else
					temp_pose2=Vector2f(3200,-1100);
			}
			else{
					temp_pose2=theControlParameter.Ball_GlobalPoseEs;
			}
		if(theControlParameter.Ball_Seen_flag)
			HeadPanTurn(theControlParameter.Ball_RobotPose.angle());
		else
			HeadScan(20_deg);
		temp_pose1.x()=theRobotPose.translation.x()-temp_pose2.x();
		temp_pose1.y()=theRobotPose.translation.y()-temp_pose2.y();
		temp_dis=temp_pose1.norm();
		temp_pose1.x()=(temp_pose1.x()/temp_dis)*800+temp_pose2.x();
		temp_pose1.y()=(temp_pose1.y()/temp_dis)*800+temp_pose2.y();
		temp_pose_aw=Transformation::fieldToRobot(theRobotPose,temp_pose1);
		
		
		temp_pose1.x()=-4500-temp_pose2.x();
		temp_pose1.y()=0-temp_pose2.y();
		temp_dis=temp_pose1.norm();
		temp_pose1.x()=(temp_pose1.x()/temp_dis)*800+temp_pose2.x();
		temp_pose1.y()=(temp_pose1.y()/temp_dis)*800+temp_pose2.y();
		temp_pose_df=Transformation::fieldToRobot(theRobotPose,temp_pose1);
		if(theControlParameter.Ball_RobotPose.norm()<800)
			goto farAway;
		else 
			goto defense;
	}
	initial_state(start){
		transition{
			float temp_dis;
		Vector2f temp_pose1;
		Vector2f temp_pose2;
		if(option_time>=40000&&kind==0)
			goto end;
			if(kind==0){
			if(theControlParameter.Ball_Seen_flag)
					temp_pose2=theControlParameter.Ball_GlobalPoseEs;
			else if(theRobotPose.translation.y()>0)
		      temp_pose2=Vector2f(3200,1100);
			else
					temp_pose2=Vector2f(3200,-1100);
			}
			else{
					temp_pose2=theControlParameter.Ball_GlobalPoseEs;
			}
		if(theControlParameter.Ball_Seen_flag)
			HeadPanTurn(theControlParameter.Ball_RobotPose.angle());
		else
			HeadScan(20_deg);
		temp_pose1.x()=theRobotPose.translation.x()-temp_pose2.x();
		temp_pose1.y()=theRobotPose.translation.y()-temp_pose2.y();
		temp_dis=temp_pose1.norm();
		temp_pose1.x()=(temp_pose1.x()/temp_dis)*800+temp_pose2.x();
		temp_pose1.y()=(temp_pose1.y()/temp_dis)*800+temp_pose2.y();
		temp_pose_aw=Transformation::fieldToRobot(theRobotPose,temp_pose1);
		
		
		temp_pose1.x()=-4500-temp_pose2.x();
		temp_pose1.y()=0-temp_pose2.y();
		temp_dis=temp_pose1.norm();
		temp_pose1.x()=(temp_pose1.x()/temp_dis)*800+temp_pose2.x();
		temp_pose1.y()=(temp_pose1.y()/temp_dis)*800+temp_pose2.y();
		temp_pose_df=Transformation::fieldToRobot(theRobotPose,temp_pose1);
		if(theControlParameter.Ball_RobotPose.norm()<800)
			goto farAway;
		else 
			goto defense;
		}
	}
	state(farAway){
		action{
			WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theControlParameter.Ball_RobotPose.angle(),temp_pose_aw));
		}
	}
	state(defense){
		action{
			WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(theControlParameter.Ball_RobotPose.angle(),temp_pose_df));
		}
	}
	target_state(end){
		OUTPUT_TEXT("dianqiu end");
		goto start;
	}
	
}

option(penltyKick){
	static Vector2f goal_pose;
	initial_state(start){
		transition{
			if(state_time>2000)
				goto walkToBAll;
		}
		action{
			goal_pose=theControlParameter.PosChange(2,Vector2f(0,0),Vector2f(0,0));
		}
	}
	state(walkToBAll){
		transition{
			if(action_done)
				goto Kick;
		}
		action{
			HeadPanTurn(theControlParameter.Ball_RobotPose.angle());
			WalkToBall(goal_pose);
		}
	}
	state(Kick){
		transition{
			if(action_done)
				goto end;
		}
		action{
			 kick(3);
		}
	}
	target_state(end){
		transition{
			if(state_time>100)
				goto start;
		}
		action{
			OUTPUT_TEXT("kick:::::endp");
		}
	}
	}
