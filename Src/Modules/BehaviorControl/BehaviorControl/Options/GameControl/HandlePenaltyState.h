/** Handle penalty state (and the actions to be performed after leaving the penalty state).
 *   This option is one level higher than the main game state handling as penalties
 *   might occur in most game states. */
option(HandlePenaltyState)
{
  /** By default, the robot is not penalized and plays soccer according to the current game state.
  The chestbutton has to be pushed AND released to manually penalize a robot */
	static Vector2f temp_pose;
 initial_state(notPenalized)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
    }
    action
    {
      //OUTPUT_TEXT("HandlePenaltyState_notPenalized_action");
      //theHeadControlMode = HeadControl::lookLeftAndRight;
      HandleGameState();
    }
  }
  /** In case of any penalty, the robots stands still. */
  state(penalized)
  {
    transition
    {
      if(theRobotInfo.penalty == PENALTY_NONE)
        goto preUnpenalize;
    }
    action
    {
      PlaySound("penalized.wav");
      //OUTPUT_TEXT("HandlePenaltyState_penalized_action");
      theHeadControlMode = HeadControl::lookForward;
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }
  state(preUnpenalize)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      else if(theGameInfo.state == STATE_INITIAL || state_time > 500)
      {
		  // temp_pose=Transformation::robotToField(theRobotPose,Vector2f(1500,0));
      //      switch(theRobotInfo.number)
      //       {
			// case 2: //if(theControlParameter.Ball_Seen_flag)
			// 			//goto striker2_attack;
			// 		//else 
			// 			goto Striker2goToField;
			// 		break;
					
			// case 5:// if(theControlParameter.Ball_Seen_flag)
			// 			//goto striker5_attack;
			// 		//else 
			// 			goto Striker5goToField;
			// 		break;
			// default:goto  notPenalized;
      //       }	
      goto penltyKick1;	
      }
    }
    action
    {
      PlaySound("notPenalized.wav");
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }
  
  state(penltyKick1){
	action{
		penltyKick();
	}
}
  
  
  state(penltyKick){
	transition{
		if(action_done)
		  goto notPenalized;
	}
	action{
		penltyKick();
	}
}




  state(striker2_attack){
	  action{
		  Striker2_attack();
	  }
  }
    state(striker5_attack){
	  action{
		  Striker5_attack();
	  }
  }
  
  state(Striker2goToField)
  {
	  transition
	  {
		  if(state_time>4000)  
			goto notPenalized;
		  if(action_done)
			goto wait2;
	  }
	  action
	  {
		  HeadPanTurn(0);
		  WalkToTargetDes(temp_pose);
      }
  }
  
    state(Striker5goToField)
  {
	  transition
	  {
		  if(state_time>4000)  
			goto notPenalized;
		  if(action_done)
			goto wait1;
	  }
	  action
	  {
		  HeadPanTurn(0);
		  WalkToTargetDes(temp_pose);
      }
  }
  state(wait1){
	  transition{
		  if(theControlParameter.Ball_Seen_flag)
			goto notPenalized;
		  if(state_time>1500)
			  goto notPenalized;
	  }
	  action{
		  Stand();
	  }
  }
   state(wait2){
	  transition{
		  if(theControlParameter.Ball_Seen_flag)
			goto notPenalized;
		  if(state_time>1500)
			  goto notPenalized;
	  }
	  action{
		  Stand();
	  }
  }
}
