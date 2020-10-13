option(LookLeftandRight, (float) (1.0) rightPan, (float) (-1.0) leftPan, (float) (0.38f) tilt,(float)(0.7*pi) speed, (HeadMotionRequest::CameraControlMode)(HeadMotionRequest::autoCamera) camera)
{
  /** Set the head motion request. */
  initial_state(Request1)
  {
    transition
    {
      if(action_done)
        goto pausetime1;
    }
    action
    {
      SetHeadPanTilt(rightPan, tilt, speed);
    }
  }
  state(pausetime1){
	  
	 transition
    {
      if(action_done)
        goto pausetime2;
    }
    action
    {
      SetHeadPanTilt(leftPan, tilt, speed);
    } 
	  
  }
   state(pausetime2){
	  
	 transition
    {
      if(action_done)
        goto pausetime3;
    }
    action
    {
      SetHeadPanTilt(0, tilt, speed);
    } 
	  
  }
     state(pausetime3){
		 
   transition{
	   goto Request1;
   }
     
	  
  }
  
  
 
  /** This state "notifies" the caller that the requested head angles are reached */

}