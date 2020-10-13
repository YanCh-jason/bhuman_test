option(kick,(int)(1)kind){
    initial_state(start){
        transition{
			if(kind==1){
                goto kick2;
            }
            else if(kind==2) {
                goto kick3;
            }
            else if(kind==3) {
                goto kick1;
            }
		}
		   
		action{
			OUTPUT_TEXT("kick::init");
			Stand();
		}
    }
    state(kick1){
        transition{			
            if(state_time>3000){
				goto end;
			}
        }
        action{
			OUTPUT_TEXT("kick::kick shoot");
           if(state_time<500)
               Stand();
           else    
               Shoot(KickRequest::greatKickForward); //
        }    
    }
    state(kick2){
        transition{
              if(state_time>2000){ 
				goto end;
			}
        }
        action{
			OUTPUT_TEXT("kick::kick go");
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED+0.1,MOVE_Y_SPEED+0.1),Pose2f(0,300,0));
        }    
    }
     state(kick3){
        transition{
              if(state_time>2000){
				goto end;
			}
        }
        action{
			OUTPUT_TEXT("kick::kick fastkick");
          Shoot(KickRequest::fastkick);
        }    
    }
    target_state(end){
        transition{
            if(state_time>20)
                goto start;
        }
        action{
            OUTPUT_TEXT("kick::end");
        }
    }

}