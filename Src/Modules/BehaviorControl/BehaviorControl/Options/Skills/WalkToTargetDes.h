option(WalkToTargetDes,(Vector2f)target){ 
    static Vector2f pos_temp;
    common_transition{
        pos_temp=Transformation::fieldToRobot(theRobotPose,target);
    }
    initial_state(start){
        transition{
            goto TurnState;
        }
    }
    state(TurnState){
        transition{
            if(std::abs(pos_temp.angle())<10_deg)
                goto WalkState;
            if(std::abs(pos_temp.angle())>30_deg)    
                goto TurnState1;
        }
        action{
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(pos_temp.angle(),pos_temp));
        }
    }
     state(TurnState1){
        transition{
            if(std::abs(pos_temp.angle())<10_deg)
                goto WalkState;
        }
        action{
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(pos_temp.angle(),0,0));
        }
    }
    state(WalkState){
        transition{
            if(std::abs(pos_temp.angle())>15_deg)
                goto TurnState;
            if(pos_temp.norm()<DIS_NEAR_TARGET)
                goto end;
        }
        action{
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(0,pos_temp));
        }
    }
    
    target_state(end){
        transition{
            goto start;
        }
        action{
           OUTPUT_TEXT("WalkToTargetDes::end"); 
        }
    }

}