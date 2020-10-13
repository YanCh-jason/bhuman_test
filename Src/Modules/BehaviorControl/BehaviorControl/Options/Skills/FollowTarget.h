
option(FollowTarget_B){
    float temp_angle;
    common_transition{
        temp_angle=Transformation::fieldToRobot(theRobotPose,theControlParameter.Ball_GlobalPoseEs).angle();
        if(std::abs(temp_angle)>25.0)
            goto turnToBall;
    }
    initial_state(start){
        action{
            OUTPUT_TEXT("FollowTarget_B::start");
            HeadPanTurn(temp_angle,HEAD_PAN_SPEED);
            Stand();
        }
    }
    state(turnToBall){
        transition{
            temp_angle=Transformation::fieldToRobot(theRobotPose,theControlParameter.Ball_GlobalPoseEs);
            if(std::abs(temp_angle)<5_deg)
                goto start;
        }
        action{
            OUTPUT_TEXT("FollowTarget_B::turnToBall");
            HeadPanTurn(temp_angle,HEAD_PAN_SPEED);
            WalkToTarget(Pose2f(MOVE_ROTATION_SPEED,MOVE_X_SPEED,MOVE_Y_SPEED),Pose2f(temp_angle,0.0,0.0));
        }
    }
}