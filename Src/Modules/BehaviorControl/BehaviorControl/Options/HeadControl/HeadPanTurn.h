option(HeadPanTurn,(float)(0)pan,(float)(HEAD_TILT_JS)Tilt,(float)(HEAD_PAN_SPEED) speed)
{
    initial_state(start){
        transition
        {  
                if(action_done)
                    goto end_state;
        }
        action
        {
            OUTPUT_TEXT("tilt:");
            OUTPUT_TEXT(Tilt);
            OUTPUT_TEXT("///");
            SetHeadPanTilt(pan,Tilt, speed);
        }
    }
    state(end_state){
        transition{
                goto start;
        }
    }
}