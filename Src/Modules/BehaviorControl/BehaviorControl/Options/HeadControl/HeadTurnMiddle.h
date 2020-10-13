option(HeadScan,(float)(0.f)range,(float)(HEAD_TILT_JS)Tilt,(bool)(false)LR){   
    static int times_temp;
    static float range_temp;
    initial_state(start){
        transition{
            OUTPUT_TEXT("tilt:");
            OUTPUT_TEXT(Tilt);
            OUTPUT_TEXT("///");
            if(range>60_deg)
                range_temp=60_deg;
            else
                range_temp=range;
            if(!LR)
                goto left;
            else
                goto right;
        }
    }
    state(right){
        transition{
            if(action_done)
                goto left; 
        }
        action{
            SetHeadPanTilt(-range_temp,Tilt, HEAD_PAN_SPEED_SCAN);
        }
    }
    state(left){
        transition{
            if(action_done)
                goto right; 
        }
        action{
            SetHeadPanTilt(range_temp,Tilt, HEAD_PAN_SPEED_SCAN);
        }
    }
}