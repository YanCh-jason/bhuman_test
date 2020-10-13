#include "ControlParameterProvider.h"
#include "Tools/Math/Eigen.h"
//#include "Representations/Communication/TeamData.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"
MAKE_MODULE(ControlParameterProvider, behaviorControl);


void ControlParameterProvider::update(ControlParameter& ControlParameter)
{   
    Vector2f pose_temp;
/**Ball position deal**/
    bool ball_judge;
    float df_dis1,df_dis2;
    float  max_validity=-1.0;
    int temp_team_other=0;
    ControlParameter.SinceBallWasOut_time = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
    for(auto const& teammate : theTeamData.teammates){
        switch(teammate.number)
        {
            case 1:
                ControlParameter.keeper = teammate; break;
            case 2:
                ControlParameter.striker1 = teammate; break;
            case 3:
                ControlParameter.defender1 = teammate; break;
            case 4:
                ControlParameter.defender2 = teammate; break;
            case 5:
                ControlParameter.striker2 = teammate; break;
            default: break;
        }
    }
    
    ball_judge=false;
    df_dis1=df_dis2=99999.f;
    if(theTeamBallModel.isValid){
            for(auto const& teamballmodel : theTeamBallModel.balls){
                switch(teamballmodel.playerNumber)
                {
                    case 1:
                        if(theFrameInfo.getTimeSince(ControlParameter.keeper.timeWhenLastPacketReceived)>8000)
                            break; 
                        if(ControlParameter.keeper.theRobotPose.validity>max_validity){
                            ControlParameter.Ball_GlobalPoseEs=teamballmodel.position;
                            max_validity=ControlParameter.keeper.theRobotPose.validity;
                        }
                        ball_judge=true;
                        break;
                    case 2:
                        if(theRobotInfo.number==2)
                            break;
                        temp_team_other=2;
                        if(theFrameInfo.getTimeSince(ControlParameter.striker1.timeWhenLastPacketReceived)>8000)
                            break;    
                        if(ControlParameter.striker1.theRobotPose.validity>max_validity){
                            ControlParameter.Ball_GlobalPoseEs=teamballmodel.position;
                            max_validity=ControlParameter.striker1.theRobotPose.validity;
                        }
                        ball_judge=true;
                        break;
                    case 3:
                        if(theFrameInfo.getTimeSince(ControlParameter.defender1.timeWhenLastPacketReceived)>8000)
                            break;
                        df_dis1=ControlParameter.defender1.theBallModel.estimate.position.norm();
                        ControlParameter.df_x1=ControlParameter.defender1.theRobotPose.translation.x();
                        if(ControlParameter.defender1.theRobotPose.validity>max_validity){
                            ControlParameter.Ball_GlobalPoseEs=teamballmodel.position;
                            max_validity=ControlParameter.defender1.theRobotPose.validity;
                        }
                        ball_judge=true;
                        break;
                    case 4:
                        if(theFrameInfo.getTimeSince(ControlParameter.defender2.timeWhenLastPacketReceived)>8000)
                            break;
                        df_dis2=ControlParameter.defender2.theBallModel.estimate.position.norm();
                        ControlParameter.df_x2=ControlParameter.defender2.theRobotPose.translation.x();
                        if(ControlParameter.defender2.theRobotPose.validity>max_validity){
                            ControlParameter.Ball_GlobalPoseEs=teamballmodel.position;
                            max_validity=ControlParameter.defender2.theRobotPose.validity;
                        }
                        ball_judge=true;
                        break;
                    case 5:
                        if(theRobotInfo.number==5)
                            break;
                        temp_team_other=5;
                        if(theFrameInfo.getTimeSince(ControlParameter.striker2.timeWhenLastPacketReceived)>8000)
                            break;
                        if(ControlParameter.striker2.theRobotPose.validity>max_validity){
                            ControlParameter.Ball_GlobalPoseEs=teamballmodel.position;
                            max_validity=ControlParameter.striker2.theRobotPose.validity;
                        }
                        ball_judge=true;
                        break;
                    default: break;
                }
            }
        ControlParameter.Ball_RobotPose=Transformation::fieldToRobot(theRobotPose,ControlParameter.Ball_GlobalPoseEs);       
    }
    ControlParameter.defender1_dis=df_dis1;
    ControlParameter.defender2_dis=df_dis2;
    if(ControlParameter.SinceBallWasOut_time<1200){
        ControlParameter.Ball_RobotPose=theBallModel.estimate.position;
        ControlParameter.Ball_GlobalPoseEs=Transformation::robotToField(theRobotPose,theBallModel.estimate.position);    
        ball_judge=true;
    }
    ControlParameter.Other_Robot_Seen=temp_team_other;
    if(ball_judge)
        ControlParameter.Ball_Seen_flag=true;
    else
        ControlParameter.Ball_Seen_flag=false;     



    if(theFrameInfo.time-ControlParameter.time_now_temp>600){
        Vector2f temp_pose_M;
        temp_pose_M.x()=ControlParameter.Ball_GlobalPoseEs.x()-ControlParameter.Ball_GlobalPoseEs_last.x();
        temp_pose_M.y()=ControlParameter.Ball_GlobalPoseEs.y()-ControlParameter.Ball_GlobalPoseEs_last.y();
        if(temp_pose_M.norm()>100)
            ControlParameter.Ball_Moved_flag=true;
        else 
            ControlParameter.Ball_Moved_flag=false;
        ControlParameter.Ball_GlobalPoseEs_last=ControlParameter.Ball_GlobalPoseEs;
        ControlParameter.time_now_temp=theFrameInfo.time;
    }
/**Ball position deal**/
    if(ControlParameter.SinceBallWasOut_time<1200||theTeamBallModel.isValid){
        //防御条件更新150150
        pose_temp.x()=ControlParameter.Ball_GlobalPoseEs.x()-theRobotPose.translation.x();
        pose_temp.y()=ControlParameter.Ball_GlobalPoseEs.y()-theRobotPose.translation.y();    
        ControlParameter.theRobotToGoal_dis=pose_temp.norm();
        if(ControlParameter.Other_Robot_Seen==5){
            pose_temp.x()=ControlParameter.Ball_GlobalPoseEs.x()-ControlParameter.striker2.theRobotPose.translation.x();
            pose_temp.y()=ControlParameter.Ball_GlobalPoseEs.y()-ControlParameter.striker2.theRobotPose.translation.y();
            ControlParameter.otherRobotToGoal_dis=pose_temp.norm();
        }
        else if(ControlParameter.Other_Robot_Seen==2){
            pose_temp.x()=ControlParameter.Ball_GlobalPoseEs.x()-ControlParameter.striker1.theRobotPose.translation.x();
            pose_temp.y()=ControlParameter.Ball_GlobalPoseEs.y()-ControlParameter.striker1.theRobotPose.translation.y();
            ControlParameter.otherRobotToGoal_dis=pose_temp.norm();
        }
        else{
            ControlParameter.otherRobotToGoal_dis=999999999;
        }
    }     

     ControlParameter.PosChange =[&](int kind,const Vector2f& pos,const Vector2f& Ball_pose) ->Vector2f
    {
        Vector2f temp_pose_re;
        
       // flag=0;
        if(kind==0){ //坐标转化
            float temp_angle_t;
            Vector2f temp_pose_t;
            temp_pose_t.x()=Ball_pose.x()-ControlParameter.Ball_GlobalPoseEs.x();
            temp_pose_t.y()=Ball_pose.y()-ControlParameter.Ball_GlobalPoseEs.y();
            temp_angle_t=pos.angle()+temp_pose_t.angle();
            temp_pose_re.x()=pos.norm()*std::cos(temp_angle_t)+ControlParameter.Ball_GlobalPoseEs.x();
            temp_pose_re.y()=pos.norm()*std::sin(temp_angle_t)+ControlParameter.Ball_GlobalPoseEs.y();
            return temp_pose_re;
        }
        else if(kind==1){ //动态目标点
            bool FrontOb=false;
            bool test_flag=false;
            int flag=0;
            bool flag_mid=false;
            int i=0;
            Vector2f temp_pose1,temp_pose2,temp_pose3,temp_pose4;
            temp_pose3=Vector2f(4700,0);
            temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
            temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
            for(const auto& obstacle : theObstacleModel.obstacles){
                    temp_pose1=Transformation::robotToField(theRobotPose,obstacle.center);
                    temp_pose1.x()=temp_pose1.x()-ControlParameter.Ball_GlobalPoseEs.x();
                    temp_pose1.y()=temp_pose1.y()-ControlParameter.Ball_GlobalPoseEs.y();
                    test_flag=temp_pose1.norm()*std::sin(std::abs(temp_pose1.angle()-temp_pose2.angle()))<300&&temp_pose1.norm()<2000;
                    if(test_flag)
                        break;
            }
            if(test_flag){
                flag=2;
                 OUTPUT_TEXT("hello2");
                 temp_pose3=Vector2f(4700,600);
                temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                temp_pose3=Vector2f(4700,-600);
                temp_pose4.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                temp_pose4.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();

                if(temp_pose2.norm()<=temp_pose4.norm()){
                    OUTPUT_TEXT("test");
                    temp_pose3=Vector2f(4700,0);
                    temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                    temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                    for(const auto& obstacle : theObstacleModel.obstacles){
                        flag_mid=false;
                        temp_pose1=Transformation::robotToField(theRobotPose,obstacle.center);
                        temp_pose1.x()=temp_pose1.x()-ControlParameter.Ball_GlobalPoseEs.x();
                        temp_pose1.y()=temp_pose1.y()-ControlParameter.Ball_GlobalPoseEs.y();
                        while(temp_pose1.norm()*std::sin(std::abs(temp_pose1.angle()-temp_pose2.angle()))<180){
                            if(temp_pose3.y()<=-600){
                                temp_pose3.y()=300;
                                temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                                temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                                flag=1;
                                break;
                            }
                            
                            if(!flag_mid){
                                temp_pose3.y()+=150;
                                temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                                temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                                if(std::abs(temp_pose3.y()+150)>600&&!flag_mid)
                                    flag_mid=true; 
                                if(temp_pose3.y()==600)
                                    temp_pose3=Vector2f(4700,0);
                            }
                            else
                            {
                                temp_pose3.y()-=150;
                                temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                                temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            }
                        }
                    }
                }
                else{
                    OUTPUT_TEXT("test123");
                    temp_pose3=Vector2f(4700,0);
                    temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                    temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                    for(const auto& obstacle : theObstacleModel.obstacles){
                        flag_mid=false;
                        temp_pose1=Transformation::robotToField(theRobotPose,obstacle.center);
                        temp_pose1.x()=temp_pose1.x()-ControlParameter.Ball_GlobalPoseEs.x();
                        temp_pose1.y()=temp_pose1.y()-ControlParameter.Ball_GlobalPoseEs.y();
                        while(temp_pose1.norm()*std::sin(std::abs(temp_pose1.angle()-temp_pose2.angle()))<180){
                            if(temp_pose3.y()>=600){
                                temp_pose3.y()=-300;
                                temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                                temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                                flag=1;
                                break;
                            }
                            
                            if(!flag_mid){
                                temp_pose3.y()-=150;
                                temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                                temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                                if(std::abs(temp_pose3.y()-150)>600&&!flag_mid)
                                flag_mid=true; 
                                if(temp_pose3.y()==-600)
                                    temp_pose3=Vector2f(4700,0);
                            }
                            else
                            {
                                temp_pose3.y()+=150;
                                temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                                temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            }
                        }
                    }
                }
            }
            if(flag==1){
                OUTPUT_TEXT("test123");
                flag=2;
                temp_pose3=Vector2f(4700,0);
                temp_pose2.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                temp_pose2.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                temp_pose4=temp_pose2;
                for(const auto& obstacle : theObstacleModel.obstacles){
                    temp_pose1=Transformation::robotToField(theRobotPose,obstacle.center);
                    temp_pose1.x()=temp_pose1.x()-ControlParameter.Ball_GlobalPoseEs.x();
                    temp_pose1.y()=temp_pose1.y()-ControlParameter.Ball_GlobalPoseEs.y();
                    while(temp_pose1.norm()<1400&&temp_pose1.norm()*std::sin(std::abs(temp_pose1.angle()-temp_pose2.angle()))<250&&ControlParameter.Ball_GlobalPoseEs.x()<3300&&temp_pose4.norm()>1700){
                        OUTPUT_TEXT("hello450");
                        flag=1;
                        if(i==0){
                            temp_pose3=Vector2f(4700,0);
                            temp_pose3.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                            temp_pose3.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            temp_pose2.x()=std::cos(temp_pose3.angle()-35_deg)*1000.0;
                            temp_pose2.y()=std::sin(temp_pose3.angle()-35_deg)*1000.0;
                            i++;
                        }
                        else if(i==1){
                            temp_pose3=Vector2f(4700,0);
                            temp_pose3.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                            temp_pose3.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            temp_pose2.x()=std::cos(temp_pose3.angle()+35_deg)*1000.0;
                            temp_pose2.y()=std::sin(temp_pose3.angle()+35_deg)*1000.0;
                            i++;
                        }
                        else if(i==2){
                            temp_pose3=Vector2f(4700,0);
                            temp_pose3.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                            temp_pose3.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            temp_pose2.x()=std::cos(temp_pose3.angle()+45_deg)*1000.0;
                            temp_pose2.y()=std::sin(temp_pose3.angle()+45_deg)*1000.0;
                            i++;
                        }
                        else if(i==3){
                            temp_pose3=Vector2f(4700,0);
                            temp_pose3.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                            temp_pose3.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            temp_pose2.x()=std::cos(temp_pose3.angle()-45_deg)*1000.0;
                            temp_pose2.y()=std::sin(temp_pose3.angle()-45_deg)*1000.0;
                            i++;
                        }
                        else if(i==4){
                            temp_pose3=Vector2f(4700,0);
                            temp_pose3.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                            temp_pose3.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            temp_pose2.x()=std::cos(temp_pose3.angle()+60_deg)*1000.0;
                            temp_pose2.y()=std::sin(temp_pose3.angle()+60_deg)*1000.0;
                            i++;
                        }
                        else if(i==5){
                            temp_pose3=Vector2f(4700,0);
                            temp_pose3.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                            temp_pose3.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            temp_pose2.x()=std::cos(temp_pose3.angle()-60_deg)*1000.0;
                            temp_pose2.y()=std::sin(temp_pose3.angle()-60_deg)*1000.0;
                            i++;
                        }
                        else if(i==6){
                            temp_pose3=Vector2f(4700,0);
                            temp_pose3.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                            temp_pose3.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            temp_pose2.x()=std::cos(temp_pose3.angle()+70_deg)*1000.0;
                            temp_pose2.y()=std::sin(temp_pose3.angle()+70_deg)*1000.0;
                            i++;
                        }
                        else if(i==7){
                            temp_pose3=Vector2f(4700,0);
                            temp_pose3.x()=temp_pose3.x()-ControlParameter.Ball_GlobalPoseEs.x();
                            temp_pose3.y()=temp_pose3.y()-ControlParameter.Ball_GlobalPoseEs.y();
                            temp_pose2.x()=std::cos(temp_pose3.angle()-70_deg)*1000.0;
                            temp_pose2.y()=std::sin(temp_pose3.angle()-70_deg)*1000.0;
                            i++;
                        }
                        else{
                            i=0;
                            flag=2;
                            break;
                        }
                    }
                }
            }
            if(flag==1)
                ControlParameter.ball_goal=true;
            else
                ControlParameter.ball_goal=false;
            if(flag==0){
                OUTPUT_TEXT("angle");
                temp_pose_re=Vector2f(4700,0);
            }
            else
            {
                temp_pose_re.x()=temp_pose2.x()+ControlParameter.Ball_GlobalPoseEs.x();
                temp_pose_re.y()=temp_pose2.y()+ControlParameter.Ball_GlobalPoseEs.y();
            }
            if(std::abs(temp_pose_re.angle()-90_deg)>70_deg)
                temp_pose_re=Vector2f(4700,300);
            ControlParameter.Pose_Goal=temp_pose_re;
            return temp_pose_re;
        }
        else if(kind==2){
            bool flag1=false,flag2=false,flag3=false;
            Vector2f temp_pose4,temp_pose5;
            Vector2f temp_pose1=Vector2f(2400,0);
            Vector2f temp_pose2=Vector2f(2400,-320);
            Vector2f temp_pose3=Vector2f(2400,320);
            for(const auto& obstacle : theObstacleModel.obstacles){
                    temp_pose5.x()=obstacle.center.x()-theBallModel.estimate.position.x();
                    temp_pose5.y()=obstacle.center.y()-theBallModel.estimate.position.y();
                    temp_pose4.x()=temp_pose1.x()-theBallModel.estimate.position.x();
                    temp_pose4.y()=temp_pose1.y()-theBallModel.estimate.position.y();
                    if(temp_pose5.norm()*std::sin(std::abs(temp_pose4.angle()-temp_pose5.angle()))<200){
                        flag1=true;
                    }
                    temp_pose4.x()=temp_pose2.x()-theBallModel.estimate.position.x();
                    temp_pose4.y()=temp_pose2.y()-theBallModel.estimate.position.y();
                    if(temp_pose5.norm()*std::sin(std::abs(temp_pose4.angle()-temp_pose5.angle()))<200){
                        flag2=true;
                    }
                    temp_pose4.x()=temp_pose3.x()-theBallModel.estimate.position.x();
                    temp_pose4.y()=temp_pose3.y()-theBallModel.estimate.position.y();
                    if(temp_pose5.norm()*std::sin(std::abs(temp_pose4.angle()-temp_pose5.angle()))<200){
                        flag3=true;
                    }
                }
            if(!flag1)
                return Transformation::robotToField(theRobotPose,temp_pose1);
            else if(!flag3)
                return Transformation::robotToField(theRobotPose,temp_pose3);    
            else if(!flag2)
                return Transformation::robotToField(theRobotPose,temp_pose2);
            else
                return Transformation::robotToField(theRobotPose,temp_pose2);
        } 
    };






    bool judge_flag_front=false,judge_flag_arm=false,judge_flag_ball_Near=false;
    int  judge_flag_ball=0;
    pose_temp.x()=ControlParameter.Ball_GlobalPoseEs.x()-4500;
    pose_temp.y()=ControlParameter.Ball_GlobalPoseEs.y();
    for(const auto& obstacle : theObstacleModel.obstacles){
            if(obstacle.center.norm()<300||obstacle.center.norm()<400&&std::abs(obstacle.center.angle())<50_deg)
                judge_flag_front=true;
            if(judge_Arm(obstacle.center))
                judge_flag_arm=true; 
            if(obstacle.center.norm()<600&&obstacle.type==Obstacle::teammate&&theRobotInfo.number==5&&theRobotPose.translation.x()<10)
                judge_flag_ball_Near=true;       
            if((std::abs(obstacle.center.angle())<75_deg&&obstacle.center.norm()<500)||obstacle.center.norm()<600&&ControlParameter.Ball_GlobalPoseEs.x()>3300){
                judge_flag_ball=1;
                break;
            }
            else if((std::abs(obstacle.center.angle())<75_deg&&obstacle.center.norm()<500)||ControlParameter.Ball_GlobalPoseEs.x()>3300){ 
                judge_flag_ball=2;  
                break;
            }
	}
    if(judge_flag_front)
        ControlParameter.Obstacle_Front=true;
    else
        ControlParameter.Obstacle_Front=false;
    if(judge_flag_arm)
        ControlParameter.Obstacle_Arm=true;    
    else
        ControlParameter.Obstacle_Arm=false;
    if(judge_flag_ball==1)
        ControlParameter.Obstacle_Ball=1;
    else if(ControlParameter.ball_goal||judge_flag_ball==2)
        ControlParameter.Obstacle_Ball=2;
    else  
        ControlParameter.Obstacle_Ball=3;

    if(judge_flag_ball_Near)
        ControlParameter.Obstacle_Front_Near=true;
    else
        ControlParameter.Obstacle_Front_Near=false;
   // OUTPUT_TEXT(ControlParameter.Obstacle_Ball);          
    // OUTPUT_TEXT(ControlParameter.Obstacle_Arm);
    // OUTPUT_TEXT(ControlParameter.Obstacle_Front);
    //  OUTPUT_TEXT(ControlParameter.Obstacle_Ball);
    // };
    /*坐标系转化*/
   
    
    /***障碍物检测函数*/
    DECLARE_DEBUG_DRAWING("test", "drawingOnField");
    CROSS("test", ControlParameter.Pose_Goal.x(),ControlParameter.Pose_Goal.y(), 50, 50, Drawings::solidPen, ColorRGBA::yellow);
    LINE("test", ControlParameter.Ball_GlobalPoseEs.x(), ControlParameter.Ball_GlobalPoseEs.y(), ControlParameter.Pose_Goal.x(), ControlParameter.Pose_Goal.y(), 40, Drawings::solidPen, ColorRGBA::gray);

}


bool ControlParameterProvider::judge_Arm(const Vector2f& Ob_pos){
    if(Ob_pos.norm()<500)
        return true;
    return false;
}


