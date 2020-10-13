/**
 * @file ControlParameterProvider.h
 */

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/Libraries/ControlParameter.h"
//#include "Representations/BehaviorControl/Libraries/GoalParameter.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/TeamBallModel.h"

//#include "Representations/Infrastructure/GameInfo.h"
MODULE(ControlParameterProvider,
{,
  REQUIRES(BallModel),				/**球的信息*/
  REQUIRES(FieldDimensions),		/**场地的信息*/
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),	
  REQUIRES(RobotPose),				/**机器人在全局坐标中坐标及其方向*/
  REQUIRES(ObstacleModel),			/**关于障碍物检测的数据流*/
  REQUIRES(TeamBallModel),			/**队友传过来的球的信息*/
  REQUIRES(TeamData),
//  REQUIRES(GameInfo),
 /// REQUIRES(GoalParameter), 
  PROVIDES(ControlParameter),  
});

class ControlParameterProvider : public ControlParameterProviderBase
{
  void update(ControlParameter& ControlParameter);
  bool judge_Arm(const Vector2f& Ob_pos);
};
