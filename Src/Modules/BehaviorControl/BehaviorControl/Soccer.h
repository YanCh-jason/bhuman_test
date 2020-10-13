/**
 * @file Soccer.h
 *
 * This file declares the root behavior option (playing soccer).
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorOption.h"

#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif
#include<string.h>


#include "Platform/SystemCall.h"
#include "ParameterDefine.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/ControlParameter.h"
//#include "Representations/BehaviorControl/Libraries/GoalParameter.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/FootBumperState.h"
#include "Tools/Math/Transformation.h"





#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX Soccer::
#endif
#include "Tools/Cabsl.h"


float angletest=10;
using namespace Transformation;

BEHAVIOR_OPTION(Soccer, BehaviorOptionInterface,
{,
  REQUIRES(ArmContactModel),
  REQUIRES(BallModel),
  REQUIRES(BehaviorParameters),
  REQUIRES(CameraStatus),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(GroundContactState),
  REQUIRES(HeadMotionEngineOutput),
  REQUIRES(JointRequest),
  REQUIRES(KeyStates),
  REQUIRES(ControlParameter),
 // REQUIRES(GoalParameter),
  REQUIRES(MotionInfo),
  REQUIRES(RobotHealth),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(ActivationGraph),
  REQUIRES(ArmKeyFrameEngineOutput),
  REQUIRES(ArmMotionInfo),
  REQUIRES(ArmMotionSelection),
  REQUIRES(BallSpecification),
  //REQUIRES(CameraCalibrationRating),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FieldDimensions),
  REQUIRES(FootBumperState),
  REQUIRES(GlobalFieldCoverage),
  REQUIRES(JointAngles),
  REQUIRES(KickEngineOutput),
  //REQUIRES(LibDemo),
  REQUIRES(WalkGenerator),
  REQUIRES(LegMotionSelection),
  REQUIRES(ObstacleModel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PathPlanner),
  REQUIRES(SideConfidence),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamData),
  REQUIRES(WalkingEngineOutput),
  
  MODIFIES(ArmMotionRequest),
  MODIFIES(BehaviorStatus),
  MODIFIES(HeadControlMode),
  MODIFIES(HeadMotionRequest),
  MODIFIES(MotionRequest),
});

class Soccer : public SoccerBase, public Cabsl<Soccer>
{
public:
  /** Constructor. */
  Soccer();
private:
  /** Executes the behavior. */
  void execute() override;

#include "Options.h"
};