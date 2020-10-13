/** All option files that belong to the current behavior have to be included by this file. */


#include "Options/Soccer.h"

#include "Options/GameControl/HandleGameState.h"
#include "Options/GameControl/HandlePenaltyState.h"
#include "Options/GameControl/Ready.h"
#include "Options/GameControl/test.h"
#include "Options/GameControl/FinishedState.h"

#include "Options/HeadControl/LookForward.h"
#include "Options/HeadControl/FollowTheTarget.h"
#include "Options/HeadControl/FixPointRotate.h"
#include "Options/HeadControl/LookLeftandRight.h"
#include "Options/HeadControl/HeadPanTurn.h"
#include "Options/HeadControl/HeadTurnMiddle.h"







#include "Options/Output/Activity.h"
#include "Options/Output/Annotation.h"
#include "Options/Output/HeadControlMode.h"
#include "Options/Output/HeadMotionRequest/SetHeadPanTilt.h"
#include "Options/Output/HeadMotionRequest/SetHeadTarget.h"
#include "Options/Output/HeadMotionRequest/SetHeadTargetOnGround.h"
#include "Options/Output/MotionRequest/InWalkKick.h"
#include "Options/Output/MotionRequest/SpecialAction.h"
#include "Options/Output/MotionRequest/Stand.h"
#include "Options/Output/MotionRequest/WalkAtAbsoluteSpeed.h"
#include "Options/Output/MotionRequest/WalkAtRelativeSpeed.h"
#include "Options/Output/MotionRequest/WalkToTarget.h"
#include "Options/Output/MotionRequest/GetUpEngine.h"
//#include "Options/Output/MotionRequest/WalkToDestination.h"
#include "Options/Output/MotionRequest/Shoot.h"
#include "Options/Output/PlaySound.h"

#include "Options/Roles/Striker2/Striker2.h"

 #include "Options/Roles/Striker5/Striker5_attack.h"
 #include "Options/Roles/Striker2/Striker2_attack.h"

#include "Options/Roles/Striker5/Striker5.h"
#include "Options/seniorSkill/Kick.h"

#include "Options/Skills/GetUp.h"
#include "Options/Skills/WalkRotate.h"
#include "Options/seniorSkill/SearchForBallNoBallState.h"
#include "Options/seniorSkill/SearchForBallStay.h"
#include "Options/Skills/WalkToTargetDes.h"
#include "Options/seniorSkill/WalkToBallState.h"




#include "Options/Tools/ButtonPressedAndReleased.h"
#include "Options/Tools/USBCheck.h"
#include "Options/Tools/Cout_text.h"