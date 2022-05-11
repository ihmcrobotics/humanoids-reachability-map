package us.ihmc.reachabilityMap;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapRobotInformation;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapTools;
import us.ihmc.commons.FormattingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieReachabilityMapVisualizer
{
   public static void main(String[] args)
   {
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      String robotName = FormattingTools.underscoredToCamelCase(valkyrieRobotModel.getSimpleRobotName(), true);
      ValkyrieJointMap jointMap = valkyrieRobotModel.getJointMap();
      RobotDefinition robotDefinition = valkyrieRobotModel.getRobotDefinition();
      robotDefinition.setName(robotName);
      ReachabilityMapRobotInformation robotInformation = new ReachabilityMapRobotInformation(robotDefinition,
                                                                                             jointMap.getChestName(),
                                                                                             jointMap.getHandName(RobotSide.LEFT));
      ReachabilityMapTools.loadVisualizeReachabilityMap(robotInformation);
   }
}
