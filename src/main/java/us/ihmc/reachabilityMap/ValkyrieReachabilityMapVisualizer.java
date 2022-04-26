package us.ihmc.reachabilityMap;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapTools;
import us.ihmc.commons.FormattingTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilityMapVisualizer
{
   public static void main(String[] args)
   {
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      String robotName = FormattingTools.underscoredToCamelCase(valkyrieRobotModel.getSimpleRobotName(), true);
      FullHumanoidRobotModel fullRobotModel = valkyrieRobotModel.createFullRobotModel();
      ReachabilityMapTools.loadVisualizeReachabilityMap(robotName, valkyrieRobotModel.getRobotDefinition(), fullRobotModel);
   }
}
