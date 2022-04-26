package us.ihmc.reachabilityMap;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapTools;
import us.ihmc.commons.FormattingTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class AtlasReachabilityMapVisualizer
{
   public static void main(String[] args)
   {
      String robotName = FormattingTools.underscoredToCamelCase(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), true);
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      FullHumanoidRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();
      ReachabilityMapTools.loadVisualizeReachabilityMap(robotName, atlasRobotModel.getRobotDefinition(), fullRobotModel);
   }
}
