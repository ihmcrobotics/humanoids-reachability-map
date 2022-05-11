package us.ihmc.reachabilityMap;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapRobotInformation;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapTools;
import us.ihmc.commons.FormattingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AtlasReachabilityMapVisualizer
{
   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      AtlasJointMap jointMap = robotModel.getJointMap();
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      robotDefinition.setName(FormattingTools.underscoredToCamelCase(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), true));
      ReachabilityMapRobotInformation robotInformation = new ReachabilityMapRobotInformation(robotDefinition,
                                                                                             jointMap.getChestName(),
                                                                                             jointMap.getHandName(RobotSide.LEFT));
      ReachabilityMapTools.loadVisualizeReachabilityMap(robotInformation);
   }
}
