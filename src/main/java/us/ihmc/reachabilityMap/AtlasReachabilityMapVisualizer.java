package us.ihmc.reachabilityMap;

import javafx.application.Platform;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapRobotInformation;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapVisualizer;
import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AtlasReachabilityMapVisualizer
{
   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      AtlasJointMap jointMap = robotModel.getJointMap();
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      robotDefinition.setName(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString());
      ReachabilityMapRobotInformation robotInformation = new ReachabilityMapRobotInformation(robotDefinition,
                                                                                             jointMap.getChestName(),
                                                                                             jointMap.getHandName(RobotSide.LEFT));

      ReachabilityMapVisualizer visualizer = new ReachabilityMapVisualizer(robotInformation);

      if (!visualizer.loadReachabilityMapFromLatestFile(AtlasReachabilitySphereMapSimulation.class))
      {
         Platform.exit();
         return;
      }

      visualizer.setVisualizePositionReach(false);
      visualizer.setVisualizeRayReach(true);
      visualizer.setVisualizePoseReach(false);

      // Only visualizing reach from the left side
      visualizer.setRayFilter(ReachabilityMapVisualizer.newHemisphereFilter(Axis3D.Y.negated()));

      visualizer.visualize();
   }
}
