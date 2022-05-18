package us.ihmc.reachabilityMap;

import javafx.application.Platform;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapRobotInformation;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapVisualizer;
import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieReachabilityMapVisualizer
{
   public static void main(String[] args)
   {
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      ValkyrieJointMap jointMap = valkyrieRobotModel.getJointMap();
      RobotDefinition robotDefinition = valkyrieRobotModel.getRobotDefinition();
      robotDefinition.setName("Valkyrie");
      ReachabilityMapRobotInformation robotInformation = new ReachabilityMapRobotInformation(robotDefinition,
                                                                                             jointMap.getChestName(),
                                                                                             jointMap.getHandName(RobotSide.LEFT));

      ReachabilityMapVisualizer visualizer = new ReachabilityMapVisualizer(robotInformation);

      if (!visualizer.loadReachabilityMapFromLatestFile(ValkyrieReachabilitySphereMapSimulation.class))
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
