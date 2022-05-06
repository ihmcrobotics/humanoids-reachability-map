package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasReachabilitySphereMapSimulation
{
   public AtlasReachabilitySphereMapSimulation() throws IOException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);

      String chestName = robotModel.getJointMap().getChestName();
      String leftHandName = robotModel.getJointMap().getHandName(RobotSide.LEFT);
      ReachabilitySphereMapSimulationHelper simHelper = new ReachabilitySphereMapSimulationHelper(robotModel.getRobotDefinition(), chestName, leftHandName);
      simHelper.setGridParameters(25, 0.025, 50, 1);
      simHelper.setEvaluateDReachability(false);
      simHelper.setEvaluateD0Reachability(false);
      simHelper.setPalmOrthogonalAxis(Axis3D.X);
      simHelper.setGridPosition(0.4, 0.4, 0.5);
      simHelper.setControlFramePoseInParentJoint(robotModel.getJointMap().getHandControlFrameToWristTransform(RobotSide.LEFT));

      simHelper.start();

      String robotName = FormattingTools.underscoredToCamelCase(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), true);
      simHelper.exportDataToFile(robotName, getClass());
   }

   public static void main(String[] args) throws IOException
   {
      new AtlasReachabilitySphereMapSimulation();
   }
}
