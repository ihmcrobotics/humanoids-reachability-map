package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapRobotInformation;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper;
import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AtlasReachabilitySphereMapSimulation
{
   public AtlasReachabilitySphereMapSimulation() throws IOException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);

      String chestName = robotModel.getJointMap().getChestName();
      String leftHandName = robotModel.getJointMap().getHandName(RobotSide.LEFT);

      RobotDefinition robotDefinition = robotModel.createRobotDefinition();
      robotDefinition.setName(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString());
      ReachabilityMapRobotInformation robotInformation = new ReachabilityMapRobotInformation(robotDefinition, chestName, leftHandName);
      robotInformation.setControlFramePoseInParentJoint(robotModel.getJointMap().getHandControlFrameToWristTransform(RobotSide.LEFT));
      robotInformation.setOrthogonalToPalm(Axis3D.X);

      ReachabilitySphereMapSimulationHelper simHelper = new ReachabilitySphereMapSimulationHelper(robotInformation);
      simHelper.setGridParameters(25, 0.025, 50, 1);
      simHelper.setEvaluateRReachability(false);
      simHelper.setEvaluateR2Reachability(false);
      simHelper.setGridPosition(0.4, 0.4, 0.5);

      if (simHelper.start())
      {
         simHelper.exportDataToFile(getClass());
      }
   }

   public static void main(String[] args) throws IOException
   {
      new AtlasReachabilitySphereMapSimulation();
   }
}
