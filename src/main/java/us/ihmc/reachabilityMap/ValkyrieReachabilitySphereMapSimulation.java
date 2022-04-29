package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilitySphereMapSimulation
{
   public ValkyrieReachabilitySphereMapSimulation() throws IOException
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

      String chestName = robotModel.getJointMap().getChestName();
      String leftHandName = robotModel.getJointMap().getHandName(RobotSide.LEFT);
      ReachabilitySphereMapSimulationHelper simHelper = new ReachabilitySphereMapSimulationHelper(robotModel.getRobotDefinition(), chestName, leftHandName);
      simHelper.setGridParameters(30, 0.05, 50, 1);
      simHelper.setAngularSelection(false, true, true);
      simHelper.setGridPosition(0.5, 0.2, 0.32);
      RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
      controlFrameToWristTransform.getTranslation().set(0.025, 0.07, 0.0);
      simHelper.setControlFramePoseInParentJoint(controlFrameToWristTransform);

      simHelper.start();

      String robotName = "Valkyrie";
      simHelper.exportDataToFile(robotName, getClass());
   }

   public static void main(String[] args) throws IOException
   {
      new ValkyrieReachabilitySphereMapSimulation();
   }
}
