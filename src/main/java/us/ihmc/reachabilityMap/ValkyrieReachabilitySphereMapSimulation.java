package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapListener;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilitySphereMapSimulation
{
   public ValkyrieReachabilitySphereMapSimulation() throws IOException
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      FloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(sdfRobot, fullRobotModel);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot, parameters);
      scs.setGroundVisible(false);

      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT));
      ReachabilitySphereMapCalculator calculator = new ReachabilitySphereMapCalculator(armJoints, scs);
      calculator.setupCalculatorToRecordInFile("Valkyrie", getClass());
      RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
      controlFrameToWristTransform.setTranslation(0.025, 0.07, 0.0);
      calculator.setControlFramePose(controlFrameToWristTransform);
      calculator.setAngularSelection(false, true, true);
      calculator.setGridParameters(40, 0.05, 50, 1);

      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), armJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(calculator.getGridSizeInMeters() / 2.5, 0.2, 0.0);
      calculator.setGridFramePose(gridFramePose);


      ReachabilityMapListener listener = new ReachabilityMapListener()
      {
         @Override
         public void hasReachedNewConfiguration()
         {
            jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
         }
      };

      calculator.attachReachabilityMapListener(listener);
      scs.startOnAThread();

      calculator.buildReachabilitySpace();
   }

   public static void main(String[] args) throws IOException
   {
      new ValkyrieReachabilitySphereMapSimulation();
   }
}
