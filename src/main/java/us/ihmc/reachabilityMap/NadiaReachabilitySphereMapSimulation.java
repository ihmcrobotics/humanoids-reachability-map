package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapListener;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.nadia.parameters.NadiaModelFactory;
import us.ihmc.nadia.parameters.robotVersions.NadiaVersion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class NadiaReachabilitySphereMapSimulation
{
   public NadiaReachabilitySphereMapSimulation() throws IOException
   {
      
      NadiaVersion nadiaVersion = NadiaVersion.V16_RIGHT_ARM;
      NadiaModelFactory modelFactory = new NadiaModelFactory(nadiaVersion, nadiaVersion.getJointMap(), null);
      FloatingRootJointRobot sdfRobot = modelFactory.createSdfRobot();
      FullRobotModel fullRobotModel = modelFactory.createFullRobotModel();

      OneDoFJointBasics firstShoulderJoint = fullRobotModel.getOneDoFJointByName("RIGHT_SHOULDER_Y");
      OneDoFJointBasics lastWristJoint = fullRobotModel.getOneDoFJointByName("RIGHT_WRIST_Y");
      
      
      
      final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(sdfRobot, fullRobotModel);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot, parameters);
      scs.setGroundVisible(false);

      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(firstShoulderJoint.getPredecessor(), lastWristJoint.getSuccessor());
      ReachabilitySphereMapCalculator calculator = new ReachabilitySphereMapCalculator(armJoints, scs);
      calculator.setupCalculatorToRecordInFile("Nadia", getClass());
      RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
      controlFrameToWristTransform.setTranslation(0.0, 0.0, -0.14);
      calculator.setControlFramePose(controlFrameToWristTransform);
      calculator.setAngularSelection(false, true, true);
      calculator.setGridParameters(40, 0.05, 50, 1);

      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), armJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(calculator.getGridSizeInMeters() / 4.0, 0.2, 0.0);
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
      new NadiaReachabilitySphereMapSimulation();
   }
}
