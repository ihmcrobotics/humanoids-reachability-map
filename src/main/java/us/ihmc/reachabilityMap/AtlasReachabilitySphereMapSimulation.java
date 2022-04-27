package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapCalculator;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AtlasReachabilitySphereMapSimulation
{
   public AtlasReachabilitySphereMapSimulation() throws IOException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      FloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(sdfRobot, fullRobotModel);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot, parameters);
      scs.setGroundVisible(false);

      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT));
      ReachabilitySphereMapCalculator calculator = new ReachabilitySphereMapCalculator(armJoints, scs);
      sdfRobot.setController(calculator);
      sdfRobot.setController(new RobotController()
      {
         @Override
         public void initialize()
         {
         }
         
         @Override
         public YoRegistry getYoRegistry()
         {
            return new YoRegistry("dummy");
         }
         
         @Override
         public void doControl()
         {
            jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
         }
      });
      calculator.setControlFramePose(fullRobotModel.getHandControlFrame(RobotSide.LEFT)
                                                   .getTransformToDesiredFrame(fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame()));
      FramePose3D palmCenter = new FramePose3D(fullRobotModel.getHandControlFrame(RobotSide.LEFT));
      palmCenter.changeFrame(fullRobotModel.getEndEffector(RobotSide.LEFT, LimbName.ARM).getBodyFixedFrame());
      RigidBodyTransform transformFromPalmCenterToHandBodyFixedFrame = new RigidBodyTransform();
      palmCenter.get(transformFromPalmCenterToHandBodyFixedFrame);
      //      reachabilitySphereMapCalculator.setTransformFromControlFrameToEndEffectorBodyFixedFrame(transformFromPalmCenterToHandBodyFixedFrame);
      String robotName = FormattingTools.underscoredToCamelCase(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), true);
      calculator.setupCalculatorToRecordInFile(robotName, getClass());
      //      calculator.setGridParameters(25, 0.075, 50, 1);
      calculator.setGridParameters(5, 0.075, 5, 1);
      calculator.setAngularSelection(false, true, true);

      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), armJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(calculator.getGridSizeInMeters() / 2.5, 0.15, -0.20);
      calculator.setGridFramePose(gridFramePose);

      scs.setSimulateDoneCriterion(() -> calculator.isDone());
      scs.startOnAThread();
      scs.simulate();
   }

   public static void main(String[] args) throws IOException
   {
      new AtlasReachabilitySphereMapSimulation();
   }
}
