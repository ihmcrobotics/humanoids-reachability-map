package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapCalculator;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;

public class AtlasReachabilitySphereMapSimulation
{
   public AtlasReachabilitySphereMapSimulation() throws IOException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      String robotName = FormattingTools.underscoredToCamelCase(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), true);

      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      robotDefinition.ignoreAllJoints();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      RigidBodyBasics base = fullRobotModel.getChest();
      RigidBodyBasics endEffector = fullRobotModel.getHand(RobotSide.LEFT);
      ReferenceFrame controlFrame = fullRobotModel.getHandControlFrame(RobotSide.LEFT);

      SimulationSession session = new SimulationSession("Reachability Analysis - Atlas");
      session.initializeBufferSize(16000);
      Robot robot = session.addRobot(robotDefinition);
      ReachabilitySphereMapCalculator calculator = setupCalculator(robotName, base, endEffector, controlFrame, robot.getControllerOutput());

      robot.addController(calculator);
      session.addYoGraphicDefinition(calculator.getYoGraphicVisuals());

      SessionVisualizerControls guiControls = SessionVisualizer.startSessionVisualizer(session);
      guiControls.setCameraFocusPosition(calculator.getGridFramePose().getX(), calculator.getGridFramePose().getY(), calculator.getGridFramePose().getZ());
      guiControls.setCameraOrientation(Math.toRadians(15.0), Math.toRadians(170.0), 0.0);
      calculator.setStaticVisualConsumer(guiControls::addStaticVisual);

      SimulationSessionControls simControls = session.getSimulationSessionControls();
      simControls.addExternalTerminalCondition(calculator::isDone);
      simControls.simulate(Integer.MAX_VALUE);
   }

   private ReachabilitySphereMapCalculator setupCalculator(String robotName,
                                                           RigidBodyBasics base,
                                                           RigidBodyBasics endEffector,
                                                           ReferenceFrame controlFrame,
                                                           ControllerOutput controllerOutput)
         throws IOException
   {
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);
      ReachabilitySphereMapCalculator calculator = new ReachabilitySphereMapCalculator(armJoints, controllerOutput);

      calculator.setControlFramePose(controlFrame.getTransformToDesiredFrame(endEffector.getBodyFixedFrame()));
      //      FramePose3D palmCenter = new FramePose3D(controlFrame);
      //      palmCenter.changeFrame(endEffector.getBodyFixedFrame());
      //      RigidBodyTransform transformFromPalmCenterToHandBodyFixedFrame = new RigidBodyTransform();
      //      palmCenter.get(transformFromPalmCenterToHandBodyFixedFrame);
      //      calculator.setTransformFromControlFrameToEndEffectorBodyFixedFrame(transformFromPalmCenterToHandBodyFixedFrame);

      calculator.setupCalculatorToRecordInFile(robotName, getClass());
            calculator.setGridParameters(25, 0.075, 50, 1);
//      calculator.setGridParameters(5, 0.075, 5, 1);
      calculator.setAngularSelection(false, true, true);

      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), armJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(calculator.getGridSizeInMeters() / 2.5, 0.15, -0.20);
      calculator.setGridFramePose(gridFramePose);

      return calculator;
   }

   public static void main(String[] args) throws IOException
   {
      new AtlasReachabilitySphereMapSimulation();
   }
}
