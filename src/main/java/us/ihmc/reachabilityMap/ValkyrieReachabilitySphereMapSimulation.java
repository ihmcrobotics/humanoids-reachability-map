package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapTools;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapCalculator;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilitySphereMapSimulation
{
   public ValkyrieReachabilitySphereMapSimulation() throws IOException
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      String robotName = "Valkyrie";

      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      robotDefinition.ignoreAllJoints();

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      RigidBodyBasics base = fullRobotModel.getChest();
      RigidBodyBasics endEffector = fullRobotModel.getHand(RobotSide.LEFT);
      RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
      controlFrameToWristTransform.getTranslation().set(0.025, 0.07, 0.0);

      SimulationSession session = new SimulationSession("Reachability Analysis - Atlas");
      session.initializeBufferSize(16000);
      Robot robot = session.addRobot(robotDefinition);
      ReachabilitySphereMapCalculator calculator = setupCalculator(robotName, base, endEffector, controlFrameToWristTransform, robot.getControllerOutput());

      robot.addController(calculator);
      session.addYoGraphicDefinition(calculator.getYoGraphicVisuals());

      SessionVisualizerControls guiControls = SessionVisualizer.startSessionVisualizer(session);
      guiControls.setCameraFocusPosition(calculator.getGridFramePose().getX(), calculator.getGridFramePose().getY(), calculator.getGridFramePose().getZ());
      guiControls.setCameraOrientation(Math.toRadians(15.0), Math.toRadians(170.0), 0.0);
      guiControls.addStaticVisuals(ReachabilityMapTools.createReachibilityColorScaleVisuals());
      guiControls.addStaticVisuals(ReachabilityMapTools.createBoundingBoxVisuals(calculator.getVoxel3DGrid()));
      calculator.setStaticVisualConsumer(guiControls::addStaticVisual);

      SimulationSessionControls simControls = session.getSimulationSessionControls();
      simControls.addExternalTerminalCondition(calculator::isDone);
      simControls.simulate(Integer.MAX_VALUE);
   }

   private ReachabilitySphereMapCalculator setupCalculator(String robotName,
                                                           RigidBodyBasics base,
                                                           RigidBodyBasics endEffector,
                                                           RigidBodyTransform controlFrameTransform,
                                                           ControllerOutput controllerOutput)
         throws IOException
   {
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);
      ReachabilitySphereMapCalculator calculator = new ReachabilitySphereMapCalculator(armJoints,
                                                                                       controllerOutput,
                                                                                       Voxel3DGrid.newVoxel3DGrid(40, 0.05, 50, 1));

      calculator.setControlFramePose(controlFrameTransform);
      calculator.setupCalculatorToRecordInFile(robotName, getClass());
      calculator.setAngularSelection(false, true, true);

      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), armJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(calculator.getGridSizeInMeters() / 2.5, 0.2, 0.0);
      calculator.setGridFramePose(gridFramePose);

      return calculator;
   }

   public static void main(String[] args) throws IOException
   {
      new ValkyrieReachabilitySphereMapSimulation();
   }
}
