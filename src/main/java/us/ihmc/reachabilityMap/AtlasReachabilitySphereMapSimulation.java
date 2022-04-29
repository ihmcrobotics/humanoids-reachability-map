package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapFileWriter;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapTools;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapCalculator;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid;
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
      Voxel3DGrid voxel3DGrid = Voxel3DGrid.newVoxel3DGrid(25, 0.025, 50, 1);

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
      ReachabilitySphereMapCalculator calculator = setupCalculator(robotName, base, endEffector, controlFrame, robot.getControllerOutput(), voxel3DGrid);

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
      simControls.simulateNow(Integer.MAX_VALUE);

      ReachabilityMapFileWriter.exportVoxel3DGridToFile(robotName, getClass(), calculator.getRobotArmJoints(), voxel3DGrid);
   }

   private ReachabilitySphereMapCalculator setupCalculator(String robotName,
                                                           RigidBodyBasics base,
                                                           RigidBodyBasics endEffector,
                                                           ReferenceFrame controlFrame,
                                                           ControllerOutput controllerOutput,
                                                           Voxel3DGrid voxel3DGrid)
         throws IOException
   {
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);
      ReachabilitySphereMapCalculator calculator = new ReachabilitySphereMapCalculator(armJoints, controllerOutput, voxel3DGrid);

      calculator.setControlFramePose(controlFrame.getTransformToDesiredFrame(endEffector.getBodyFixedFrame()));
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
