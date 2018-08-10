package us.ihmc.reachabilityMap;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapFileLoader;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReachabilityMapVisualizer
{
   public static void main(String[] args)
   {
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
      String robotName = FormattingTools.underscoredToCamelCase(valkyrieRobotModel.getSimpleRobotName(), true);
      FullHumanoidRobotModel fullRobotModel = valkyrieRobotModel.createFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      long startTime = System.nanoTime();
      System.out.println("Loading reachability map");
//      File fileToLoad = new File("C://Users//Sylvain//Desktop//Workspace1//Atlas//resources//us//ihmc//atlas//reachabilityMap//atlasReachabilitySphereMapSimulation//20150319_220135_AtlasUnpluggedV5DualRobotiq.xls");
      ReachabilityMapFileLoader reachabilityMapFileLoader = new ReachabilityMapFileLoader(robotName, fullRobotModel.getElevator(), referenceFrames);
      
      System.out.println("Done loading reachability map. Took: " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) + " seconds.");
      
      Voxel3DGrid grid = reachabilityMapFileLoader.getLoadedGrid();
      SphereVoxelShape sphereVoxelShape = grid.getSphereVoxelShape();
      
      HumanoidFloatingRootJointRobot robot = valkyrieRobotModel.createHumanoidFloatingRootJointRobot(false);
      valkyrieRobotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, valkyrieRobotModel.getJointMap());
      SDFPerfectSimulatedSensorReader sensorReader = new SDFPerfectSimulatedSensorReader(robot, fullRobotModel, referenceFrames);
      sensorReader.read();
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.startOnAThread();

      int numberOfVoxel = grid.getNumberOfVoxelsPerDimension();
      grid.getReferenceFrame().update();
      System.out.println(grid.getReferenceFrame().getTransformToWorldFrame()); 
      FramePoint3D voxelLocation = new FramePoint3D();

      for (int xIndex = 0; xIndex < numberOfVoxel; xIndex++)
      {
         for (int yIndex = 0; yIndex < numberOfVoxel; yIndex++)
         {
            for (int zIndex = 0; zIndex < numberOfVoxel; zIndex++)
            {
               double reachabilityValue = grid.getD(xIndex, yIndex, zIndex);

               if (reachabilityValue > 0.001)
               {
                  grid.getVoxel(voxelLocation, xIndex, yIndex, zIndex);
                  System.out.println("xIndex: " + xIndex + ", yIndex: " + yIndex + ", zIndex: " + zIndex + ", voxel location: " + voxelLocation);
                  Graphics3DObject voxelViz = sphereVoxelShape.createVisualization(voxelLocation, 0.25, reachabilityValue);
                  scs.addStaticLinkGraphics(voxelViz);
               }
            }
         }
      }
      
      ThreadTools.sleepForever();
   }
}
