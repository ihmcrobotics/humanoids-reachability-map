package us.ihmc.reachabilityMap;

import us.ihmc.avatar.reachabilityMap.ReachabilityMapFileLoader;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.nadia.parameters.NadiaModelFactory;
import us.ihmc.nadia.parameters.robotVersions.NadiaVersion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class NadiaReachabilityMapVisualizer
{
   public static void main(String[] args)
   {
      NadiaVersion nadiaVersion = NadiaVersion.V16_RIGHT_ARM;
      NadiaModelFactory modelFactory = new NadiaModelFactory(nadiaVersion, nadiaVersion.getJointMap(), null);
      FloatingRootJointRobot sdfRobot = modelFactory.createSdfRobot();
      FullRobotModel fullRobotModel = modelFactory.createFullRobotModel();

      long startTime = System.nanoTime();
      System.out.println("Loading reachability map");
      ReachabilityMapFileLoader reachabilityMapFileLoader = new ReachabilityMapFileLoader("Nadia", fullRobotModel.getElevator(), null);
      
      System.out.println("Done loading reachability map. Took: " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) + " seconds.");
      
      Voxel3DGrid grid = reachabilityMapFileLoader.getLoadedGrid();
      SphereVoxelShape sphereVoxelShape = grid.getSphereVoxelShape();
      
      SDFPerfectSimulatedSensorReader sensorReader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, null);
      sensorReader.read();
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot, parameters);
      scs.setGroundVisible(false);
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
