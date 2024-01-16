package us.ihmc.reachabilityMap;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilityMapRobotInformation;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper;
import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.collision.CollidableHelper;

import java.io.IOException;

public class AtlasReachabilitySphereMapSimulation
{
   public AtlasReachabilitySphereMapSimulation() throws IOException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);

      String chestName = robotModel.getJointMap().getChestName();
      String leftHandName = robotModel.getJointMap().getHandName(RobotSide.LEFT);

      RobotDefinition robotDefinition = robotModel.createRobotDefinition();
      robotDefinition.setName(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString());
      addCollisions(robotDefinition);
      ReachabilityMapRobotInformation robotInformation = new ReachabilityMapRobotInformation(robotDefinition, chestName, leftHandName);
      robotInformation.setControlFramePoseInParentJoint(robotModel.getJointMap().getHandControlFrameToWristTransform(RobotSide.LEFT));
      robotInformation.setOrthogonalToPalm(Axis3D.X);

      ReachabilitySphereMapSimulationHelper simHelper = new ReachabilitySphereMapSimulationHelper(robotInformation);
      simHelper.setGridParameters(35, 0.05, 40, 5);
      simHelper.setEvaluateRReachability(true);
      simHelper.setEvaluateR2Reachability(true);
      simHelper.enableJointTorqueAnalysis(false);
      simHelper.setGridPosition(0.4, 0.4, 0.5);

      if (simHelper.start())
      {
         simHelper.exportDataToMatlabFile(getClass());
      }
   }

   public static void addCollisions(RobotDefinition robotDefinition)
   {
      CollidableHelper helper = new CollidableHelper();
      long body = helper.getCollisionMask("body");
      long arm = helper.getCollisionMask("arm");

      {
         RigidBodyDefinition torso = robotDefinition.getRigidBodyDefinition("pelvis");
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Box3DDefinition(0.3, 0.3, 0.2));
         collision.getOriginPose().getTranslation().set(0, 0, 0.02);
         collision.setCollisionGroup(arm);
         collision.setCollisionMask(body);
         torso.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition torso = robotDefinition.getRigidBodyDefinition("utorso");
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Box3DDefinition(0.48, 0.34, 0.68));
         collision.getOriginPose().getTranslation().set(-0.06, 0.0, 0.34);
         collision.setCollisionGroup(arm);
         collision.setCollisionMask(body);
         torso.getCollisionShapeDefinitions().add(collision);

         collision = new CollisionShapeDefinition(new Box3DDefinition(0.1, 0.45, 0.15));
         collision.getOriginPose().getTranslation().set(0.23, 0.0, 0.45);
         collision.setCollisionGroup(arm);
         collision.setCollisionMask(body);
         torso.getCollisionShapeDefinitions().add(collision);

         collision = new CollisionShapeDefinition(new Box3DDefinition(0.09, 0.28, 0.15));
         collision.getOriginPose().getTranslation().set(0.225, 0.0, 0.3);
         collision.setCollisionGroup(arm);
         collision.setCollisionMask(body);
         torso.getCollisionShapeDefinitions().add(collision);

         collision = new CollisionShapeDefinition(new Box3DDefinition(0.07, 0.2, 0.1));
         collision.getOriginPose().getTranslation().set(0.215, 0.0, 0.175);
         collision.setCollisionGroup(arm);
         collision.setCollisionMask(body);
         torso.getCollisionShapeDefinitions().add(collision);

         collision = new CollisionShapeDefinition(new Box3DDefinition(0.2, 0.23, 0.15));
         collision.getOriginPose().getTranslation().set(0.15, 0.0, 0.75);
         collision.setCollisionGroup(arm);
         collision.setCollisionMask(body);
         torso.getCollisionShapeDefinitions().add(collision);

         collision = new CollisionShapeDefinition(new Capsule3DDefinition(0.3, 0.06));
         collision.getOriginPose().getRotation().setToRollOrientation(0.5 * Math.PI);
         collision.getOriginPose().getTranslation().set(0.25, 0.0, 0.62);
         collision.setCollisionGroup(arm);
         collision.setCollisionMask(body);
         torso.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition upperArm = robotDefinition.getJointDefinition("l_arm_shx").getSuccessor();
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Capsule3DDefinition(0.15, 0.07));
         collision.getOriginPose().getRotation().setToRollOrientation(0.5 * Math.PI);
         collision.getOriginPose().getTranslation().set(-0.01, 0.16, -0.02);
         collision.setCollisionGroup(body);
         collision.setCollisionMask(arm);
         upperArm.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition forearm = robotDefinition.getJointDefinition("l_arm_elx").getSuccessor();
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Capsule3DDefinition(0.22, 0.06));
         collision.getOriginPose().getRotation().setToRollOrientation(0.5 * Math.PI);
         collision.getOriginPose().getTranslation().set(0.0, 0.18, -0.02);
         collision.setCollisionGroup(body);
         collision.setCollisionMask(arm);
         forearm.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition forearm = robotDefinition.getJointDefinition("l_arm_wry2").getSuccessor();
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Box3DDefinition(0.12, 0.1, 0.12));
         collision.getOriginPose().getTranslation().set(0.0, 0.17, 0.0);
         collision.setCollisionGroup(body);
         collision.setCollisionMask(arm);
         forearm.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition forearm = robotDefinition.getJointDefinition("l_arm_wry2").getSuccessor();
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Box3DDefinition(0.12, 0.1, 0.12));
         collision.getOriginPose().getTranslation().set(0.0, 0.17, 0.0);
         collision.setCollisionGroup(body);
         collision.setCollisionMask(arm);
         forearm.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition forearm = robotDefinition.getJointDefinition("l_arm_wry2").getSuccessor();
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Box3DDefinition(0.035, 0.15, 0.035));
         collision.getOriginPose().getRotation().setQuaternion(0.2, 0, 0, 1);
         collision.getOriginPose().getTranslation().set(0.0, 0.25, 0.075);
         collision.setCollisionGroup(body);
         collision.setCollisionMask(arm);
         forearm.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition forearm = robotDefinition.getJointDefinition("l_arm_wry2").getSuccessor();
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Box3DDefinition(0.035, 0.15, 0.035));
         collision.getOriginPose().getRotation().setQuaternion(-0.2, 0, 0, 1);
         collision.getOriginPose().getTranslation().set(0.035, 0.25, -0.075);
         collision.setCollisionGroup(body);
         collision.setCollisionMask(arm);
         forearm.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition forearm = robotDefinition.getJointDefinition("l_arm_wry2").getSuccessor();
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Box3DDefinition(0.035, 0.15, 0.035));
         collision.getOriginPose().getRotation().setQuaternion(-0.2, 0, 0, 1);
         collision.getOriginPose().getTranslation().set(-0.035, 0.25, -0.075);
         collision.setCollisionGroup(body);
         collision.setCollisionMask(arm);
         forearm.getCollisionShapeDefinitions().add(collision);
      }

      {
         RigidBodyDefinition forearm = robotDefinition.getJointDefinition("l_arm_wry2").getSuccessor();
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Sphere3DDefinition(0.05));
         collision.getOriginPose().getTranslation().set(0.0, 0.07, 0.0);
         collision.setCollisionGroup(body);
         collision.setCollisionMask(arm);
         forearm.getCollisionShapeDefinitions().add(collision);
      }
   }

   public static void main(String[] args) throws IOException
   {
      new AtlasReachabilitySphereMapSimulation();
   }
}
