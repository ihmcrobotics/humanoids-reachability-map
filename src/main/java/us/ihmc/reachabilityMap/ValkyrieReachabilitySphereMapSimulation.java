package us.ihmc.reachabilityMap;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieReachabilitySphereMapSimulation
{
   public ValkyrieReachabilitySphereMapSimulation() throws IOException
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

      ValkyrieJointMap jointMap = robotModel.getJointMap();
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      //      createCollisions(robotDefinition, jointMap);
      robotDefinition.getOneDoFJointDefinition("leftShoulderRoll").setEffortLimits(5.0);

      String chestName = jointMap.getChestName();
      String leftHandName = jointMap.getHandName(RobotSide.LEFT);
      ReachabilitySphereMapSimulationHelper simHelper = new ReachabilitySphereMapSimulationHelper(robotDefinition, chestName, leftHandName);
      simHelper.setGridParameters(30, 0.05, 50, 1);
      simHelper.setPalmOrthogonalAxis(Axis3D.X);
      simHelper.setEvaluateDReachability(false);
      simHelper.setEvaluateD0Reachability(false);
      simHelper.enableJointTorqueAnalysis(true);
      simHelper.setGridPosition(0.5, 0.2, 0.32);
      RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
      controlFrameToWristTransform.getTranslation().set(0.025, 0.07, 0.0);
      simHelper.setControlFramePoseInParentJoint(controlFrameToWristTransform);

      if (simHelper.start())
      {
         String robotName = "Valkyrie";
         simHelper.exportDataToFile(robotName, getClass());
      }
   }

   private static void createCollisions(RobotDefinition robotDefinition, ValkyrieJointMap jointMap)
   {
      double modelScale = jointMap.getModelScale();
      CollidableHelper helper = new CollidableHelper();
      long body = helper.getCollisionMask("body");
      long arm = helper.getCollisionMask("arm");

      { // Head
         RigidBodyDefinition head = robotDefinition.getRigidBodyDefinition(jointMap.getHeadName());
         CollisionShapeDefinition collision = new CollisionShapeDefinition(new Sphere3DDefinition(0.15 * modelScale));
         collision.setName("headCollision");
         collision.getOriginPose().getTranslation().set(0.077, 0.0, 0.001);
         collision.getOriginPose().getTranslation().scale(modelScale);
         collision.setCollisionMask(body);
         collision.setCollisionGroup(arm);
         head.getCollisionShapeDefinitions().add(collision);
      }

      RobotSide robotSide = RobotSide.LEFT;
      //      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         { // Upper-arm
            JointDefinition shoulderYaw = robotDefinition.getJointDefinition(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW));
            CollisionShapeDefinition collision = new CollisionShapeDefinition(new Capsule3DDefinition(0.10 * modelScale, 0.075 * modelScale));
            collision.setName(robotSide.getCamelCaseName() + "ShoulderYawCollision");
            collision.getOriginPose().getTranslation().set(0.0, robotSide.negateIfRightSide(0.21), 0.0);
            collision.getOriginPose().getTranslation().scale(modelScale);
            collision.getOriginPose().appendRollRotation(Math.PI / 2.0);
            collision.setCollisionMask(arm);
            collision.setCollisionGroup(body);
            shoulderYaw.getSuccessor().getCollisionShapeDefinitions().add(collision);
         }

         { // Elbow
            JointDefinition elbow = robotDefinition.getJointDefinition(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH));
            CollisionShapeDefinition collision = new CollisionShapeDefinition(new Capsule3DDefinition(0.08 * modelScale, 0.05 * modelScale));
            collision.setName(robotSide.getCamelCaseName() + "ElbowCollision");
            collision.getOriginPose().getTranslation().set(0.0, 0.0, 0.0);
            collision.getOriginPose().getTranslation().scale(modelScale);
            collision.setCollisionMask(arm);
            collision.setCollisionGroup(body);
            elbow.getSuccessor().getCollisionShapeDefinitions().add(collision);
         }

         { // Forearm
            JointDefinition elbow = robotDefinition.getJointDefinition(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH));
            CollisionShapeDefinition collision = new CollisionShapeDefinition(new Capsule3DDefinition(0.15 * modelScale, 0.075 * modelScale));
            collision.setName(robotSide.getCamelCaseName() + "ForearmCollision");
            collision.getOriginPose().getTranslation().set(-0.03, robotSide.negateIfRightSide(0.14), 0.0);
            collision.getOriginPose().getTranslation().scale(modelScale);
            collision.getOriginPose().appendRollRotation(Math.PI / 2.0);
            collision.setCollisionMask(arm);
            collision.setCollisionGroup(body);
            elbow.getSuccessor().getCollisionShapeDefinitions().add(collision);
         }

         { // Hand
            RigidBodyDefinition hand = robotDefinition.getRigidBodyDefinition(jointMap.getHandName(robotSide));
            CollisionShapeDefinition collision = new CollisionShapeDefinition(new Capsule3DDefinition(0.02 * modelScale, 0.055 * modelScale));
            collision.setName(robotSide.getCamelCaseName() + "HandCollision");
            collision.getOriginPose().getTranslation().set(-0.007, robotSide.negateIfRightSide(0.062), -0.01);
            collision.getOriginPose().getTranslation().scale(modelScale);
            collision.getOriginPose().appendRollRotation(Math.PI / 2.0);
            collision.setCollisionMask(arm);
            collision.setCollisionGroup(body);
            hand.getCollisionShapeDefinitions().add(collision);
         }
      }

      { // Torso
         RigidBodyDefinition torso = robotDefinition.getRigidBodyDefinition(jointMap.getChestName());
         CollisionShapeDefinition collisionFront = new CollisionShapeDefinition(new Capsule3DDefinition(0.13 * modelScale, 0.12 * modelScale));
         collisionFront.setName("chestFrontCollision");
         collisionFront.getOriginPose().getTranslation().set(0.034, 0.0, 0.253);
         collisionFront.getOriginPose().getTranslation().scale(modelScale);
         collisionFront.getOriginPose().appendRollRotation(Math.PI / 2.0);
         collisionFront.setCollisionMask(body);
         collisionFront.setCollisionGroup(arm);
         torso.getCollisionShapeDefinitions().add(collisionFront);

         Box3DDefinition collisionBackShape = new Box3DDefinition(0.35, 0.35, 0.40);
         //         collisionBackShape.setMargins(1.0e-5, 4.0e-4);
         CollisionShapeDefinition collisionBack = new CollisionShapeDefinition(collisionBackShape);
         collisionBack.setName("chestBackCollision");
         collisionBack.getOriginPose().getTranslation().set(-0.111, 0.0, 0.208);
         collisionBack.getOriginPose().getTranslation().scale(modelScale);
         collisionBack.setCollisionMask(body);
         collisionBack.setCollisionGroup(arm);
         torso.getCollisionShapeDefinitions().add(collisionBack);

      }

      { // Pelvis
         RigidBodyDefinition pelvis = robotDefinition.getRigidBodyDefinition(jointMap.getPelvisName());
         CollisionShapeDefinition collisionTop = new CollisionShapeDefinition(new Capsule3DDefinition(0.2 * modelScale, 0.1 * modelScale));
         collisionTop.setName("pelvisTopCollision");
         collisionTop.getOriginPose().getTranslation().set(-0.005, 0.0, -0.076);
         collisionTop.getOriginPose().getTranslation().scale(modelScale);
         collisionTop.getOriginPose().appendRollRotation(Math.PI / 2.0);
         collisionTop.setCollisionMask(body);
         collisionTop.setCollisionGroup(arm);
         pelvis.getCollisionShapeDefinitions().add(collisionTop);

         CollisionShapeDefinition collisionFront = new CollisionShapeDefinition(new Capsule3DDefinition(0.19 * modelScale, 0.1 * modelScale));
         collisionFront.setName("pelvisFrontCollision");
         collisionFront.getOriginPose().getTranslation().set(0.025, 0.0, -0.154);
         collisionFront.getOriginPose().getTranslation().scale(modelScale);
         collisionFront.setCollisionMask(body);
         collisionFront.setCollisionGroup(arm);
         pelvis.getCollisionShapeDefinitions().add(collisionFront);

         CollisionShapeDefinition collisionBack = new CollisionShapeDefinition(new Capsule3DDefinition(0.19 * modelScale, 0.1 * modelScale));
         collisionBack.setName("pelvisBackCollision");
         collisionBack.getOriginPose().getTranslation().set(-0.015, 0.0, -0.154);
         collisionBack.getOriginPose().getTranslation().scale(modelScale);
         collisionBack.setCollisionMask(body);
         collisionBack.setCollisionGroup(arm);
         pelvis.getCollisionShapeDefinitions().add(collisionBack);
      }

      //      for (RobotSide robotSide : RobotSide.values)
      { // Legs
         { // Hip
            JointDefinition hipYaw = robotDefinition.getJointDefinition(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW));
            CollisionShapeDefinition collision = new CollisionShapeDefinition(new Sphere3DDefinition(0.13 * modelScale));
            collision.setName(robotSide.getCamelCaseName() + "WaistCollision");
            collision.getOriginPose().getTranslation().set(0.0, 0.0, -0.03);
            collision.getOriginPose().getTranslation().scale(modelScale);
            collision.setCollisionMask(body);
            collision.setCollisionGroup(arm);
            hipYaw.getSuccessor().getCollisionShapeDefinitions().add(collision);
         }

         { // Thigh
            RigidBodyDefinition thigh = robotDefinition.getJointDefinition(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).getPredecessor();
            CollisionShapeDefinition collisionUpper = new CollisionShapeDefinition(new Capsule3DDefinition(0.2 * modelScale, 0.1 * modelScale));
            collisionUpper.setName(robotSide.getCamelCaseName() + "ThighUpperCollision");
            collisionUpper.getOriginPose().getTranslation().set(0.0195, robotSide.negateIfRightSide(0.086), -0.093);
            collisionUpper.getOriginPose().getTranslation().scale(modelScale);
            EuclidGeometryTools.orientation3DFromZUpToVector3D(new Vector3D(-0.15, robotSide.negateIfRightSide(-0.05), 1.0),
                                                               collisionUpper.getOriginPose().getRotation());
            collisionUpper.setCollisionMask(body);
            collisionUpper.setCollisionGroup(arm);
            thigh.getCollisionShapeDefinitions().add(collisionUpper);

            CollisionShapeDefinition collisionFront = new CollisionShapeDefinition(new Capsule3DDefinition(0.15 * modelScale, 0.095 * modelScale));
            collisionFront.setName(robotSide.getCamelCaseName() + "ThighFrontCollision");
            collisionFront.getOriginPose().getTranslation().set(0.0424, robotSide.negateIfRightSide(0.081), -0.258);
            collisionFront.getOriginPose().getTranslation().scale(modelScale);
            EuclidGeometryTools.orientation3DFromZUpToVector3D(new Vector3D(0.1, 0.0, 1.0), collisionFront.getOriginPose().getRotation());
            collisionFront.setCollisionMask(body);
            collisionFront.setCollisionGroup(arm);
            thigh.getCollisionShapeDefinitions().add(collisionFront);

            CollisionShapeDefinition collisionLower = new CollisionShapeDefinition(new Capsule3DDefinition(0.25 * modelScale, 0.09 * modelScale));
            collisionLower.setName(robotSide.getCamelCaseName() + "ThighLowerCollision");
            collisionLower.getOriginPose().getTranslation().set(0.017, robotSide.negateIfRightSide(0.091), -0.288);
            collisionLower.getOriginPose().getTranslation().scale(modelScale);
            EuclidGeometryTools.orientation3DFromZUpToVector3D(new Vector3D(0.1, robotSide.negateIfRightSide(0.05), 1.0),
                                                               collisionLower.getOriginPose().getRotation());
            collisionLower.setCollisionMask(body);
            collisionLower.setCollisionGroup(arm);
            thigh.getCollisionShapeDefinitions().add(collisionLower);
         }
      }
   }

   public static void main(String[] args) throws IOException
   {
      new ValkyrieReachabilitySphereMapSimulation();
   }
}
