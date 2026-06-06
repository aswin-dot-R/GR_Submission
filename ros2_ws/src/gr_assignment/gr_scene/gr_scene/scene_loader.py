"""Pushes countertop / faucet / mirror collision objects into the MoveIt
PlanningScene via the /apply_planning_scene service. Reads sizes/poses from
the ROS parameters declared in scene.yaml.
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


def make_box(name: str, size, xyz, frame_id="base_link") -> CollisionObject:
    co = CollisionObject()
    co.id = name
    co.header.frame_id = frame_id
    co.operation = CollisionObject.ADD
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(s) for s in size]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz]
    pose.orientation.w = 1.0
    co.primitives.append(prim)
    co.primitive_poses.append(pose)
    return co


class SceneLoader(Node):
    def __init__(self):
        super().__init__("scene_loader")
        self.declare_parameter("frame_id", "base_link")
        for n in ("countertop", "faucet", "mirror"):
            self.declare_parameter(f"scene.{n}.size", [0.0, 0.0, 0.0])
            self.declare_parameter(f"scene.{n}.pose", [0.0, 0.0, 0.0])

        self.cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/apply_planning_scene not available — is move_group running?")
            raise SystemExit(1)

        frame = self.get_parameter("frame_id").value
        scene = PlanningScene()
        scene.is_diff = True
        for name in ("countertop", "faucet", "mirror"):
            size = self.get_parameter(f"scene.{name}.size").value
            xyz = self.get_parameter(f"scene.{name}.pose").value
            scene.world.collision_objects.append(make_box(name, size, xyz, frame))

        req = ApplyPlanningScene.Request(scene=scene)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() and future.result().success:
            self.get_logger().info(f"Loaded {len(scene.world.collision_objects)} collision objects")
        else:
            self.get_logger().error("ApplyPlanningScene failed")


def main():
    rclpy.init()
    SceneLoader()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
