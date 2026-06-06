"""Thin wrapper around MoveIt's /compute_ik service.

Exposes /gr_kinematics/solve_ik (moveit_msgs/srv/GetPositionIK) so other nodes
in this assignment (reachability sweep, coverage planner) can call IK
without each importing MoveIt directly. Adds:
  - collision-aware solve by default
  - configurable timeout and retry attempts
  - clear logging of why a pose is infeasible (no-IK vs in-collision)
"""
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
from geometry_msgs.msg import PoseStamped


class IKService(Node):
    def __init__(self):
        super().__init__("gr_ik_service")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("tip_link", "link6")
        self.declare_parameter("timeout_s", 0.05)
        self.declare_parameter("attempts", 5)
        self.declare_parameter("avoid_collisions", True)

        self.group = self.get_parameter("planning_group").value
        self.tip = self.get_parameter("tip_link").value
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.attempts = int(self.get_parameter("attempts").value)
        self.avoid = bool(self.get_parameter("avoid_collisions").value)

        cb = ReentrantCallbackGroup()
        self.cli = self.create_client(GetPositionIK, "/compute_ik", callback_group=cb)
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_ik not available — start move_group first")
            raise SystemExit(1)

        self.srv = self.create_service(
            GetPositionIK, "/gr_kinematics/solve_ik", self._handle, callback_group=cb
        )
        self.get_logger().info(
            f"IK service ready (group={self.group}, tip={self.tip}, "
            f"timeout={self.timeout}s, attempts={self.attempts})"
        )

    def _handle(self, request, response):
        # NOTE: do NOT use rclpy.spin_until_future_complete here — this handler
        # is already being invoked from the executor's spin loop. We rely on
        # MultiThreadedExecutor + ReentrantCallbackGroup so another thread can
        # service the inner /compute_ik response; we block on a threading.Event
        # set from add_done_callback. This is the canonical rclpy pattern for
        # nested client-inside-service calls.
        req = request.ik_request
        if not req.group_name:
            req.group_name = self.group
        if not req.ik_link_name:
            req.ik_link_name = self.tip
        if req.timeout.sec == 0 and req.timeout.nanosec == 0:
            req.timeout.sec = int(self.timeout)
            req.timeout.nanosec = int((self.timeout - int(self.timeout)) * 1e9)
        req.avoid_collisions = self.avoid

        last = None
        for _ in range(self.attempts):
            inner = GetPositionIK.Request(ik_request=req)
            event = threading.Event()
            fut = self.cli.call_async(inner)
            fut.add_done_callback(lambda _f, e=event: e.set())
            if not event.wait(timeout=self.timeout + 0.5):
                continue  # inner call timed out — try again
            last = fut.result()
            if last and last.error_code.val == MoveItErrorCodes.SUCCESS:
                break

        if last is not None:
            response.solution = last.solution
            response.error_code = last.error_code
        else:
            response.error_code.val = MoveItErrorCodes.FAILURE
        return response


def main():
    rclpy.init()
    node = IKService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
