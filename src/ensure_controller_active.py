#!/usr/bin/env python3

import argparse
import sys
from typing import Optional

import rclpy
from controller_manager_msgs.srv import ConfigureController, ListControllers, LoadController, SwitchController
from rclpy.duration import Duration
from rclpy.node import Node


class ControllerActivator(Node):
    def __init__(self, controller_name: str, controller_manager: str, timeout_sec: float) -> None:
        super().__init__("ensure_controller_active")
        self.controller_name = controller_name
        self.timeout_sec = timeout_sec
        manager_prefix = controller_manager.rstrip("/")
        self.list_client = self.create_client(ListControllers, f"{manager_prefix}/list_controllers")
        self.load_client = self.create_client(LoadController, f"{manager_prefix}/load_controller")
        self.configure_client = self.create_client(
            ConfigureController, f"{manager_prefix}/configure_controller"
        )
        self.switch_client = self.create_client(SwitchController, f"{manager_prefix}/switch_controller")

    def wait_for_services(self) -> bool:
        deadline = self.get_clock().now() + Duration(seconds=self.timeout_sec)
        service_clients = (
            ("list_controllers", self.list_client),
            ("load_controller", self.load_client),
            ("configure_controller", self.configure_client),
            ("switch_controller", self.switch_client),
        )
        for service_name, client in service_clients:
            while rclpy.ok() and self.get_clock().now() < deadline:
                if client.wait_for_service(timeout_sec=1.0):
                    break
                self.get_logger().info(f"waiting for {service_name} service...")
            else:
                self.get_logger().error(f"timeout waiting for {service_name} service")
                return False
        return True

    def get_state(self) -> Optional[str]:
        for attempt in range(3):
            request = ListControllers.Request()
            future = self.list_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
            if future.done() and future.result() is not None:
                for controller in future.result().controller:
                    if controller.name == self.controller_name:
                        return controller.state
                return ""

            self.get_logger().warn(f"list_controllers attempt {attempt + 1} failed, retrying...")
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().error("failed to list controllers")
        return None

    def load(self) -> bool:
        request = LoadController.Request()
        request.name = self.controller_name
        future = self.load_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        response = future.result()
        if response is None:
            self.get_logger().warn(f"load_controller call failed for {self.controller_name}")
            return False
        return response.ok

    def configure(self) -> bool:
        request = ConfigureController.Request()
        request.name = self.controller_name
        future = self.configure_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        response = future.result()
        if response is None:
            self.get_logger().warn(f"configure_controller call failed for {self.controller_name}")
            return False
        return response.ok

    def activate(self) -> bool:
        request = SwitchController.Request()
        request.activate_controllers = [self.controller_name]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout.sec = int(self.timeout_sec)
        request.timeout.nanosec = int((self.timeout_sec - int(self.timeout_sec)) * 1e9)
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        response = future.result()
        if response is None:
            self.get_logger().warn(f"switch_controller call failed for {self.controller_name}")
            return False
        if not response.ok:
            self.get_logger().error(
                f"controller manager rejected activation of {self.controller_name}: {response.message}"
            )
        return response.ok

    def ensure_active(self) -> bool:
        if not self.wait_for_services():
            return False

        state = self.get_state()
        if state is None:
            return False
        if state == "active":
            self.get_logger().info(f"{self.controller_name} is already active")
            return True

        if state == "":
            self.get_logger().info(f"loading controller {self.controller_name}")
            if not self.load():
                self.get_logger().warn(
                    f"load_controller returned no success for {self.controller_name}, checking state directly"
                )
            state = self.get_state()

        if state is None:
            return False
        if state == "active":
            self.get_logger().info(f"{self.controller_name} became active during load")
            return True

        if state == "unconfigured":
            self.get_logger().info(f"configuring controller {self.controller_name}")
            if not self.configure():
                self.get_logger().warn(
                    f"configure_controller returned no success for {self.controller_name}, checking state directly"
                )
            state = self.get_state()

        if state is None:
            return False
        if state != "active":
            self.get_logger().info(f"activating controller {self.controller_name}")
            if not self.activate():
                self.get_logger().warn(
                    f"switch_controller returned no success for {self.controller_name}, checking state directly"
                )
                refreshed_state = self.get_state()
                if refreshed_state != "active":
                    return False

        final_state = self.get_state()
        if final_state != "active":
            self.get_logger().error(
                f"controller {self.controller_name} is still not active (state={final_state})"
            )
            return False

        self.get_logger().info(f"{self.controller_name} is active")
        return True


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller-name", required=True)
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=None)
    node = ControllerActivator(args.controller_name, args.controller_manager, args.timeout_sec)
    try:
        return 0 if node.ensure_active() else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
