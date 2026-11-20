"""Apply all four session fields through one standard ROS atomic parameter call.

This configures only: it never starts speech or enables the microphone.
"""
import argparse
import json


def main(argv=None):
    parser = argparse.ArgumentParser(description="Assign the next storytelling session without restarting")
    parser.add_argument("--participant", required=True)
    parser.add_argument("--condition", required=True, choices=("customized", "uncustomized"))
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--controller", default="/storytelling_controller")
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically

    assignment = {"participant": args.participant, "condition": args.condition,
                  "robot_name": args.robot_name, "prompt_id": args.prompt}
    rclpy.init(args=[])
    node = rclpy.create_node("storytelling_assignment_client")
    try:
        service = args.controller.rstrip("/") + "/set_parameters_atomically"
        client = node.create_client(SetParametersAtomically, service)
        if not client.wait_for_service(timeout_sec=5.0):
            print("Controller not found. Start its launch and check that both terminals use the same ROS_DOMAIN_ID.")
            return 1
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()
                              for name, value in assignment.items()]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
        if not future.done():
            print("No response received. Assignment status is unknown; inspect the controller parameters before retrying.")
            return 1
        response = future.result()
        if response is None:
            print("No usable response. Inspect the controller parameters before retrying.")
            return 1
        if not response.result.successful:
            print("ASSIGNMENT REJECTED: " + response.result.reason)
            print("The previous assignment was retained.")
            return 2
        print("ASSIGNMENT ACCEPTED — activity has not started.")
        print(json.dumps(assignment, indent=2, ensure_ascii=False))
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())