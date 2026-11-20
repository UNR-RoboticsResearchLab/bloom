"""Researcher control page for the existing ROS storytelling controller.

Run alongside the quiet or real speech launch. Python standard library HTTP;
ROS service calls execute in the ROS executor, never the HTTP threads.
"""
import argparse
from collections import OrderedDict
from concurrent.futures import Future
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlsplit
import uuid


class CommandError(Exception):
    pass


def assignment_from_form(payload, default_name):
    result = {key: payload.get(key) for key in ("participant", "condition", "robot_name", "prompt_id")}
    if result["condition"] == "uncustomized":
        result["robot_name"] = default_name
    for key, value in result.items():
        if not isinstance(value, str) or not value.strip():
            raise CommandError(f"Please enter {key.replace('_', ' ')}.")
        result[key] = value.strip()
    if result["condition"] not in {"customized", "uncustomized"}:
        raise CommandError("Select a valid robot condition.")
    return result


class Gateway:
    def __init__(self, bridge, default_name, mode):
        self.bridge, self.default_name, self.mode = bridge, default_name, mode
        self.lock = threading.Lock()
        self.completed = OrderedDict()

    def snapshot(self):
        return {**self.bridge.snapshot(), "default_name": self.default_name,
                "mode": self.mode, "command_pending": self.lock.locked()}

    def command(self, payload):
        command = payload.get("command")
        if command not in {"start", "pause", "resume", "finish", "stop", "reset"}:
            raise CommandError("Unknown control command.")
        request_id = payload.get("request_id")
        try:
            uuid.UUID(request_id)
        except (ValueError, TypeError, AttributeError):
            raise CommandError("Missing request ID. Reload the page.")
        fingerprint = json.dumps(payload, sort_keys=True)
        if not self.lock.acquire(blocking=False):
            raise CommandError("Another control request is pending. Wait for the status to update.")
        try:
            if request_id in self.completed:
                old, result = self.completed[request_id]
                if old != fingerprint:
                    raise CommandError("Request ID was reused with different settings.")
                return result
            try:
                state = self.bridge.snapshot()
                if not state["connected"]:
                    raise CommandError("Controller disconnected. Wait for it to reconnect.")
                assignment = assignment_from_form(payload, self.default_name) if command == "start" else None
                if assignment:
                    if not state["status"].get("assignment_editable", False):
                        raise CommandError("An activity is active. Finish or Stop it before starting another.")
                    if assignment["prompt_id"] not in {task["id"] for task in state["tasks"]}:
                        raise CommandError("The selected story is not in the controller's JSON.")
                result = self.bridge.submit(command, assignment)
            except CommandError as exc:
                result = {"success": False, "message": str(exc)}
            self.completed[request_id] = (fingerprint, result)
            while len(self.completed) > 128:
                self.completed.popitem(last=False)
            return result
        finally:
            self.lock.release()


def make_handler(gateway, html):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Never put participant settings or transcripts in HTTP logs.

        def reply(self, status, value, content_type="application/json; charset=utf-8"):
            body = value.encode("utf-8") if isinstance(value, str) else json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                self.reply(200, html, "text/html; charset=utf-8")
            elif path == "/api/state":
                self.reply(200, gateway.snapshot())
            else:
                self.reply(404, {"message": "Not found"})

        def do_POST(self):
            if self.path != "/api/command":
                self.reply(404, {"message":"Not found"})
                return
            # Same-origin JSON requests only; no CORS or form submissions.
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self.reply(403, {"success":False,"message":"Origin rejected"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8192 or self.headers.get_content_type() != "application/json":
                    raise CommandError("Expected a small JSON request.")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise CommandError("Expected a JSON object.")
                result = gateway.command(payload)
                self.reply(200 if result["success"] else 409, result)
            except (ValueError, CommandError) as exc:
                self.reply(400, {"success": False, "message": str(exc)})
            except Exception:
                self.reply(500, {"success":False,"message":"Control request failed. Check controller status before trying again."})
    return Handler


def create_ros_bridge(node, prefix, controller):
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class Bridge:
        def __init__(self):
            self.lock = threading.Lock()
            self.status = {}
            self.last_status = 0
            self.tasks = []
            self.catalogue_error = "Waiting for the story catalogue."
            self.getting_catalogue = False
            self.commands = queue.Queue()
            self.active = None
            self.parameters = node.create_client(SetParametersAtomically, controller + "/set_parameters_atomically")
            self.get_parameters = node.create_client(GetParameters, controller + "/get_parameters")
            self.controls = {name: node.create_client(Trigger, prefix + "/" + name)
                             for name in ("start","pause","resume","finish","stop","reset")}
            qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.subscription = node.create_subscription(String, prefix+"/status", self.on_status, qos)
            self.event_subscription = node.create_subscription(String, prefix+"/speech_event", self.on_event, 100)
            self.last_text = ""
            self.last_robot = ""
            self.timer = node.create_timer(0.05, self.pump)
            self.catalogue_timer = node.create_timer(3.0,self.refresh_catalogue)

        def on_status(self, msg):
            try:
                value=json.loads(msg.data)
                if not isinstance(value,dict) or "state" not in value:
                    return
                with self.lock:
                    if self.status.get("state") in {"IDLE","FINISHED","STOPPED"} and value["state"]=="SPEAKING":
                        self.last_text=""
                    self.status=value
                    self.last_status=time.monotonic()
            except (ValueError,TypeError):
                pass

        def on_event(self,msg):
            try:
                event=json.loads(msg.data)
                if event.get("kind")=="final":
                    with self.lock:
                        self.last_text=event.get("text","")
            except (ValueError,TypeError,AttributeError):
                pass

        def snapshot(self):
            with self.lock:
                return {"connected": bool(self.status) and time.monotonic()-self.last_status < 4.0,
                        "status":dict(self.status),"tasks":list(self.tasks),
                        "catalogue_error":self.catalogue_error,"last_text":self.last_text}

        def refresh_catalogue(self):
            if self.getting_catalogue or not self.get_parameters.service_is_ready():
                return
            request=GetParameters.Request()
            request.names=["prompts_file"]
            self.getting_catalogue=True
            future=self.get_parameters.call_async(request)
            def done(future):
                try:
                    path=future.result().values[0].string_value
                    data=json.loads(Path(path).read_text(encoding="utf-8"))
                    tasks=[{key:task.get(key,"") for key in ("id","genre","topic","spoken_prompt")}
                           for task in data["tasks"]]
                    with self.lock:
                        self.tasks=tasks
                        self.catalogue_error=None
                except Exception:
                    with self.lock:
                        self.tasks=[]
                        self.catalogue_error="Could not read the controller's story JSON. Check its prompts_file setting."
                finally:
                    self.getting_catalogue=False
            future.add_done_callback(done)

        def submit(self, command, assignment):
            future=Future()
            self.commands.put((command,assignment,future,time.monotonic()+15.0))
            try:
                return future.result(timeout=16.0)
            except TimeoutError:
                raise CommandError("No confirmation received. Outcome unknown: check the activity status before trying again.")

        def complete(self, operation, success, message):
            if not operation[2].done():
                operation[2].set_result({"success":success,"message":message})
            if self.active is operation:
                self.active=None

        def pump(self):
            if self.active is not None:
                if time.monotonic()>self.active[3]:
                    self.complete(self.active,False,"No confirmation received. Outcome unknown: check activity status before trying again.")
                return
            try:
                operation=self.commands.get_nowait()
            except queue.Empty:
                return
            self.active=operation
            if time.monotonic()>operation[3]:
                self.complete(operation,False,"Request expired before it could run.")
                return
            command,assignment,_,_=operation
            if assignment:
                if not self.parameters.service_is_ready():
                    self.complete(operation,False,"Controller assignment service is unavailable.")
                    return
                request=SetParametersAtomically.Request()
                request.parameters=[Parameter(key,value=value).to_parameter_msg() for key,value in assignment.items()]
                future=self.parameters.call_async(request)
                def assigned(future):
                    if self.active is not operation:
                        return
                    try:
                        result=future.result().result
                        if not result.successful:
                            self.complete(operation,False,result.reason)
                        else:
                            self.trigger(operation)
                    except Exception:
                        self.complete(operation,False,"Assignment was not confirmed. Check controller status.")
                future.add_done_callback(assigned)
            else:
                self.trigger(operation)

        def trigger(self,operation):
            if self.active is not operation or time.monotonic()>operation[3]:
                self.complete(operation,False,"Request timed out. Assignment may have changed; activity was not started by this request.")
                return
            client=self.controls[operation[0]]
            if not client.service_is_ready():
                self.complete(operation,False,"The requested controller service is unavailable.")
                return
            future=client.call_async(Trigger.Request())
            def done(future):
                try:
                    response=future.result()
                    self.complete(operation,response.success,response.message)
                except Exception:
                    self.complete(operation,False,"Control outcome unknown. Check controller status.")
            future.add_done_callback(done)

    return Bridge()


def main(argv=None):
    parser=argparse.ArgumentParser(description="M storytelling researcher webpage")
    parser.add_argument("--mode",choices=("quiet","voice"),default="quiet")
    parser.add_argument("--controller",default="/storytelling_controller")
    parser.add_argument("--default-name",default="M")
    parser.add_argument("--host",default="0.0.0.0")
    parser.add_argument("--port",type=int,default=8765)
    args=parser.parse_args(argv)
    import rclpy
    rclpy.init(args=[])
    node=rclpy.create_node("storytelling_web")
    prefix="/study_test" if args.mode=="quiet" else "/story_voice"
    bridge=create_ros_bridge(node,prefix,args.controller.rstrip("/"))
    gateway=Gateway(bridge,args.default_name,args.mode)
    html=Path(__file__).with_name("storytelling_control.html").read_text(encoding="utf-8")
    server=ThreadingHTTPServer((args.host,args.port),make_handler(gateway,html))
    server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print(f"Control page: http://localhost:{args.port} — mode: {args.mode}",flush=True)
    print("From your Mac: use the lab computer's network IP instead of localhost.",flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()