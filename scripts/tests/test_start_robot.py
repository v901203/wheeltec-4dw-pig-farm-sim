"""Exercise the launcher with fake executables, never Docker, DDS or UART."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "start_robot.sh"
FAKE = r'''#!/usr/bin/python3
import json, os, pathlib, signal, subprocess, sys, time
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ["CALLS"], "a") as f:
    f.write(json.dumps([name, *args]) + "\n")
if name == "docker":
    if args[0] == "inspect":
        if os.environ.get("NO_CONTAINER"): sys.exit(1)
        if "-f" in args:
            if "NetworkMode" in args[2]: print(os.environ.get("NETWORK_MODE", "host"))
            else: print("false" if os.environ.get("STOPPED") else "true")
    elif args[0] == "exec":
        pathlib.Path(os.environ["CAPTURE"]).write_text(sys.stdin.read())
elif name == "ip":
    state = pathlib.Path(os.environ["IP_STATE"])
    if args[:2] == ["link", "show"] or args[:3] == ["-o", "link", "show"]:
        if os.environ.get("NO_INTERFACE"): sys.exit(1)
        print("2: eth0: <BROADCAST,MULTICAST,UP" +
              (">" if os.environ.get("NO_LINK") else ",LOWER_UP>") + " mtu 1500")
    elif args[:4] == ["-o", "-4", "addr", "show"]:
        for address in json.loads(state.read_text()) if state.exists() else []:
            print("2: eth0 inet " + address + " scope global eth0")
    elif args[:2] == ["addr", "add"]:
        if os.environ.get("DENY_IP"): sys.exit(1)
        addresses = json.loads(state.read_text()) if state.exists() else []
        addresses.append(args[2])
        state.write_text(json.dumps(addresses))
    elif args[:3] == ["-4", "route", "get"]:
        print(args[3] + " dev " + ("wlan0" if os.environ.get("BAD_ROUTE") else "eth0") + " src 192.168.1.100")
    else: sys.exit(1)
elif name == "sudo":
    if args and args[0] == "-n": args.pop(0)
    os.execvp(args[0], args)
elif name == "pgrep":
    if os.environ.get("LEGACY") and "ecanum_driver" in args[-1]:
        print("999 mecanum_driver")
    else: sys.exit(1)
elif name == "ros2":
    try:
        os.fstat(9)
    except OSError:
        pass
    else:
        with open(os.environ["CALLS"], "a") as f:
            f.write(json.dumps(["event", "ros_inherited_lock", *args]) + "\n")
    if os.environ.get("FAKE_DAEMON") and not pathlib.Path(os.environ["FAKE_DAEMON"]).exists():
        daemon = subprocess.Popen(["/bin/sleep", "30"], close_fds=False,
                                  stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
        pathlib.Path(os.environ["FAKE_DAEMON"]).write_text(str(daemon.pid))
    if args[:2] == ["pkg", "prefix"]: print(os.environ["PREFIX"])
    elif args[:2] == ["topic", "info"]: print("Publisher count: 0")
    elif args[:2] == ["param", "get"]:
        print("Double value is: 0.2")
        if os.environ.get("CANCEL_AFTER_BRIDGE"):
            pathlib.Path(os.environ["RUNTIME"], "stop_requested").touch()
elif name == "python3":
    if "-u" in args:
        print("fake patrol started", flush=True)
        time.sleep(.3)
        with open(os.environ["CALLS"], "a") as f:
            f.write(json.dumps(["event", "fsm_exit"]) + "\n")
    else:
        code = sys.stdin.read()
        if "get_publishers_info_by_topic" in code and os.environ.get("PUBLISHER"):
            print("existing /cmd_vel publisher", file=sys.stderr)
            sys.exit(1)
        if "load_config" in code and os.environ.get("CANCEL"):
            pathlib.Path(os.environ["RUNTIME"], "stop_requested").touch()
elif name == "wheeltec_uart_bridge_node":
    if os.environ.get("BAD_SERIAL"): sys.exit(1)
    print("Opened fake-port at 115200 8N1; waiting for a new /cmd_vel", flush=True)
    def stopped(*_):
        with open(os.environ["CALLS"], "a") as f:
            f.write(json.dumps(["event", "bridge_stop"]) + "\n")
        sys.exit(0)
    signal.signal(signal.SIGTERM, stopped)
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    while True: signal.pause()
'''


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.workspace = self.root / "workspace"
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.calls = self.root / "calls.jsonl"
        self.capture = self.root / "container.sh"
        self.runtime = self.root / "runtime"
        self.setup = self.root / "ros_setup.bash"
        self.setup.write_text(":\n")
        for path in ("install/local_setup.bash", "rl_pig_pen/fsm_lidar_patrol.py",
                     "rl_pig_pen/patrol_config.json",
                     "src/wheeltec_uart_bridge/config/uart_bridge.yaml"):
            target = self.workspace / path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(":\n" if path.endswith(".bash") else "{}\n")
        self.port = self.root / "fake-port"
        self.port.touch()
        prefix = self.root / "install"
        bridge = prefix / "lib/wheeltec_uart_bridge/wheeltec_uart_bridge_node"
        bridge.parent.mkdir(parents=True)
        for tool in [*(self.bin / n for n in ("docker", "pgrep", "ros2", "python3", "ip", "sudo")), bridge]:
            tool.write_text(FAKE)
            tool.chmod(0o755)
        self.env = dict(os.environ, PATH=f"{self.bin}:/usr/bin:/bin", CALLS=str(self.calls),
                        CAPTURE=str(self.capture), PREFIX=str(prefix), RUNTIME=str(self.runtime),
                        IP_STATE=str(self.root / "ip.json"), RADAR_LINK_WAIT_SEC="0")
        source = SCRIPT.read_text()
        self.remote = source.split("<<'CONTAINER_SCRIPT'\n", 1)[1].rsplit("\nCONTAINER_SCRIPT", 1)[0]
        self.remote = self.remote.replace("runtime=/tmp/pig_pen_start_robot",
                                          "runtime=" + shlex.quote(str(self.runtime)))

    def commands(self, name):
        if not self.calls.exists(): return []
        return [row[1:] for row in map(json.loads, self.calls.read_text().splitlines()) if row[0] == name]

    def host_run(self, action, **overrides):
        return subprocess.run(["bash", str(SCRIPT), action], env=dict(self.env, **overrides),
                              capture_output=True, text=True, timeout=10)

    def test_network_setup_adds_both_ips_once_without_docker(self):
        for _ in range(2):
            result = self.host_run("setup-network")
            self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([c for c in self.commands("ip") if c[:2] == ["addr", "add"]],
                         [["addr", "add", "192.168.1.100/24", "dev", "eth0"],
                          ["addr", "add", "192.168.1.102/24", "dev", "eth0"]])
        self.assertFalse(self.commands("docker"))

    def test_network_failure_blocks_container_exec(self):
        for flag in ("NO_INTERFACE", "NO_LINK", "BAD_ROUTE", "DENY_IP"):
            with self.subTest(flag=flag):
                result = self.host_run("start-patrol", **{flag: "1"})
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertFalse(self.capture.exists())
                state = Path(self.env["IP_STATE"])
                if state.exists(): state.unlink()

    def test_invalid_addresses_and_non_host_network_are_rejected(self):
        for settings in ({"RADAR_HOST_IP": "999.1.1.1/24"},
                         {"RADAR_HOST_IP": "192x168x1x100/24"},
                         {"NETWORK_MODE": "bridge"}):
            result = self.host_run("start", **settings)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(self.capture.exists())
        self.assertFalse(any(c[:2] == ["addr", "add"] for c in self.commands("ip")))

    def test_compat_ip_can_be_omitted_and_start_configures_before_exec(self):
        result = self.host_run("start", RADAR_COMPAT_IP="")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.capture.exists())
        self.assertEqual(len([c for c in self.commands("ip") if c[:2] == ["addr", "add"]]), 1)

    def test_status_stop_and_base_do_not_configure_network(self):
        for action in ("status", "stop", "start-base"):
            self.assertEqual(self.host_run(action).returncode, 0)
        self.assertFalse(self.commands("ip"))

    def remote_run(self, action, **overrides):
        return subprocess.run(["bash", "-s", "--", action, str(self.workspace), str(self.port),
                               "/x10/scan", str(self.setup)], input=self.remote,
                              env=dict(self.env, **overrides), text=True,
                              capture_output=True, timeout=15)

    def test_both_shell_layers_parse(self):
        subprocess.run(["bash", "-n", str(SCRIPT)], check=True)
        subprocess.run(["bash", "-n"], input=self.remote, text=True, check=True)

    def test_host_passes_explicit_deployment_settings_without_running_container_code(self):
        result = subprocess.run(["bash", str(SCRIPT), "status"], env=dict(self.env,
            ROBOT_CONTAINER="example", ROBOT_PORT="/dev/example", ROBOT_SCAN_TOPIC="/custom/scan"),
            capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.commands("docker")[-1]
        self.assertEqual(args[:7], ["exec", "-i", "example", "bash", "-s", "--", "status"])
        self.assertIn("/dev/example", args)
        self.assertIn("/custom/scan", args)
        self.assertTrue(self.capture.exists())

    def test_missing_or_stopped_container_is_not_created_or_started(self):
        for flag in ("NO_CONTAINER", "STOPPED"):
            with self.subTest(flag=flag):
                result = subprocess.run(["bash", str(SCRIPT), "start-patrol"],
                    env=dict(self.env, **{flag: "1"}), capture_output=True, text=True, timeout=5)
                self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(c[0] in ("run", "start", "exec") for c in self.commands("docker")))

    def test_patrol_uses_fsm_yaml_limits_and_zero_before_bridge_shutdown(self):
        result = self.remote_run("start-patrol")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        bridge = self.commands("wheeltec_uart_bridge_node")
        self.assertEqual(len(bridge), 1)
        self.assertIn(str(self.workspace / "src/wheeltec_uart_bridge/config/uart_bridge.yaml"), bridge[0])
        self.assertIn("port:=" + str(self.port), bridge[0])
        patrol = [c for c in self.commands("python3") if "-u" in c]
        self.assertEqual(len(patrol), 1)
        self.assertIn(str(self.workspace / "rl_pig_pen/fsm_lidar_patrol.py"), patrol[0])
        self.assertIn("use_sim_time:=false", patrol[0])
        self.assertIn("/scan:=/x10/scan", patrol[0])
        self.assertTrue(any(c[:2] == ["topic", "pub"] for c in self.commands("ros2")))
        rows = list(map(json.loads, self.calls.read_text().splitlines()))
        self.assertFalse(any(row[:2] == ["event", "ros_inherited_lock"] for row in rows))
        zero_index = next(i for i, row in enumerate(rows) if row[:3] == ["ros2", "topic", "pub"])
        self.assertLess(rows.index(["event", "fsm_exit"]), zero_index)
        self.assertLess(zero_index, rows.index(["event", "bridge_stop"]))
        self.assertTrue(list((self.workspace / "logs/robot").glob("*/patrol.log")))

    def test_persistent_ros_daemon_does_not_block_next_start(self):
        daemon_file = self.root / "daemon.pid"
        def stop_daemon():
            if daemon_file.exists():
                try:
                    os.kill(int(daemon_file.read_text()), 15)
                except ProcessLookupError:
                    pass
        self.addCleanup(stop_daemon)
        result = self.remote_run("start", FAKE_DAEMON=str(daemon_file))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(daemon_file.exists())
        os.kill(int(daemon_file.read_text()), 0)
        # Real kernel flock, while the fake ROS daemon is still alive.
        lock = subprocess.run(["flock", "-n", str(self.runtime / "control.lock"), "true"],
                              capture_output=True, text=True, timeout=5)
        self.assertEqual(lock.returncode, 0, "ROS daemon retained launcher lock")
        result = self.remote_run("start")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_test_safe_isolated_topics_and_no_bridge(self):
        result = self.remote_run("test-safe")
        self.assertEqual(result.returncode, 0, result.stderr)
        patrol = [c for c in self.commands("python3") if "-u" in c][0]
        self.assertIn("/cmd_vel:=/test_cmd_vel", patrol)
        self.assertIn("/patrol/state:=/test_patrol/state", patrol)
        self.assertFalse(self.commands("wheeltec_uart_bridge_node"))

    def test_legacy_or_existing_publisher_prevents_uart_start(self):
        for flag in ("LEGACY", "PUBLISHER"):
            with self.subTest(flag=flag):
                result = self.remote_run("start-patrol", **{flag: "1"})
                self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.commands("wheeltec_uart_bridge_node"))

    def test_missing_yaml_never_falls_back_to_compiled_limits(self):
        (self.workspace / "src/wheeltec_uart_bridge/config/uart_bridge.yaml").unlink()
        result = self.remote_run("start-base")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.commands("wheeltec_uart_bridge_node"))

    def test_serial_start_failure_never_starts_patrol(self):
        result = self.remote_run("start-patrol", BAD_SERIAL="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any("-u" in c for c in self.commands("python3")))

    def test_stop_during_preflight_cancels_uart_start(self):
        result = self.remote_run("start-patrol", CANCEL="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.commands("wheeltec_uart_bridge_node"))

    def test_stop_after_bridge_open_prevents_patrol_start(self):
        result = self.remote_run("start-patrol", CANCEL_AFTER_BRIDGE="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(self.commands("wheeltec_uart_bridge_node"))
        self.assertFalse(any("-u" in c for c in self.commands("python3")))
        self.assertIn(["bridge_stop"], self.commands("event"))

    def test_stale_pid_record_does_not_signal_unrelated_process(self):
        with subprocess.Popen(["sleep", "10"]) as unrelated:
            try:
                self.runtime.mkdir()
                (self.runtime / "fsm.pid").write_text(f"{unrelated.pid} 0\n")
                result = self.remote_run("stop")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIsNone(unrelated.poll())
            finally:
                unrelated.terminate()

    def test_stop_without_owned_processes_does_not_publish_or_stop_container(self):
        result = self.remote_run("stop")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.commands("ros2"))
        self.assertFalse(self.commands("docker"))


if __name__ == "__main__":
    unittest.main()
