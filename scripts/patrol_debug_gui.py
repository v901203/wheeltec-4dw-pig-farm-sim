#!/usr/bin/env python3
"""Host desktop monitor; isolated read-only ROS worker runs inside robot-dev."""
import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time


def ros_worker():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from rcl_interfaces.msg import Log
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = rclpy.create_node('patrol_debug_monitor_' + str(os.getpid()))
    eof = threading.Event()
    def stdin_watch():
        for _ in sys.stdin:
            pass
        eof.set()
    threading.Thread(target=stdin_watch, daemon=True).start()
    def emit(kind, data):
        print(json.dumps({'kind': kind, 'data': data}, ensure_ascii=False), flush=True)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Twist, '/cmd_vel', lambda m: emit('cmd',
        {'v': m.linear.x, 'y': m.linear.y, 'w': m.angular.z}), 10)
    node.create_subscription(String, '/patrol/state', lambda m: emit('state', m.data), qos)
    node.create_subscription(String, '/patrol/debug', lambda m: emit('debug', m.data), qos)
    node.create_subscription(String, '/control/status', lambda m: emit('control', m.data), qos)
    def log(m):
        if m.name in ('pig_pen_lidar_patrol', 'wheeltec_uart_bridge'):
            emit('log', m.name + ': ' + m.msg)
    node.create_subscription(Log, '/rosout', log, qos)
    node.create_timer(1., lambda: emit('heartbeat', 'ROS 監看已連線'))
    try:
        while rclpy.ok() and not eof.is_set():
            rclpy.spin_once(node, timeout_sec=.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def worker_command(container, project, rmw="rmw_fastrtps_cpp"):
    # Pass the mounted project as an argument, never interpolate shell code.
    script = '''monitor_rmw="$RMW_IMPLEMENTATION"
for setup in /opt/ros/humble/install/setup.bash /opt/ros/humble/setup.bash; do
if [ -r "$setup" ]; then source "$setup"; break; fi
done
export RMW_IMPLEMENTATION="$monitor_rmw"
exec python3 -u "$1/scripts/patrol_debug_gui.py" --ros-worker'''
    return ['docker', 'exec', '-i', '-e', 'RMW_IMPLEMENTATION=' + rmw,
            container, 'bash', '--noprofile', '--norc',
            '-c', script, 'monitor', project]


def service_command(action):
    if action not in ('stop', 'restart'):
        raise ValueError('Unsupported service action')
    return ['sudo', '-n', '/usr/bin/systemctl', action, 'pig-pen-patrol.service']


class Monitor:
    def __init__(self, root, args):
        import tkinter as tk
        from tkinter import ttk, scrolledtext
        self.root, self.args = root, args
        self.events = queue.Queue(maxsize=2000)
        self.closed = threading.Event()
        self.busy = False
        self.received = {}
        self.values = {}
        root.title('豬舍巡邏 — 除錯監看')
        root.geometry('1000x740')
        panel = ttk.Frame(root, padding=14)
        panel.pack(fill='both', expand=True)
        for key, title in [('service', '系統服務'), ('connection', 'ROS 連線'),
                           ('state', '巡邏路段'), ('cmd', 'cmd_vel 指令（非實際速度）'),
                           ('progress', '巡邏進度'), ('reason', '等待／異常原因'),
                           ('control', '控制模式'), ('action', '操作結果')]:
            self.values[key] = tk.StringVar(value='等待資料')
            ttk.Label(panel, text=title, font=('', 11, 'bold')).pack(anchor='w')
            ttk.Label(panel, textvariable=self.values[key], wraplength=950).pack(anchor='w', pady=(0, 7))
        buttons = ttk.Frame(panel)
        buttons.pack(fill='x', pady=6)
        self.stop_button = ttk.Button(buttons, text='停止巡邏服務', command=lambda: self.action('stop'))
        self.stop_button.pack(side='left', padx=5)
        self.restart_button = ttk.Button(buttons, text='回到起點後重新出發', command=lambda: self.action('restart'))
        self.restart_button.pack(side='left', padx=5)
        ttk.Button(buttons, text='讀取最近服務日誌', command=self.read_journal).pack(side='left', padx=5)
        ttk.Label(panel, text='關閉視窗不會停止車子。停止按鈕會請求停止服務，不能取代實體急停。').pack(anchor='w')
        self.log = scrolledtext.ScrolledText(panel, height=13, state='disabled', wrap='word')
        self.log.pack(fill='both', expand=True, pady=8)
        copy_buttons = ttk.Frame(panel)
        copy_buttons.pack(fill='x')
        ttk.Button(copy_buttons, text='複製選取日誌', command=self.copy_logs).pack(side='left', padx=5)
        ttk.Button(copy_buttons, text='複製全部日誌',
                   command=lambda: self.copy_logs(all_lines=True)).pack(side='left', padx=5)
        self.copy_status = tk.StringVar()
        ttk.Label(copy_buttons, textvariable=self.copy_status).pack(side='left', padx=5)
        self.log.bind('<Control-c>', self.copy_logs)
        self.log.bind('<Control-C>', self.copy_logs)
        self.log.bind('<Control-a>', self.select_logs)
        self.log_menu = tk.Menu(self.log, tearoff=False)
        self.log_menu.add_command(label='複製選取日誌', command=self.copy_logs)
        self.log_menu.add_command(label='複製全部日誌', command=lambda: self.copy_logs(all_lines=True))
        self.log_menu.add_command(label='全選', command=self.select_logs)
        self.log.bind('<Button-3>', self.show_log_menu)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.observer = threading.Thread(target=self.observe, daemon=True)
        self.observer.start()
        threading.Thread(target=self.poll_service, daemon=True).start()
        self.journal_thread = threading.Thread(target=self.follow_journal, daemon=True)
        self.journal_thread.start()
        root.after(100, self.refresh)

    def select_logs(self, event=None):
        self.log.tag_add('sel', '1.0', 'end-1c')
        self.log.focus_set()
        return 'break'

    def show_log_menu(self, event):
        try:
            self.log_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.log_menu.grab_release()
        return 'break'

    def copy_logs(self, event=None, *, all_lines=False):
        import tkinter as tk
        if all_lines:
            text = self.log.get('1.0', 'end-1c')
        elif self.log.tag_ranges('sel'):
            text = self.log.get('sel.first', 'sel.last')
        else:
            self.copy_status.set('請先選取文字，或按「複製全部日誌」')
            return 'break'
        if not text:
            self.copy_status.set('目前沒有日誌可複製')
            return 'break'
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
            self.copy_status.set(f'已複製 {len(text)} 個字元')
        except tk.TclError as exc:
            self.copy_status.set('剪貼簿無法使用：' + str(exc))
        return 'break'

    def enqueue(self, kind, data):
        try:
            self.events.put_nowait((kind, data, time.monotonic()))
        except queue.Full:
            pass

    def append_log(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', time.strftime('%H:%M:%S ') + str(text) + '\n')
        if int(self.log.index('end-1c').split('.')[0]) > 600:
            self.log.delete('1.0', '101.0')
        self.log.see('end')
        self.log.configure(state='disabled')

    def observe(self):
        while not self.closed.is_set():
            proc = None
            thread = None
            try:
                proc = subprocess.Popen(worker_command(self.args.container, self.args.project, self.args.rmw),
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1)
                reader_done = threading.Event()
                def reader(proc=proc, reader_done=reader_done):
                    try:
                        for line in proc.stdout:
                            try:
                                packet = json.loads(line)
                                self.enqueue(packet['kind'], packet['data'])
                            except (ValueError, KeyError, TypeError):
                                self.enqueue('log', line.rstrip())
                    finally:
                        reader_done.set()
                thread = threading.Thread(target=reader, daemon=True)
                thread.start()
                while not reader_done.wait(.2) and not self.closed.is_set():
                    pass
            except OSError as exc:
                self.enqueue('log', str(exc))
            finally:
                if proc is not None:
                    if proc.stdin:
                        proc.stdin.close()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.terminate()
                        try:
                            proc.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                            proc.wait()
                    if thread is not None:
                        thread.join(timeout=1)
                self.enqueue('connection', '監看中斷，將自動重新連線')
            self.closed.wait(3)

    def poll_service(self):
        while not self.closed.is_set():
            try:
                result = subprocess.run(['systemctl', 'show', 'pig-pen-patrol.service',
                    '-p', 'ActiveState', '-p', 'SubState', '-p', 'Result'],
                    capture_output=True, text=True, timeout=5)
                self.enqueue('service', (result.stdout + result.stderr).strip())
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.enqueue('service', str(exc))
            self.closed.wait(2)

    def follow_journal(self):
        """Follow host service stderr/stdout even if ROS startup itself fails."""
        while not self.closed.is_set():
            proc = None
            reader = None
            try:
                proc = subprocess.Popen(['journalctl', '-u', 'pig-pen-patrol.service',
                    '-b', '-n', '35', '-f', '--no-pager'], stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, bufsize=1)
                def read(stream=proc.stdout):
                    for line in stream:
                        self.enqueue('log', '服務日誌：' + line.rstrip())
                reader = threading.Thread(target=read, daemon=True)
                reader.start()
                while proc.poll() is None and not self.closed.wait(.2):
                    pass
            except OSError as exc:
                self.enqueue('log', '無法追蹤服務日誌：' + str(exc))
            finally:
                if proc is not None:
                    if proc.poll() is None:
                        proc.terminate()
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
                    if reader is not None:
                        reader.join(timeout=1)
                    proc.stdout.close()
            self.closed.wait(5)

    def read_journal(self):
        def read():
            try:
                r = subprocess.run(['journalctl', '-u', 'pig-pen-patrol.service', '-b',
                                    '-n', '35', '--no-pager'], capture_output=True, text=True, timeout=8)
                self.enqueue('log', '最近服務日誌（含歷史事件）：\n' + r.stdout + r.stderr)
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.enqueue('log', str(exc))
        threading.Thread(target=read, daemon=True).start()

    def action(self, action):
        from tkinter import messagebox
        if self.busy:
            return
        if action == 'restart' and not messagebox.askyesno('確認重新出發',
                '車子已放回起點、對準主幹道且周圍可通行？\n重新出發會清除原路線進度，並可能立即移動。'):
            return
        self.busy = True
        self.stop_button.configure(state='disabled')
        self.restart_button.configure(state='disabled')
        self.values['action'].set('正在執行服務操作，請等待完成')
        def run():
            try:
                result = subprocess.run(service_command(action), capture_output=True, text=True)
                text = ('服務請求完成；請確認最新巡邏與速度狀態' if result.returncode == 0 else
                        '操作失敗（若尚未設定免密碼權限，請先執行 install_patrol_gui_permissions.sh）：' + (result.stderr or result.stdout))
                self.enqueue('action_done', text)
            except OSError as exc:
                self.enqueue('action_done', str(exc))
        threading.Thread(target=run, daemon=True).start()

    def refresh(self):
        for _ in range(200):
            try:
                kind, data, stamp = self.events.get_nowait()
            except queue.Empty:
                break
            self.received[kind] = stamp
            if kind == 'log':
                self.append_log(data)
            elif kind == 'cmd':
                self.values['cmd'].set('linear.x={v:+.3f} m/s   linear.y={y:+.3f} m/s   angular.z={w:+.3f} rad/s'.format(**data))
            elif kind in ('debug', 'control'):
                try:
                    payload = json.loads(data)
                    if kind == 'debug':
                        self.values['progress'].set('路口：{junctions_done}/{junctions_total}　支線端點：{endpoints_reached}'.format(**payload))
                        self.values['reason'].set(payload['reason'])
                        self.values['state'].set(payload['state_label'] + ' (' + payload['state'] + ')')
                    else:
                        self.values['control'].set(str(payload.get('mode')) + ' / ' + str(payload.get('reason')))
                except (ValueError, KeyError, TypeError) as exc:
                    self.append_log('狀態格式錯誤：' + str(exc))
            elif kind == 'heartbeat':
                self.values['connection'].set(data)
            elif kind == 'action_done':
                self.busy = False
                self.stop_button.configure(state='normal')
                self.restart_button.configure(state='normal')
                self.values['action'].set(data)
                self.read_journal()
            elif kind in self.values:
                self.values[kind].set(str(data))
        now = time.monotonic()
        if now - self.received.get('cmd', 0) > 1:
            self.values['cmd'].set('未收到新指令／已逾時，無法確認目前輸出')
        if now - self.received.get('state', 0) > 3:
            self.values['state'].set('巡邏狀態失聯（不可依舊狀態判斷）')
        if now - self.received.get('debug', 0) > 3:
            self.values['reason'].set('詳細狀態尚未收到或已失聯；請查看日誌')
            self.values['progress'].set('未知')
        if now - self.received.get('control', 0) > 3:
            self.values['control'].set('未收到三模式控制狀態；完整巡邏入口可能未使用此介面')
        if now - self.received.get('heartbeat', 0) > 3:
            self.values['connection'].set('ROS 監看未連線／失聯，正在重試')
        if not self.closed.is_set():
            self.root.after(100, self.refresh)

    def close(self):
        self.closed.set()
        self.root.withdraw()
        def finish():
            if self.observer.is_alive() or self.journal_thread.is_alive():
                self.root.after(100, finish)
            else:
                self.root.destroy()
        finish()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-worker', action='store_true')
    parser.add_argument('--install-autostart', action='store_true',
                        help='Install desktop login autostart for the current user')
    parser.add_argument('--desktop-shortcut', action='store_true',
                        help='Also create a shortcut on the desktop with --install-autostart')
    parser.add_argument('--rmw', choices=('rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp'),
                        default=os.environ.get('ROBOT_RMW', 'rmw_fastrtps_cpp'))
    parser.add_argument('--container', default=os.environ.get('ROBOT_CONTAINER', 'robot-dev'))
    parser.add_argument('--project', default='/workspace/pig_patrol')
    args = parser.parse_args()
    if args.install_autostart:
        def desktop_quote(value):
            value = str(value).replace('%', '%%')
            for char in ('\\', '"', '`', '$'):
                value = value.replace(char, '\\' + char)
            return '"' + value + '"'
        target = Path.home() / '.config/autostart/pig-pen-debug.desktop'
        target.parent.mkdir(parents=True, exist_ok=True)
        command = ' '.join(desktop_quote(x) for x in (
            '/usr/bin/python3', Path(__file__).resolve(), '--container', args.container,
            '--project', args.project, '--rmw', args.rmw))
        target.write_text('[Desktop Entry]\nType=Application\nName=豬舍巡邏除錯\n'
                          + 'Exec=' + command + '\nTerminal=false\n'
                          + 'X-GNOME-Autostart-enabled=true\n', encoding='utf-8')
        print('已設定登入桌面後開啟 GUI：' + str(target))
        if args.desktop_shortcut:
            desktop = Path.home() / 'Desktop'
            try:
                result = subprocess.run(['xdg-user-dir', 'DESKTOP'], capture_output=True,
                                        text=True, timeout=3)
                if result.returncode == 0 and result.stdout.strip():
                    candidate = Path(result.stdout.strip())
                    if candidate.is_absolute() and candidate != Path.home():
                        desktop = candidate
            except (OSError, subprocess.TimeoutExpired):
                pass
            desktop.mkdir(parents=True, exist_ok=True)
            shortcut = desktop / target.name
            shortcut.write_text(target.read_text(), encoding='utf-8')
            shortcut.chmod(0o755)
            print('桌面捷徑：' + str(shortcut))
    elif args.ros_worker:
        ros_worker()
    else:
        import tkinter as tk
        root = tk.Tk()
        Monitor(root, args)
        root.mainloop()


if __name__ == '__main__':
    main()
