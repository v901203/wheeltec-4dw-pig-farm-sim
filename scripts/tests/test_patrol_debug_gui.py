import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch, Mock

spec = importlib.util.spec_from_file_location('patrol_debug_gui',
    Path(__file__).resolve().parents[1] / 'patrol_debug_gui.py')
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)

class MonitorTests(unittest.TestCase):
    def test_worker_only_starts_read_only_monitor(self):
        args = gui.worker_command('robot-dev', '/workspace/a; echo BAD')
        self.assertEqual(args[-1], '/workspace/a; echo BAD')
        self.assertNotIn('echo BAD', args[-3])
        self.assertIn('--ros-worker', args[-3])
        self.assertIn('RMW_IMPLEMENTATION=rmw_fastrtps_cpp', args)

    def test_actions_are_limited_to_patrol_service(self):
        self.assertEqual(gui.service_command('stop'),
            ['sudo', '-n', '/usr/bin/systemctl', 'stop', 'pig-pen-patrol.service'])
        with self.assertRaises(ValueError):
            gui.service_command('start-other')

    def test_window_telemetry_expiry_and_restart_confirmation(self):
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        try:
            # Never run docker, journalctl, systemctl or ROS in this UI test.
            with patch.object(gui.threading.Thread, 'start'):
                m = gui.Monitor(root, Mock(container='test', project='/test'))
                m.enqueue('cmd', {'v': .12, 'y': 0, 'w': -.3})
                m.enqueue('state', 'MAIN')
                m.enqueue('debug', json.dumps(dict(junctions_done=2, junctions_total=5,
                    endpoints_reached=4, reason='footprint_wait', state='MAIN', state_label='主幹道前進')))
                m.refresh()
                self.assertIn('+0.120', m.values['cmd'].get())
                self.assertIn('2/5', m.values['progress'].get())
                self.assertEqual(m.values['reason'].get(), 'footprint_wait')
                with patch('tkinter.messagebox.askyesno', return_value=False):
                    m.action('restart')
                self.assertFalse(m.busy)
                with patch('tkinter.messagebox.askyesno', return_value=True):
                    m.action('restart')
                self.assertTrue(m.busy)
                m.enqueue('action_done', '授權取消')
                m.received['cmd'] = 0
                m.received['debug'] = 0
                m.refresh()
                self.assertFalse(m.busy)
                self.assertIn('逾時', m.values['cmd'].get())
                self.assertEqual(m.values['progress'].get(), '未知')
        finally:
            root.destroy()

if __name__ == '__main__':
    unittest.main()
