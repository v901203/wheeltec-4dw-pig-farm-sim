#!/usr/bin/env python3
"""Read-only ROS 2 movement-rule diagnostics. Never publishes motion commands."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import time
from collections import deque
import numpy as np
from navigation import SensorFault, load_config, scan_features, lidar_ignore_mask, command_clearance_masks


def inspect_scan(scan, cfg, probes, measured_speed=0.):
    """Use exactly the project's mask/swept-footprint calculations."""
    features = scan_features(scan, cfg)
    raw = np.asarray(scan.ranges, dtype=float)
    ignored = lidar_ignore_mask(features.angles, cfg)
    known = ((np.isfinite(raw) & (raw >= scan.range_min) & (raw <= scan.range_max)) |
             np.isposinf(raw) | ignored)
    result = dict(invalid_unmasked_rays=int(np.sum(~known)), ignored_rays=int(ignored.sum()),
                  front_m=float(features.front_m), rear_wall_m=float(features.rear_wall_m),
                  left_m=float(features.left_edge_m), right_m=float(features.right_edge_m), probes={})
    for name, command in probes.items():
        masks, limits = command_clearance_masks(command, features, cfg, measured_speed)
        hits = []
        for guard, mask in masks.items():
            indices = np.flatnonzero(mask)
            for index in sorted(indices, key=lambda i: features.ranges[i])[:3]:
                angle = float(features.angles[index]); distance = float(features.ranges[index])
                hits.append(dict(guard=guard, ray=int(index), range_m=distance,
                                 body_angle_deg=math.degrees(angle),
                                 x_m=distance*math.cos(angle), y_m=distance*math.sin(angle)))
        result['probes'][name] = dict(command=list(command), blocked=bool(hits),
                                      limits=limits, hits=hits)
    return result


def recent_log_lines(directory):
    """Historical clues, never treated as current controller state."""
    if directory is None:
        pointer = Path('/tmp/pig_pen_start_robot/latest_log_dir')
        if not pointer.is_file(): return []
        directory = Path(pointer.read_text().strip())
    lines=[]
    for name in ('patrol.log','bridge.log','lidar.log'):
        path=directory/name
        if not path.is_file(): continue
        with path.open('rb') as stream:
            stream.seek(0,2); size=stream.tell(); stream.seek(max(0,size-24000))
            text=stream.read().decode(errors='replace')
        matches=[line for line in text.splitlines() if any(token in line for token in
                 ('[WARN]','[ERROR]','原因：','Package not found','not found:','Failed','Error'))]
        lines.extend(dict(file=str(path),text=line[:1800]) for line in matches[-3:])
    return lines


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-topic',default='/x10/scan')
    parser.add_argument('--cmd-topic',default='/cmd_vel')
    parser.add_argument('--odom-topic',default='/odom')
    parser.add_argument('--config',type=Path)
    parser.add_argument('--log-dir',type=Path)
    parser.add_argument('--duration',type=float,default=30.)
    parser.add_argument('--interval',type=float,default=2.)
    parser.add_argument('--probe-v',type=float,default=.05)
    parser.add_argument('--probe-w',type=float,default=.2)
    args=parser.parse_args()
    if (not all(math.isfinite(v) for v in (args.duration,args.interval,args.probe_v,args.probe_w)) or
        args.duration<0 or args.interval<=0 or not 0<args.probe_v<=.2 or not 0<args.probe_w<=.6):
        parser.error('Require duration>=0, interval>0, 0<probe-v<=.2, 0<probe-w<=.6')
    cfg=load_config(args.config) if args.config else load_config()
    # Patrol node's braking horizon includes scan aging and watchdog period.
    configured_dt=cfg.control_dt
    cfg=replace(cfg,control_dt=max(cfg.control_dt,.5))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    from rcl_interfaces.msg import Log

    class Diagnostics(Node):
        def __init__(self):
            super().__init__('pig_pen_motion_diagnostics')
            self.scan=None; self.scan_at=self.cmd_at=self.state_at=self.control_at=self.odom_at=None
            self.command=None; self.state=None; self.control=None; self.speed=0.
            self.last_stamp=None; self.stamp_increasing=True
            self.peer_stop=None; self.peer_at=None
            self.intervals=deque(maxlen=16); self.warnings=deque(maxlen=5)
            self.create_subscription(LaserScan,args.scan_topic,self.on_scan,qos_profile_sensor_data)
            self.create_subscription(Twist,args.cmd_topic,self.on_command,qos_profile_sensor_data)
            self.create_subscription(Odometry,args.odom_topic,self.on_odom,qos_profile_sensor_data)
            latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String,'/patrol/state',self.on_state,latched)
            self.create_subscription(String,'/control/status',self.on_control,latched)
            self.create_subscription(Log,'/rosout',self.on_log,qos_profile_sensor_data)
            self.create_subscription(Bool,'/pig_face/stop_requested',self.on_peer,10)
        def on_scan(self,msg):
            now=time.monotonic()
            if self.scan_at is not None and 0<now-self.scan_at<.5:
                self.intervals.append(now-self.scan_at)
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            self.stamp_increasing=math.isfinite(stamp) and (self.last_stamp is None or stamp>self.last_stamp)
            self.last_stamp=stamp;self.scan=msg;self.scan_at=now
        def on_command(self,msg):
            self.command=(msg.linear.x,msg.angular.z);self.cmd_at=time.monotonic()
        def on_odom(self,msg):
            self.speed=abs(msg.twist.twist.linear.x);self.odom_at=time.monotonic()
        def on_state(self,msg):self.state=msg.data;self.state_at=time.monotonic()
        def on_control(self,msg):
            try:self.control=json.loads(msg.data);self.control_at=time.monotonic()
            except (ValueError,TypeError):pass
        def on_peer(self,msg):
            self.peer_stop=msg.data;self.peer_at=time.monotonic()
        def on_log(self,msg):
            if msg.level>=30 and msg.name in ('pig_pen_lidar_patrol','wheeltec_uart_bridge'):
                self.warnings.append(dict(received_at=time.monotonic(),node=msg.name,text=msg.msg))
        def report(self):
            now=time.monotonic(); problems=[]
            age=lambda received:None if received is None else round(now-received,3)
            report=dict(route_state=self.state,route_state_age_s=age(self.state_at),
                        control_status=self.control,control_status_age_s=age(self.control_at),
                        scan_topic=args.scan_topic,scan_age_s=age(self.scan_at),
                        commanded=self.command if self.command is None or np.isfinite(self.command).all() else None,
                        command_age_s=age(self.cmd_at),peer_stop_requested=self.peer_stop,peer_age_s=age(self.peer_at),
                        reaction_horizon_s=cfg.control_dt,vision_required_in_config=cfg.vision_required,
                        ignore_sectors_deg=cfg.lidar_ignore_sectors_deg)
            publishers=self.get_publishers_info_by_topic(args.cmd_topic)
            subscribers=self.get_subscriptions_info_by_topic(args.cmd_topic)
            report['cmd_publishers']=[p.node_namespace+'/'+p.node_name for p in publishers]
            report['cmd_subscribers']=[p.node_namespace+'/'+p.node_name for p in subscribers]
            if not publishers:problems.append('沒有速度指令發布者：巡邏／控制程式可能未啟動，或 ROS domain 不一致')
            if len(publishers)>1:problems.append('速度指令有多個發布者，可能互相覆寫')
            if not subscribers:problems.append('速度 Topic 沒有訂閱者：底盤 bridge 可能未啟動，或 topic/domain 不一致')
            if self.cmd_at is None or now-self.cmd_at>.5:
                problems.append('未收到新速度指令超過 0.5 秒：預設 UART watchdog 會送零；實際逾時以 bridge 設定為準')
            if self.command is not None and not np.isfinite(self.command).all():
                problems.append('速度指令含 NaN/Inf，不符合有限數值規則')
            if self.command is not None and np.isfinite(self.command).all() and max(abs(v) for v in self.command)<1e-6:
                report['zero_command_note']='目前收到零速度；這是控制程式的輸出結果，需結合狀態及日誌判斷原始停止原因。'
            if self.state=='FAILED':problems.append('FSM 已鎖定 FAILED，需先排除原因；障礙移除不代表可自動解除鎖定')
            if self.state=='DONE':problems.append('FSM 已完成路線，正常停止')
            if self.control and self.control_at is not None and now-self.control_at<2.:
                if self.control.get('mode')=='STOP':problems.append('控制模式為 STOP：'+str(self.control.get('reason')))
                elif self.control.get('reason') not in ('AUTO_RUNNING','MANUAL_COMMAND'):
                    report['reported_controller_reason']=self.control.get('reason')
            if self.scan is None:problems.append('沒有收到雷達：檢查驅動、topic、ROS domain')
            else:
                pause=min(.4,max(.15,2*configured_dt,
                    1.5*float(np.quantile(self.intervals,.9))+.02 if len(self.intervals)>=3 else 0.))
                report['estimated_scan_pause_s']=pause
                if now-self.scan_at>pause:problems.append(f'雷達接收間隔超過估計暫停門檻 {pause:.3f}s（控制節點實際歷史可能不同）')
                if not self.stamp_increasing:problems.append('最近兩筆雷達時間戳未遞增')
                probes=dict(假設前進=(args.probe_v,0.),假設後退=(-args.probe_v,0.),
                            假設左旋=(0.,args.probe_w),假設右旋=(0.,-args.probe_w))
                if self.command and np.isfinite(self.command).all():probes['目前已發布指令']=self.command
                fresh_odom=self.odom_at is not None and now-self.odom_at<.5 and math.isfinite(self.speed)
                report['measured_speed_m_s']=self.speed if fresh_odom else None
                if not fresh_odom:report['speed_note']='沒有新鮮 odom，檢查暫以量測速度 0 計算，通過不代表實車煞停距離足夠'
                try:
                    report['safety']=inspect_scan(self.scan,cfg,probes,self.speed if fresh_odom else 0.)
                    if report['safety']['invalid_unmasked_rays']:
                        problems.append('遮罩外有無效雷達回波，不符合巡邏入口的全有效檢查')
                    if report['safety']['probes'].get('目前已發布指令',{}).get('blocked'):
                        problems.append('目前發布指令不通過此診斷的雷達安全檢查，詳見 hits 的 guard/角度/距離')
                except (SensorFault,ValueError,FloatingPointError) as exc:
                    problems.append('雷達資料無法解析：'+str(exc))
            if self.peer_stop is not None:
                report['peer_note']='收到停車請求不一定立即停車，是否延後處理由當前 FSM 決定；失聯也不會自動取消已接受的停車。'
            report['problems']=problems
            report['recent_warnings']=[dict(age_s=round(now-r['received_at'],1),node=r['node'],text=r['text']) for r in self.warnings]
            report['historical_log_clues']=recent_log_lines(args.log_dir)
            report['limitations']='假設動作只作離線安全試算，不會送指令。已發布零速度無法反推原本遭擋的動作；FSM 內部幾何／計數／恢復狀態須以 patrol.log 為準。無底盤回授不能證實車輪是否移動。'
            print(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),flush=True)

    rclpy.init(args=[]);node=Diagnostics();start=time.monotonic();last=start
    print('唯讀移動診斷已啟動：不發布速度、不呼叫控制服務。',flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
            now=time.monotonic()
            if now-last>=args.interval:node.report();last=now
            if args.duration and now-start>=args.duration:break
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
