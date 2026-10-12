#!/usr/bin/env python3
"""Top-level mission sequencing for the CAC rover.

State flow (one excavate->dump cycle per mission.num_cycles):

  START -> LOCALIZE -> SURVEY -> TRAVEL_TO_EXCAVATION -> EXCAVATION -> TRAVEL_TO_CONSTRUCTION
        -> DUMP -> (TRAVEL_TO_EXCAVATION ... if more cycles) -> RETURN -> COMPLETE
  any state -> FAILED (terminal, with a logged reason)

How each transition is earned:
  LOCALIZE   turns in place in steps (rotate, pause) until marker_localizer has a
             fix from the start-zone ArUco markers (status "initialized").
             The start pose is generated with turn-in-place clearance.
  SURVEY     before the first leg, turns in place to face each of
             mission.survey_headings (map frame, toward the obstacle zone) and
             holds still there, so the depth camera sweeps the ground next to
             the start zone and crater_detector confirms craters BEFORE the
             first path is planned. Without it, craters near the start were
             seen only if the marker search happened to spin past them (seed
             10: an immediate marker fix, no spin, a wheel in a crater 1.2 m
             from the start).
  TRAVEL_*   sends the leg's goal (arena frame == map frame) straight to Nav2's
             NavigateToPose; the global planner chooses the route through the
             costmap. A leg succeeds ONLY when Nav2 returns SUCCEEDED *and* the
             localized pose is within mission.arrival_* of the goal *and* the
             leg's zone condition holds (mission_logic.zone_ok). Failures are
             retried up to mission.max_goal_attempts, switching to a validated
             alternate goal after GOAL_OCCUPIED / NO_VALID_PATH; then FAILED.
             After START_OCCUPIED (the padded footprint covers a lethal cell --
             Nav2 has no recovery for it) the rover first makes the smallest
             costmap-checked turn or short drive that clears it
             (mission_logic.escape_motion).
             The arm is stowed (and verified) before every leg.
  EXCAVATION / DUMP  are simulation abstractions: the arm (if present) runs its
             dig/dump joint sequence and PayloadManager counts a notional
             payload. No sand is moved; logs say "simulated".

Goals come from config/arena_params.yaml `mission:` -- fixed zone geometry known
before the run. This node never sees boulder/crater positions (those reach Nav2
only through sensed data in the costmap) and never receives the spawn pose:
localization comes from the start-zone markers + odometry.
"""
import enum
import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from cac_sim import mission_logic as ml
from cac_sim.payload_manager import PayloadManager


class State(enum.Enum):
    START = 'START'
    LOCALIZE = 'LOCALIZE'
    SURVEY = 'SURVEY'
    TRAVEL_TO_EXCAVATION = 'TRAVEL_TO_EXCAVATION'
    EXCAVATION = 'EXCAVATION'
    TRAVEL_TO_CONSTRUCTION = 'TRAVEL_TO_CONSTRUCTION'
    DUMP = 'DUMP'
    RETURN = 'RETURN'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'


_LEG_OF_STATE = {State.TRAVEL_TO_EXCAVATION: 'excavation',
                 State.TRAVEL_TO_CONSTRUCTION: 'construction',
                 State.RETURN: 'return'}
_GOAL_PARAM = {'excavation': 'mission.excavation_approach',
               'construction': 'mission.construction_approach',
               'return': 'mission.return_approach'}


class MissionStateMachine(Node):

    def __init__(self):
        super().__init__('mission_state_machine')
        dp = self.declare_parameter
        dp('autonomy', True)
        dp('arena.length', 9.0)
        dp('arena.width', 5.0)
        dp('zones.starting_zone', [-4.5, -2.7])
        dp('zones.excavation_zone', [2.0, 3.5])
        dp('zones.construction_zone', [3.5, 4.5])
        dp('robot.footprint_front', 0.66)
        dp('robot.footprint_rear', 0.47)
        dp('robot.footprint_half_width', 0.37)
        dp('mission.excavation_approach', [2.35, 0.0, 0.0])
        dp('mission.construction_approach', [3.15, 0.0, 0.0])
        dp('mission.zone_cross_margin', 0.10)
        dp('mission.return_approach', [-3.55, 0.0, math.pi])
        dp('mission.arrival_xy_tolerance', 0.30)
        dp('mission.arrival_yaw_tolerance', 0.60)
        dp('mission.max_goal_attempts', 3)
        dp('mission.leg_timeout_sec', 240.0)
        dp('mission.num_cycles', 1)
        dp('mission.excavation_duration_sec', 8.0)
        dp('mission.dump_duration_sec', 5.0)
        dp('localize_spin_rate', 0.35)       # rad/s while searching for markers
        dp('localize_timeout_sec', 90.0)
        # Obstacle-zone survey (deg, map frame; 0 = east, toward the obstacle zone).
        # 30 deg steps with the camera's 66 deg view: about -93..+93 deg covered.
        dp('mission.survey_headings', [-60.0, -30.0, 0.0, 30.0, 60.0])
        dp('mission.survey_hold_sec', 1.2)       # still time per heading (detector: 2 frames at 8 Hz)
        dp('mission.survey_turn_rate', 0.35)     # rad/s, the measured no-slip turn rate
        dp('mission.survey_timeout_sec', 60.0)
        dp('mission.footprint_padding', 0.02)   # = Nav2 footprint_padding (test_consistency)
        dp('escape_timeout_sec', 15.0)
        # Final turn: the lattice planner reaches a goal heading only by driving a
        # 2.6-3.3 m loop when the rover is already at the goal position but turned
        # (planner_route_test, seed 14) -- through ground the camera never saw (a
        # wheel in an undetected crater). Within final_turn_radius of the goal and
        # more than final_turn_min_yaw off its heading, the mission cancels Nav2 and
        # turns in place instead (if the sweep is clear in the costmap).
        dp('mission.final_turn_radius', 0.25)
        dp('mission.final_turn_min_yaw', 0.40)    # = Nav2 goal checker yaw tolerance
        dp('mission.final_turn_timeout_sec', 20.0)
        # Arm (optional: used only if these joints appear on /joint_states).
        dp('arm.joints', ['boom_joint', 'stick_joint', 'bucket_joint'])
        dp('arm.stow', [0.0, 0.0, 0.0])
        dp('arm.dig_sequence', [0.0, 0.0, 0.0])    # flattened list of poses
        dp('arm.dump_sequence', [0.0, 0.0, 0.0])
        dp('arm.pose_tolerance', 0.08)
        dp('arm.pose_timeout_sec', 8.0)

        g = self.get_parameter
        self._enabled = bool(g('autonomy').value)
        self._fp = ml.Footprint(g('robot.footprint_front').value, g('robot.footprint_rear').value,
                                g('robot.footprint_half_width').value)
        self._zones = ml.Zones(tuple(g('zones.starting_zone').value), tuple(g('zones.excavation_zone').value),
                               tuple(g('zones.construction_zone').value),
                               g('arena.length').value / 2.0, g('arena.width').value / 2.0)
        self._max_attempts = int(g('mission.max_goal_attempts').value)
        self._cycles_left = int(g('mission.num_cycles').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # bt_navigator's action server exists before the node is ACTIVE, and
        # goals sent then are rejected; legs wait for ACTIVE instead of burning
        # attempts (its global costmap needs the map frame, i.e. the marker fix).
        self._bt_state = self.create_client(GetState, '/bt_navigator/get_state')
        self._nav_active = False
        self._nav_query = None
        dp('nav2_ready_timeout_sec', 180.0)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._state_pub = self.create_publisher(String, 'mission_state', 10)
        self._status_pub = self.create_publisher(String, 'mission_status', 10)
        self._payload_pub = self.create_publisher(Float32, 'payload_level', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._costmap = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, '_costmap', m), latched)
        self._loc_status = {}
        self.create_subscription(String, '/localization/status', self._on_loc_status, 10)
        self._joint_pos = {}
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20)
        self._arm_pubs = {j: self.create_publisher(Float64, f'/arm/{j}/cmd', 10)
                          for j in g('arm.joints').value}

        self._payload = PayloadManager()
        self._state = None
        self._state_t0 = self.get_clock().now()
        self._history = []          # [(leg, outcome, reason)]
        self._fail_reason = ''
        self._leg = None
        self._goal_handle = None
        self._nav_result = None     # (status, error_code) once the action finishes
        self._goal_seq = 0          # results of older (cancelled/replaced) goals are ignored
        self._arm_queue = []
        self._arm_target = None
        self._arm_t0 = None
        self._set_state(State.START)
        self.create_timer(0.1, self._tick)
        if not self._enabled:
            self.get_logger().info('autonomy:=false -- mission state machine idling, no goals will be sent.')

    # ----------------------------------------------------------------- helpers

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _set_state(self, state: State, note: str = ''):
        if state != self._state:
            self.get_logger().info(f'Mission state: {self._state.value if self._state else "-"} -> '
                                   f'{state.value}{" (" + note + ")" if note else ""}')
        self._state = state
        self._state_t0 = self.get_clock().now()
        self._state_pub.publish(String(data=state.value))
        self._publish_status(note)

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_t0).nanoseconds / 1e9

    def _publish_status(self, note: str = ''):
        self._status_pub.publish(String(data=json.dumps({
            'state': self._state.value, 'note': note, 'cycles_left': self._cycles_left,
            'history': self._history, 'fail_reason': self._fail_reason})))

    def _fail(self, reason: str):
        self._cmd_vel_pub.publish(Twist())
        self._fail_reason = reason
        self.get_logger().error(f'MISSION FAILED: {reason}')
        self._set_state(State.FAILED, reason)

    def _pose(self):
        """Localized (x, y, yaw) of base_footprint in map, or None."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _on_loc_status(self, msg: String):
        try:
            self._loc_status = json.loads(msg.data)
        except ValueError:
            pass

    def _on_joints(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self._joint_pos[n] = p

    # --------------------------------------------------------------------- arm

    def _arm_present(self) -> bool:
        return all(j in self._joint_pos for j in self._arm_pubs)

    def _arm_poses(self, param: str) -> list:
        flat = list(self.get_parameter(param).value)
        n = len(self._arm_pubs)
        return [flat[k:k + n] for k in range(0, len(flat) - n + 1, n)]

    def _arm_start(self, poses: list):
        """Queue joint-space poses; _arm_step() drives through them."""
        self._arm_queue = list(poses) if self._arm_present() else []
        self._arm_target = None

    def _arm_step(self) -> str:
        """'running' | 'done' | 'error:<reason>'. Each pose must be reached
        (|error| < arm.pose_tolerance on every joint) within arm.pose_timeout_sec."""
        if self._arm_target is None:
            if not self._arm_queue:
                return 'done'
            self._arm_target = self._arm_queue.pop(0)
            self._arm_t0 = self._now_s()
        for (j, pub), q in zip(self._arm_pubs.items(), self._arm_target):
            pub.publish(Float64(data=float(q)))
        tol = self.get_parameter('arm.pose_tolerance').value
        if all(abs(self._joint_pos.get(j, 1e9) - q) < tol for j, q in zip(self._arm_pubs, self._arm_target)):
            self._arm_target = None
            return 'running' if self._arm_queue else 'done'
        if self._now_s() - self._arm_t0 > self.get_parameter('arm.pose_timeout_sec').value:
            err = {j: round(self._joint_pos.get(j, float('nan')) - q, 3)
                   for j, q in zip(self._arm_pubs, self._arm_target)}
            return f'error:arm did not reach {self._arm_target} (joint errors {err})'
        return 'running'

    # --------------------------------------------------------------------- legs

    def _start_leg(self, state: State):
        leg = _LEG_OF_STATE[state]
        goal = tuple(self.get_parameter(_GOAL_PARAM[leg]).value)
        self._leg = {'name': leg, 'candidates': ml.candidate_goals(leg, goal, self._zones, self._fp),
                     'cand': 0, 'attempt': 0, 'phase': 'stow', 'retry_at': None}
        self._arm_start(self._arm_poses('arm.stow'))
        self._set_state(state, f'goal {self._leg["candidates"][0]}')

    def _nav2_ready(self) -> bool:
        if self._nav_active:
            return True
        if self._nav_query is None and self._bt_state.service_is_ready():
            self._nav_query = self._bt_state.call_async(GetState.Request())
        elif self._nav_query is not None and self._nav_query.done():
            res = self._nav_query.result()
            self._nav_active = res is not None and res.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE
            self._nav_query = None
        return self._nav_active

    def _send_leg_goal(self):
        leg = self._leg
        if not self._nav2_ready():
            if self._elapsed() > self.get_parameter('nav2_ready_timeout_sec').value:
                self._fail('Nav2 (bt_navigator) did not become active')
            else:
                self.get_logger().info('Waiting for Nav2 to become active...', throttle_duration_sec=10.0)
            return
        if self._goal_blocked(leg['candidates'][leg['cand']]):
            idx = self._next_free_candidate(leg)
            if idx != leg['cand'] and not self._goal_blocked(leg['candidates'][idx]):
                note = f'{leg["candidates"][leg["cand"]]} blocked in the costmap, using {leg["candidates"][idx]}'
                self.get_logger().warn(f'[{leg["name"]}] goal {note}')
                self._history.append([leg['name'], 'skip_blocked_goal', note])
                leg['cand'] = idx
        gx, gy, gyaw = leg['candidates'][leg['cand']]
        if not self._nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('navigate_to_pose action server not available yet.', throttle_duration_sec=5.0)
            return
        leg['attempt'] += 1
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)
        self._nav_result = None
        self._goal_handle = None
        self._goal_seq += 1
        leg['phase'] = 'navigating'
        leg['sent_at'] = self._now_s()
        leg['blocked_checks'] = 0
        leg['next_block_check'] = self._now_s() + 1.0
        leg['final_turn_done'] = False
        self.get_logger().info(f'[{leg["name"]}] attempt {leg["attempt"]}/{self._max_attempts}: '
                               f'goal ({gx:.2f}, {gy:.2f}, yaw {math.degrees(gyaw):.0f} deg)')
        seq = self._goal_seq
        self._nav_client.send_goal_async(goal).add_done_callback(lambda f: self._on_goal_response(f, seq))

    def _on_goal_response(self, future, seq):
        # responses/results of a goal that was cancelled or replaced are ignored
        if seq != self._goal_seq:
            return
        handle = future.result()
        if not handle.accepted:
            self._nav_result = (GoalStatus.STATUS_ABORTED, ml.NONE)
            return
        self._goal_handle = handle

        def on_result(f):
            if seq == self._goal_seq:
                self._nav_result = (f.result().status, f.result().result.error_code)
        handle.get_result_async().add_done_callback(on_result)

    def _goal_blocked(self, goal) -> bool:
        """The planner's own goal test: a lethal cell under the padded footprint at the goal."""
        return self._costmap is not None and bool(ml.lethal_cells(
            self._costmap, goal, self._fp, pad=self.get_parameter('mission.footprint_padding').value))

    def _run_leg(self, on_success):
        leg = self._leg
        if leg['phase'] == 'stow':
            arm = self._arm_step()
            if arm.startswith('error'):
                return self._fail(f'[{leg["name"]}] {arm[6:]}')
            if arm == 'done':
                leg['phase'] = 'send'
            return
        if leg['phase'] == 'escape':
            return self._escape_step(leg)
        if leg['phase'] == 'final_turn':
            return self._final_turn_step(leg)
        if leg['phase'] == 'wait_retry':
            if self._now_s() >= leg['retry_at']:
                leg['phase'] = 'send'
            return
        if leg['phase'] == 'send':
            return self._send_leg_goal()
        # navigating
        if self._nav_result is None and not leg['final_turn_done'] and self._start_final_turn(leg):
            return
        if self._nav_result is None and self._now_s() >= leg['next_block_check']:
            # A goal that became blocked (an obstacle sensed next to it) makes Nav2
            # replan and run recoveries until it gives up; end the attempt instead,
            # as GOAL_OCCUPIED, so the next try uses a free alternate goal.
            leg['next_block_check'] = self._now_s() + 1.0
            leg['blocked_checks'] = leg['blocked_checks'] + 1 if self._goal_blocked(
                leg['candidates'][leg['cand']]) else 0
            if leg['blocked_checks'] >= 2:
                self.get_logger().warn(f'[{leg["name"]}] goal {leg["candidates"][leg["cand"]]} is now blocked '
                                       'in the costmap; cancelling it.')
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
                self._goal_seq += 1
                self._nav_result = (GoalStatus.STATUS_ABORTED, ml.GOAL_OCCUPIED)
        timed_out = self._now_s() - leg['sent_at'] > self.get_parameter('mission.leg_timeout_sec').value
        if self._nav_result is None and not timed_out:
            return
        if self._nav_result is None:          # timeout: cancel and count the attempt
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._goal_seq += 1
            status, code = GoalStatus.STATUS_CANCELED, ml.NONE
        else:
            status, code = self._nav_result
        self._cmd_vel_pub.publish(Twist())
        goal = leg['candidates'][leg['cand']]
        pose = self._pose()
        arrived = pose is not None and ml.arrival_ok(
            goal, pose, self.get_parameter('mission.arrival_xy_tolerance').value,
            self.get_parameter('mission.arrival_yaw_tolerance').value)
        in_zone = pose is not None and ml.zone_ok(leg['name'], pose, self._fp, self._zones,
                                                  cross_margin=self.get_parameter('mission.zone_cross_margin').value)
        d = ml.decide_nav_result(status == GoalStatus.STATUS_SUCCEEDED, code, arrived, in_zone,
                                 leg['attempt'], self._max_attempts, timed_out=self._nav_result is None)
        pose_txt = f'({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f} deg)' if pose else 'unknown'
        if d.action == 'success':
            self.get_logger().info(f'[{leg["name"]}] ARRIVED (verified): pose {pose_txt}')
            self._history.append([leg['name'], 'arrived', pose_txt])
            return on_success()
        self.get_logger().warn(f'[{leg["name"]}] {d.reason}; pose {pose_txt}')
        self._history.append([leg['name'], d.action, d.reason])
        if d.action == 'fail':
            return self._fail(f'leg "{leg["name"]}" failed: {d.reason}')
        if d.action == 'retry_alternate':
            leg['cand'] = self._next_free_candidate(leg)
        leg['phase'] = 'wait_retry'
        leg['retry_at'] = self._now_s() + 3.0      # let the costmap settle before retrying
        if self._nav_result is not None and code == ml.START_OCCUPIED and pose is not None:
            self._start_escape(leg, pose)

    def _start_final_turn(self, leg) -> bool:
        goal, pose = leg['candidates'][leg['cand']], self._pose()
        if pose is None or math.hypot(pose[0] - goal[0], pose[1] - goal[1]) > \
                self.get_parameter('mission.final_turn_radius').value:
            return False
        if abs(ml.angle_diff(goal[2], pose[2])) <= self.get_parameter('mission.final_turn_min_yaw').value:
            return False
        if self._costmap is None or not ml.turn_in_place_safe(
                self._costmap, pose, goal[2], self._fp, self.get_parameter('mission.footprint_padding').value):
            return False
        leg['final_turn_done'] = True
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_seq += 1
        txt = f'at ({pose[0]:.2f}, {pose[1]:.2f}), {math.degrees(ml.angle_diff(goal[2], pose[2])):.0f} deg to go'
        self.get_logger().info(f'[{leg["name"]}] near the goal: turning in place to its heading ({txt}).')
        self._history.append([leg['name'], 'final_turn', txt])
        leg['phase'] = 'final_turn'
        leg['turn_t0'] = self._now_s()
        return True

    def _final_turn_step(self, leg):
        """Turn to the goal heading closed-loop, then let the normal arrival check decide."""
        goal, pose, twist = leg['candidates'][leg['cand']], self._pose(), Twist()
        timed_out = self._now_s() - leg['turn_t0'] > self.get_parameter('mission.final_turn_timeout_sec').value
        if pose is not None and not timed_out:
            err = ml.angle_diff(goal[2], pose[2])
            if abs(err) > math.radians(3.0):
                twist.angular.z = math.copysign(min(0.35, max(0.1, abs(err))), err)
        self._cmd_vel_pub.publish(twist)
        if twist.angular.z == 0.0:
            leg['phase'] = 'navigating'
            self._nav_result = (GoalStatus.STATUS_SUCCEEDED, ml.NONE)   # verified by arrival + zone checks

    def _start_escape(self, leg, pose):
        pad = self.get_parameter('mission.footprint_padding').value
        # Lethal cells under the start: clear them with the smallest motion. None
        # (the planner refused a start this costmap copy shows free): move to the
        # clearest reachable pose instead of a token 5 deg turn.
        objective = 'smallest' if self._costmap is not None and ml.lethal_cells(
            self._costmap, pose, self._fp, pad) else 'clearest'
        motion = None if self._costmap is None else ml.escape_motion(
            self._costmap, pose, self._fp, pad=pad, objective=objective)
        if motion is None:
            self.get_logger().warn(f'[{leg["name"]}] START_OCCUPIED: no collision-free escape motion found.')
            self._history.append([leg['name'], 'escape', 'none found'])
            return
        kind, amount = motion
        txt = f'turn {math.degrees(amount):.0f} deg' if kind == 'turn' else f'drive {amount:+.2f} m'
        self.get_logger().warn(f'[{leg["name"]}] START_OCCUPIED: escaping with {txt} before retrying.')
        self._history.append([leg['name'], 'escape', txt])
        leg['phase'] = 'escape'
        leg['escape'] = {'kind': kind, 'amount': amount, 'start': pose, 't0': self._now_s()}

    def _escape_step(self, leg):
        """Drive the escape motion closed-loop on the localized pose; then wait and retry."""
        e, pose, twist = leg['escape'], self._pose(), Twist()
        timed_out = self._now_s() - e['t0'] > self.get_parameter('escape_timeout_sec').value
        if pose is not None and not timed_out:
            if e['kind'] == 'turn':
                err = ml.angle_diff(e['start'][2] + e['amount'], pose[2])
                if abs(err) > math.radians(2.0):
                    twist.angular.z = math.copysign(min(0.3, max(0.1, abs(err))), err)
            else:
                sx, sy, syaw = e['start']
                done = (pose[0] - sx) * math.cos(syaw) + (pose[1] - sy) * math.sin(syaw)
                if abs(e['amount'] - done) > 0.01:
                    twist.linear.x = math.copysign(0.08, e['amount'] - done)
        self._cmd_vel_pub.publish(twist)
        if twist.angular.z == 0.0 and twist.linear.x == 0.0:
            if timed_out:
                self.get_logger().warn(f'[{leg["name"]}] escape motion timed out.')
            leg['phase'] = 'wait_retry'
            leg['retry_at'] = self._now_s() + 3.0

    def _next_free_candidate(self, leg) -> int:
        """Next candidate with no inscribed cost under the footprint (clear margin);
        failing that, the next one the planner would accept; failing that, the next."""
        n = len(leg['candidates'])
        order = [(leg['cand'] + k) % n for k in range(1, n + 1)]
        for idx in order:
            if self._costmap is None or ml.footprint_cost_ok(self._costmap, leg['candidates'][idx], self._fp):
                return idx
        for idx in order:
            if not self._goal_blocked(leg['candidates'][idx]):
                return idx
        return order[0]

    # --------------------------------------------------------------------- fsm

    def _tick(self):
        if self._enabled:
            getattr(self, f'_handle_{self._state.name.lower()}')()

    def _handle_start(self):
        # Wait for the simulation + EKF (odom->base_footprint) before turning to
        # look for markers. Nav2 is NOT awaited here: its global costmap needs
        # the map frame, which only exists after the marker fix.
        try:
            self._tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        self.get_logger().info('Mission started. Localizing from the start-zone markers...')
        self._set_state(State.LOCALIZE)

    def _handle_localize(self):
        if self._loc_status.get('initialized') and self._pose() is not None:
            self._cmd_vel_pub.publish(Twist())
            pose = self._pose()
            self.get_logger().info(f'Localized from markers: ({pose[0]:.2f}, {pose[1]:.2f}, '
                                   f'{math.degrees(pose[2]):.0f} deg), {self._loc_status.get("fixes", 0)} fixes.')
            return self._start_survey(pose)
        if self._elapsed() > self.get_parameter('localize_timeout_sec').value:
            return self._fail('no marker fix within localize_timeout_sec')
        # Turn in steps: rotate 1.2 s, then hold still 1.0 s. marker_localizer
        # ignores frames taken while turning (image/odometry timing errors), so
        # the pauses are when the markers are actually measured.
        twist = Twist()
        if self._elapsed() % 2.2 < 1.2:
            twist.angular.z = float(self.get_parameter('localize_spin_rate').value)
        self._cmd_vel_pub.publish(twist)

    def _start_survey(self, pose):
        heads = [math.radians(h) for h in self.get_parameter('mission.survey_headings').value]
        if not heads:
            return self._start_leg(State.TRAVEL_TO_EXCAVATION)
        # sweep from the end nearer the current heading
        if abs(ml.angle_diff(heads[-1], pose[2])) < abs(ml.angle_diff(heads[0], pose[2])):
            heads = heads[::-1]
        self._survey = {'heads': heads, 'k': 0, 'hold_since': None}
        self._set_state(State.SURVEY, f'{len(heads)} headings')

    def _handle_survey(self):
        sv, pose, twist = self._survey, self._pose(), Twist()
        if self._elapsed() > self.get_parameter('mission.survey_timeout_sec').value:
            self.get_logger().warn('Survey timed out; continuing with what was seen.')
            self._cmd_vel_pub.publish(twist)
            return self._start_leg(State.TRAVEL_TO_EXCAVATION)
        if pose is None:
            return self._cmd_vel_pub.publish(twist)
        err = ml.angle_diff(sv['heads'][sv['k']], pose[2])
        if abs(err) > math.radians(3.0) and sv['hold_since'] is None:
            rate = self.get_parameter('mission.survey_turn_rate').value
            twist.angular.z = math.copysign(min(rate, max(0.1, abs(err))), err)
            return self._cmd_vel_pub.publish(twist)
        self._cmd_vel_pub.publish(twist)
        if sv['hold_since'] is None:
            sv['hold_since'] = self._now_s()
        elif self._now_s() - sv['hold_since'] >= self.get_parameter('mission.survey_hold_sec').value:
            sv['k'] += 1
            sv['hold_since'] = None
            if sv['k'] >= len(sv['heads']):
                self._history.append(['survey', 'done', f'{len(sv["heads"])} headings'])
                return self._start_leg(State.TRAVEL_TO_EXCAVATION)

    def _handle_travel_to_excavation(self):
        def arrived():
            self._payload.start(self._now_s())
            self._arm_start(self._arm_poses('arm.dig_sequence'))
            self._set_state(State.EXCAVATION, 'simulated excavation')
        self._run_leg(arrived)

    def _handle_excavation(self):
        arm = self._arm_step()
        if arm.startswith('error'):
            return self._fail(f'excavation: {arm[6:]}')
        level = self._payload.tick_excavate(self._now_s())
        self._payload_pub.publish(Float32(data=level))
        if arm == 'done' and (self._elapsed() >= self.get_parameter('mission.excavation_duration_sec').value
                              or self._payload.is_full()):
            self.get_logger().info(f'Excavation step complete (SIMULATED payload={level:.2f}).')
            self._history.append(['excavation', 'simulated', f'payload {level:.2f}'])
            self._start_leg(State.TRAVEL_TO_CONSTRUCTION)

    def _handle_travel_to_construction(self):
        def arrived():
            self._payload.start(self._now_s())
            self._arm_start(self._arm_poses('arm.dump_sequence'))
            self._set_state(State.DUMP, 'simulated dump')
        self._run_leg(arrived)

    def _handle_dump(self):
        arm = self._arm_step()
        if arm.startswith('error'):
            return self._fail(f'dump: {arm[6:]}')
        level = self._payload.tick_dump(self._now_s())
        self._payload_pub.publish(Float32(data=level))
        if arm == 'done' and (self._elapsed() >= self.get_parameter('mission.dump_duration_sec').value
                              or self._payload.is_empty()):
            self.get_logger().info('Dump step complete (SIMULATED -- no sand is physically moved).')
            self._history.append(['dump', 'simulated', 'payload 0'])
            self._cycles_left -= 1
            self._start_leg(State.TRAVEL_TO_EXCAVATION if self._cycles_left > 0 else State.RETURN)

    def _handle_return(self):
        def arrived():
            self._set_state(State.COMPLETE, 'all legs verified')
        self._run_leg(arrived)

    def _handle_complete(self):
        if self._elapsed() < 0.15:
            self.get_logger().info(f'Mission complete. Legs: {self._history}')

    def _handle_failed(self):
        self._cmd_vel_pub.publish(Twist())


def main():
    rclpy.init()
    node = MissionStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
