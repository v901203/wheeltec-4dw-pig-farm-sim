import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import math
import unittest
from unittest.mock import Mock,patch
import numpy as np
from continuous_env import ContinuousJunctionEnv,exit_measurement,completed_exit
from continuous_common import convert_observations,contract
from tasks import TASKS,CENTRE_Y


class ContinuousTests(unittest.TestCase):
    def test_conversion_erases_phase_and_preserves_task_geometry(self):
        x=np.zeros((2,161),dtype=np.float32)
        x[:,152]=1.;x[:,159]=1.;x[:,155]=.2
        y=convert_observations(x,'main_to_left')
        np.testing.assert_array_equal(y[:,151:155],0.)
        np.testing.assert_allclose(y[:,157],.5)
        np.testing.assert_allclose(y[:,155],.2)
        np.testing.assert_array_equal(x[:,152],1.)
        reverse=convert_observations(x,'left_to_right_reverse')
        np.testing.assert_array_equal(reverse[:,160],1.)
    def test_exit_requires_measured_depth_and_stable_alignment(self):
        self.assertEqual(completed_exit(0.,0.,0.,(0.,0.),2),0)
        self.assertEqual(completed_exit(1.,0.,0.,(0.,0.),2),3)
        self.assertEqual(completed_exit(1.,.2,0.,(0.,0.),2),0)
        self.assertEqual(completed_exit(1.,0.,.4,(0.,0.),2),0)
        depth,_=exit_measurement(np.array([1.,0.]),math.pi)
        self.assertAlmostEqual(depth,1.)
    def test_model_action_not_stopped_by_fsm_phase_or_failure(self):
        env=object.__new__(ContinuousJunctionEnv)
        env.cfg=Mock()
        env.teacher=Mock()
        env.teacher.command.side_effect=AssertionError('FSM must not run after takeover')
        env.pose=np.array([0.,CENTRE_Y-.2,math.pi/2])
        env.task=TASKS[0];env.previous=np.zeros(2)
        env.geometry_valid=True;env.cached_context=np.zeros(10)
        env.skill_steps=0;env.exit_frames=0;env.geometry_hold=0
        env.continuous_stall=0;env.best_route_error=env.route_error(env.pose)
        env.exit_depth=0.;env.exit_lateral=0.;env.exit_heading=1.
        env.skill_max_steps=300;env.dt=.2;env.active=True
        env._prepare_decision=Mock();env._features=Mock()
        def transition(instance, action):
            np.testing.assert_allclose(action,[.4,.3])
            instance.pose[1]+=.01
            return np.zeros(151),0.,False,False,dict(collision=False,off_route=False,
                wrong_heading=False,safety_blocked=False,executed_action=[.08,.18],task=instance.task.name)
        with patch('continuous_env.RouteEnv.step',transition),patch('continuous_env.incoming_corridor',return_value=None):
            obs,reward,done,truncated,info=env.step([.4,.3])
        self.assertEqual(obs.shape,(161,));self.assertFalse(done)
        env.teacher.command.assert_not_called()
        self.assertEqual(info['route_state'],'CONTINUOUS_JUNCTION')
    def test_contract_is_distinct(self):
        self.assertEqual(contract()['schema'],'pig_pen_continuous_junction_v1')

if __name__=='__main__': unittest.main()
