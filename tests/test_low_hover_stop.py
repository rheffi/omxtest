"""Open-gripper low hover, no prompt or return; all devices are fake."""
import contextlib
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/picking'))
import omx_pick as app
import pick_core as core
import pick_robot as robot
import pick_vision as vision
from test_omx_picking import fixture, FakeBus


class LowHoverTest(unittest.TestCase):
    def test_low_hover_skips_high_ik_and_stops_open_before_contact(self):
        cfg, cal, target, saved = fixture()
        cfg['unfold'] = dict(elbow_pulse=2418, shoulder_pulse=1856)
        saved['shelf_approach'] = [2050,2000,2100,3000,2048]
        solve = Mock(return_value=[2048]*5)
        plan = core.build_plan(cfg,cal,target,saved,solve,hover_only=True,
                               from_folded=True,final_pick_z_mm=42,hover_stop_mm=5)
        self.assertEqual([s['name'] for s in plan['stages']],
                         ['ELBOW_FIRST','SHOULDER_SECOND','HIGH','OPEN','APPROACH','LOW_HOVER'])
        self.assertEqual(plan['low_hover_xyz_mm'],(190,0,47))
        solve.assert_called_once_with((190,0,47))
        self.assertEqual(plan['stages'][3]['target'],cfg['gripper']['open_pulse'])

    def test_live_cli_has_no_move_prompt_opens_gripper_and_holds_final_target(self):
        cfg, cal, target, saved = fixture()
        cfg['classes']['milk']['grasp_reference_pulses']=[2048]*5
        saved['shelf_approach']=[2048]*5
        bus=FakeBus()
        camera=Mock(__enter__=Mock(),__exit__=Mock(return_value=False))
        frame=SimpleNamespace(shape=(800,1000,3))
        args=app.arguments(['--class','milk','--hover','--live','--hover-mm','3'])
        output=io.StringIO()
        with patch.object(app,'calibration_for',return_value=(cal,Path('unused'))), \
             patch.object(app,'saved_for',return_value=saved), \
             patch.object(app,'solver_for',return_value=lambda xyz:[2000+round(xyz[2])]*5), \
             patch.object(vision,'Detector'),patch.object(vision,'Camera',return_value=camera), \
             patch.object(vision,'capture_target',return_value=(target,frame,0)), \
             patch.object(robot,'DynamixelBus',return_value=bus),patch.object(robot.time,'sleep'), \
             patch('builtins.input',side_effect=AssertionError('Unexpected MOVE prompt')), \
             contextlib.redirect_stdout(output):
            app.run_pipeline(args,SimpleNamespace(name='arm_a',port=Path('/dev/fake')),cfg)
        self.assertEqual([bus.read(i,116,4) for i in robot.ARM_IDS],[2033]*5)
        self.assertEqual(bus.read(16,116,4),cfg['gripper']['open_pulse'])
        self.assertTrue(all(bus.read(i,64,1)==1 for i in [*robot.ARM_IDS,16]))
        self.assertNotIn('[ARM] RETURN_HIGH',output.getvalue())
        self.assertNotIn('[GRIPPER] CLOSE',output.getvalue())
        self.assertIn('[HOVER HOLD]',output.getvalue())

    def test_full_pick_and_legacy_high_hover_keep_existing_sequences(self):
        cfg, cal, target, saved = fixture()
        solve=lambda xyz:[2048]*5
        plan=core.build_plan(cfg,cal,target,saved,solve)
        names=[s['name'] for s in plan['stages']]
        self.assertLess(names.index('CLOSE'),names.index('LIFT'))
        self.assertLess(names.index('DROP_RELEASE'),names.index('RELEASE'))
        self.assertEqual(names[-1],'RETURN_HIGH')
        high=core.build_plan(cfg,cal,target,saved,solve,hover_only=True)
        self.assertEqual(high['stages'][-1]['name'],'RETURN_HIGH')
        self.assertTrue(all(s['kind']=='arm' for s in high['stages']))


if __name__=='__main__':
    unittest.main()
