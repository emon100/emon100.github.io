"""Audit parser tests: state-machine events must not impersonate physical onset."""
import copy, json, unittest
from pathlib import Path
from launch_timing_audit import measure_recording
import core

class TimingAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data=json.loads((Path(__file__).parent/'results.json').read_text())
        cls.recording=cls.data['tasks']['stationary_kickflip']['comparisons']['untuned']
    def test_real_recorded_launch_distinguishes_command_actuation_and_motion(self):
        r=measure_recording(self.recording,self.data['config'],core.UNTUNED,core.PARAMETERS,'stationary_kickflip','untuned')
        self.assertEqual(r['rollPhaseEntry']['t'],.1)
        self.assertEqual(r['firstNonzeroRollCommand']['t'],.1)
        self.assertEqual(r['firstRealizedRollTorque']['t'],.12)
        self.assertEqual(r['firstRollMotion']['t'],.18)
        self.assertEqual(r['firstNonzeroRollCommand']['heightCG'],self.recording['frames'][5]['pos'][2])
    def test_synthetic_phase_with_zero_rate_does_not_count_as_a_roll_command(self):
        # Deliberately altered fixture tests only extraction semantics, not dynamics.
        fixture=copy.deepcopy(self.recording)
        for f in fixture['frames']:
            if f['controlObservationTime']<.2 and f['goal']['phase']=='flip':f['goal']['commandRollRate']=0.
        r=measure_recording(fixture,self.data['config'],core.UNTUNED,core.PARAMETERS,'stationary_kickflip','synthetic-parser-fixture')
        self.assertEqual(r['rollPhaseEntry']['t'],.1)
        self.assertEqual(r['firstNonzeroRollCommand']['t'],.2)
        self.assertEqual(r['firstRealizedRollTorque']['t'],.12)

if __name__=='__main__':unittest.main()
