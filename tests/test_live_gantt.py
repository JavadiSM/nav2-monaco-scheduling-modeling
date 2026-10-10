import unittest
from tools.live_bridge.viewer import LiveGantt

class LiveGanttHistoryTests(unittest.TestCase):
    def test_pruning_preserves_visible_pixels_and_active_completion(self):
        view=LiveGantt({'vehicle_count':1,'core_layouts':{'vehicle':['A7'],'server':['A15']}})
        view.origin=0;view.tick=2500
        view.jobs={1:dict(task='bt_tick',device=0,core=0,finished=True),
                   2:dict(task='control_iteration',device=0,core=0,finished=True),
                   3:dict(task='local_costmap_update',device=0,core=0),
                   4:dict(task='planning_request',device=0)}
        view.segments=[(1,100,101),(2,2100,2110)]
        view.active={3:2400};view.cooling_spans=[(0,200,300)]
        before=view.image(2.).tobytes();view.prune_before(2000)
        self.assertEqual(view.image(2.).tobytes(),before)
        self.assertNotIn(1,view.jobs);self.assertIn(4,view.jobs)
        view.consume(dict(kind='budget_complete',tick=2501,job_id=3))
        view.consume(dict(kind='finish',tick=2501,job_id=3))
        self.assertTrue(view.jobs[3]['finished']);self.assertNotIn(3,view.active)

if __name__=='__main__':unittest.main()
