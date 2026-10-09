"""Explicit lane queues for the fixed-mapping validation scheduler."""
from collections import defaultdict


class QueuePolicy:
    def _rebuild_core_queues(self):
        lanes = defaultdict(list)
        for job_id in self.task_nodes:
            job = self.state[job_id]
            if job['status'] not in ('completed', 'dropped'):
                lanes[job['device_id'], job['core_id']].append(job_id)
        self.core_queues = dict(lanes)
        self.core_running = {(device_id, core_id): None
                             for device_id, device in self.devices.items()
                             for core_id in device.cores}

    def _queue_head(self, lane, time_s):
        def key(job_id):
            return (-int(self._is_hi(job_id)), self.rank[job_id], job_id)
        waiting = (n for n in self.core_queues.get(lane, ())
                   if self.state[n]['status'] == 'planned'
                   and self._dependencies_satisfied(n, time_s))
        return min(waiting, key=key, default=None)

    def _dispatch_device(self, device_id, time_s):
        for core_id in sorted(self.devices[device_id].cores):
            lane = device_id, core_id
            if self.core_running[lane] is None:
                job_id = self._queue_head(lane, time_s)
                if job_id is not None:
                    self._try_start(job_id, time_s)
