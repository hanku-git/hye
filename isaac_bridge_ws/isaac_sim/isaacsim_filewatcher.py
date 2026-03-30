import json
import os
import numpy as np
import omni.kit.app
from isaacsim.core.prims import SingleArticulation

CMD_FILE = "/tmp/isaac_joint_cmd.json"

class FileWatcher:
    def __init__(self):
        self._robot = None
        self._last_mtime = 0
        self._start_pos = None
        self._target_pos = None
        self._duration = 0.0
        self._elapsed = 0.0
        self._moving = False

    def on_update(self, e):
        dt = e.payload["dt"]

        if self._robot is None:
            try:
                self._robot = SingleArticulation("/World/m1013")
                self._robot.initialize()
                print("Robot ready")
            except:
                return

        # 새 명령 확인
        if os.path.exists(CMD_FILE):
            try:
                mtime = os.path.getmtime(CMD_FILE)
                if mtime > self._last_mtime:
                    self._last_mtime = mtime
                    with open(CMD_FILE) as f:
                        cmd = json.load(f)
                    cur = self._robot.get_joint_positions()
                    self._start_pos = cur.tolist()
                    self._target_pos = cmd["target"]
                    self._duration = max(cmd["duration"], 0.1)
                    self._elapsed = 0.0
                    self._moving = True
            except:
                pass

        # 보간 실행
        if self._moving and self._target_pos is not None:
            self._elapsed += dt
            t = min(self._elapsed / self._duration, 1.0)
            interp = [s + (g - s) * t
                      for s, g in zip(self._start_pos, self._target_pos)]
            self._robot.set_joint_positions(np.array(interp, dtype=np.float64))
            if t >= 1.0:
                self._moving = False

watcher = FileWatcher()
sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    watcher.on_update, name="joint_cmd_watcher"
)
print("FileWatcher ready - smooth interpolation at sim rate")
