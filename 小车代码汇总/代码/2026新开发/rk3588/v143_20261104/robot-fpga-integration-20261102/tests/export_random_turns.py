"""Export real random-walk commands for the Runtime/Gateway integration test."""
import json
import math
import sys
from acg138_frontier.semantic_random_walk import SemanticRandomWalk

scan = (tuple(math.inf for _ in range(360)), -math.pi, math.tau / 360, .05, 10.)
commands = []
for seed in range(12):
    ranges=tuple(.32 if abs(-math.pi+i*math.tau/360)<math.radians(20) else 3. for i in range(360))
    command = SemanticRandomWalk(seed=seed).update((ranges,*scan[1:]), (0., 0., 0.))
    if command['op'] == 'rotate':
        commands.append(command)
assert {c['direction'] for c in commands} == {'cw', 'ccw'}
assert all(0 < c['speed'] <= .25 for c in commands)
for error in (-math.pi, -1., -.4, -.1, 0., .1, .4, 1., math.pi):
    walk = SemanticRandomWalk(seed=1)
    walk.state = walk.DRIVING_SEGMENT
    walk.target_yaw = error; walk.start_xy = (0., 0.); walk.segment_length = .6
    command = walk.update(scan, (0., 0., 0.))
    assert command['op'] == 'drive' and abs(command['steer']) <= .15
    commands.append(command)
json.dump(commands, sys.stdout)
