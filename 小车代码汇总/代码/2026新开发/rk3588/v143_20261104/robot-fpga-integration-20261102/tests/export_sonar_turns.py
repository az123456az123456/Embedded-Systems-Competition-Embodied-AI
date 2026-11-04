"""Production sonar policy outputs for the real Gateway fixture (no hardware)."""
import json
import math
import sys
from acg138_frontier.semantic_sonar_walk import SemanticSonarWalk
scan=(tuple([math.inf]*360),-math.pi,math.tau/360,.05,10.)
commands=[]
for ranges in [(.8,.5,.6),(.09,.6,.4),(.09,.4,.6),(.5,.09,.7),(.5,.7,.09)]:
    walk=SemanticSonarWalk()
    commands.append(walk.update(scan,(0.,0.,0.),ranges,(1,1,1)))
    commands.append(walk.update(scan,(0.,0.,.2),(.4,.4,.4),(2,2,2)))
    for yaw in (-.15,.15):
        commands.append(walk.update(scan,(.1,0.,yaw),(.4,.4,.4),(3,3,3)))
assert {c['direction'] for c in commands if c['op']=='rotate'}=={'cw','ccw'}
assert all(c['motion_requested'] for c in commands)
walk=SemanticSonarWalk()
wall=[]
for i in range(720):
    c=math.cos(-math.pi+i*math.tau/720)
    wall.append(.18/c if c>0 else math.inf)
back=walk.update((tuple(wall),-math.pi,math.tau/720,.05,12.),(0.,0.,0.),(.09,.6,.4),(1,1,1))
assert back['motion_requested'] and back['throttle']<0
commands.append(back)
turn=walk.update(scan,(-.06,0.,0.),(.2,.6,.4),(2,2,2))
assert turn['op']=='rotate' and turn['direction']=='ccw'
commands.append(turn)
forward=walk.update(scan,(-.06,0.,1.2),(.4,.6,.4),(3,3,3))
assert forward['throttle']>0
commands.append(forward)
json.dump(commands,sys.stdout)
