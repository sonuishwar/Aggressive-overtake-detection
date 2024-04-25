# imp

import sys
try:
    sys.path.append("./libs/carla-0.9.15-py3.7-linux-x86_64.egg")
except IndexError:
    pass

import scenic
# import verfai
# param map = localPath('../../assets/maps/CARLA/Town03.xodr')  
# param carla_map = 'Town03'
param map = localPath('maps/CARLA/Town03.xodr')  # or other CARLA map that definitely works

param carla_map = 'Town03'
param weather = 'ClearNoon' 
model scenic.domains.driving.model
model scenic.simulators.carla.model


MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = 4 #VerifaiRange(2, 4)

param ADV_DIST = VerifaiRange(-25, -10)
param ADV_SPEED = VerifaiRange(7, 10)

BYPASS_DIST = [15, 2]
INIT_DIST = 50
TERM_TIME = 5

# param EGO_SPEED = 6
initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
# INIT_DIST = 50
# BYPASS_DIST = [15, 4]
# TERM_TIME = 10

# param ADV_DIST = -15
# param ADV_SPEED = 8

behavior adversaryBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)
    interrupt when withinDistanceToAnyObjs(self, BYPASS_DIST[0]):
        fasterLaneSec = self.laneSection.fasterLane
        do LaneChangeBehavior(
                laneSectionToSwitch=fasterLaneSec,
                target_speed=globalParameters.ADV_SPEED)
        do FollowLaneBehavior(
                target_speed=globalParameters.ADV_SPEED,
                laneToFollow=fasterLaneSec.lane) \
            until (distance to adversary) > BYPASS_DIST[1]
        slowerLaneSec = self.laneSection.slowerLane
        do LaneChangeBehavior(
                laneSectionToSwitch=slowerLaneSec,
                target_speed=globalParameters.ADV_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for TERM_TIME seconds
        terminate 


ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    

adversary = new Car following roadDirection for globalParameters.ADV_DIST,
    with blueprint MODEL,
    with behavior adversaryBehavior()
    
# require adversary can see ego

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is not None)