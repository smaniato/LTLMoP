# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1
requestExplore, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
DemoDayNoVICON

Customs: # List of custom propositions
explore_room_done
explore
needs_resynthesis

RegionFile: # Relative path of region description file
finalloop.update2.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
NEW_DOORWAY, 1
BUSY_EXPLORING, 1
region_added, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
newRegion1 = p4
r3 = p1
r1 = p3
r2 = p2
others = 

Spec: # Specification in structured English
robot starts in newRegion1 with resynthesize and explore_room_done and requestExplore and explore and not needs_resynthesis
### assumptions ###
### main specification ###
#group patrol_locations is empty
group patrol_locations is newRegion1, r1, r2, r3
### exploration settings ###
always do explore
add to patrol_locations if and only if you are activating explore_room_done
if you are activating explore_room_done then do resynthesize
# --------- begin auto-generated exploration spec -------
# keep track of places you need to explore, at all times (TODO: BFS vs DFS?)
group unexplored_rooms is newRegion1
add to unexplored_rooms if and only if you are sensing start of region_added
remove from unexplored_rooms if and only if you are activating explore_room_done
# resynthesize as appropriate (if we are not exploring, we can delay)
needs_resynthesis is set on region_added and reset on resynthesize
if you are activating explore and needs_resynthesis then do resynthesize
# make sure we visit and explore the new places (if told to)
if you are activating explore and not region_added then visit all unexplored_rooms and requestExplore and explore_room_done at least once
if you are not in any unexplored_room then do not requestExplore
if you were activating requestExplore or you are activating requestExplore then stay there
# environment assumptions
if you are activating requestExplore then infinitely often do BUSY_EXPLORING
if you are activating requestExplore then infinitely often do not BUSY_EXPLORING
# react instantly to UKR-related sensors
#if you are sensing region_added then stay there
if you are sensing region_added or you were sensing region_added then stay there
#if you are activating explore_room_done then stay there
do explore_room_done if and only if end of BUSY_EXPLORING
if you are not sensing region_added then visit all patrol_locations

