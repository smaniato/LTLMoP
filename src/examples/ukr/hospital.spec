# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
explore_room, 1
raise_flag, 1

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
newmap

Customs: # List of custom propositions
resynthesize
explore
needs_resynthesis

RegionFile: # Relative path of region description file
hospital2.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
region_added, 1
explore_room_done, 1
carrying_chart, 1
room_category_OR, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
hall2 = p3
hall3 = p2
office = p1
hall1 = p4

Spec: # Specification in structured English
### assumptions ###

Environment starts with not explore_room_done and not region_added
Robot starts with explore

### main specification ###

#group patrol_locations is empty

group patrol_locations is hall1, hall2, hall3


do raise_flag if and only if you are sensing carrying_chart and you are in office

### exploration settings ###

do explore if and only if you are not sensing carrying_chart

add to patrol_locations if and only if you are sensing explore_room_done and room_category_OR

if you are sensing explore_room_done and room_category_OR then do resynthesize

if you are sensing start of carrying_chart or end of carrying_chart then stay there









# --------- begin auto-generated exploration spec -------

# keep track of places you need to explore, at all times (TODO: BFS vs DFS?)
group unexplored_rooms is empty
add to unexplored_rooms if and only if you are sensing start of region_added
remove from unexplored_rooms if and only if you are sensing explore_room_done

# resynthesize as appropriate (if we are not exploring, we can delay)
needs_resynthesis is set on region_added and reset on resynthesize
if you are activating explore and needs_resynthesis then do resynthesize

# make sure we visit and explore the new places (if told to)
if you are activating explore and not region_added then visit all unexplored_rooms and explore_room and explore_room_done at least once
if you are not in any unexplored_room then do not explore_room

if you were activating explore_room or you are activating explore_room then stay there

# environment assumptions
if you are activating explore_room then infinitely often do explore_room_done
if you were not activating explore_room then do not explore_room_done

# react instantly to UKR-related sensors
if you are sensing region_added or you were sensing region_added then stay there
if you are sensing explore_room_done or you were sensing explore_room_done then stay there




######################

#if you are not sensing carrying_chart or region_added then visit all patrol_locations

#if you are sensing carrying_chart and not region_added then visit office

