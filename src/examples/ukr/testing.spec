# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
explore, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions
resynthesize

RegionFile: # Relative path of region description file
exp10.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
region_added, 1
explore_done, 1
force_explore, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
classroom1 = p4
others = 
hall2 = p2
office1 = p1
hall1 = p3

Spec: # Specification in structured English
Environment starts with false
Robot starts with false

group rooms is hall1, hall2, office1, classroom1
if you are not sensing force_explore and you are not sensing region_added then visit all rooms

group unexplored_rooms is empty
add to unexplored_rooms if and only if you are sensing region_added

if you are sensing force_explore and not region_added then visit all unexplored_rooms and explore and explore_done at least once

if you are activating explore then infinitely often explore_done
if you were activating explore or you are activating explore then stay there

# react instantly to mode-switch
if you are sensing start of force_explore or end of force_explore then stay there
if you are sensing region_added or you were sensing region_added then stay there

do resynthesize if and only if you are sensing start of force_explore

