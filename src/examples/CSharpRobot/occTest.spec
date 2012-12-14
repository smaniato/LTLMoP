# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1
IDLE_MODE, 0
EXPLORE_MODE, 0
requestExplore, 1
EXPLORE_SETUP_MODE, 0
RESYNTHESIZE_MODE, 0

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
pioneer loop

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
loop.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
LIDAR, 0
ARTAG64, 0
ARTAG103, 0
NEW_DOORWAY, 1
BUSY_EXPLORING, 1
exploreDone, 0
region_added, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
r1 = p3
r2 = p2
r3 = p1

Spec: # Specification in structured English
# This examples demonstrates the use of a few locative prepositions
group region is r1, r2, r3

Environment starts with false
Robot starts with false


if you are sensing (NEW_DOORWAY and not BUSY_EXPLORING) then do requestExplore
if you are sensing BUSY_EXPLORING then stay

if you are sensing (not NEW_DOORWAY and not BUSY_EXPLORING) then visit all region

