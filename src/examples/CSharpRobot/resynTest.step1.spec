# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1
IDLE_MODE, 1
EXPLORE_MODE, 1
requestExplore, 1
EXPLORE_SETUP_MODE, 1
RESYNTHESIZE_MODE, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
pioneer loop

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
resynTest.update1.regions

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
r3 = p1
r1 = p3
r2 = p2
others = 

Spec: # Specification in structured English
robot starts in r2 with not resynthesize and EXPLORE_SETUP_MODE and not requestExplore and IDLE_MODE and not RESYNTHESIZE_MODE and not EXPLORE_MODE
# This examples demonstrates the use of a few locative prepositions
group region is r1, r2, r3

# unexplored region stuff
group unexplored_rooms is r2
add to unexplored_rooms if and only if you are sensing (IDLE_MODE and region_added)



IDLE_MODE is set on (not BUSY_EXPLORING and not region_added and not RESYNTHESIZE_MODE and not EXPLORE_SETUP_MODE and not EXPLORE_MODE) and reset on (region_added and not BUSY_EXPLORING)
RESYNTHESIZE_MODE is set on (region_added and not BUSY_EXPLORING) and reset on (not region_added and not BUSY_EXPLORING and not resynthesize)
EXPLORE_SETUP_MODE is set on (not region_added and not BUSY_EXPLORING and not resynthesize) and reset on any unexplored_rooms
EXPLORE_MODE is set on (any unexplored_rooms and not IDLE_MODE and not EXPLORE_SETUP_MODE) and reset on (not NEW_DOORWAY and not BUSY_EXPLORING)

if you are activating (not EXPLORE_SETUP_MODE and not EXPLORE_MODE and not RESYNTHESIZE_MODE and not BUSY_EXPLORING) then visit all region
if you are activating RESYNTHESIZE_MODE then do resynthesize
visit all unexplored_rooms if and only if you are activating EXPLORE_SETUP_MODE
if you are activating EXPLORE_MODE then stay there
do requestExplore if and only if you are sensing start of EXPLORE_MODE

