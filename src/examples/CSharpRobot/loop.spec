# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1
IDLE_MODE, 1
EXPLORE_MODE, 1
requestExplore, 0
CHASE_BEAR, 0
EXPLORE_SETUP_MODE, 1

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
regionAdded, 1
foundBear, 0
killedBear, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
r1 = p3
r2 = p2
r3 = p1

Spec: # Specification in structured English
# This examples demonstrates the use of a few locative prepositions
group region is r1, r2, r3
Robot starts in any region
Robot starts with false and IDLE_MODE

IDLE_MODE is set on (not NEW_DOORWAY and not BUSY_EXPLORING and not regionAdded) and reset on (regionAdded and not BUSY_EXPLORING)
EXPLORE_SETUP_MODE is set on (regionAdded and not BUSY_EXPLORING) and reset on (not NEW_DOORWAY and BUSY_EXPLORING and not regionAdded)
EXPLORE_MODE is set on (not NEW_DOORWAY and BUSY_EXPLORING and not regionAdded) and reset on (not NEW_DOORWAY and not BUSY_EXPLORING)

if you are activating (IDLE_MODE and not EXPLORE_SETUP_MODE and not EXPLORE_MODE) then visit all region
if you activated EXPLORE_SETUP_MODE then do resynthesize
if you are activating EXPLORE_MODE then stay there

