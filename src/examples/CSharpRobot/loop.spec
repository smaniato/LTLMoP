# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1
explore, 1

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

if you are not sensing BUSY_EXPLORING and you are not sensing NEW_DOORWAY then visit r1
if you are not sensing BUSY_EXPLORING and you are not sensing NEW_DOORWAY then visit r3
if you are sensing NEW_DOORWAY and you are not sensing BUSY_EXPLORING then do explore
if you are sensing BUSY_EXPLORING then stay
#if you are not sensing NEW_DOORWAY then do resynthesize

