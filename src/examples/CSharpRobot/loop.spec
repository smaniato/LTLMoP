# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

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


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r3 = p1
r1 = p3
r2 = p2
others = 

Spec: # Specification in structured English
# This examples demonstrates the use of a few locative prepositions
group region is r1, r2, r3
#Env starts with false
Robot starts in any region

visit r1
visit r3
#if you are in r2 then visit r1
#if you are in r1 then visit r2

