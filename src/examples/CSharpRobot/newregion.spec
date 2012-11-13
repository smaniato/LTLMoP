# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
New Region Exploration

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
loop.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
LIDAR, 0
ARTAG64, 0
ARTAG103, 0
NEWDOOR, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
bedroom = p3
hallway = p2
kitchen = p1
others = 

Spec: # Specification in structured English
group livingspace is kitchen,bedroom
Robot starts in any livingspace

#if you are activating ARTAG64 then stay
#if you are activating ARTAG103 then stay
#if you are not activating ARTAG64 and you are not activating ARTAG103 then visit all regions
#visit r1
#visit r2
#visit r3
#visit r4

