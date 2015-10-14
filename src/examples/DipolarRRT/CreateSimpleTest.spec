# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
CreateRRT

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
CreateSimpleTest.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
r1 = p3
r2 = p2
r3 = p1

Spec: # Specification in structured English
visit r1
visit r3

