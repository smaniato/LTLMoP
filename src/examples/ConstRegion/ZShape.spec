# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: False
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
SimulatedDipolar

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
ZShape.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
Start = p3
others = 
Goal = p4
r2 = p1, p4
r1 = p2, p3

Spec: # Specification in structured English
visit Goal
visit Start

