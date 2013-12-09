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
DipolarSimulation

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
ComplexMap.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p1, p3
r5 = p3
r1 = p6
r2 = p5
r3 = p4
others = 

Spec: # Specification in structured English
visit r1
visit r3
visit r5

