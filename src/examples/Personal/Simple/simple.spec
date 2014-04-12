# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
fastslow: False
decompose: True

CurrentConfigName:
BasicExperiment

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
simple.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p3
r2 = p2
others = p4, p5, p6

Spec: # Specification in structured English
go to r1

