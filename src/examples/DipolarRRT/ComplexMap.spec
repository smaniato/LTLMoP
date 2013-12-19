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
CreateRRT

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
Fall13FinalDemo_adjusted.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
end = p7
r1 = p5
r2 = p3
start = p2
middle = p6
others = p1

Spec: # Specification in structured English
visit start
visit middle
visit end

