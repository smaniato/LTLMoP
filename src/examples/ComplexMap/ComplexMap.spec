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
Untitled configuration

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
ComplexMap.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
GoToField, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
field = p3
b3 = p5
b1 = p7
b2 = p6
others = p1

Spec: # Specification in structured English

if you are not sensing GoToField then visit b1
if you are not sensing GoToField then visit b2
if you are not sensing GoToField then visit b3

visit field if and only if you are sensing GoToField

