# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
NotifyReachedR1, 1

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
DipolarRRT

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
..\..\..\..\..\..\..\Downloads\fgd.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
Go, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r0 = p5
r1 = p4
others = p1, p6, p7, p8

Spec: # Specification in structured English
#visit r1 if and only if you are not sensing Go
#visit r3 if and only if you are sensing Go

#do NotifyReachedR1 if and only if you are in r1

visit r1
visit r0

