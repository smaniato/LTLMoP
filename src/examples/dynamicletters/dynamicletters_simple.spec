# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1

CompileOptions:
convexify: True
parser: nltk
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
basicsim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
slurp_hospital.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
letter1, 1
letter2, 1
newLetter, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
hallN = p15, p16
r5 = p2
r6 = p1
r1 = p6
closet = p12
r3 = p4
mailRoom = p7
hallC = p11
lounge = p8
r2 = p5
others = 
hallW = p13, p14
r4 = p3

Spec: # Specification in structured English
#### Group definitions ####

# Letters that we can detect
Group Letters is letter1, letter2

# Regions that each letter needs to end up in.
Group Destinations is r1, r2

# Regions to patrol if you are not carrying letters
Group PatrolRooms is mailRoom, hallW, hallN

# C(letters) = {Destinations}
Letters correspond to Destinations

#robot starts in mailRoom with false
#environment starts with false

#### Spec-rewriting and resynthesis mechanics ####

# This is only triggered if we see a letter we haven't ever seen before
If you are sensing newLetter then add to group Letters and resynthesize

#### Letter delivery specification ####
if you are sensing any Letters and you are not sensing newLetter then go to the corresponding Destination

# No spurious pickups
#Do pick_up if and only if you are sensing any Letters

# Go back to the mailroom if we have nothing else to do
If you are not sensing any Letters and you are not sensing newLetter then go to mailRoom

# Environment assumption: Be nice don't show us letters all the time
#Infinitely often not newLetter

# F/S stuffs
If you are sensing newLetter then stay there

