# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1

CompileOptions:
convexify: True
parser: nltk
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
Simulation

Customs: # List of custom propositions
carrying_flammable
carrying_hot
_add_to_items

OpenWorld: # List of OpenWorld propositions/correspondences
items
items->hallways

RegionFile: # Relative path of region description file
laboratory.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
newItem, 1
flammable, 1
hot, 1
pipette, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
hallN = p15, p16
storageRoom = p1
r6 = p2
r1 = p7
closet = p12
r5 = p3
r3 = p5
hallC = p11
lounge = p8
r2 = p6
others = 
hallW = p13, p14
r4 = p4

Spec: # Specification in structured English
Group Items is pipette
Group StorageRooms is storageRoom

Group HazardousProperties is flammable, hot

Group LabHallways is hallW, hallC, hallN

Items correspond to StorageRooms

### Initial Conditions ###

environment starts with false
robot starts in (closet) with false

### Mission tasks ###

# If you are not carrying any items, patrol the laboratory's hallways:
If you are not sensing any Items then go to hallW
If you are not sensing any Items then go to hallN

# Items that you are carrying must eventually be dropped off in the storage room
If you are sensing any Items then go to the corresponding StorageRoom

# Be aware of any hazardous items you may be carrying
carrying_flammable is toggled on (newItem and flammable and resynthesize and not hot and not carrying_hot)
carrying_hot is toggled on (newItem and hot and resynthesize and not flammable and not carrying_flammable)

# The same item cannot have a combination of hazardous properties
Always not (flammable and hot)

### Open-world settings ###

# If there is no reason not to, accept new items
If you are sensing (newItem and not hot and not flammable) then add to group Items
If you are sensing (newItem and flammable) and you were not activating carrying_hot then add to group Items
#If you are sensing (newItem and hot) and you were not activating carrying_flammable then add to group Items

# However, carrying hazardous combinations of items is strictly prohibited
If you are sensing (newItem and flammable) and you were activating carrying_hot then do not resynthesize
If you are sensing (newItem and hot) and you were activating carrying_flammable then do not resynthesize

# Various open-world mechanics
If you are not sensing newItem then do not resynthesize
Do resynthesize if and only if you are activating _add_to_items

