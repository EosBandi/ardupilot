# This directory can be used for vendor specific DSDL for ArduPilot

Data type IDs must not collide with those in the DroneCAN DSDL submodule
(modules/DroneCAN/DSDL) within the same namespace root.

Current contents:

- `ardupilot/gnss/20008.Integrity.uavcan` - GNSS jamming/spoofing/authentication
  state, mirroring the MAVLink GNSS_INTEGRITY enums. Pending upstream inclusion
  in DroneCAN/DSDL; remove from here once the submodule carries it.
