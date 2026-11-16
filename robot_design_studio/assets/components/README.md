# Reference component CAD

Source files are geometry data. Models are displayed at native millimetre scale; the registration in `models.json` rotates and centres them but never rescales them. Planning envelopes, soldering, headers, connector insertion and actual lab hardware remain separate, unmeasured inputs.

## Pico 2

Official bare-board STEP from the [Raspberry Pi product information portal](https://pip.raspberrypi.com/categories/1005-raspberry-pi-pico-2). Reused the previously downloaded file only after verifying its SHA256 against the retained source metadata. The official portal was checked again for this update. The archive's MIT license is retained in `pico2/LICENSE.txt`. Native extents including USB overhang: 21 × 52.3 × 3.73 mm. No headers; boards with headers need a matching model, not a scaled copy of this one.

## MPU6050

Third-party reference CAD by Nicola Cugola, [original model](https://grabcad.com/library/mpu6050-1), retrieved via the [Usini sensor library](https://github.com/usini/usini_kicad_sensors/blob/e2d03afffcb95ee2b2ff4183b6573658f2684229/usini_sensors.pretty/MPU_6050.step). Native extents: 20.5 × 16 × 3 mm, without fitted pin header. The laboratory module's manufacturer and PCB variant are unconfirmed; this CAD is identified as a GY-521-style reference, not a measured model of the user's board.

The source README explicitly distinguishes its GrabCAD-derived models from the CC0 models. This MPU6050 model is not identified as CC0. The original attribution and stated usage terms are retained in `mpu6050/SOURCE_README.md` and `ATTRIBUTION.txt`; no MIT or other open redistribution license is asserted for it. Included here for the user's local reference visualization; verify rights before publishing this third-party asset elsewhere.

## Conversion

Gmsh 4.13.1 (already present in the geometry environment), STEP surface triangulation with mesh size 0.15–0.6 mm, first-order triangles. Native CAD face colours are retained. Visual surface meshes are not asserted to be closed manufacturing solids; collision checks and assumed mass continue to use the separately registered closed envelopes. Sources, SHA256, transforms and variant evidence travel with the display asset records.

Power supplies and the distribution module have no selected physical model. They remain explicitly labelled envelopes.

## Registered reference PCB mounts (0.9.2)

The model registry now records native STEP PCB-hole axes and board planes. `component_visuals.registration` is shared by CAD and generated screw carriers; source files are unchanged and no rescaling is used. A 6 mm underside allowance reserves the explicit prototype mount zone. Source CAD holes are checked by ray queries. Pico uses four 2.1 mm reference holes and MPU uses two 3 mm reference holes; physical laboratory variants remain unconfirmed. Candidate fastener geometry is not a measured/purchased part specification. See `docs/REGISTERED_PCB_MOUNTS_2026-11-16.md` and `evidence/registered_mounts_2026-11-16/cad-hole-audit.json`.
