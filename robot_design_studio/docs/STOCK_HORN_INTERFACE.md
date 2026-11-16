# Stock SG90 horn and printed output adaptor

The actuator's stock cross horn retains the manufactured spline and original
centre screw. The circular output adaptor is a **printed part**, integrated
with the moving shell; it is not a purchased circular servo horn.

The versioned `horn_profile.adaptor_slots` record now drives the output plate's
two longitudinal slots, the reference horn's pilot holes and the screw/tool
axes together. Both axes lie on the longitudinal arm centreline. Cross and
symmetric bar profiles share this interface; a different physical horn must
have its dimensions registered and be regenerated rather than assumed to fit.

Current SG90 defaults remain engineering assumptions: cross outer radius 9 mm,
arm width 6 mm, thickness 2 mm; slot centre endpoints at radii 4.5 and 6.5 mm,
slot radius 1 mm, pilot radius 0.6 mm. These are not measurements of the user's
parts. The actual hole radius/position, horn outline, hub, centre screw access,
small attachment screws and thread engagement need physical verification.
The template rejects slots that cross the centre opening or leave the horn's
arm or radial envelope. No printed shaft spline is substituted.

Assembly: set the unloaded servo to neutral; secure the supplied cross horn
using its original centre screw; fasten the printed adaptor through its slots
to two opposing holes in the stock horn with screws matching those holes.
The body-closure M3 screws are not the horn attachment screws. Follow the
saved assembly sequence for access and close the shell afterwards.

Tests cover differently sized cross/bar reference solids, slot/pilot/tool-axis
coincidence, contact material and invalid narrow-arm profiles. Geometry passes
do not certify actual hardware dimensions or physical load capability.
