# Native output datum and final mating surfaces

All rules below apply to registered motor templates and arbitrary input geometry; no source-model names or coordinates enter the solver.

## Problem identified

The previous open-cradle solver allowed up to three module-depth-derived axial shifts. This preserved the infinite rotation axis, but did not preserve the annotated joint location. The previous 200 mm test selected 42.39 mm shifts for both arms and 28.26 mm for the base. A clear screw approach did not independently verify that the final load-bearing seat survived all cuts.

## Revised constraints

- For rectangular servos, the native output origin must coincide with the annotated joint point; shaft axis must be collinear with the joint axis. The body centre is not the shaft centre. No axial-shift fallback is allowed. Roll and registered retainer installation directions are searched at the same origin.
- The parent shell carries the integral mounting-ear cradle. Each ear has its own planar seating witness, aligned with its registered hole. The candidate 0.05 mm assembly clearance remains explicitly unverified physically. The retainer clamps the ears with two bolts and captive nuts.
- The child shell carries an integral output adaptor. Its flat mating face meets the stock horn. The horn's centre screw fixes it to the output shaft; separate adaptor screws join the horn to the child-side adaptor.
- After motion trimming, tool openings and cover creation, each seating witness must remain in its declared final shell, and that shell must be connected. Clear holes alone cannot pass this check. Existing collision, screw access, insertion, support and motion checks still apply.
- Support search samples directionally distinct shell patches within the existing maximum span, in addition to nearby vertices. A dense mesh patch must not monopolize all candidate attachment targets.
- Golden preview overlays are intersections with the final shell, rather than untrimmed copies of the original fixtures.

These are geometry and assembly constraints, not physical strength or motor-fit certification. A failed bounded search does not prove that no mechanical design exists. It must reject the candidate and retain diagnostics, rather than move the motor away from the joint to make a result appear feasible.

## Tests

`tests.joint_registration_geometry`: parallel-but-displaced axes rejected; rotated correct ownership accepted; missing seats, floating shells and wrong owners rejected. Existing fastener tests remain applicable. `tests.open_cradle_integration` exercises complete single- and paired-joint source-generated models.

## Initial validation result (superseded)

Single-joint fixture: 47/47 checks passed. Paired-joint fixture: 85/85 checks passed. Regression tests: 12 passed. Live 04 re-generation timed out in the base-joint support search at the existing 360-second job limit; no corrected printable 04 is claimed. The unresolved task is a generic feasible shell-support and assembly-opening solution at the anchored output datum.

## Follow-up resolved the bounded-search blocker

Live 04 revision 17 now passes all 124 geometry/assembly checks after generic surface-support search and assembly-order fixes. See [surface support and assembly order](SURFACE_SUPPORT_AND_ASSEMBLY_ORDER_2026-11-16.md). Physical fit/load and electronic layout remain unverified/incomplete.
