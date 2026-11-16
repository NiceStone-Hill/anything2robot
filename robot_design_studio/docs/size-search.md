# Bounded model enlargement

Internal layout offers **If components do not fit, try larger copies** (enabled
by default). `POST /api/projects/{id}/size-search` accepts the project revision
and the same complete component envelopes as layout generation. The returned
ordinary run can be polled and cancelled through `/api/runs/{id}`.

The workflow first tries the current size, then +5%, +10%, +15%, +20%, +25%
relative to its initial model. Copies inherit a `size_search_origin` event;
starting again from one of those copies retains the original limit. Dimensions
are uniformly scaled; the 25% limit applies to linear dimensions, not volume.
Each larger candidate is a separate saved project. Existing project sizes and
historical runs are retained. The first size reuses the current project and
creates new structure/layout revisions there.

Unsuccessful component packing with recorded placement-search attempts, or an
explicit actuator `_module_room` failure, triggers growth. Missing service openings, invalid inputs, structural failures,
worker failures, screw/export checks do not establish insufficient space.
Search exhaustion is not proof that fitting is mathematically impossible.

Every trial invokes normal source scaling, articulation, motor selection,
electrical generation, assembly and layout services. Motor hardware, PCB
profiles, mounting-hole dimensions and power configuration stay at native
size; no per-model coordinates are introduced. All components are copied.
Unsupported hardware is not silently removed or substituted. Connections,
screw access, collisions and export checks remain enforced.

A layout can fit while structure remains unverified. Such a result is reported
as `fit_structure_unverified`, never as print-ready. Other terminal reasons
include `structure_failed`, `non_packing_failure`, `electrical_failed` and
`limit_reached`. Results record baseline, every candidate project, actual size,
factor and child run IDs. At the limit the search stops; it does not exceed 25%.
The first passing candidate is the smallest tested size, not a globally optimal
solution. Physical fit, loads, wiring and electrical validation remain separate.
