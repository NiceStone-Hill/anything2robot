# Fastener relocation and assembly order

The same solver runs after structure generation and after electronics supports are added. It uses registered part ownership, split frames, screw interfaces and actual meshes. It contains no model names, project IDs or preset world positions.

## Position search

When generating an M3 split connection, retain the exact material removed from each shell half, in world coordinates. Also retain the section-derived candidate positions. If a later tool-access check fails, search nearby candidates (at most 48 per failed connection):

1. Restore the previous hole from its recorded removed material. Historical holes with no record cannot be reconstructed by guessing.
2. Keep at least 9 mm separation from the other registered screws and at least 60% of the previous fastening footprint area. Build both seats, the shared clearance hole, head entry and hexagonal nut seat at the proposed axis.
3. Clip seats to the owner domain. Reject interference with neighboring parts, disconnected shells, overlapping shell halves, blocked nut entry, missing bearing material, blocked screw/tool paths or degradation of another previously valid connection.
4. Commit both meshes and their shared interface record together. Rejected candidates change neither geometry nor hole positions. Existing boss material can remain; the old hole is filled using its recorded stock.
5. Export the updated positions, search attempts and source/restoration information. Existing downstream collision, movement, PCB registration and STL checks still run.

The current relocation template is the generated M3 shell connection. An unregistered thread/stock combination is reported as unsupported instead of being resized implicitly.

## Assembly order

If relocation fails, only explicitly registered removable covers may be postponed. Add an edge from the obstructed cover closure to the obstructing cover closure. Preserve all existing actuator installation, joint fastening and cover dependencies. Topologically sort the entire operation graph; reject missing operations or cycles. Do not excuse actual interference with an installed screw.

Records retain the blocked assets and specify that access is valid only with those covers deferred. This is an assembly-stage tool-access check; it does not certify an arbitrary manual removal trajectory. Straight cover/component extraction remains separately visible as an advisory check.

Structure generation resolves order after its complete operation graph exists. Electronics layout uses the saved structure operation graph, and exports the resulting steps in its assembly instructions.

## Verification

`tests/test_fastener_search.py` verifies relocation and restoration on two different-sized translated shells, immutable failure without stock, exhausted-search rollback, dependency cycle/missing-operation rejection and installed-screw interference rejection. The existing layout and carrier-depth tests also remain applicable.

## 05 system verification

Assembly `run_f5041f42e45c40a5` relocated the obstructed connection by 2.549 mm, restored its previous hole, and retained 98.2% of the fastening footprint area. Layout `run_8c0bfbd22d434fd1` placed both boards and passed every hard layout check, including cover screw access. Structure support checks and one STL export-deviation check remain unresolved; this is a layout draft, not a manufacturing release. See `evidence/packing_diagnostics/05-fastener-search.json`.
