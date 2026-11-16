# Reference component models and shared harness routing — 0.9.0

The existing example `project_95d1e73d971c4ba1` (1c4ba1) now has a new saved version 12. Its English display name remains `Example · head joint · 500 mm`. Refresh the tool at `http://127.0.0.1:8768/`, open the example, and use **5 Internal layout** or **6 Wire routing**. The **Focus** selector can isolate Pico 2 or MPU6050; **Toggle shell** restores the body context.

## Component appearance

Pico 2 uses the official manufacturer STEP. MPU6050 uses a third-party GY-521-style CAD reference with author attribution; exact laboratory PCB variant is unconfirmed. Both are triangulated from actual imported STEP geometry, at original scale, not hand-built illustrative boxes. Native source extents are 21×52.3×3.73 mm and 20.5×16×3 mm. Model registration checks the complete CAD bounds against the existing assumed envelope and falls back with an explicit reason when it does not fit; it never rescales or stretches a board.

Separate `visual_assets` contain model ID, source URL/hash, mesh hash, reference variant, scale, complete source-to-world transform and physical-verification status. The closed envelopes remain authoritative for conservative clearance and assumed mass. The preview and assembly GLB replace corresponding display boxes with CAD; separate CAD OBJ resources are used by URDF/MJCF visuals. The original closed envelope STLs are retained for checks and traceability. Unknown power/distribution hardware stays as an envelope.

Sources and original terms are preserved under `assets/components/`; see its README. Pico is MIT. The community MPU model is not claimed to be CC0 or MIT; its stated GrabCAD usage and attribution are recorded. No public upload was performed in this task.

## Shared mechanical harness

The studio now defaults to a **bundled** routing mode. It directly reuses the geometry-independent `pack`, `rooted_grid_tree`, `growing_tree`, `shared_segments` and `cable_segment_path` algorithms from `robot_interior_postprocess/harness_bundle/network.py`. The studio adapter copy in `studio/mechanics/bundle_network.py` matches the source byte for byte. Provenance hashes are recorded per run. This avoids importing the old application's workspace assumptions.

The new adapter builds occupancy from the current checked shell, motor, component envelopes and carriers, reserves conservative packed capacity for every insulated wire, chooses one assumed aggregation exit per device and attaches devices to the existing tree. Each shared segment retains exact wire IDs and is sized by its own wire membership. Wires share a mechanical corridor while their electrical nets stay separate. Wire records retain ordered segment membership, continuous shared-corridor centreline and an explicit cut-length allowance.

The run exports closed bundle corridor solids and checks their actual volume against assembly obstacles, nonincident segments and sampled moving-head poses. Geometry may merge at intended common tree junctions. As in the previous project, the bundle tube is a reserved mechanical corridor: transport of individual packed conductors through bends and junctions, actual connector fan-out, sleeve construction, ties, clips, strain relief and physical assembly remain unverified. It does not claim those manufacturing details are finished.

Legacy individual-wire outputs remain archived. New jobs use shared routing; the internal optional `routing_mode: individual` exists for regression comparison and old results are labelled as such in the UI.

## Formal case evidence

| Stage | New formal run | Checks / elapsed |
|---|---|---|
| CAD layout | run_e61b2278a72a447c | 23 passed / 27.160 s |
| Shared harness | run_d2c2e7a989b143d1 | 29 passed / 2.525 s |
| URDF/MJCF visuals | run_6f2aa3ea543b46d7 | 24 passed / 1.312 s |

The example contains **2 reference CAD models, 3 unknown-hardware envelopes, 11 wire instances and 7 shared segments**. Independent verification reloads the exported bundle STLs, verifies packing separation and containment, wire membership/continuity, hashes, CAD resource references and ZIP contents. The review ZIP has 134 entries. Source model, head joint graph and electrical nets match the prior saved example.

Changed-input verification increases insulated wire diameter from 1.2 to 1.3 mm. The maximum reserved bundle radius grows from 3.524871 to 3.748076 mm, generated solid hashes change, and the recomputed route still passes. This test runs in its own directory and does not alter the saved example.

Validation: 67 core tests, 21 geometry regression tests, and Firefox verification of all seven English pages, component focus, missing-dimension feedback, unsaved component selection and 430 px layout. No simulation engine, control program or physical manufacturing test was run.

Artifacts and reports are in `examples/cad_bundle_2026-11-16/`; browser screenshots are in `evidence/cad_bundles_2026-11-16/`. `history/version-11/` retains intermediate evidence and the workspace retains all earlier formal runs. The new review package is `robot-cad-bundles-review.zip`; standalone `assembly-with-wires.glb`, `wire-list.csv` and `simulation-model.zip` are also available.
