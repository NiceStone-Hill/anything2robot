# Simple English interface

The interface uses plain English step names, a neutral white layout, compact buttons and collapsed diagnostics. Branding, the welcome illustration, promotional copy and the footer were removed. Saved layout results appear before the dimension editor. Dimensions remain editable; missing values focus the field and open the editor. Structure failures open their diagnostic panel.

Component and generated part labels use English display aliases. Original project names, user input, catalog data and archived reports remain stored verbatim. The verified example appears as `Example · head joint · 500 mm` (1c4ba1). Original reports are available inside details or downloads.

## Accurate visualization and routing labels

Pico 2, MPU6050, power modules and the distribution module are still engineering size envelopes with generated strap mounts. They are not detailed component CAD. The SG90 motor uses third-party reference CAD whose agreement with the actual Digital variant is unverified; see `catalog/joint_modules.json`.

The current studio router generates individual obstacle-avoiding wire paths inside a fixed body. It does not create a shared trunk or packed branches. The earlier implementation in `robot_interior_postprocess/harness_bundle/` does plan shared corridors, segment membership and packing. Those capabilities have not been integrated into the studio. The interface now states this explicitly.

## Verification

- 67 core tests pass.
- Firefox opens the saved example and all seven pages, validates empty dimensions, adds/removes an unsaved component and checks 430 px width without page overflow.
- All seven default page views in the saved example have English visible copy (original user text and expandable raw reports are retained).
- Screenshots and the browser report: `evidence/simple_english_ui/`.
- Opt-in browser check from the studio directory: `PYTHONPATH=. .venv/bin/python tests/english_ui_browser.py`. Requires the existing service at 127.0.0.1:8768 and Firefox; uses an isolated browser profile.

This update changes presentation and does not add component CAD or bundled routing. Existing geometry, electrical and routing results remain unchanged.
