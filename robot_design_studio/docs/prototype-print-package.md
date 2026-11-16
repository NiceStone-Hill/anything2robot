# Prototype print geometry

The generator offers `prototype-print.zip` only after all hard structure and internal-layout checks pass. The archive contains only printable assets, with millimetre units, per-part hashes, assembly-to-print transforms, assembly instructions and outstanding advisories. Hardware and electronics reference meshes are excluded. Each part is independently oriented using its largest planar facet, then placed on Z=0; these are starting poses for slicer review, not a support-free printing guarantee.

This gate is distinct from electrical/manufacturing release. Hardware measurements, connector variants, cable routing, printer capacity, slicing, and physical fit are not inferred from closed STL geometry. `manufacturing_ready` remains false and these limitations are exported in the manifest. No printer-specific G-code is generated.

Structural supports keep actuator poses fixed but can search alternate attachment points inside their registered mounting solids. Final cover clearance holes, head entries and nut seats are reserved before rib generation, as are actuator interface voids. Later screw relocation rejects candidates that remove a registered support core.

STL repair keeps a 0.001 mm displacement limit and closed-volume round-trip requirements. The volume allowance is coupled to measured displacement and surface area, capped at 100 ppm (with the existing small absolute floor). This avoids rejecting a sub-micron surface repair solely because its accumulated volume difference exceeds an unrelated one-ppm limit. The source mesh and per-attempt measurements remain preserved.
