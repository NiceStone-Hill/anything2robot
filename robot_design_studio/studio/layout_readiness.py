"""Spatial layout can be explored separately from mechanical release.

This never changes or waives a structure check. An explicit draft request can
reuse a complete, connected, collision-checked structure for spatial packing.
"""
def draft_layout_ready(result):
    if not result.get('geometry_assets') or not result.get('layout_domains'):return False
    checks={c['name']:c for c in result.get('checks',[])}
    required={'assembly_collisions','joint_sweep','structural_paths'}
    for asset in result['geometry_assets']:
        required.update({'solid_'+asset['id'],'stl_roundtrip_'+asset['id']})
    for domain in result['layout_domains']:required.add(domain['owner_body_id']+'_final_connections')
    return all(checks.get(name,{}).get('passed') is True for name in required)
