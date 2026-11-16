"""Shared stock-horn screw registration, in the actuator output frame (mm)."""
import math


def registration(template):
    profile = template['horn_profile']
    if profile['shape'] not in {'cross', 'bar', 'disc'}:
        raise ValueError('Unsupported stock horn profile')
    slots = profile['adaptor_slots']
    lo, hi = slots['radial_centres_mm']
    radius = slots['shaft_radius_mm']
    pilot = slots['pilot_radius_mm']
    head = slots['screw_head_radius_mm']
    values = [lo, hi, radius, pilot, head, template['horn_radius'], profile['arm_width_mm']]
    if not all(isinstance(x,(int,float)) and math.isfinite(x) and x>0 for x in values):
        raise ValueError('Horn interface dimensions must be finite positive mm')
    if not (lo < hi and hi+radius < template['horn_radius'] and pilot < radius < head
            and lo-radius > 1.7 and profile['arm_width_mm']/2 > radius):
        raise ValueError('Horn slots must retain material around centre access and arm edges')
    # Both supplied horn shapes share the longitudinal arm. Use its centreline,
    # not off-axis points inherited from a circular-horn approximation.
    return [{'point_mm':[0.,sign*(lo+hi)/2,template['horn_thickness']],
             'ends_y_mm':sorted([sign*lo,sign*hi]), 'shaft_radius_mm':radius,
             'pilot_radius_mm':pilot,'head_radius_mm':head,
             'physical_fit_verified':False,'evidence':slots['evidence']}
            for sign in [-1,1]]
