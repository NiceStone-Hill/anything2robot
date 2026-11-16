"""Parametric module methods adapted from Text2Robot, new SG90 fixtures.

All interface dimensions remain engineering assumptions until measured.
Each result is a Boolean solid, with explicit fastener/access geometry.
"""
from .solid import box, cylinder, union, cut, intersect


def motor_module(y_offset=0):
    outer=box([-26,-11,-24],[8,11,10])
    pocket=box([-21.1,-6.7,-21],[3.1,6.7,15])
    screws=[cylinder(x,y,-30,16,1.2) for x in [-23.5,5.5] for y in [-8.5,8.5]]
    # Open side cable slot; cable orientation is an explicit unmeasured assumption.
    nuts=[cylinder(x,y,-24.1,-21.5,2.3,6) for x in [-23.5,5.5] for y in [-8.5,8.5]]
    seats=[cylinder(x,y,9.9,10.4,2) for x in [-23.5,5.5] for y in [-8.5,8.5]]
    outer=union(outer,*seats)
    case=cut(outer,pocket,box([-26.5,-12,-13],[-20,-5,-8]),*screws,*nuts)
    # A flat reference-body contact pad captures the motor against the bottom seat.
    # Physical case/ears/tolerances must still be measured before using this fixture.
    retainer=cut(union(box([-26,-11,10.4],[8,11,13]),box([-18,-4,8],[-11,4,10.6])),cylinder(0,0,9,15,7),*screws)
    motor=union(box([-20.5,-6.1,-21],[2.5,6.1,8]),cylinder(0,0,7.9,14,2.5))
    horn=cylinder(0,0,14,16,9)
    if y_offset:
        for m in [case,retainer,motor,horn]:m.apply_translation([0,y_offset,0])
    return {'case':case,'retainer':retainer,'motor_envelope':motor,'stock_horn_envelope':horn}


def shell_with_lid(outer,inner,side,gap,wall=2.4):
    lo,hi=outer.bounds;plane=24.
    clip=box([lo[0]-1,lo[1]-1,lo[2]-1],[(-gap if side=='fixed' else hi[0]+1),hi[1]+1,hi[2]+1]) if side=='fixed' else box([gap,lo[1]-1,lo[2]-1],[hi[0]+1,hi[1]+1,hi[2]+1])
    skin=intersect(cut(outer,inner),clip)
    base=intersect(skin,box([-500,-500,-500],[500,500,plane-.2]))
    lid=intersect(skin,box([-500,-500,plane+.2],[500,500,500]))
    xs=[lo[0]+22,-gap-16] if side=='fixed' else [gap+16,hi[0]-22]
    ys=[lo[1]+4,hi[1]-4]
    holes=[];bosses=[];lid_bosses=[];nuts=[];points=[]
    for x in xs:
        for y in ys:
            points.append([x,y,plane])
            bosses.append(intersect(cylinder(x,y,14,plane,5),outer))
            # Matching lid standoffs carry bolt compression to the base boss.
            # The external seam has clearance; the internal boss faces meet at plane.
            lid_bosses.append(intersect(cylinder(x,y,plane,hi[2]+1,5),outer))
            holes.append(cylinder(x,y,12,hi[2]+2,1.7))
            nuts.append(cylinder(x,y,13.8,17,3.3,6))
    base=cut(union(base,*bosses),*holes,*nuts)
    lid=cut(union(lid,*lid_bosses),*holes)
    return base,lid,points,skin


def fixed_carrier(case,outer,gap):
    # Ribs intersect the source shell, rather than visually touching an envelope.
    x=-gap-12
    y0,y1=outer.bounds[:,1]
    rib=intersect(box([x-4,y0,-25],[x+4,y1,-21]),outer)
    beam=box([x-4,-10,-25],[-18,10,-21])
    return union(case,rib,beam)


def moving_bridge(outer,gap):
    x=gap+14;y0,y1=outer.bounds[:,1]
    disc=cylinder(0,0,14,18,10)
    beam=box([0,-5,14],[x+4,5,18])
    rib=intersect(box([x-4,y0,14],[x+4,y1,18]),outer)
    # Adjustable stock-horn screws: no invented keyed spline.
    slots=[box([-2,y-1.2,13],[6,y+1.2,19]) for y in [-5,5]]
    return cut(union(disc,beam,rib),cylinder(0,0,13,19,1.7),*slots).apply_translation([0,0,2])


def electronics_box(outer,gap):
    x=(outer.bounds[0,0]-gap)/2-2
    tray=cut(box([x-29,-15,-15],[x+29,15,2]),box([x-26.6,-12.6,-12.6],[x+26.6,12.6,3]))
    screws=[cylinder(x+dx,y,-20,8,1.2) for dx in [-25,25] for y in [-11,11]]
    bosses=[cylinder(x+dx,y,-15,2.4,3.5) for dx in [-25,25] for y in [-11,11]]
    # Ties through the floor, adjustable to the board; not fictitious PCB holes.
    slots=[box([x+dx-1.5,y-4,-16],[x+dx+1.5,y+4,-12]) for dx in [-17,17] for y in [-6,6]]
    ears=[box([x+dx-4,y-3,-15],[x+dx+4,y+3,-12]) for dx in [-22,22] for y in [-18,18]]
    dock_holes=[cylinder(x+dx,y,-25,-10,1.7) for dx in [-22,22] for y in [-18,18]]
    lid_nuts=[cylinder(x+dx,y,-15.1,-12.7,2.3,6) for dx in [-25,25] for y in [-11,11]]
    tray=cut(union(tray,*bosses,*ears),*screws,*slots,*dock_holes,*lid_nuts)
    lid=cut(box([x-29,-15,2.4],[x+29,15,5.4]),*screws)
    rail=box([x-30,-21,-19],[x+30,21,-15])
    rib=intersect(box([x-4,outer.bounds[0,1],-19],[x+4,outer.bounds[1,1],-15]),outer)
    dock_nuts=[cylinder(x+dx,y,-19.1,-16.3,3.3,6) for dx in [-22,22] for y in [-18,18]]
    dock=cut(union(rail,rib),*dock_holes,*dock_nuts)
    return tray,lid,dock,[[x+dx,y,-12] for dx in [-22,22] for y in [-18,18]]
