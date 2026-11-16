"""Complete candidate fixtures; dimensions/evidence live in versioned templates.

Rectangular: open mounting-ear cradle and removable retaining plate.
Round: A2R axial fixed/output split, supplier candidate face patterns, no scaling.
No fixture asserts verified hardware fit or load capacity.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import trimesh as tm
from .solid import box, cylinder, union, cut, dilate, intersect
from .fastening import interfaces, cylinder_path
from .horn_interface import registration as horn_registration

TEMPLATE_PATH=Path(__file__).resolve().parents[2]/'catalog/joint_modules.json'


def module(catalog_id,library=None):
    if library is None:library=json.loads(TEMPLATE_PATH.read_text())
    if catalog_id not in library['templates']:raise ValueError('所选型号尚无完整安装模板；不能缩放其他电机孔位代替')
    t=library['templates'][catalog_id];design=library['design'];clearance=design['fit_clearance'];wall=design['wall']
    parts=[];fastener_bores=tm.Trimesh();mounting_surfaces=[];interface_voids=[]
    if t['branch']=='anything2robot_round':
        radius=t['body_diameter']/2;depth=t['body_depth'];out_depth=t['output_depth'];plate=t['plate_thickness']
        back=-depth-out_depth
        motor=cylinder(0,0,back,-out_depth,radius,64)
        rotor=cylinder(0,0,-out_depth,0,t['output_diameter']/2,64)
        def holes(pcd,count,z0,z1):
            return [cylinder(pcd/2*math.cos(math.pi/2+i*2*math.pi/count),pcd/2*math.sin(math.pi/2+i*2*math.pi/count),z0,z1,design['m3_clearance_radius']) for i in range(count)]
        fixed=cut(cylinder(0,0,back-plate,back,radius+wall,64),cylinder(0,0,back-plate-1,back+1,6),*holes(t['fixed_pcd'],t['fixed_holes'],back-plate-1,back+1))
        output=cut(cylinder(0,0,0,plate,t['output_diameter']/2,64),cylinder(0,0,-1,plate+1,5),*holes(t['output_pcd'],t['output_holes'],-1,plate+1))
        interface_voids=[{'side':'parent','mesh':union(cylinder(0,0,back-plate-1,back+1,6),*holes(t['fixed_pcd'],t['fixed_holes'],back-plate-1,back+1))},
                         {'side':'child','mesh':union(cylinder(0,0,-1,plate+1,5),*holes(t['output_pcd'],t['output_holes'],-1,plate+1))}]
        parts=[('fixed_mount','固定面安装盘','parent',fixed,True),('output_mount','输出面连接盘','child',output,True),
               ('motor','扁圆电机候选本体','parent',motor,False),('rotor','输出面候选包络','child',rotor,False)]
        cavity=union(dilate(motor,clearance),dilate(rotor,clearance),fixed,output)
        fixed_anchor=np.array([-radius-wall+2,0,back-plate/2]);moving_anchor=np.array([t['output_diameter']/2-3,0,plate/2])
        instructions=['从背面用 M3 螺钉将固定盘连接电机固定面，图纸螺纹深度 3mm；长度仍须核实',
                      '从输出端固定输出连接盘，候选 M3 螺纹深度 4mm；验证轴向正反和角向孔位后装配']
    else:
        # Reference CAD includes real mounting-ear slots. Variant matching and
        # fasteners remain unverified; never claim manufacturer-certified fit.
        ref=t['reference_cad'];path=TEMPLATE_PATH.parent/ref['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=ref['sha256']:raise ValueError('SG90 CAD 摘要不匹配')
        motor=tm.load(path,force='mesh').apply_transform(np.array(ref['source_to_module_mm']))
        motor_clearance=dilate(motor,clearance)
        horn_envelope=cylinder(0,0,0,t['horn_thickness'],t['horn_radius'])
        # Stock horns retain the manufactured spline. Outline is a configurable
        # engineering reference, not a fabricated output-shaft fit.
        profile=t.get('horn_profile',{'shape':'disc'})
        shape=profile['shape'];half=profile.get('arm_width_mm',6.)/2
        if shape not in ('disc','cross','bar'):raise ValueError('Unsupported stock horn profile')
        horn=horn_envelope
        if shape in ('cross','bar'):
            bars=[box([-half,-t['horn_radius'],0],[half,t['horn_radius'],t['horn_thickness']])]
            if shape=='cross':bars.append(box([-t['horn_radius'],-half,0],[t['horn_radius'],half,t['horn_thickness']]))
            horn=intersect(horn_envelope,union(*bars))
        seat=ref['ear_seat_z_mm'];top=ref['ear_top_z_mm']
        ear_x=[p[0] for p in ref['ear_hole_centres_mm']]
        rear=-5.9-clearance-wall
        # Two U-shaped plates grip the mounting ears, not the plastic case.
        # The open +Y side admits the motor laterally with the top plate removed.
        def ear_plate(z0,z1,contact_z0,contact_z1):
            rail=box([min(ear_x)-3,rear,z0],[max(ear_x)+3,rear+wall,z1])
            arms=[box([x-3,rear,z0],[x+3,3,z1]) for x in ear_x]
            relieved=cut(union(rail,*arms),motor_clearance)
            seats=[cylinder(x,y,contact_z0,contact_z1,3) for x,y in ref['ear_hole_centres_mm']]
            holes=[cylinder(x,y,seat-wall-1,top+wall+1,.9) for x,y in ref['ear_hole_centres_mm']]
            return cut(union(relieved,*seats),motor,*holes)
        # The reference STL ear planes differ from nominal by sub-microns.
        # Explicit assembly clearance prevents near-zero sliver solids when
        # this support is subsequently merged into the shell.
        contact_gap=t.get('ear_contact_clearance_mm',.05)
        fixed=ear_plate(seat-wall-contact_gap,seat-contact_gap,seat-wall-contact_gap,seat-contact_gap)
        # Candidate captive nuts enter before the motor. This avoids requiring
        # a socket beside the servo case while tightening the mounting screws.
        fixed=cut(fixed,*[cylinder(x,y,seat-wall-contact_gap-.1,seat-wall-contact_gap+1.5,2.1,6)
                          for x,y in ref['ear_hole_centres_mm']])
        fixed=cut(fixed,*[box([x-2.1,y-2.1,seat-wall-contact_gap-.1],[x+2.1,y+4,seat-wall-contact_gap+1.5])
                          for x,y in ref['ear_hole_centres_mm']])
        retainer=ear_plate(top+contact_gap,top+wall+contact_gap,top+contact_gap,top+wall+contact_gap)
        fastener_bores=union(*[cylinder(x,y,seat-wall-contact_gap-2,top+wall+contact_gap+2,.9) for x,y in ref['ear_hole_centres_mm']])
        # Stock horn receives screws through slots; no fabricated shaft spline.
        horn_sites=horn_registration(t)
        slots=[]
        for site in horn_sites:
            y0,y1=site['ends_y_mm'];r=site['shaft_radius_mm']
            slots.extend([cylinder(0,y0,0,10,r),cylinder(0,y1,0,10,r),box([-r,y0,0],[r,y1,10])])
        output_void=union(cylinder(0,0,0,10,1.7),*slots)
        horn=cut(horn,*[cylinder(*site['point_mm'][:2],-1,t['horn_thickness']+1,site['pilot_radius_mm']) for site in horn_sites])
        output=cut(cylinder(0,0,t['horn_thickness'],t['horn_thickness']+wall,t['horn_radius']+1),output_void)
        interface_voids=[{'side':'parent','mesh':fastener_bores}, {'side':'child','mesh':output_void}]
        # Thin solid witnesses encode actual load-bearing mating faces, not
        # merely clear screw holes. Check them again after all shell cuts.
        for index,(x,y) in enumerate(ref['ear_hole_centres_mm']):
            witness=intersect(fixed,cylinder(x,y,seat-contact_gap-.5,seat-contact_gap,3))
            mounting_surfaces.append({'id':'ear_seat_'+str(index),'side':'parent','mesh':witness,
                'normal':[0,0,1],'point':[x,y,seat-contact_gap],'assembly_gap_mm':contact_gap})
        mounting_surfaces.append({'id':'horn_seat','side':'child',
            'mesh':intersect(output,horn.copy().apply_translation([0,0,t['horn_thickness']])),
            'normal':[0,0,-1],'point':[0,0,t['horn_thickness']],'assembly_gap_mm':0.})
        parts=[('fixed_mount','开放式安装耳托架','parent',fixed,True),
            ('retainer','安装耳可拆压板','parent',retainer,True),('output_mount','打印连接片 · 配套十字/一字舵臂槽孔','child',output,True),
            ('motor','SG90 第三方 CAD · 实物版本待核实','parent',motor,False),('rotor','配套 '+shape+' 舵盘候选，尺寸待实测','child',horn,False)]
        cavity=union(motor_clearance,dilate(horn_envelope,clearance),fastener_bores,*[p[3] for p in parts if p[4]])
        fixed_anchor=np.array([np.mean(ear_x),rear+wall/2,seat-wall/2-contact_gap]);moving_anchor=np.array([t['horn_radius']-1,0,t['horn_thickness']+wall/2])
        instructions=['先沿登记侧向通道预装防转螺母，再接线、拆下压板，从托架开放侧放入电机，将两侧安装耳落在定位支座上；不沿输出轴塞入电机盒',
            '沿登记方向装回安装耳压板，以两颗候选 M1.6 螺丝和垫片贯穿压板、安装耳与托架并拧入预装螺母；实物螺母、孔位、螺丝长度和夹紧配合待核实',
            '先归零，用原配中心螺丝将原配十字舵臂固定在输出轴；圆形件是打印连接片，不需要购买圆舵盘。沿舵臂中心线用两颗匹配原配舵臂孔的螺丝穿过连接片长槽固定；不得用壳体 M3 螺丝替代。槽孔、螺丝长度及实物舵臂尺寸待核实；一字舵臂须按实物登记形状与尺寸后重算']
    exposed=t['branch']=='text2robot_rectangular'
    fastening_interfaces=interfaces(t,design)
    # Reserve the local head seating envelope as well as the shaft bore.
    # The remaining, much longer driver path is obtained by opening the shell.
    for site in fastening_interfaces:
        for tool in site['tools']:
            if tool['kind'] not in ('screw_head_and_driver','centre_screw_and_driver'):continue
            interface_voids.append({'side':site['side'],'kind':'bounded_screw_head_seat',
                'mesh':cylinder_path(tool['point_mm'],tool['direction'],wall+clearance,tool['radius_mm'])})
    # A perforated mount is not a complete cavity cutter: its existing screw
    # holes otherwise remain filled with source-shell material after union.
    # Preserve bounded template bores/slots independently of long tool sweeps.
    cavity=union(cavity,*[v['mesh'] for v in interface_voids])
    return {'registration_policy':'output_datum_at_joint' if exposed else 'axis_line',
            'interface_voids':interface_voids,
            'mounting_surfaces':mounting_surfaces,'parts':parts,'cavity':cavity,'fastener_bores':fastener_bores,'fastening_interfaces':fastening_interfaces,'hardware_clearance':union(dilate(motor,clearance),dilate(rotor if not exposed else horn,clearance)),
            'mounting_style':'open_cradle' if exposed else 'enclosed_axial','allow_exterior':exposed,
            'installation_steps':([{'role':'motor','direction':[0,1,0],'removed_roles':['retainer','rotor','output_mount']},
                                   {'role':'retainer','direction':[0,0,1],'alternative_directions':[[0,-1,0]],'removed_roles':['rotor','output_mount']}] if exposed else
                                  [{'role':'motor','direction':[0,0,1],'removed_roles':['rotor','output_mount']}]),
            'fixed_anchor':fixed_anchor,'moving_anchor':moving_anchor,
            'template':t,'template_revision':library['revision'],'instructions':instructions,'clearance_mm':clearance,'cavity_method':'hardware_surface_dilation_plus_actual_fixtures','offset_max_mm':clearance*1.259}
