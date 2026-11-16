"""Curated seed authoring tool; no manufacturer/measurement claims are inferred."""
import json
from pathlib import Path
PICO='https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf'
SG='https://towerpro.com.tw/product/sg90-7/'

def unknown(reason):
    return {'value': None, 'evidence': 'unknown', 'reason': reason}

def port(kind, voltage=None, functions=None, required=True, pin=None):
    return {'kind':kind,'voltage_v':voltage,'functions':functions or [],'required':required,'physical_pin':pin,
            'position_m':None,'exit_direction':None,'connector_id':None,'current_limit_a':None,
            'physical_evidence':'待确认实物接头、焊接方式和出线方向'}

def part(identity,name,category,source=None):
    return {'id':identity,'revision':'1','name':name,'category':category,'manufacturer':None,'product_revision':None,
      'status':'candidate','inventory':{'quantity':None,'evidence':'未清点；未知不等于零'},
      'sources':([{'url':source,'accessed':'2026-11-15','scope':'厂商公开资料，非实验室实测'}] if source else []),
      'geometry':{'units':'m','frame':identity+'_local','cad_asset':None,'display_asset':None,'collision_asset':None,
                  'dimensions_m':unknown('完整模块、接头与安装空间待实测'),'source_unit':None},
      'mounting':{'holes':None,'fasteners':None,'tool_clearance_m':None,'insertion_direction':None,'removal_direction':None},
      'physical':{'mass_kg':unknown('未测量'),'center_of_mass_m':unknown('未测量'),'inertia_kg_m2':unknown('未估算')},
      'calibration':{'zero_rad':None,'direction':None,'evidence':'未校准'},
      'electrical':{'peak_current_a':None,'current_evidence':'未确认同时动作/峰值需求','address_options':[],'feedback_actual_angle':False},
      'ports':{},'dependencies':[],'pending':['实物型号、尺寸、安装、质量和连接器待核验'],
      'verification':{'geometry':'pending','assembly':'pending','load':'pending','physical':'pending'}}
parts=[]
for state,label in [('unknown','焊接状态待确认'),('bare','裸板候选'),('straight','直排针装配候选'),('angled','弯排针装配候选')]:
 p=part('pico2_'+state,'Raspberry Pi Pico 2 · '+label,'controller',PICO)
 p['revision']='2'
 p['manufacturer']='Raspberry Pi';p['product_revision']='Pico 2 / RP2350';p['assembly_state']=state
 p['inventory']['evidence']='用户确认实验室已有 Pico 2；数量和焊接状态未知'
 p['ports']={'VSYS':port('power_in',[1.8,5.5],pin='39'),'3V3':port('power_out',[3.3,3.3],required=False,pin='36'),
             'GND':port('ground',[0,0],pin='3'), 'GP0':port('bus',[0,3.3],['I2C0_SDA'],False,'1'),
             'GP1':port('bus',[0,3.3],['I2C0_SCL'],False,'2'), 'GP2':port('signal_out',[0,3.3],['PWM1_A'],False,'4'),
             'GP3':port('signal_out',[0,3.3],['PWM1_B'],False,'5'), 'GP4':port('signal_out',[0,3.3],['PWM2_A'],False,'6')}
 p['ports']['3V3']['current_limit_a']=.3
 p['ports']['3V3']['current_evidence']='厂商建议外部负载小于 300 mA；仍依赖板内负载和 VSYS'
 p['pending']+=['Pico 输出电流预算依赖板内负载；不能直接为舵机供电','本版只开放经过登记的 GP0/GP1 I²C 与 GP2/GP3/GP4 PWM 资源']
 if state in ['straight','angled']:
  p['dependencies']=[{'catalog_id':'header_'+state,'quantity':2,'reason':'装配版本需要登记两条排针和焊接验证'}]
 parts.append(p)
p=part('mpu6050_lab','MPU6050 实验室模块 · 板卡版本待确认','imu','https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=MPU-6050')
p['inventory']['evidence']='用户确认实验室已有 MPU6050 模块；厂家、PCB 版本、数量未知'
p['sources'][0]['scope']='芯片资料入口；本次旧 PDF 跳转不可访问，不能作为开发板供电和尺寸证据'
p['ports']={'VCC':port('power_in'), 'GND':port('ground',[0,0]),'SDA':port('bus',None,['I2C_SDA']),'SCL':port('bus',None,['I2C_SCL'])}
p['electrical']['address_options']=[104,105]
p['pending']+=['模块 VCC 范围、稳压和 I²C 上拉电平未知；3.3 V 接线仅为待核实假设','AD0 状态及实际地址 0x68/0x69 待确认','端口标签是逻辑候选，未登记 PCB 排针次序；不能按裸芯片尺寸制造']
parts.append(p)
p=part('sg90_digital_towerpro','TowerPro SG90 Digital · 来源待核实','actuator',SG)
p['manufacturer']='TowerPro';p['product_revision']='SG90 Digital（非模拟版）'
p['physical']['mass_kg']={'value':.009,'evidence':'manufacturer','reason':'规格区 9g；不是实验室称重'}
p['ports']={'VCC':port('power_in',[4.8,4.8]),'GND':port('ground',[0,0]),'PWM':port('signal_in',None,['PWM'])}
p['dependencies']=[{'catalog_id':'supply_48_candidate','quantity':1,'reason':'独立舵机供电候选；不能从 Pico 3V3 取电'}]
p['pending']+=['仅按规格区 4.8 V 点值规划；允许范围、峰值电流及 3.3 V PWM 识别阈值待核验','无对外实际角度反馈；舵盘、紧固件、转角和负载未验证']
parts.append(p)
for identity,name in [('st3215_12v_candidate','Waveshare ST3215 12 V · TTL 候选'),('mg4005v2_candidate','MG4005V2 · 历史 CAD 候选')]:
 p=part(identity,name,'actuator')
 p['pending']+=['本阶段未核实全部电气参数，不开放自动接线；禁止混用 TTL / RS485 / CAN']
 parts.append(p)
for identity,name,voltage in [('supply_48_candidate','4.8 V 稳压供电 · 待选实物',4.8),('supply_5_candidate','5 V 控制器供电 · 待选实物',5)]:
 p=part(identity,name,'power')
 p['ports']={'OUT':port('power_out',[voltage,voltage]),'GND':port('ground',[0,0])}
 p['pending']+=['电压为设计目标，非实测；电流、极性、保护和接口均待确认']
 parts.append(p)
for identity,name,category in [('wire_candidate','连接线 · 规格待选','wire'),('header_straight','直排针 · 型号待选','connector'),('header_angled','弯排针 · 型号待选','connector'),('splitter_candidate','分线器 · 型号待选','distribution'),('fastener_candidate','紧固件 · 规格待选','fastener'),('battery_holder_candidate','电池盒 · 电池方案待选','power')]:
 parts.append(part(identity,name,category))

p=part('pca9685_adafruit_815','PCA9685 · Adafruit 815 参考板（实物板型待确认）','driver','https://www.adafruit.com/product/815')
p['manufacturer']='Adafruit';p['product_revision']='815; reference board, purchased variant unconfirmed'
p['sources'] += [{'url':url,'accessed':'2026-11-16','scope':'Official reference board documentation; not a measurement of purchased board'} for url in ['https://learn.adafruit.com/16-channel-pwm-servo-driver/pinouts','https://learn.adafruit.com/16-channel-pwm-servo-driver/downloads','https://github.com/adafruit/Adafruit_CAD_Parts/tree/main/815%20Servo%20Driver%2016%20Channel']]
p['geometry']['reference_board_dimensions_m']={'value':[.0625,.0254,.003],'evidence':'manufacturer','reason':'Excludes headers and terminal block; not the assembled collision envelope'}
p['geometry'].update(cad_asset='catalog/assets/pca9685_adafruit_815/reference.step',display_asset='catalog/assets/pca9685_adafruit_815/reference.stl',source_unit='mm',asset_scope='manufacturer_reference_only_not_purchased_variant')
p['geometry']['cad_source_url']='https://github.com/adafruit/Adafruit_CAD_Parts/tree/main/815%20Servo%20Driver%2016%20Channel'
p['geometry']['pcb_source_url']='https://github.com/adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB'
p['mounting']['reference_hole_diameter_m']=.0025
p['ports']={'VCC':port('power_in',[3.,5.]),'VPLUS':port('power_in',None), 'GND':port('ground',[0,0]),'SDA':port('bus',[0,3.3],['I2C_SDA']), 'SCL':port('bus',[0,3.3],['I2C_SCL']), 'OE':port('signal_in',[0,3.3],['ACTIVE_LOW_ENABLE'],False)}
for n in range(16):p['ports']['PWM'+str(n)]=port('signal_out',[0,3.3],['PWM'],False)
p['electrical'].update(address_options=[64],default_address_hex='0x40',address_selection='A0–A5; other addresses require reserved-address review',pwm_channels=16,pwm_resolution_bits=12,logic_supply_design_v=3.3,output_voltage_tracks='VCC',servo_supply_port='VPLUS',servo_supply_regulated=False,shared_pwm_frequency=True)
p['pending'] += ['参考板不是已购实物；安装孔位置、接头高度、CAD 版本与实物核实后才能制造载板','VCC 采用 3.3 V，与 Pico 共地；VPLUS 是独立舵机电源，不经过稳压','VPLUS 允许输入由具体板卡和舵机共同限制；SG90 峰值电流和信号阈值仍待验证','参考板孔径 2.5 mm；壳体的 M3 闭合螺丝不能直接穿入板孔','自动接线提供器尚需支持经此驱动器分配 PWM，不能冒称已生成接线']
parts.append(p)

Path(__file__).with_name('parts.json').write_text(json.dumps({'schema_version':'1.0','revision':'2026-11-16.3','parts':parts},ensure_ascii=False,indent=2)+'\n')
