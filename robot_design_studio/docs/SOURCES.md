# 参数来源与本次核验边界

访问日期：2026-11-15。只转录首阶段所需少量参数，不复制完整第三方手册或 CAD。公开参数不等于实物验证。

- [Pico 2 官方手册](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf)：核对 VSYS 1.8–5.5 V、3V3、电源/地物理脚号和 GPIO 3.3 V。3V3 外部负载建议小于 300 mA，受板内负载和输入条件影响。此值不是无条件供电保证。完整 USB、排针和插拔占位仍为空。
- [Raspberry Pi 硬件接口文档](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html) / [RP2350 数据手册](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)：核对 GP0/GP1 为 I2C0 SDA/SCL，GP2/GP3 为 PWM1 A/B。库只开放该子集，不意味着 Pico 只有这些引脚。
- [TowerPro SG90 Digital](https://towerpro.com.tw/product/sg90-7/)：选 Digital 明确来源候选，采用规格区 4.8 V 点值和 9 g；不从评论或其他供应商拼凑可用范围，不把该候选当实验室已有实物。PWM 输入阈值、峰值电流、装配接口和允许负载仍未知。
- [TDK MPU6050 产品入口](https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=MPU-6050)及 [MPU-6000/MPU-6050 寄存器资料](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf)：仅作为芯片/I²C 协议的历史来源入口；AD0 地址候选 0x68/0x69 需实物确认。旧 datasheet PDF 本次访问跳转失败、寄存器 PDF 返回 404（搜索索引仍可见），未据其声称开发板供电已核验。实验室模块厂家、板卡版本、上拉电压、尺寸和排针次序一律待核实。
- [Chat Completions 官方接口](https://developers.openai.com/api/reference/resources/chat/subresources/completions/methods/create)：兼容 JSON 响应适配器。服务配置决定实际模型和 API；无真实在线调用证据，模拟传输测试不当作在线验证。

旧电机与 Pico CAD 只读查看了来源文档；没有复制它们或假定其再分发许可覆盖新系统发布。Pico 官方 CAD 来源保留在旧目录；本阶段新库不声称已经创建对应三维轻量模型。

## 0.9.0 CAD 来源更新（2026-11-16）

已核验 Pico 2 官方产品门户及保留的原 STEP 摘要；导入带 MIT 许可的官方裸板模型。另从 usini 传感器库固定提交下载 Nicola Cugola 的 MPU6050 参考 STEP，保留原作者、来源与原 README，未将 GrabCAD 派生模型误标 CC0。原实物型号仍待核实。详见 `assets/components/README.md` 和 `CAD_AND_BUNDLED_ROUTING_2026-11-16.md`。以上早期“未复制 CAD”的描述仅代表当时阶段。
