顶层模块：XR_smg_interface
  ├── smg_control_module    (U1) → 生成6位BCD数字（Number_Sig）
  └── smg_interface         (U2)
        ├── XR_smg_control_module (U1) → 分时选通某一位数字
        ├── XR_smg_encode_module  (U2) → BCD→段码编码
        └── XR_smg_scan_module    (U3) → 数码管位选扫描
