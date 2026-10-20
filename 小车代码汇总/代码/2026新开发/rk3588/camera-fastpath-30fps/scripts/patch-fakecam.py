#!/usr/bin/env python3
"""Patch an isolated copy of the supplied RK3588 SDK, never the SDK itself."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
p = root / 'rkaiq/cmake/BuildOptions.cmake'
s = p.read_text()
old = 'set(RKAIQ_HAVE_FAKECAM       FALSE'
assert old in s
p.write_text(s.replace(old, 'set(RKAIQ_HAVE_FAKECAM       TRUE', 1))

p = root / 'rkaiq/hwi/fakecamera/FakeSensorHw.cpp'
s = p.read_text()
assert s.count('_timer->SetTimer(0, 100000);') == 1
s = s.replace('_timer->SetTimer(0, 100000);',
    '// Wake only pending RAW buffers promptly; do not generate camera frames.\n'
    '        _timer->SetTimer(0, 5000);')
assert 'out:\n    _vbuf_list.erase(it);' in s
assert 'pFunc(it->base_addr);' in s
s = s.replace('out:\n    _vbuf_list.erase(it);',
    'out:\n    void *completed_base = it->base_addr;\n    _vbuf_list.erase(it);')
s = s.replace('pFunc(it->base_addr);', 'pFunc(completed_base);')
p.write_text(s)

p = root / 'rkaiq/hwi/fakecamera/FakeCamHwIsp20.cpp'
s = p.read_text()
old = '        _mipi_rx_devs[i]->set_mem_type(_rx_memory_type);'
assert s.count(old) == 1
s = s.replace(old, old + '\n        _mipi_rx_devs[i]->set_use_type(_rawbuf_type == RK_AIQ_RAW_DATA ? 1 : 0);')
anchor = 'XCamReturn\nFakeCamHwIsp20::init_mipi_devices'
assert s.count(anchor) == 1
cls = '''class FakeCamCopyRxDevice : public V4l2Device {
public:
    explicit FakeCamCopyRxDevice(const char *name) : V4l2Device(name), copy_type(0) {}
    int get_use_type() override { return copy_type; }
    void set_use_type(int type) override { copy_type = type; }
private:
    int copy_type;
};

'''
s = s.replace(anchor, cls + anchor)
for name in ('rawrd2_s_path', 'rawrd0_m_path', 'rawrd1_l_path'):
    old = 'new V4l2Device (s_info->isp_info->' + name + ')'
    assert s.count(old) == 1
    s = s.replace(old, 'new FakeCamCopyRxDevice (s_info->isp_info->' + name + ')')
p.write_text(s)
print('FakeCam enabled, RAW copy/callback fixes retained, pending-buffer poll: 100ms -> 5ms')
