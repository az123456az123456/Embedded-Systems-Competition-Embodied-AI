#!/usr/bin/env python3
"""Verify all 88 actual registered Gray crossings, including optimized MSBs."""
import argparse
import hashlib
import json
from pathlib import Path
import re


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def verify(project, build):
    project = Path(project)
    build = Path(build)
    net = build / 'impl/gwsynthesis/robot_control_v21.vg'
    text = net.read_text()
    definitions = re.findall(r'module (camera_async_fifo(?:_\d+)?)\b(.*?)endmodule', text, re.S)
    assert len(definitions) == 3, len(definitions)
    connections = []
    sources = {}
    modules = dict(re.findall(r'module (\w+)\b(.*?)endmodule', text, re.S))
    instances = {}
    for instance in ('cache1', 'cache2'):
        cache_module = re.search(r'\b(\w+)\s+' + instance + r'\s*\(', modules['controller_top'])[1]
        fifo_module = re.search(r'\b(camera_async_fifo(?:_\d+)?)\s+fifo\s*\(', modules[cache_module])[1]
        instances['core/controller_top/' + instance + '/fifo'] = fifo_module
    instances['core/UDP_Send/eth_dcfifo/fifo'] = re.search(
        r'\b(camera_async_fifo(?:_\d+)?)\s+fifo\s*\(', modules['eth_dcfifo'])[1]
    for module, body in definitions:
        ff = {name: ports for name, ports in re.findall(r'DFFCE (\w+) \((.*?)\n\);', body, re.S)}
        for prefix, gray, binary in [('rq1_wgray', 'wgray', 'wbin'), ('wq1_rgray', 'rgray', 'rbin')]:
            targets = sorted((int(re.match(prefix + r'_(\d+)_s0$', n)[1]), n)
                             for n in ff if re.match(prefix + r'_(\d+)_s0$', n))
            assert len(targets) in (14, 15)
            for bit, name in targets:
                actual = re.search(r'\.D\(([^)]*)\)', ff[name])[1]
                allowed = {gray + f'[{bit}]'}
                if bit == len(targets) - 1:
                    allowed.add(binary + f'[{bit}]')
                assert actual in allowed, (module, name, actual, allowed)
                register = actual.replace('[', '_').replace(']', '_s0')
                assert register in ff, (module, actual, 'source must be registered')
                sources[module, prefix, bit] = register
                connections.append({'module': module, 'target': name, 'source_net': actual})
    assert len(connections) == 88
    groups = (build / 'impl/pnr/robot_control_v21.timing_paths').read_text().split('=====\n')
    paths = []
    for base, count in [('core/controller_top/cache1/fifo', 15),
                        ('core/controller_top/cache2/fifo', 15),
                        ('core/UDP_Send/eth_dcfifo/fifo', 14)]:
        for target, gray, binary in [('rq1_wgray', 'wgray', 'wbin'), ('wq1_rgray', 'rgray', 'rbin')]:
            for bit in range(count):
                source = f'{base}/{sources[instances[base], target, bit]}'
                dest = f'{base}/{target}_{bit}_s0'
                candidates = []
                for group in groups:
                    lines = group.strip().splitlines()
                    if len(lines) < 5 or lines[0] != 'SETUP' or source not in lines or dest not in lines:
                        continue
                    a = lines.index(source)
                    b = lines.index(dest)
                    assert b > a
                    delay = float(lines[b+1]) - float(lines[a+2])
                    slack = float(lines[1])
                    assert 0 <= delay <= 8.000 and slack >= 0, (source, dest, delay, slack)
                    candidates.append((slack, delay))
                assert candidates, f'Missing actual STA crossing: {source} -> {dest}'
                slack, delay = min(candidates)
                paths.append({'from': source + '/Q', 'to': dest + '/D',
                              'bound_ns': 8.0, 'q_to_d_network_delay_ns': round(delay, 3),
                              'setup_slack_ns': slack})
    assert len(paths) == 88
    aliases = sum(bool(re.search(r'/(wbin|rbin)_\d+_s0/Q$', p['from'])) for p in paths)
    result = {'status': 'PASS', 'paths_verified': len(paths), 'optimized_msb_paths': aliases,
              'netlist_sha256': sha(net), 'constraints_sha256': sha(project / 'robot_fmc_remote.sdc'),
              'fifo_rtl_sha256': sha(project / 'camera/camera_async_fifo.v'),
              'bitstream_sha256': sha(project / 'impl/pnr/robot_control_v21.fs'),
              'maximum_network_delay_ns': max(p['q_to_d_network_delay_ns'] for p in paths),
              'scope': 'Registered pointer Q to first synchronizer D only; later logic remains timed.',
              'hardware_tested': False, 'paths': paths}
    (project / 'reports/gray-cdc-verification.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', required=True, type=Path)
    args = parser.parse_args()
    result = verify(Path(__file__).resolve().parents[1], args.build_dir)
    print(f"PASS {result['paths_verified']} registered Gray crossings; max Q→D network delay {result['maximum_network_delay_ns']} ns")
