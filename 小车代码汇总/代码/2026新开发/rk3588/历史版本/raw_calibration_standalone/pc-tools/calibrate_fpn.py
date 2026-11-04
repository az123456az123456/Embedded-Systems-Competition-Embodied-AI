#!/usr/bin/env python3
"""Create RAW8 shared-column FPN coefficients; arithmetic runs off FPGA.

Input files are RAW8 frame stacks or NumPy HxW / FxHxW arrays. Capture dark
and uniform bright frames at IDENTICAL locked exposure/gain/white balance.
The FPGA table is dark8 + unsigned gainQ2.8, not raw Dark16/Bright16 means.
No coefficients from an ordinary scene are accepted as a flat-field image.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:
    np = None  # Buildroot RAW8 path below needs only the Python standard library.


DEFAULT_DARK_PHASE_LIMIT = 2.0


def validate_dark_phase_limit(value):
    if not math.isfinite(value) or not 0 <= value <= 255:
        raise ValueError('max_dark_phase_delta must be finite and in 0..255 RAW8 DN')


def dark_phase_diagnostics(dark_means, dark_q, gain_q, selected_limit):
    """Report the approximation; increasing a limit does not fix row FPN."""
    width = len(dark_q)
    deltas = [abs(float(dark_means[0][x]) - float(dark_means[1][x])) for x in range(width)]
    relaxed = max(deltas) > DEFAULT_DARK_PHASE_LIMIT
    return {
        'validation_mode': 'shared-column-relaxed-dark-validation' if relaxed else 'shared-column-default',
        'dark_phase_diagnostics': {
            'default_limit_dn': DEFAULT_DARK_PHASE_LIMIT,
            'selected_limit_dn': float(selected_limit),
            'columns_over_default': sum(v > DEFAULT_DARK_PHASE_LIMIT for v in deltas),
            'total_columns': width,
            'maximum_phase_delta_dn': max(deltas),
            'maximum_ideal_shared_dark_residual_dn': max(deltas) / 2,
            'maximum_integer_dark_residual_dn': max(abs(float(dark_means[p][x]) - int(dark_q[x])) for p in (0, 1) for x in range(width)),
            'maximum_residual_after_quantized_gain_before_clipping_dn': max(abs(float(dark_means[p][x]) - int(dark_q[x])) * int(gain_q[x]) / 256 for p in (0, 1) for x in range(width)),
            'scope': 'Mean dark offsets only; excludes temporal/pixel noise, clipping, output rounding, demosaic, resize and gamma.'},
        'usage_constraints': {'requires_fpga_gamma_off': True,
                              'scope': 'Temporary approximate FPN evaluation only; unsuitable with FPGA Gamma enabled.',
                              'automatically_enforced_by_v40_uploader': False} if relaxed else {},
        'warnings': ['APPROXIMATE shared-column calibration: row-phase dark difference exceeds the default 2 DN limit. Use ONLY with FPGA Gamma OFF for temporary evaluation. The selected limit does not add phase-specific dark correction; v40 does not enforce this restriction automatically.'] if relaxed else []}


def raw_means_standard_library(paths, width, height):
    """Streaming RAW8 statistics without NumPy; no full frame is retained."""
    if width < 2 or width > 1024 or width % 2 or height < 2 or height % 2:
        raise ValueError('width<=1024 and even positive Bayer dimensions required')
    sums = [[0] * width for _ in range(2)]
    saturated = [0] * width
    frame_count = 0
    for name in paths:
        path = Path(name)
        if path.suffix == '.npy':
            raise ValueError('NumPy is needed only for .npy input; use headerless RAW8 on Buildroot')
        size = path.stat().st_size
        if not size or size % (width * height):
            raise ValueError(f'{path}: incomplete RAW8 frame')
        frames = size // (width * height)
        with path.open('rb') as f:
            for _ in range(frames):
                for row in range(height):
                    data = f.read(width)
                    if len(data) != width:
                        raise ValueError(f'{path}: shortened while reading')
                    total = sums[row % 2]
                    for col, value in enumerate(data):
                        total[col] += value
                        saturated[col] += value >= 254
        frame_count += frames
    if not frame_count:
        raise ValueError('no RAW8 input frames')
    phase_count = frame_count * (height // 2)
    return [[v / phase_count for v in phase] for phase in sums], [v / (frame_count * height) for v in saturated], frame_count


def calibrate_raw_standard_library(dark_paths, bright_paths, width, height,
                                   min_span=16.0, max_phase_mismatch=.08,
                                   max_dark_phase_delta=2.0, max_saturated_fraction=.01):
    validate_dark_phase_limit(max_dark_phase_delta)
    d, _, dark_frames = raw_means_standard_library(dark_paths, width, height)
    b, saturated, bright_frames = raw_means_standard_library(bright_paths, width, height)
    span = [[b[p][x] - d[p][x] for x in range(width)] for p in range(2)]
    if min(min(v) for v in span) < min_span:
        raise ValueError('bright-dark span too small; column gain is not observable')
    if max(saturated) > max_saturated_fraction:
        raise ValueError('saturated flat-field columns cannot be calibrated')
    dark_phase_delta = max(abs(d[0][x] - d[1][x]) for x in range(width))
    if dark_phase_delta > max_dark_phase_delta:
        raise ValueError('row-phase dark offsets violate shared-column model: maximum %.6f DN > limit %.6f DN; phase-specific correction requires new FPGA/protocol support, not just a different upload order' % (dark_phase_delta, max_dark_phase_delta))
    references = [[sum(span[p][q::2]) / (width // 2) for q in range(2)] for p in range(2)]
    gains = [[references[p][x % 2] / span[p][x] for x in range(width)] for p in range(2)]
    shared = [(gains[0][x] + gains[1][x]) / 2 for x in range(width)]
    mismatch = [abs(gains[0][x] - gains[1][x]) / shared[x] for x in range(width)]
    if max(mismatch) > max_phase_mismatch:
        raise ValueError('row-phase gain mismatch violates shared-column model; phase-specific tables require new FPGA/protocol support and are not implemented by this generator')
    dark_q = [int((d[0][x] + d[1][x]) / 2 + .5) for x in range(width)]
    gain_q = [int(g * 256 + .5) for g in shared]
    if any(not 0 <= v <= 255 for v in dark_q) or any(not 1 <= v <= 1023 for v in gain_q):
        raise ValueError('coefficient out of dark8 / gainU10 Q2.8 range')
    return {
        'schema': 'FPN-COLUMN-RAW8-Q2.8-v1', 'width': width, 'height': height,
        'bayer': 'BGGR', 'raw_bits': 8, 'gain_fraction_bits': 8,
        'table_bits_per_column': 18, 'dark': dark_q, 'gain': gain_q,
        'model': 'one physical-column offset/gain shared across both Bayer row phases',
        'normalization': 'four CFA phase references before combining two row-phase gain estimates',
        'requires_locked_capture_parameters': True,
        'statistics': {'dark_frame_count': dark_frames, 'bright_frame_count': bright_frames,
                       'dark_phase_mean': d, 'bright_phase_mean': b, 'cfa_reference_spans': references,
                       'minimum_span': min(min(v) for v in span), 'maximum_gain_phase_mismatch': max(mismatch),
                       'maximum_dark_phase_delta': dark_phase_delta, 'maximum_saturated_fraction': max(saturated)},
        'limits': {'min_span': min_span, 'max_phase_mismatch': max_phase_mismatch,
                   'max_dark_phase_delta': max_dark_phase_delta, 'max_saturated_fraction': max_saturated_fraction},
        'limitations': ['No pixel/row FPN correction.',
                        'Color-dependent column response not shared between row phases is rejected.',
                        'Real sensor coefficients require RAW dark/flat captures; resized RGP1 RGB is insufficient.'],
        'calibration_backend': 'Python standard library streaming RAW8',
        **dark_phase_diagnostics(d, dark_q, gain_q, max_dark_phase_delta)}


def load_stack(paths, width, height):
    stacks = []
    for name in paths:
        path = Path(name)
        a = np.load(path, allow_pickle=False) if path.suffix == '.npy' else np.frombuffer(path.read_bytes(), np.uint8)
        if a.dtype != np.uint8:
            raise ValueError(f'{path}: RAW8 uint8 input required')
        if a.size == 0 or a.size % (width * height):
            raise ValueError(f'{path}: incomplete RAW8 frame')
        if a.ndim > 1 and a.shape[-2:] != (height, width):
            raise ValueError(f'{path}: dimensions differ from {width}x{height}')
        stacks.append(a.reshape(-1, height, width))
    return np.concatenate(stacks, axis=0)


def calibrate(dark, bright, min_span=16.0, max_phase_mismatch=0.08,
              max_dark_phase_delta=2.0, max_saturated_fraction=0.01):
    validate_dark_phase_limit(max_dark_phase_delta)
    if dark.ndim != 3 or bright.ndim != 3 or dark.shape[1:] != bright.shape[1:]:
        raise ValueError('dark/bright must have matching FxHxW frame geometry')
    _, height, width = dark.shape
    if width > 1024 or width < 2 or width % 2 or height % 2:
        raise ValueError('width<=1024 and even Bayer dimensions are required')
    # Keep both row parities independent until each CFA phase is normalized.
    d = np.stack([dark[:, p::2, :].mean(axis=(0, 1)) for p in range(2)])
    b = np.stack([bright[:, p::2, :].mean(axis=(0, 1)) for p in range(2)])
    span = b - d
    if np.any(span < min_span):
        raise ValueError('bright-dark span too small; column gain is not observable')
    saturated = (bright >= 254).mean(axis=(0, 1))
    if np.any(saturated > max_saturated_fraction):
        raise ValueError('saturated flat-field columns cannot be calibrated')
    dark_phase_delta = float(np.max(np.abs(d[0] - d[1])))
    if dark_phase_delta > max_dark_phase_delta:
        raise ValueError('row-phase dark offsets violate shared-column model: maximum %.6f DN > limit %.6f DN; phase-specific correction requires new FPGA/protocol support, not just a different upload order' % (dark_phase_delta, max_dark_phase_delta))
    gains = np.empty_like(span)
    references = np.empty((2, 2))
    for row_phase in range(2):
        for column_phase in range(2):
            references[row_phase, column_phase] = span[row_phase, column_phase::2].mean()
            gains[row_phase, column_phase::2] = references[row_phase, column_phase] / span[row_phase, column_phase::2]
    shared_gain = gains.mean(axis=0)
    phase_mismatch = np.abs(gains[0] - gains[1]) / shared_gain
    if np.any(phase_mismatch > max_phase_mismatch):
        raise ValueError('row-phase gain mismatch violates shared-column model; phase-specific tables require new FPGA/protocol support and are not implemented by this generator')
    dark_q = np.floor(d.mean(axis=0) + 0.5).astype(np.int64)
    gain_q = np.floor(shared_gain * 256.0 + 0.5).astype(np.int64)
    if np.any((dark_q < 0) | (dark_q > 255)) or np.any((gain_q < 1) | (gain_q > 1023)):
        raise ValueError('coefficient out of dark8 / gainU10 Q2.8 range')
    return {
        'schema': 'FPN-COLUMN-RAW8-Q2.8-v1', 'width': width, 'height': height,
        'bayer': 'BGGR', 'raw_bits': 8, 'gain_fraction_bits': 8,
        'table_bits_per_column': 18, 'dark': dark_q.tolist(), 'gain': gain_q.tolist(),
        'model': 'one physical-column offset/gain shared across both Bayer row phases',
        'normalization': 'four CFA phase references before combining two row-phase gain estimates',
        'requires_locked_capture_parameters': True,
        'statistics': {'dark_frame_count': dark.shape[0], 'bright_frame_count': bright.shape[0],
                       'dark_phase_mean': d.tolist(), 'bright_phase_mean': b.tolist(),
                       'cfa_reference_spans': references.tolist(),
                       'minimum_span': float(span.min()), 'maximum_gain_phase_mismatch': float(phase_mismatch.max()),
                       'maximum_dark_phase_delta': float(np.abs(d[0] - d[1]).max()),
                       'maximum_saturated_fraction': float(saturated.max())},
        'limits': {'min_span': min_span, 'max_phase_mismatch': max_phase_mismatch,
                   'max_dark_phase_delta': max_dark_phase_delta, 'max_saturated_fraction': max_saturated_fraction},
        'limitations': ['No pixel/row FPN correction.',
                        'Color-dependent column response not shared between row phases is rejected.',
                        'Real sensor coefficients require RAW dark/flat captures; resized RGP1 RGB is insufficient.'],
        **dark_phase_diagnostics(d, dark_q, gain_q, max_dark_phase_delta)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dark', nargs='+', required=True)
    ap.add_argument('--bright', nargs='+', required=True)
    ap.add_argument('--width', type=int, default=960)
    ap.add_argument('--height', type=int, default=1080)
    ap.add_argument('--eye', choices=['left', 'right'], required=True)
    ap.add_argument('--locked-capture', action='store_true', required=True,
                    help='assert exposure/gain/white balance and RAW transfer curve were identical and locked')
    ap.add_argument('--min-span', type=float, default=16)
    ap.add_argument('--max-phase-mismatch', type=float, default=.08)
    ap.add_argument('--max-dark-phase-delta', type=float, default=DEFAULT_DARK_PHASE_LIMIT,
                    help='maximum odd/even-row dark mean difference in RAW8 DN (default 2.0); raising this permits approximate shared-column evaluation, not phase-specific correction')
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    if np is None:
        table = calibrate_raw_standard_library(args.dark, args.bright, args.width, args.height,
                                              min_span=args.min_span, max_phase_mismatch=args.max_phase_mismatch,
                                              max_dark_phase_delta=args.max_dark_phase_delta)
    else:
        table = calibrate(load_stack(args.dark, args.width, args.height), load_stack(args.bright, args.width, args.height),
                          min_span=args.min_span, max_phase_mismatch=args.max_phase_mismatch,
                          max_dark_phase_delta=args.max_dark_phase_delta)
    for warning in table['warnings']:
        print('FPN warning: ' + warning, file=sys.stderr)
    table['eye'] = args.eye
    table['input_sha256'] = {str(Path(p)): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in args.dark + args.bright}
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(table, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    # Packed ROM form is convenient for simulation inspection. FPGA runtime
    # upload must still use BEGIN, ordered WRITE and COMMIT with packet CRC.
    words = [(d << 10) | g for d, g in zip(table['dark'], table['gain'])]
    out.with_suffix('.hex').write_text(''.join(f'{v:05x}\n' for v in words), encoding='ascii')
    print(f'PASS eye={args.eye} columns={len(words)} q=2.8 output={out}')


if __name__ == '__main__':
    main()
