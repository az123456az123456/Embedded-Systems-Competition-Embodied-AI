"""Session-local map evidence, latched after one second of fresh observations.

This is not a collision veto: current hits always win in DynamicMap. A later
one-second sequence can replace history, so moving objects are not immortal.
Unobserved cells, duplicate stamps and wall-clock waiting never add evidence.
"""
import numpy as np


class StableHistory:
    HOLD_NS = 1_000_000_000
    MAX_GAP_NS = 250_000_000

    def __init__(self, size):
        self.values = np.full(size, -1, dtype=np.int8)
        # Bounded by the existing one-million-cell map cap: 18 bytes/cell.
        # Vectorized timestamps avoid a Python loop over every traversed cell.
        self.candidate = np.full(size, -1, dtype=np.int8)
        self.since = np.zeros(size, dtype=np.uint64)
        self.seen = np.zeros(size, dtype=np.uint64)
        self.pending = np.empty(0, dtype=np.int32)
        self.last_stamp = 0
        self.free_count = self.occupied_count = 0

    def break_confirmation(self):
        self.candidate[self.pending] = -1
        self.pending = np.empty(0, dtype=np.int32)
        # Keep last_stamp: a duplicate cannot become fresh after a TF correction.

    def invalidate_free(self, indices):
        if len(indices) == 0:
            return
        i = np.asarray(indices, dtype=np.int32)
        # A new hit interrupts provisional FREE confirmation, but never turns
        # committed evidence back into UNKNOWN. DynamicMap applies the hit as
        # an immediate collision veto independently of the retained history.
        self.candidate[i[self.candidate[i] == 0]] = -1

    def observe(self, stamp, free, hits):
        if stamp <= self.last_stamp or stamp <= 0:
            return False
        contiguous = stamp - self.last_stamp <= self.MAX_GAP_NS
        pending = []
        hit_indices = np.fromiter(hits, dtype=np.int32, count=len(hits))
        self.invalidate_free(hit_indices)
        for value, cells in ((0, free - hits), (100, hits)):
            i = hit_indices if value == 100 else np.fromiter(cells, dtype=np.int32, count=len(cells))
            same = (self.seen[i] == self.last_stamp) & (self.candidate[i] == value) & contiguous
            self.since[i[~same]] = stamp
            self.seen[i] = stamp
            self.candidate[i] = value
            pending.append(i)
            eligible = i[(np.uint64(stamp) - self.since[i]) >= self.HOLD_NS]
            old = self.values[eligible]
            self.free_count += (len(eligible) if value == 0 else 0) - int(np.count_nonzero(old == 0))
            self.occupied_count += (len(eligible) if value == 100 else 0) - int(np.count_nonzero(old == 100))
            self.values[eligible] = value
        self.pending = np.concatenate(pending)
        self.last_stamp = stamp
        return True

    def reindex(self, old_width, old_height, width, height, ox, oy):
        x0, y0 = max(0, -ox), max(0, -oy)
        x1, y1 = min(old_width, width - ox), min(old_height, height - oy)
        for name, fill in (('values', -1), ('candidate', -1), ('since', 0), ('seen', 0)):
            old = getattr(self, name)
            new = np.full((height, width), fill, dtype=old.dtype)
            if x1 > x0 and y1 > y0:
                new[y0+oy:y1+oy, x0+ox:x1+ox] = old.reshape(old_height, old_width)[y0:y1, x0:x1]
            setattr(self, name, new.reshape(-1))
        x, y = self.pending % old_width + ox, self.pending // old_width + oy
        valid = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        self.pending = y[valid] * width + x[valid]
        self.free_count = int(np.count_nonzero(self.values == 0))
        self.occupied_count = int(np.count_nonzero(self.values == 100))

    def diagnostics(self):
        return dict(stable_confirmation_ns=self.HOLD_NS,
                    committed_history_policy='retain_known_until_explicit_session_reset',
                    stable_free_cells=self.free_count,
                    stable_occupied_cells=self.occupied_count,
                    pending_confirmation_cells=len(self.pending))
