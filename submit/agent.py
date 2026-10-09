import os
import numpy as np

# (dx, dy, dz) index permutation per allowed orientation.
# Mirrors src/ground_handling/utils.py::get_half_ext
_ORN = ((0, 1, 2), (0, 2, 1), (2, 1, 0), (1, 0, 2), (1, 2, 0), (2, 0, 1))

# Evaluation platform margins (README "PlacementValidator" section).
_SAFETY = 0.015
_START_MARGIN = 0.01
_CEILING = 0.018
_START_Z = 0.08
_INCLUSION = -0.005

# The simulator measures true closest points, we measure per-axis gaps, so keep
# enough slack that our estimate is never the optimistic one.
_PAD = 0.002

# Containment uses a negative margin, so a box flush against a wall is rejected.
# Stay a little further in than the limit, otherwise the test is a coin flip.
_FIT = 0.007

_CELL = 0.015
_FLAT_TOL = 0.03
_MAX_CAND = 16000
_STAMP = 0.025
_LIFT = 0.001

# Relative weight of each term in the placement key. Layers dominate, then
# profile coherence, then the sideways-slide rule, then depth, then flushness.
_W_LEVEL = 1e15
_W_COHER = 1e13
_W_TALL = 0.0
_W_SLIDE = 1e10
_W_DEPTH = 1e6
_W_FLUSH = 1e3
_W_SOFT = 1e18


def _permuted(l, w, h, orn):
    p = _ORN[orn]
    return (l, w, h)[p[0]], (l, w, h)[p[1]], (l, w, h)[p[2]]


def _qmat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class _Container:
    """Mirrors the simulator's container geometry, plus our packing state."""

    def __init__(self, info):
        off = float(info['center'][0])
        n = np.asarray(info['n_vecs'], dtype=float)
        p = np.asarray(info['points'], dtype=float) - np.array([off, 0.0, 0.0])

        self.index = int(info['index'])
        self.length = float(info['length'])
        self.width = float(info['width'])
        self.height = float(info['height'])
        self.thickness = float(info['thickness'])
        self.cut_x = float(info['cut_x'])
        self.is_prioritized = bool(info.get('is_prioritized', False))
        self.require_shelf = bool(info.get('shelf', False))

        # Interior bounds read straight off the planes the simulator uses for its
        # own containment test, so nothing about buffer/cut sizing is assumed.
        lo = np.full(3, -1e9)
        hi = np.full(3, 1e9)
        for k in range(3):
            e = np.zeros(3)
            e[k] = 1.0
            pos = np.all(np.abs(n - e) < 1e-6, axis=1)
            neg = np.all(np.abs(n + e) < 1e-6, axis=1)
            if pos.any():
                hi[k] = p[pos, k].min()
            if neg.any():
                lo[k] = p[neg, k].max()
        self.lo, self.hi = lo, hi
        self.buffer = float(lo[2] - self.thickness)
        # The containment test uses a negative margin, so a box resting exactly on
        # a wall fails it. Start the height map a hair inside the floor instead.
        self.floor_z = float(lo[2]) + _FIT
        self.open_y = float(lo[1])

        self.n_vecs = n
        self.plane_d = np.einsum('ij,ij->i', n, p)
        self.abs_n = np.abs(n)

        # Fixed obstacles that block the push-in path (they do not affect the
        # containment test, which is why the simulator treats them separately).
        shelf_z = self.height / 2.0 + self.thickness / 2.0 + self.buffer
        obstacles = [[-self.length / 2.0 + self.cut_x / 2.0 + self.thickness,
                      0.0, shelf_z, self.cut_x / 2.0,
                      self.width / 2.0 - self.thickness, self.thickness / 2.0]]
        if self.require_shelf:
            obstacles.append([0.0, self.width / 4.0, shelf_z,
                              self.length / 2.0 - self.thickness / 2.0,
                              self.width / 4.0 - self.thickness, self.thickness / 2.0])
        self.obstacles = np.array(obstacles, dtype=float)

        self.nx = max(2, int(round((hi[0] - lo[0]) / _CELL)))
        self.ny = max(2, int(round((hi[1] - lo[1]) / _CELL)))
        self.x0 = float(lo[0]) + _CELL * 0.5
        self.y0 = float(lo[1]) + _CELL * 0.5
        self.reset_state()

    def reset_state(self):
        self.boxes = []                                     # cx cy cz hx hy hz soft pri
        self.hmap = np.full((self.ny, self.nx), self.floor_z)
        self.layer_dz = None

    def _track_layer(self):
        """Height class of the surface currently being built on."""
        if not self.boxes:
            self.layer_dz = None
            return
        top = max(b[2] + b[5] for b in self.boxes)
        dz = [2.0 * b[5] for b in self.boxes if abs(b[2] + b[5] - top) < 1e-6]
        self.layer_dz = max(set(dz), key=dz.count) if dz else None

    def sync(self, packed_items):
        """Adopt the simulator's authoritative settled poses."""
        self.boxes = []
        for it in packed_items:
            pos, orn = it.get('pos'), it.get('orn')
            if pos is None:
                continue
            l, w, h = it['length'], it['width'], it['height']
            q = orn
            if q is None:
                hx, hy, hz = l / 2.0, w / 2.0, h / 2.0
            else:
                q = np.asarray(q, dtype=float)
                if q.shape != (4,) or abs(float(np.linalg.norm(q)) - 1.0) > 1e-3:
                    hx, hy, hz = l / 2.0, w / 2.0, h / 2.0
                else:
                    hx, hy, hz = (np.abs(_qmat(q)) @ np.array([l, w, h]) / 2.0).tolist()
            self.boxes.append([float(pos[0]), float(pos[1]), float(pos[2]),
                               float(hx), float(hy), float(hz),
                               bool(it.get('is_soft', False)),
                               bool(it.get('is_prioritized', False))])
        self.hmap = self._rebuild()

    def _rebuild(self):
        hmap = np.full((self.ny, self.nx), self.floor_z)
        for b in self.boxes:
            self._stamp(hmap, b[0], b[1], b[2], b[3], b[4], b[5])
        return hmap

    def _stamp(self, hmap, x, y, z, hx, hy, hz):
        # Grow by the packing gap so that the sliver of empty floor between two
        # neighbours reads as being at their level. Without this nothing wider than
        # a single placed item can ever rest on top of a finished layer.
        m = _STAMP
        i0 = max(0, int(np.floor((x - hx - m - self.x0) / _CELL)) + 1)
        j0 = max(0, int(np.floor((y - hy - m - self.y0) / _CELL)) + 1)
        i1 = min(self.nx, max(i0 + 1, int(np.ceil((x + hx + m - self.x0) / _CELL))))
        j1 = min(self.ny, max(j0 + 1, int(np.ceil((y + hy + m - self.y0) / _CELL))))
        if i1 > i0 and j1 > j0:
            np.maximum(hmap[j0:j1, i0:i1], z + hz, out=hmap[j0:j1, i0:i1])

    # ---------------- simulator-mirroring checks ----------------

    def _entry(self, pos, half):
        """Reproduce the simulator's insertion height and insertion x."""
        hx, hy, hz = half[:, 0], half[:, 1], half[:, 2]

        eff = np.full(len(pos), _START_Z)
        bottom = pos[:, 2] - hz
        for r in (self.thickness, self.height / 2.0 + self.thickness + self.buffer):
            hit = (eff > 0) & (bottom - r >= 0.0) & (bottom - r <= 0.05)
            eff = np.where(hit, 0.0, eff)
        top = pos[:, 2] + hz
        for c in (self.height / 2.0 + self.buffer,
                  self.height + self.buffer - self.thickness):
            clearance = c - top
            hit = (eff > 0.0) & (clearance >= 0.0) & (clearance < (eff + _CEILING))
            eff = np.where(hit, np.maximum(0.0, clearance - _CEILING - 0.0005), eff)
        ze = np.minimum(self.hi[2] - hz - _START_MARGIN, pos[:, 2] + eff)

        x_min = -self.length / 2.0 + self.thickness + self.cut_x + hx + _START_MARGIN
        x_max = self.length / 2.0 - self.thickness - hx - _START_MARGIN
        xe = np.clip(pos[:, 0], x_min, x_max)
        return xe, ze

    def contained(self, pos, half):
        dots = self.n_vecs @ pos.T - self.plane_d[:, None] + self.abs_n @ half.T
        return np.all(dots <= _INCLUSION, axis=0)

    def _collide(self, corners, margins):
        if self.boxes:
            arr = np.asarray(self.boxes, dtype=float)
            cen, hei = arr[:, 0:3], arr[:, 3:6]
        else:
            cen = np.zeros((0, 3))
            hei = np.zeros((0, 3))
        if self.obstacles.size:
            cen = np.concatenate([cen, self.obstacles[:, 0:3]], axis=0)
            hei = np.concatenate([hei, self.obstacles[:, 3:6]], axis=0)
        out = np.empty(len(corners), dtype=bool)
        if len(cen) == 0:
            out[:] = False
            return out
        cen = cen[None, :, 0:3]
        hei = hei[None, :, :]
        # chunked so a large candidate batch cannot blow up memory
        step = max(1, 2 ** 22 // max(1, len(self.boxes) + len(self.obstacles)))
        for a in range(0, len(corners), step):
            b = min(a + step, len(corners))
            blk = corners[a:b]
            overlap = np.abs(blk[:, None, 0:3] - cen) < (
                blk[:, None, 3:6] + hei + margins[a:b][:, None, :])
            out[a:b] = np.any(np.all(overlap, axis=2), axis=1)
        return out

    def feasible(self, pos, half):
        """Containment plus the full push-in corridor, batched over candidates."""
        ok = self.contained(pos, half)
        if not ok.any():
            return ok
        xe, ze = self._entry(pos, half)
        hx, hy, hz = half[:, 0], half[:, 1], half[:, 2]

        # sweep A: opening -> target depth, travelling at the insertion height
        y_lo = self.open_y - hy
        y_hi = pos[:, 1] + hy
        # sweep B: lateral slide to the final x, at the target depth
        x_lo = np.minimum(xe, pos[:, 0]) - hx
        x_hi = np.maximum(xe, pos[:, 0]) + hx
        # plus the resting position itself, which the simulator only discovers
        # through interpenetration once it has settled.
        corners = np.concatenate([
            np.stack([xe, (y_lo + y_hi) / 2.0, ze, hx, (y_hi - y_lo) / 2.0, hz], axis=1),
            np.stack([(x_lo + x_hi) / 2.0, pos[:, 1], ze, (x_hi - x_lo) / 2.0, hy, hz], axis=1),
            np.concatenate([pos, half], axis=1),
        ], axis=0)
        m = _SAFETY + _PAD
        sweep = np.array([m, m, m])
        # A resting box sits on top of whatever carries it, so it needs no vertical
        # slack at all; only the lateral gap to its neighbours matters.
        rest = np.array([0.004, 0.004, 0.0])
        margins = np.concatenate([np.repeat(sweep[None], 2 * len(pos), axis=0),
                                  np.repeat(rest[None], len(pos), axis=0)])
        blocked = self._collide(corners, margins)
        n = len(pos)
        ok &= ~blocked[:n] & ~blocked[n:2 * n] & ~blocked[2 * n:]
        return ok


class Agent:
    def __init__(self, module_path: str):
        self.containers = []
        self.plan = None
        self.plan_i = 0

    # ---------------- setup ----------------

    def get_init_states(self, init_states: dict):
        self.containers = [_Container(c) for c in init_states['container_list']]
        self.plan = None
        self.plan_i = 0
        return True

    def optimize(self, item_list: list):
        try:
            for c in self.containers:
                c.reset_state()
            plan = self._run_pack(list(item_list))
        except Exception:
            if os.environ.get('BAG_DEBUG'):
                raise
            plan = []
        # The platform requires a permutation of every index, so anything the
        # planner could not place still has to be appended in some order.
        order = [s['index'] for s in plan]
        placed = set(order)
        order += [it['index'] for it in item_list if it['index'] not in placed]
        self.plan = plan
        self.plan_i = 0
        return order

    # ---------------- policy ----------------

    def policy(self, observation: dict):
        pool = observation.get('pool_list', [])
        if not pool:
            return self._fallback()

        for c, info in zip(self.containers, observation.get('container_list', [])):
            c.sync(info.get('packed_items', []))

        act = self._from_plan(pool)
        if act is not None:
            return act

        act = self._online(pool)
        if act is None:
            return self._fallback()
        return act

    def _fallback(self):
        return {'item_idx': 0, 'container_idx': 0,
                'place_pos': np.array([0.0, 0.0, 0.05], dtype=np.float32),
                'orientation': 0}

    def _from_plan(self, pool):
        if not self.plan or self.plan_i >= len(self.plan):
            return None
        step = self.plan[self.plan_i]
        slot = next((i for i, p in enumerate(pool) if p['index'] == step['index']), None)
        if slot is None:
            return None
        self.plan_i += 1
        return {'item_idx': int(slot), 'container_idx': int(step['container_idx']),
                'place_pos': np.array(step['pos'], dtype=np.float32),
                'orientation': int(step['orn'])}

    def _online(self, pool):
        best = None
        for slot, item in enumerate(pool):
            move = self._best_move(item)
            if move is not None and (best is None or move['key'] < best[0]['key']):
                best = (move, slot)
        if best is None:
            return None
        move, slot = best
        return {'item_idx': int(slot), 'container_idx': int(move['container_idx']),
                'place_pos': np.array(move['pos'], dtype=np.float32),
                'orientation': int(move['orn'])}

    # ---------------- packing core ----------------

    def _run_pack(self, items):
        steps = []
        remaining = [dict(it) for it in items]
        while remaining:
            best = None
            for item in remaining:
                move = self._best_move(item)
                if move is not None and (best is None or move['key'] < best[1]['key']):
                    best = (item, move)
            if best is None:
                break
            item, move = best
            self._commit(move)
            steps.append({'index': item['index'], 'container_idx': move['container_idx'],
                          'pos': list(move['pos']), 'orn': move['orn']})
            remaining = [it for it in remaining if it['index'] != item['index']]
        return steps

    def _commit(self, move):
        c = self.containers[move['container_idx']]
        x, y, z = move['pos']
        hx, hy, hz = move['half']
        c.boxes.append([x, y, z, hx, hy, hz, move['soft'], move['pri']])
        c._stamp(c.hmap, x, y, z, hx, hy, hz)
        c._track_layer()

    def _best_move(self, item):
        parts = []
        for c in self.containers:
            cand = self._candidates(c, item)
            if cand is not None and len(cand):
                parts.append(cand)
        if not parts:
            return None
        allc = np.concatenate(parts, axis=0)
        key, ci, x, y, z, orn = allc.T
        order = np.lexsort((key, ci))

        soft = bool(item.get('is_soft', False))
        pri = bool(item.get('is_prioritized', False))
        best = None
        starts = np.searchsorted(ci[order], np.arange(len(self.containers) + 1))
        for k, c in enumerate(self.containers):
            rows = order[starts[k]:starts[k + 1]][:_MAX_CAND]
            if not len(rows):
                continue
            pos = np.stack([x[rows], y[rows], z[rows]], axis=1)
            half = np.array([_half_ext(item, int(orn[t])) for t in rows])
            ok = c.feasible(pos, half)
            if not ok.any():
                continue
            t = rows[int(np.argmax(ok))]
            if best is None or key[t] < best['key']:
                best = {'container_idx': k, 'pos': (float(x[t]), float(y[t]), float(z[t])),
                        'orn': int(orn[t]), 'key': float(key[t]),
                        'half': _half_ext(item, int(orn[t])), 'soft': soft, 'pri': pri}
        return best

    def _candidates(self, c, item):
        """Array of [key, container, x, y, z, orientation] for every resting spot."""
        l, w, h = item['length'], item['width'], item['height']
        hmap = c.hmap
        soft = bool(item.get('is_soft', False))
        xc = (c.lo[0] + c.hi[0]) / 2.0
        span = np.array([c.hi[0] - c.lo[0], c.hi[1] - c.lo[1], c.hi[2] - c.lo[2]])
        if c.boxes:
            barr = np.asarray(c.boxes, dtype=float)
            bx, bhx = barr[:, 0], barr[:, 3]
        else:
            bx = np.zeros(0)
            bhx = np.zeros(0)
        gap = _SAFETY + _PAD
        out = []

        for orn in range(6):
            dx, dy, dz = _permuted(l, w, h, orn)
            hx, hy, hz = dx / 2.0, dy / 2.0, dz / 2.0
            if dx > span[0] or dy > span[1] or dz > span[2] + _FIT:
                continue
            # Only ever lay an item on its smallest face. A box stood on end seals
            # the push-in corridor in front of it for everything behind it, which
            # costs far more than the extra orientation is ever worth.
            if dz > min(l, w, h) + 1e-9:
                continue

            gx = max(1, int(np.ceil(dx / _CELL)))
            gy = max(1, int(np.ceil(dy / _CELL)))
            if gx > c.nx or gy > c.ny:
                continue

            sw = np.lib.stride_tricks.sliding_window_view
            hmax, hmin = hmap, hmap
            if gx > 1:
                hmax = np.max(sw(hmax, gx, axis=1), axis=-1)
                hmin = np.min(sw(hmin, gx, axis=1), axis=-1)
            if gy > 1:
                hmax = np.max(sw(hmax, gy, axis=0), axis=-1)
                hmin = np.min(sw(hmin, gy, axis=0), axis=-1)

            base = hmax
            ok = ((hmax - hmin) <= _FLAT_TOL) & (base + dz <= c.hi[2] - _FIT)
            if not ok.any():
                continue

            jj, ii = np.nonzero(ok)
            zs = base[jj, ii] + hz + _LIFT
            # A window starting at cell j covers cells j..j+gx-1, so the box
            # centre sits at the window's midpoint, not its first cell.
            xs = np.clip(c.x0 + (ii + (gx - 1) * 0.5) * _CELL,
                         c.lo[0] + hx + _FIT, c.hi[0] - hx - _FIT)
            ys = np.clip(c.y0 + (jj + (gy - 1) * 0.5) * _CELL,
                         c.lo[1] + hy + _FIT, c.hi[1] - hy - _FIT)

            # Prefer sitting flush against a wall or a neighbour. Seeking the
            # centre on its own opens dead strips along the walls that no later
            # item is narrow enough to fill.
            flush = np.concatenate([
                np.array([c.lo[0] + hx + _FIT, c.hi[0] - hx - _FIT]),
                bx + bhx + hx + gap, bx - bhx - hx - gap,
            ])
            flush = flush[(flush > c.lo[0] + hx + _FIT - 1e-9) &
                          (flush < c.hi[0] - hx - _FIT + 1e-9)]
            if len(flush):
                # Quantised to whole cells so that "flush" always beats "nearly
                # flush", no matter how close to the centre the latter happens to be.
                fdist = np.round(np.min(np.abs(xs[:, None] - flush[None, :]), axis=1) / _CELL)
            else:
                fdist = np.zeros(len(xs))

            # A position outside [x_min, x_max] cannot be entered straight: the box
            # has to slide sideways at its own depth, which seals that whole band
            # of the row for anything placed after it. Fill those first.
            x_min = -c.length / 2.0 + c.thickness + c.cut_x + hx + _START_MARGIN
            x_max = c.length / 2.0 - c.thickness - hx - _START_MARGIN
            slide = ((xs < x_min - 1e-9) | (xs > x_max + 1e-9)).astype(float)

            # Order by flat layer, then the thinnest profile so each layer stays
            # level, then sideways-slide positions (see above), then deepest first
            # which keeps the push-in corridor open for whatever comes next. Soft
            # items go last so nothing ends up stacked on them.
            level = np.round(base[jj, ii] / _CELL)
            depth = np.round((c.hi[1] - ys) / _CELL)
            # Keep one item height per layer: a layer of mixed heights leaves the
            # next layer with nowhere level to rest, which is what stalls a purely
            # greedy packer.
            coher = 0.0 if (c.layer_dz is None or abs(dz - c.layer_dz) < 1e-9) else 1.0
            key = (level * _W_LEVEL + coher * _W_COHER - dz * _W_TALL
                   + (1.0 - slide) * _W_SLIDE + depth * _W_DEPTH + fdist * _W_FLUSH
                   + np.abs(xs - xc) + (_W_SOFT if soft else 0.0))

            out.append(np.stack([key, np.full(len(key), float(c.index)),
                                 xs, ys, zs, np.full(len(key), float(orn))], axis=1))

        if not out:
            return None
        return np.concatenate(out, axis=0)


def _half_ext(item, orn):
    dx, dy, dz = _permuted(item['length'], item['width'], item['height'], orn)
    return np.array([dx / 2.0, dy / 2.0, dz / 2.0])