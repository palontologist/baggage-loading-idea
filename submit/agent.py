import time

import numpy as np

# (dx, dy, dz) index permutation per allowed orientation.
# Mirrors src/ground_handling/utils.py::get_half_ext
_ORN = ((0, 1, 2), (0, 2, 1), (2, 1, 0), (1, 0, 2), (1, 2, 0), (2, 0, 1))

# Evaluation platform margins, from the README's PlacementValidator section.
_SAFETY = 0.015
_START_MARGIN = 0.01
_CEILING = 0.018
_START_Z = 0.08
_INCLUSION = -0.005

# The containment test uses a negative margin, so a box flush against a wall is
# rejected. Aim a little inside it: the test is a floating point comparison, and
# the scoring pass repeats it against the settled pose afterwards.
_CONTAIN = -0.006
_FIT = 0.008

# The simulator measures true closest points while we compare axis aligned gaps,
# so keep slack to make sure our estimate is never the optimistic one.
_PAD = 0.002

# The underside of a new item meets the top face of whatever carries it; the
# small lift keeps that contact off the exact boundary.
_LIFT = 0.001

# Height map resolution and how far outside a box its top is still treated as
# available surface.
_CELL = 0.015
_STAMP = 0.025

# A box has to be held up by enough of its underside, and that support has to sit
# under its middle, or it tips over when released.
_MIN_SUPPORT = 0.45
_CENTRE_TOL = 0.25

_MAX_CAND = 16000

# Placement ranking, in order of precedence. Layer first, then depth within a row,
# then how level the support is, then flushness against a wall or neighbour, then
# a pull towards the container's centre to keep the centre of gravity low. Soft
# items go last so nothing ends up stacked on them.
_W_LEVEL = 1e15
_W_SLIDE = 0.0
_W_DEPTH = 1e6
_W_BRIDGE = 1e3
_W_FLUSH = 1e3
_W_SOFT = 1e18
_W_PRI = 1e17

# Offline planning budget. The platform allows 180 s for optimize(); stay well
# inside it and keep policy() well inside its own limit.
_OPT_BUDGET = 150.0
_POLICY_BUDGET = 3.0


def _sliding_max(a, gx, gy):
    sw = np.lib.stride_tricks.sliding_window_view
    out = a
    if gx > 1:
        out = np.max(sw(out, gx, axis=1), axis=-1)
    if gy > 1:
        out = np.max(sw(out, gy, axis=0), axis=-1)
    return out


def _sliding_min(a, gx, gy):
    sw = np.lib.stride_tricks.sliding_window_view
    out = a
    if gx > 1:
        out = np.min(sw(out, gx, axis=1), axis=-1)
    if gy > 1:
        out = np.min(sw(out, gy, axis=0), axis=-1)
    return out


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
        self.offset_x = float(info['center'][0])
        n = np.asarray(info['n_vecs'], dtype=float)
        p = np.asarray(info['points'], dtype=float) - np.array([self.offset_x, 0.0, 0.0])

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
            # packed_items report world poses; everything here is container local
            self.boxes.append([float(pos[0]) - self.offset_x, float(pos[1]), float(pos[2]),
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
        return np.all(dots <= _CONTAIN, axis=0)

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

    def _stable(self, pos, half):
        """Reject spots that would not hold the box up once it is released.

        Two things go wrong without this. A spot whose underside only grazes a
        neighbouring box prints as supported on the height map, then tips and
        falls the full height of the container. And the settle test failing ends
        the whole episode, so one teetering box costs every item after it.
        """
        hx, hy, hz = half[:, 0], half[:, 1], half[:, 2]
        bottom = pos[:, 2] - hz
        on_floor = bottom <= self.floor_z + 0.02
        if not self.boxes:
            return on_floor

        arr = np.asarray(self.boxes, dtype=float)
        cen = np.concatenate([arr[:, 0:3], self.obstacles[:, 0:3]], axis=0)
        hei = np.concatenate([arr[:, 3:6], self.obstacles[:, 3:6]], axis=0)
        bx0 = cen[None, :, 0] - hei[None, :, 0]
        bx1 = cen[None, :, 0] + hei[None, :, 0]
        by0 = cen[None, :, 1] - hei[None, :, 1]
        by1 = cen[None, :, 1] + hei[None, :, 1]
        x0 = (pos[:, 0] - hx)[:, None]
        x1 = (pos[:, 0] + hx)[:, None]
        y0 = (pos[:, 1] - hy)[:, None]
        y1 = (pos[:, 1] + hy)[:, None]

        ox = np.maximum(0.0, np.minimum(x1, bx1) - np.maximum(x0, bx0))
        oy = np.maximum(0.0, np.minimum(y1, by1) - np.maximum(y0, by0))
        # only boxes whose top face is the underside we are resting on count
        touching = np.abs((cen[None, :, 2] + hei[None, :, 2]) - bottom[:, None]) < 0.02
        area = ox * oy * touching
        sup = area.sum(axis=1)

        mx = (area * (np.maximum(x0, bx0) + np.minimum(x1, bx1)) * 0.5).sum(axis=1)
        my = (area * (np.maximum(y0, by0) + np.minimum(y1, by1)) * 0.5).sum(axis=1)
        safe = np.maximum(sup, 1e-9)
        centred = ((np.abs(mx / safe - pos[:, 0]) <= _CENTRE_TOL * hx)
                   & (np.abs(my / safe - pos[:, 1]) <= _CENTRE_TOL * hy))
        held = (sup >= _MIN_SUPPORT * (4.0 * hx * hy)) & centred
        # Anything resting on a soft bag is a bad idea: soft cargo crushes under
        # load and the platform penalises having anything stacked on it.
        return on_floor | held

    def feasible(self, pos, half, relax=False):
        """Containment plus the full push-in corridor, batched over candidates."""
        ok = self.contained(pos, half)
        if not relax:
            ok = ok & self._stable(pos, half)
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
        self.has_priority = False
        self._pri_pen = 0.0

    # ---------------- setup ----------------

    def get_init_states(self, init_states: dict):
        self.containers = [_Container(c) for c in init_states['container_list']]
        self.has_priority = any(c.is_prioritized for c in self.containers)
        return True

    def optimize(self, item_list: list):
        """Decide the order the items should arrive in.

        Only the ordering matters downstream: policy() re-derives every position
        from the live, settled state, because replaying coordinates computed here
        breaks as soon as anything shifts by a millimetre while settling.
        """
        try:
            for c in self.containers:
                c.reset_state()
            order = self._run_pack([dict(it) for it in item_list])
        except Exception:
            order = []
        placed = set(order)
        # The platform requires a permutation of every index, so anything the
        # planner could not place still has to be appended in some order.
        return order + [it['index'] for it in item_list if it['index'] not in placed]

    # ---------------- policy ----------------

    def policy(self, observation: dict):
        pool = observation.get('pool_list', [])
        if not pool:
            return self._fallback()

        infos = observation.get('container_list', [])
        if not self.containers and infos:
            # The runner starts a fresh agent if optimize() overruns its budget, and
            # it does not call get_init_states again. Rebuild from the observation,
            # which carries the same geometry, rather than failing every action.
            self.containers = [_Container(c) for c in infos]
            self.has_priority = any(c.is_prioritized for c in self.containers)
        for c, info in zip(self.containers, infos):
            c.sync(info.get('packed_items', []))

        # Positions are always recomputed against the live, settled state. The
        # offline pass only supplies the item ordering: replaying its coordinates
        # verbatim breaks as soon as anything settles even a few millimetres.
        act = self._online(pool)
        return act if act is not None else self._fallback()

    def _fallback(self):
        return {'item_idx': 0, 'container_idx': 0,
                'place_pos': np.array([0.0, 0.0, 0.05], dtype=np.float32),
                'orientation': 0}

    def _online(self, pool):
        """Best available spot for anything currently on the conveyor."""
        best = None
        best_key = None
        deadline = time.time() + _POLICY_BUDGET
        for slot, item in enumerate(pool):
            move = self._best_move(item)
            if move is not None and (best_key is None or move['key'] < best_key):
                best, best_key = (move, slot), move['key']
            if time.time() > deadline:
                break
        if best is None:
            # Nothing is safely supported anywhere. Rather than end the episode on
            # an impossible placement, take the best spot that at least passes the
            # simulator's containment and push-in checks.
            for slot, item in enumerate(pool):
                move = self._best_move(item, relax=True)
                if move is not None:
                    best = (move, slot)
                    break
        if best is None:
            return None
        move, slot = best
        return {'item_idx': int(slot), 'container_idx': int(move['container_idx']),
                'place_pos': np.array(move['pos'], dtype=np.float32),
                'orientation': int(move['orn'])}

    # ---------------- packing core ----------------

    def _run_pack(self, items):
        """Pack every item greedily; only the resulting order is used downstream."""
        order = []
        remaining = [dict(it) for it in items]
        deadline = time.time() + _OPT_BUDGET
        while remaining:
            best = None
            for item in remaining:
                move = self._best_move(item)
                if move is not None and (best is None or move['key'] < best[1]['key']):
                    best = (item, move)
            if best is None or time.time() > deadline:
                break
            item, move = best
            self._commit(move)
            order.append(item['index'])
            remaining = [it for it in remaining if it['index'] != item['index']]
        return order

    def _commit(self, move):
        c = self.containers[move['container_idx']]
        x, y, z = move['pos']
        hx, hy, hz = move['half']
        c.boxes.append([x, y, z, hx, hy, hz, move['soft'], move['pri']])
        c._stamp(c.hmap, x, y, z, hx, hy, hz)

    def _best_move(self, item, relax=False):
        self._pri_pen = (_W_PRI if (self.has_priority
                                   and item.get('is_prioritized', False)) else 0.0)
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
            ok = c.feasible(pos, half, relax)
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
            if dx > span[0] or dy > span[1] or dz > span[2] - _FIT:
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

            hmax = _sliding_max(hmap, gx, gy)
            hmin = _sliding_min(hmap, gx, gy)

            base = hmax
            ok = (base + dz <= c.hi[2] - _FIT)
            if not ok.any():
                continue

            jj, ii = np.nonzero(ok)
            zs = base[jj, ii] + hz + _LIFT
            # A window starting at cell j covers cells j..j+gx-1, so the box
            # centre sits at the window's midpoint, not its first cell.
            xs = np.clip(c.x0 + (ii + (gx - 1) * 0.5) * _CELL,
                         c.lo[0] + hx + _FIT, c.hi[0] - hx - _FIT)
            # Only boxes whose centre lies in this span go in straight. Anything
            # outside it has to be slid in sideways at its own depth, and that
            # slide seals the whole band for every later box at the same height.
            entry_lo = -c.length / 2.0 + c.thickness + c.cut_x + hx + _START_MARGIN
            entry_hi = c.length / 2.0 - c.thickness - hx - _START_MARGIN
            slide = ((xs < entry_lo - 1e-9) | (xs > entry_hi + 1e-9)).astype(float)
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

            # Order by flat layer, then the thinnest profile so each layer stays
            # level, then sideways-slide positions (see above), then deepest first
            # which keeps the push-in corridor open for whatever comes next. Soft
            # items go last so nothing ends up stacked on them.
            level = np.round(base[jj, ii] / _CELL)
            depth = np.round((c.hi[1] - ys) / _CELL)
            # Bridging a void lets the next layer span several boxes at once, which
            # is worth a lot of extra fill, but a box teetering on a pedestal tips
            # over and ends the episode. Take a level support whenever one is on
            # offer and only bridge when nothing else fits.
            bridge = np.minimum(np.round((hmax - hmin)[jj, ii] / _CELL), 40.0)
            # A priority item left in an ordinary container is penalised by the
            # platform whenever a dedicated priority container exists.
            wrong = 0.0 if (c.is_prioritized or not self._pri_pen) else self._pri_pen
            key = (level * _W_LEVEL + slide * _W_SLIDE
                   + depth * _W_DEPTH + bridge * _W_BRIDGE
                   + fdist * _W_FLUSH + np.abs(xs - xc) + wrong
                   + (_W_SOFT if soft else 0.0))

            out.append(np.stack([key, np.full(len(key), float(c.index)),
                                 xs, ys, zs, np.full(len(key), float(orn))], axis=1))

        if not out:
            return None
        return np.concatenate(out, axis=0)


def _half_ext(item, orn):
    dx, dy, dz = _permuted(item['length'], item['width'], item['height'], orn)
    return np.array([dx / 2.0, dy / 2.0, dz / 2.0])