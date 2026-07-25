import numpy as np
import math

class Agent():
    def __init__(self, module_path: str):
        self.container_info = None
        self.packed_local = []
        self.orientations = 6

    def get_init_states(self, init_states: dict):
        self.container_info = init_states.get('container_list', [{}])[0]
        self.packed_local = []
        return True

    def optimize(self, item_list: list):
        sorted_items = sorted(item_list, key=lambda x: (
            -x.get('is_prioritized', False),
            -(x['length'] * x['width'] * x['height']),
            -x.get('mass', 0)
        ))
        return [item['index'] for item in sorted_items]

    def policy(self, observation: dict):
        pool_list = observation.get('pool_list', [])
        if not pool_list:
            return self._default_action()

        target_item = pool_list[0]
        container = observation.get('container_list', [{}])[0]
        self._update_packed(container)

        length = container.get('length', 2.0)
        width = container.get('width', 1.45)
        height = container.get('height', 1.61)
        thickness = container.get('thickness', 0.04)
        cut_x = container.get('cut_x', 0.3)
        cut_y = container.get('cut_y', 0.3)
        offset_x = container.get('offset_x', 0.0)

        inner_len = length - 2 * thickness
        inner_wid = width - 2 * thickness
        inner_hgt = height - thickness
        il_max = inner_len / 2
        iw_max = inner_wid / 2

        best_action = None
        best_z = float('inf')

        for orn_idx in range(self.orientations):
            h_ext = self._half_ext(target_item, orn_idx)
            hl, hw, hh = h_ext

            if hl > il_max or hw > iw_max or hh > inner_hgt:
                continue

            z_floor = thickness + hh
            shelf_zs = [z_floor]
            for p in self.packed_local:
                top_z = p['z'] + p['ehh']
                shelf_zs.append(top_z + hh + 0.005)

            for z in sorted(shelf_zs):
                if z < z_floor - 0.01 or z > inner_hgt - hh + 0.01:
                    continue

                step_x = target_item['length'] * 0.45 + 0.01
                step_y = target_item['width'] * 0.45 + 0.01

                x_min = -inner_len / 2 + hl + 0.01
                x_max = inner_len / 2 - hl - 0.01
                y_min = -inner_wid / 2 + hw + 0.01
                y_max = inner_wid / 2 - hw - 0.01

                if x_min > x_max or y_min > y_max:
                    continue

                x_positions = [x_min + i * step_x for i in range(int((x_max - x_min) / step_x) + 1)]
                x_positions.append(x_max)

                y_positions = [y_min + i * step_y for i in range(int((y_max - y_min) / step_y) + 1)]
                y_positions.append(y_max)

                for x in x_positions:
                    for y in y_positions:
                        if self._in_cut_corner(x, y, inner_len, inner_wid, thickness, cut_x, cut_y):
                            continue
                        if self._collides(x, y, z, hl, hw, hh):
                            continue
                        if z < best_z:
                            best_z = z
                            best_action = {
                                'item_idx': 0,
                                'container_idx': 0,
                                'place_pos': np.array([x, y, z], dtype=np.float32),
                                'orientation': orn_idx
                            }
                        if best_z == z_floor:
                            return best_action

        if best_action is not None:
            return best_action

        return self._default_action()

    def _half_ext(self, item, orn_idx):
        l, w, h = item['length'], item['width'], item['height']
        if orn_idx == 0:
            return [l/2, w/2, h/2]
        elif orn_idx == 1:
            return [l/2, h/2, w/2]
        elif orn_idx == 2:
            return [h/2, w/2, l/2]
        elif orn_idx == 3:
            return [w/2, l/2, h/2]
        elif orn_idx == 4:
            return [w/2, h/2, l/2]
        elif orn_idx == 5:
            return [h/2, l/2, w/2]

    def _update_packed(self, container):
        self.packed_local = []
        offset_x = container.get('offset_x', 0.0)
        for p in container.get('packed_items', []):
            wpos = p.get('pos')
            if wpos is None or len(wpos) < 3:
                continue
            self.packed_local.append({
                'x': wpos[0] - offset_x,
                'y': wpos[1],
                'z': wpos[2],
                'l': p.get('length', 0),
                'w': p.get('width', 0),
                'h': p.get('height', 0),
                'ehl': p.get('length', 0) / 2,
                'ehw': p.get('width', 0) / 2,
                'ehh': p.get('height', 0) / 2,
            })

    def _in_cut_corner(self, x, y, inner_len, inner_wid, thickness, cut_x, cut_y):
        wall_x = -inner_len / 2
        wall_y = inner_wid / 2
        if x <= wall_x + cut_x and y >= wall_y - cut_y:
            return True
        return False

    def _collides(self, x, y, z, hl, hw, hh):
        for p in self.packed_local:
            dx = abs(x - p['x'])
            dy = abs(y - p['y'])
            dz = abs(z - p['z'])
            if dx < hl + p['ehl'] + 0.01 and dy < hw + p['ehw'] + 0.01 and dz < hh + p['ehh'] + 0.01:
                return True
        return False

    def _default_action(self):
        return {
            'item_idx': 0,
            'container_idx': 0,
            'place_pos': np.array([0.0, 0.0, 0.2], dtype=np.float32),
            'orientation': 0
        }