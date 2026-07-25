import numpy as np
import subprocess
import json
import os

class Agent():
    def __init__(self, module_path: str):
        abs_module_path = os.path.abspath(module_path)
        self.binary_path = os.path.join(abs_module_path, "rust_brain_wrapper.sh")
        self.env = os.environ.copy()
        self.rust_works = True
        try:
            self.process = subprocess.Popen(
                [self.binary_path],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1, env=self.env
            )
        except Exception:
            self.rust_works = False
        self.container_info = None
        self.packed_local = []

    def _send_request(self, request):
        try:
            line = json.dumps(request) + "\n"
            self.process.stdin.write(line)
            self.process.stdin.flush()
            response_line = self.process.stdout.readline()
            return json.loads(response_line) if response_line else None
        except Exception:
            return None

    def get_init_states(self, init_states: dict):
        self.container_info = init_states.get('container_list', [{}])[0]
        self.packed_local = []
        return True

    def optimize(self, item_list: list):
        if self.rust_works:
            rust_items = [{"id": i['index'], "size": {"width": i['length'], "height": i['height'], "depth": i['width']}, "weight": i['mass'], "is_soft": i.get('is_soft', False), "is_priority": i.get('is_prioritized', False)} for i in item_list]
            response = self._send_request({"type": "Optimize", "payload": {"items": rust_items}})
            if response and response.get('type') == 'Optimize':
                return response['payload']

        sorted_items = sorted(item_list, key=lambda x: (
            -x.get('is_prioritized', False),
            -(x['length'] * x['width'] * x['height']),
            -x.get('mass', 0)
        ))
        return [item['index'] for item in sorted_items]

    def policy(self, observation: dict):
        pool_list = observation.get('pool_list', [])

        if not pool_list:
            return {'item_idx': 0, 'container_idx': 0, 'place_pos': np.array([0, 0, 0], dtype=np.float32), 'orientation': 0}

        target_item = pool_list[0]
        container = observation.get('container_list', [{}])[0]

        if self.rust_works:
            rust_baggage = {
                "id": target_item['index'],
                "size": {"width": target_item['length'], "height": target_item['height'], "depth": target_item['width']},
                "weight": target_item['mass'],
                "is_soft": target_item.get('is_soft', False),
                "is_priority": target_item.get('is_prioritized', False)
            }
            rust_containers = [{
                "id": c['index'],
                "dimensions": {
                    "width": c['length'],
                    "height": c['height'],
                    "depth": c['width']
                },
                "loaded_items": []
            } for c in observation.get('container_list', [])]

            response = self._send_request({
                "type": "Policy",
                "payload": {
                    "baggage": rust_baggage,
                    "containers": rust_containers,
                    "offline_plan_idx": None
                }
            })

            if response and response.get('type') == 'Policy' and response['payload']:
                p = response['payload']
                return {
                    'item_idx': 0,
                    'container_idx': p['container_id'],
                    'place_pos': np.array([p['position']['x'], p['position']['y'], p['position']['z']], dtype=np.float32),
                    'orientation': 0
                }

        return self._python_policy(observation, target_item, container)

    def _python_policy(self, observation, target_item, container):
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
        il_half = inner_len / 2
        iw_half = inner_wid / 2

        best_action = None
        best_z = float('inf')

        for orn_idx in range(6):
            h_ext = self._half_ext(target_item, orn_idx)
            hl, hw, hh = h_ext

            if hl > il_half or hw > iw_half or hh > inner_hgt:
                continue

            min_z = thickness + hh
            max_z = inner_hgt - hh

            shelf_zs = [min_z]
            for p in self.packed_local:
                top_z = p['z'] + p['ebh']
                shelf_zs.append(top_z + hh + 0.008)

            shelf_zs = sorted(set(shelf_zs))

            for z in shelf_zs:
                if z < min_z - 0.001 or z > max_z + 0.001:
                    continue

                item_l = target_item['length']
                item_w = target_item['width']
                step_x = max(hl * 2 * 0.55, 0.03)
                step_y = max(hw * 2 * 0.55, 0.03)

                x_start = -il_half + hl + 0.015
                x_end = il_half - hl - 0.015
                y_start = -iw_half + hw + 0.015
                y_end = iw_half - hw - 0.015

                if x_start > x_end or y_start > y_end:
                    continue

                xp = x_start
                while xp <= x_end + 1e-9:
                    yp = y_start
                    while yp <= y_end + 1e-9:
                        if self._in_cut_corner(xp, yp, inner_len, inner_wid, thickness, cut_x, cut_y):
                            yp += step_y
                            continue
                        if self._collides(xp, yp, z, hl, hw, hh):
                            yp += step_y
                            continue
                        if z < best_z:
                            best_z = z
                            best_action = {
                                'item_idx': 0,
                                'container_idx': 0,
                                'place_pos': np.array([xp, yp, z], dtype=np.float32),
                                'orientation': orn_idx
                            }
                            return best_action
                        yp += step_y
                    xp += step_x

        if best_action is not None:
            return best_action

        return {
            'item_idx': 0,
            'container_idx': 0,
            'place_pos': np.array([0.0, 0.0, 0.2], dtype=np.float32),
            'orientation': 0
        }

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
            orn = p.get('orn')
            l = p.get('length', 0)
            w = p.get('width', 0)
            h = p.get('height', 0)
            ehl, ehw, ehh = self._effective_half_ext(l, w, h, orn)
            self.packed_local.append({
                'x': wpos[0] - offset_x,
                'y': wpos[1],
                'z': wpos[2],
                'ehl': ehl,
                'ehw': ehw,
                'ehh': ehh,
                'ebh': ehh,
            })

    def _effective_half_ext(self, l, w, h, orn):
        if orn is None:
            return [l/2, w/2, h/2]
        try:
            wq = orn[0]
            xq = orn[1]
            yq = orn[2]
            zq = orn[3]
        except (IndexError, TypeError):
            return [l/2, w/2, h/2]
        R = np.array([
            [1-2*(yq*yq+zq*zq), 2*(xq*yq-wq*zq), 2*(xq*zq+wq*yq)],
            [2*(xq*yq+wq*zq), 1-2*(xq*xq+zq*zq), 2*(yq*zq-wq*xq)],
            [2*(xq*zq-wq*yq), 2*(yq*zq+wq*xq), 1-2*(xq*xq+yq*yq)],
        ], dtype=np.float64)
        half = np.array([l/2, w/2, h/2], dtype=np.float64)
        eff = np.zeros(3, dtype=np.float64)
        for i in range(3):
            eff[i] = np.sum(np.abs(R[i]) * half)
        return [float(eff[0]), float(eff[1]), float(eff[2])]

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
            if dx < hl + p['ehl'] + 0.012 and dy < hw + p['ehw'] + 0.012 and dz < hh + p['ehh'] + 0.012:
                return True
        return False