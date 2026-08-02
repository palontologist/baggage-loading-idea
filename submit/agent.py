import numpy as np
import subprocess
import json
import os
import time

MAX_REQUESTS_PER_SECOND = 2.0
COOLDOWN_SECONDS = 1.0 / MAX_REQUESTS_PER_SECOND

def _validate_item(item):
    if not isinstance(item, dict):
        return False
    required_keys = {'index', 'length', 'height', 'width', 'mass'}
    if not required_keys.issubset(item.keys()):
        return False
    for key in ('index', 'length', 'height', 'width', 'mass'):
        val = item[key]
        if not isinstance(val, (int, float)):
            return False
        if isinstance(val, bool):
            return False
    return True

def _validate_container(container):
    if not isinstance(container, dict):
        return False
    required_keys = {'index', 'length', 'height', 'width'}
    if not required_keys.issubset(container.keys()):
        return False
    for key in ('index', 'length', 'height', 'width'):
        val = container[key]
        if not isinstance(val, (int, float)):
            return False
        if isinstance(val, bool):
            return False
    return True

def _sanitize_request(request):
    if not isinstance(request, dict):
        return None
    if 'type' not in request or 'payload' not in request:
        return None
    if not isinstance(request['type'], str):
        return None
    if not isinstance(request['payload'], dict):
        return None
    return request

class Agent():
    def __init__(self, module_path: str):
        abs_module_path = os.path.abspath(module_path)
        self.binary_path = os.path.join(abs_module_path, "rust_brain_wrapper.sh")
        self.env = os.environ.copy()
        self.rust_works = True
        self._last_request_time = 0.0
        try:
            self.process = subprocess.Popen(
                [self.binary_path],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1, env=self.env
            )
        except Exception as e:
            print(f"Failed to spawn Rust binary: {e}")
            self.rust_works = False

    def _send_request(self, request):
        if not _sanitize_request(request):
            return None
        current_time = time.monotonic()
        elapsed = current_time - self._last_request_time
        if elapsed < COOLDOWN_SECONDS:
            time.sleep(COOLDOWN_SECONDS - elapsed)
            current_time = time.monotonic()
        self._last_request_time = current_time
        try:
            line = json.dumps(request) + "\n"
            self.process.stdin.write(line)
            self.process.stdin.flush()
            response_line = self.process.stdout.readline()
            return json.loads(response_line) if response_line else None
        except:
            return None

    def get_init_states(self, init_states: dict):
        self.num_containers = len(init_states.get('container_list', []))
        return True

    def optimize(self, item_list: list):
        if self.rust_works:
            valid_items = [i for i in item_list if _validate_item(i)]
            if valid_items:
                rust_items = [{"id": i['index'], "size": {"width": i['length'], "height": i['height'], "depth": i['width']}, "weight": i['mass'], "is_soft": i.get('is_soft', False), "is_priority": i.get('is_prioritized', False)} for i in valid_items]
                response = self._send_request({"type": "Optimize", "payload": {"items": rust_items}})
                if response and response.get('type') == 'Optimize':
                    return response['payload']

        sorted_items = sorted(item_list, key=lambda x: (
            -x.get('is_priority', False),
            -x['mass'],
            -(x['length'] * x['width'] * x['height'])
        ))
        return [item['index'] for item in sorted_items]

    def policy(self, observation: dict):
        pool_list = observation.get('pool_list', [])

        if not pool_list:
            return {'item_idx': 0, 'container_idx': 0, 'place_pos': np.array([0, 0, 0], dtype=np.float32), 'orientation': 0}

        target_item = pool_list[0]

        if self.rust_works and _validate_item(target_item):
            rust_baggage = {"id": target_item['index'], "size": {"width": target_item['length'], "height": target_item['height'], "depth": target_item['width']}, "weight": target_item['mass'], "is_soft": target_item.get('is_soft', False), "is_priority": target_item.get('is_prioritized', False)}
            container_list = observation.get('container_list', [])
            valid_containers = [c for c in container_list if _validate_container(c)]
            rust_containers = [{"id": c['index'], "dimensions": {"width": c['length'], "height": c['height'], "depth": c['width']}, "loaded_items": []} for c in valid_containers]

            response = self._send_request({"type": "Policy", "payload": {"baggage": rust_baggage, "containers": rust_containers, "offline_plan_idx": None}})

            if response and response.get('type') == 'Policy' and response['payload']:
                p = response['payload']
                return {'item_idx': 0, 'container_idx': p['container_id'], 'place_pos': np.array([p['position']['x'], p['position']['y'], p['position']['z']], dtype=np.float32), 'orientation': 0}

        return {'item_idx': 0, 'container_idx': 0, 'place_pos': np.array([0.5, 0.5, 0.5], dtype=np.float32), 'orientation': 0}