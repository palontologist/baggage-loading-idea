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
        except Exception as e:
            print(f"Failed to spawn Rust binary: {e}")
            self.rust_works = False

    def _send_request(self, request):
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
            rust_items = [{"id": i['index'], "size": {"width": i['length'], "height": i['height'], "depth": i['width']}, "weight": i['mass'], "is_soft": i.get('is_soft', False), "is_priority": i.get('is_prioritized', False)} for i in item_list]
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
        
        if self.rust_works:
            rust_baggage = {"id": target_item['index'], "size": {"width": target_item['length'], "height": target_item['height'], "depth": target_item['width']}, "weight": target_item['mass'], "is_soft": target_item.get('is_soft', False), "is_priority": target_item.get('is_prioritized', False)}
            rust_containers = [{"id": c['index'], "dimensions": {"width": c['length'], "height": c['height'], "depth": c['width']}, "loaded_items": []} for c in observation.get('container_list', [])]
            
            response = self._send_request({"type": "Policy", "payload": {"baggage": rust_baggage, "containers": rust_containers, "offline_plan_idx": None}})
            
            if response and response.get('type') == 'Policy' and response['payload']:
                p = response['payload']
                return {'item_idx': 0, 'container_idx': p['container_id'], 'place_pos': np.array([p['position']['x'], p['position']['y'], p['position']['z']], dtype=np.float32), 'orientation': 0}
        
        return {'item_idx': 0, 'container_idx': 0, 'place_pos': np.array([0.5, 0.5, 0.5], dtype=np.float32), 'orientation': 0}
