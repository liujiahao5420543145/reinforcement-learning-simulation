"""One-time export of the existing SB3 policy; C++ runtime uses only the text file."""
from pathlib import Path
from stable_baselines3 import PPO
root = Path(__file__).resolve().parents[1]
model = PPO.load(root / 'models/ppo_car')
layers = [model.policy.mlp_extractor.policy_net[0],
          model.policy.mlp_extractor.policy_net[2], model.policy.action_net]
with (root / 'models/ppo_car.txt').open('w') as f:
    f.write(f'{len(layers)}\n')
    for layer in layers:
        f.write(f'{layer.in_features} {layer.out_features}\n')
        for tensor in (layer.weight, layer.bias):
            f.write(' '.join(format(float(v), '.9g') for v in tensor.detach().cpu().flatten()) + '\n')
print('Exported models/ppo_car.txt')
