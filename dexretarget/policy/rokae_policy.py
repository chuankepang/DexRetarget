"""The same base-frame TCP6 + native-hand6 contract for training and inference."""

import dataclasses

import numpy as np


def parse_image(value):
    image = np.asarray(value)
    if image.ndim != 3:
        raise ValueError("Expected RGB image with three dimensions")
    if image.shape[0] == 3 and image.shape[-1] != 3:
        image = np.moveaxis(image, 0, -1)
    if image.shape[-1] != 3:
        raise ValueError("Expected three RGB channels")
    if np.issubdtype(image.dtype, np.floating):
        image = np.clip(image * 255, 0, 255).round().astype(np.uint8)
    if image.dtype != np.uint8:
        raise ValueError("Expected uint8 RGB or float RGB in [0,1]")
    return image


@dataclasses.dataclass(frozen=True)
class RokaeInputs:
    def __call__(self, data):
        state = np.asarray(data["observation.state"], dtype=np.float32)
        if state.shape != (12,) or not np.all(np.isfinite(state)):
            raise ValueError("State must contain measured TCP6 and actual hand6")
        global_image = parse_image(data["observation.images.global"])
        wrist_image = parse_image(data["observation.images.wrist"])
        result = {
            "state": state,
            "image": {"base_0_rgb": global_image, "left_wrist_0_rgb": wrist_image,
                      "right_wrist_0_rgb": np.zeros_like(global_image)},
            "image_mask": {"base_0_rgb": np.True_, "left_wrist_0_rgb": np.True_, "right_wrist_0_rgb": np.False_},
        }
        if "action" in data:
            result["actions"] = np.asarray(data["action"], dtype=np.float32)
        if "prompt" in data:
            result["prompt"] = data["prompt"]
        return result


@dataclasses.dataclass(frozen=True)
class RokaeOutputs:
    def __call__(self, data):
        # OpenPI reverses normalization before this output transform. These
        # are absolute TCP references, not measured future states or deltas.
        actions = np.asarray(data["actions"])[..., :12].copy()
        if not np.all(np.isfinite(actions)):
            raise ValueError("Nonfinite policy output")
        actions[..., 6:] = np.clip(actions[..., 6:], 0, 1)
        return {"actions": actions}


def make_train_config(repo_id):
    """Call from a compatible OpenPI installation; keep full finetuning defaults."""
    from openpi import transforms
    from openpi.models.pi0_config import Pi0Config
    from openpi.training.config import DataConfig, SimpleDataConfig, TrainConfig
    from openpi.training.weight_loaders import CheckpointWeightLoader

    return TrainConfig(
        name="pi05_rokae", model=Pi0Config(pi05=True, action_dim=32, action_horizon=20),
        data=SimpleDataConfig(
            repo_id=repo_id, base_config=DataConfig(prompt_from_task=True, action_sequence_keys=("action",)),
            data_transforms=make_data_transforms,
        ),
        weight_loader=CheckpointWeightLoader("gs://openpi-assets/checkpoints/pi05_base/params"),
        batch_size=8, num_train_steps=20000, wandb_enabled=False,
    )


def make_data_transforms(model_config):
    from openpi import transforms

    return transforms.Group(inputs=[RokaeInputs()], outputs=[RokaeOutputs()])
