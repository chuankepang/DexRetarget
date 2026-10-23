import numpy as np

from dexretarget.policy.rokae_policy import RokaeInputs, RokaeOutputs


def test_lerobot_chw_float_images_and_live_rgb_use_same_input_contract():
    rgb = np.full((48, 64, 3), 128, dtype=np.uint8)
    state = np.arange(12, dtype=np.float32)
    base = {"observation.state": state, "prompt": "pick up the cup"}
    live = RokaeInputs()({**base, "observation.images.global": rgb, "observation.images.wrist": rgb})
    training = RokaeInputs()({**base, "observation.images.global": np.moveaxis(rgb.astype("f4") / 255, -1, 0),
                              "observation.images.wrist": np.moveaxis(rgb.astype("f4") / 255, -1, 0),
                              "action": np.ones((20, 12), dtype="f4")})
    np.testing.assert_equal(live["image"]["base_0_rgb"], training["image"]["base_0_rgb"])
    np.testing.assert_equal(live["state"], training["state"])
    assert not live["image_mask"]["right_wrist_0_rgb"]
    assert training["actions"].shape == (20, 12)
    output = RokaeOutputs()({"actions": np.zeros((20, 32))})
    assert output["actions"].shape == (20, 12)
