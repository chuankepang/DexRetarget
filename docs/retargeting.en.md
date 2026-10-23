# Hand retargeting reference

Commands below run from the project root in the existing `dexteleop` Conda environment. The application workflow is documented in [usage.md](usage.md).

## Supported Robots

Config files are organized by **optimizer type** and **input source**:

```
config/retargeting/
├── adaptive/          # AdaptiveOptimizerAnalytical (default)
│   ├── mediapipe/     # camera / video / replay input
│   ├── avp/           # Apple Vision Pro input
│   ├── quest3/        # Meta Quest 3 input
│   ├── pico4/         # Pico 4 input
│   └── noitom/        # Noitom PNS-G gloves
└── vector/            # KeyVectorOptimizer
    ├── mediapipe/
    ├── avp/
    ├── quest3/
    ├── pico4/
    └── noitom/
```

| Robot | `--robot` value | Config suffix | Description |
|-------|------------------|---------------|-------------|
| **Shadow Hand** | `shadow` | `shadow_hand` | Shadow Hand with MuJoCo Menagerie meshes (default sim target) |
| **Wuji Hand** | `wuji` | `wuji_hand` | Wuji Hand, 5 fingers / 20 DOF |
| **Allegro Hand** | `allegro` | `allegro_hand` | Allegro Hand, 4 fingers / 16 DOF |
| **Inspire Hand** | `inspire` | `inspire_hand` | Inspire Hand with mimic joints |
| **Ability Hand** | `ability` | `ability_hand` | Ability Hand with mimic joints |
| **Leap Hand** | `leap` | `leap_hand` | Leap Hand, 4 fingers / 16 DOF |
| **SVH Hand** | `svh` | `svh_hand` | Schunk SVH Hand with mimic joints |
| **LinkerHand L21** | `linkerhand_l21` | `linkerhand_l21` | LinkerHand L21 |
| **Linker L20** | `linker_l20` | `linker_l20` | DexForce Linker L20, 5 fingers / 21 revolute joints with mimic joints |
| **ROHand** | `rohand` | `rohand` | ROHand |
| **Unitree Dex5** | `unitree_dex5` | `unitree_dex5_hand` | Unitree Dex5 |
| **Sharpa Hand** | `sharpa` | `sharpa_hand` | Sharpa Wave Hand, 5 fingers / 22 DOF |
| **Gaia Hand20** | `gaia` | `gaia_hand20` | Gaia Hand20, 5 fingers |

> **Note on Noitom configs:** Only `shadow_hand`, `wuji_hand`, and `inspire_hand` have been roughly calibrated for Noitom input. If you need to fine-tune the mapping accuracy between your hand and the robot hand, run `debug_skeleton.py` to visualize three skeletons side-by-side: **Blue** = raw input, **Green** = after scaling, **Red** = retargeted FK result. Compare the skeleton sizes and adjust the corresponding YAML config parameters (`scaling`, `segment_scaling`, `key_vectors[].scale`, etc.) accordingly.
>
> ```bash
> cd example
> python test/debug_skeleton.py --robot inspire --input noitom --noitom-local-ip 192.168.5.25
> ```

## Quick Start

The repository currently includes:

- `integrations/data/right.mp4`: sample input video
- `integrations/data/avp1.pkl`: optional recorded hand-tracking replay

### Simulation

```bash
cd example

# Run the included sample video (adaptive optimizer, default)
python teleop_sim.py --video data/right.mp4 --robot shadow --hand right

# Gaia Hand20 (right/left both supported)
python teleop_sim.py --video data/right.mp4 --robot gaia --hand right

# Pico 4 direct mode (PC broadcasts itself and accepts the headset connection)
python teleop_sim.py --input pico4 --pico4-mode direct --robot gaia --hand right

# Pico 4 relay mode (default; run input/pico4_daemon.py in another terminal first)
python teleop_sim.py --input pico4 --robot gaia --hand right

# Switch to KeyVector optimizer
python teleop_sim.py --video data/right.mp4 --robot shadow --hand right --optimizer vector

# Replay the optional sample recording
python teleop_sim.py --play data/avp1.pkl --robot shadow --hand right

# Real-time with laptop camera (MediaPipe)
python teleop_sim.py --input camera --robot shadow --hand right

# Real-time with Vision Pro
python teleop_sim.py --input visionpro --robot shadow --ip <vision-pro-ip> --hand right

# Real-time with Quest 3 (via Hand Tracking Streamer)
python teleop_sim.py --input quest3 --robot shadow --port 9000 --hand right

# Real-time with RealSense
python teleop_sim.py --realsense --robot shadow --hand right --show-video

# Noitom PNS-G gloves
python teleop_sim.py --input noitom --robot inspire --hand right --noitom-local-ip 192.168.5.25

# Replay your own recording (.pkl)
python teleop_sim.py --play path/to/record.pkl --robot shadow --hand right
```

### Real Hardware

`teleop_real.py` provides real-hardware output drivers for **Wuji Hand**, **Shadow Hand** (TCP bridge), **Inspire Hand** (serial), and **Gaia Hand20** (official HandSDK).

```bash
cd example

# Live Vision Pro -> Wuji Hand (adaptive)
python teleop_real.py --robot wuji --input visionpro --ip <vision-pro-ip> --hand right

# Live Vision Pro -> Wuji Hand (vector optimizer)
python teleop_real.py --robot wuji --input visionpro --ip <vision-pro-ip> --hand right --optimizer vector

# Noitom PNS-G gloves -> Inspire Hand
python teleop_real.py --robot inspire --input noitom --hand right --noitom-local-ip 192.168.5.25

# Pico 4 relay -> right Gaia Hand20
python teleop_real.py --robot gaia --input pico4 --hand right --pico4-mode relay \
  --gaia-port /dev/ttyACM0

# Replay the optional sample recording -> Wuji Hand
python teleop_real.py --robot wuji --play data/avp1.pkl --hand right

# Linux USB permission (Inspire / Gaia examples)
sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/ttyACM0
```

#### Gaia Hand20 setup

Install the Gaia HandSDK wheel matching the Python version and host architecture. For the recommended Python 3.10 Linux x86_64 environment:

```bash
conda activate /data/miniconda3/envs/dexteleop
pip install /path/to/gaia_hand/02.HandSDK/packages/02.Linux/x86_64/v1.1.1/handsdk-1.1.1-cp310-cp310-manylinux_2_35_x86_64.whl
python -c "import hand; print('Gaia HandSDK OK')"
```


### Command Reference

#### Input Source

| Option | Default | Description |
|--------|---------|-------------|
| `--input` | - | `teleop_sim.py`: `visionpro` / `quest3` / `pico4` / `noitom` / `camera` / `realsense` / `video` / `mediapipe_replay` |
| `--input` | - | `teleop_real.py`: `visionpro` / `quest3` / `pico4` / `noitom` / `camera` / `realsense` / `video` / `mediapipe_replay` |
| `--hand` | `right` | Hand side (`left`/`right`) |
| `--realsense` | off | Shortcut for `--input realsense` |
| `--play FILE` | - | Replay recording (shortcut for `--input mediapipe_replay`) |
| `--video FILE` | - | Video file input with MediaPipe hand detection |
| `--ip` | `192.168.50.127` | Vision Pro IP |
| `--port` | `9000` | Quest 3 HTS listener port |
| `--protocol` | `udp` | Quest 3 HTS transport protocol (`udp`/`tcp`) |
| `--noitom-local-ip` | `192.168.5.25` | Noitom: local IP (this machine) |
| `--noitom-local-port` | `8000` | Noitom: local UDP port |
| `--noitom-server-ip` | `192.168.5.33` | Noitom: Axis Studio IP (Windows) |
| `--noitom-server-port` | `9000` | Noitom: Axis Studio port |

#### Optimizer

| Option | Default | Description |
|--------|---------|-------------|
| `--optimizer` | `adaptive` | Optimizer type: `adaptive` or `vector` |
| `--config` | auto-select | Configuration file (overrides `--robot` and `--optimizer`) |

#### Robot Hand & Output

| Option | Default | Description |
|--------|---------|-------------|
| `--robot` | `shadow` (sim) / `wuji` (real) | Robot hand type; real output supports `wuji`, `shadow`, `inspire`, and `gaia` |
| `--record` | - | Record input data |
| `--output FILE` | - | Output file path for recording |
| `--show-video` | off | Show RGB / landmark preview for supported inputs |
| `--speed` | `1.0` | Playback speed |
| `--no-loop` | - | Disable looping for replay |
| `--headless` | off | Run simulation without GUI viewer |
| `--save-sim FILE` | - | Save offscreen simulation video |
| `--save-qpos FILE` | - | Save target / simulated qpos trajectory |

### Debug & Visualization Tools

#### debug_skeleton.py

Compare three hand skeletons in the MuJoCo viewer to debug retargeting issues:

- **Blue**: Raw MediaPipe skeleton (after coordinate transform, before scaling)
- **Green**: Scaled target skeleton (what the optimizer tries to match)
- **Red**: Robot FK skeleton (retargeting result)

```bash
cd example

# With camera input
python test/debug_skeleton.py --robot leap --input camera

# With video file
python test/debug_skeleton.py --robot leap --video data/right.mp4

# With optional sample recording, compare optimizers
python test/debug_skeleton.py --robot shadow --play data/avp1.pkl --optimizer adaptive
python test/debug_skeleton.py --robot shadow --play data/avp1.pkl --optimizer vector

# With RealSense D435
python test/debug_skeleton.py --robot sharpa --input realsense --hand right

# With Vision Pro
python test/debug_skeleton.py --robot sharpa --input avp --avp-ip 192.168.5.32 --hand right

# With Noitom PNS-G gloves
python test/debug_skeleton.py --robot inspire --input noitom --noitom-local-ip 192.168.5.25

# With Noitom + KeyVector optimizer
python test/debug_skeleton.py --robot inspire --input noitom --optimizer vector --noitom-local-ip 192.168.5.25

# With your own recorded data
python test/debug_skeleton.py --robot shadow --play path/to/record.pkl
```

#### calibrate_scaling.py

Calibrate `segment_scaling` for any robot hand and input source. Collects data while the user holds their hand flat, then computes the ratio between robot FK and human bone distances.

```bash
cd example

# Calibrate with RealSense
python test/calibrate_scaling.py --robot sharpa --input mediapipe

# Calibrate with video
python test/calibrate_scaling.py --robot shadow --input mediapipe --video data/right.mp4

# Calibrate with Vision Pro
python test/calibrate_scaling.py --robot wuji --input avp --avp-ip 192.168.5.32

# Calibrate with Noitom
python test/calibrate_scaling.py --robot inspire --input noitom

# Calibrate with Quest 3
python test/calibrate_scaling.py --robot shadow --input quest3
```

#### visualize_scaling.py

Visualize how `scaling` and `segment_scaling` parameters affect MediaPipe keypoints.

```bash
cd example

python test/visualize_scaling.py --robot leap --video data/right.mp4 --hand right
python test/visualize_scaling.py --robot allegro --play data/avp1.pkl --hand right
```

## API Reference

### Basic Usage

```python
from anydexretarget import Retargeter

# Load from config file
retargeter = Retargeter.from_yaml("config/adaptive/mediapipe/mediapipe_shadow_hand.yaml", hand_side="right")

# Retarget: (21, 3) MediaPipe keypoints -> joint angles
qpos = retargeter.retarget(raw_keypoints)

# With verbose output
qpos, info = retargeter.retarget_verbose(raw_keypoints)
print(f"Cost: {info['cost']:.4f}")
print(f"Pinch alphas: {info.get('pinch_alphas')}")  # adaptive only
```

### Advanced Usage

```python
# Direct optimizer access
optimizer = retargeter.optimizer

# Compute cost for given pose
cost = optimizer.compute_cost(qpos, mediapipe_keypoints)

# Get timing statistics
stats = optimizer.get_timing_stats()
print(f"Average time: {stats.get_avg()['total_ms']:.2f} ms")
```

## Citation

```bibtex
@software{anydexretarget2025,
  title={AnyDexRetarget},
  author={Shiquan Qiu},
  year={2025},
  url={https://github.com/qqsq12321/AnyDexRetarget},
}
```

## Acknowledgement

- [MuJoCo](https://mujoco.org/) - Physics simulation
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) - Shadow Hand models
- [dex-retargeting](https://github.com/dexsuite/dex-retargeting) - Retargeting algorithms
- [DexPilot](https://arxiv.org/abs/1910.03135) - Vision-based teleoperation
- [VisionProTeleop](https://github.com/Improbable-AI/VisionProTeleop) - Apple Vision Pro streaming
- [wuji-retargeting](https://github.com/wuji-technology/wuji-retargeting) - Wuji retargeting

## Contact

For questions, please open an issue on [Gitee](https://gitee.com/gx_robot/AnyDexRetarget/issues) / [GitHub](https://github.com/qqsq12321/AnyDexRetarget/issues) or contact the author via 932851972@qq.com.
