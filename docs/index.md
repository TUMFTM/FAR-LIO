<p align="center">
  <img src="assets/farlio-banner.png" alt="FAR-LIO" width="660">
</p>

<p align="center">
  A highly efficient, CUDA-accelerated framework for Fast, Accurate, and Robust LiDAR-Inertial Odometry.
</p>

<p align="center">
  <a href="https://arxiv.org/abs/2606.26010"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2606.26010-b31b1b?logo=arxiv&logoColor=white"></a>
  <a href="https://github.com/TUMFTM/FAR-LIO/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/license-Apache%202.0-blue"></a>
  <a href="https://github.com/TUMFTM/FAR-LIO/actions/workflows/docker.yml"><img alt="Docker build" src="https://github.com/TUMFTM/FAR-LIO/actions/workflows/docker.yml/badge.svg"></a>
  <a href="https://github.com/TUMFTM/FAR-LIO/actions/workflows/docs.yml"><img alt="Docs build" src="https://github.com/TUMFTM/FAR-LIO/actions/workflows/docs.yml/badge.svg"></a>
  <br />
  <img alt="Docker" src="https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white">
  <img alt="ROS 2 Jazzy" src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white">
  <img alt="C++20" src="https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus&logoColor=white">
</p>

## Racetrack Deployment (A2RL)

An excerpt of FAR-LIO running on an autonomous race car on a racetrack as part of the
[Abu Dhabi Autonomous Racing League (A2RL)](https://a2rl.io/).

<div style="text-align:center;white-space:nowrap;overflow-x:auto">
  <span style="display:inline-block;white-space:normal;vertical-align:top;margin:0 0.5rem">
    <img src="https://github.com/user-attachments/assets/76415cfc-4ba3-4187-afee-672ece0ad899" alt="A2RL onboard footage" style="height:240px!important;width:auto!important;max-width:none!important" />
    <br /><sub>Onboard camera footage</sub>
  </span>
  <span style="display:inline-block;white-space:normal;vertical-align:top;margin:0 0.5rem">
    <img src="https://github.com/user-attachments/assets/6504064d-b9d0-4411-96c4-ed91ed6b8f72" alt="A2RL FAR-LIO ego view" style="height:240px!important;width:auto!important;max-width:none!important" />
    <br /><sub>FAR-LIO's odometry</sub>
  </span>
</div>

## Architecture

FAR-LIO has two main components: a **LiDAR Scan Pipeline** that registers each incoming scan
against a local map on the GPU, and a **Sensor Fusion** backend that fuses the registered poses
with high-frequency IMU data. Green modules are GPU-accelerated (CUDA); blue modules run on the CPU.

<p align="center">
  <img src="assets/architecture.svg" alt="FAR-LIO architecture" width="820">
</p>

- **LiDAR Scan Pipeline (GPU)** — CUDA-accelerated preprocessing and undistortion, followed by a
  sparsity-aware Generalized ICP on a novel CUDA voxel hashmap (`cuVoxelMap`), registering each scan
  against an adaptive local submap.
- **Sensor Fusion (CPU)** — a 100 Hz kinematic Extended Kalman Filter fuses the registered LiDAR
  poses with the IMU stream, with delay compensation for smooth, low-latency output. Its estimate is
  fed back to the scan pipeline as the initial guess and for undistortion.

## References

If you use FAR-LIO in your research, please cite our paper:

```bibtex
@misc{2026far-lio,
      title={FAR-LIO: Enabling High-Speed Autonomy through Fast, Accurate, and Robust LiDAR-Inertial Odometry}, 
      author={Maximilian Leitenstern and Marcel Weinmann and Patrick Haft and Tobias Lasser and Dominik Kulmer and Markus Lienkamp},
      year={2026},
      eprint={2606.26010},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2606.26010},
}
```

### Core Developers

Marcel Weinmann [:material-home:](https://github.com/MarcelWeinmann) [:material-linkedin:](https://www.linkedin.com/in/marcel-weinmann/) [:material-mail:](mailto:marcel.weinmann@tum.de)  
Maximilian Leitenstern [:material-home:](https://github.com/mleitenstern) [:material-linkedin:](https://www.linkedin.com/in/maximilian-leitenstern-a8a4551b1/) [:material-mail:](mailto:maxi.leitenstern@tum.de)  
Institute of Automotive Technology, School of Engineering and Design, Technical University of Munich, 85748 Garching, Germany

### Acknowledgements

We thank Patrick Haft and Tobias Lasser ([NVIDIA Corporation](https://www.nvidia.com/)) for their assistance during the CUDA development.
