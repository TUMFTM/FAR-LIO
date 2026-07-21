<p align="center">
  <img src="docs/assets/farlio-banner.png" alt="FAR-LIO" width="660">
</p>

<p align="center">
  A highly efficient, CUDA-accelerated framework for Fast, Accurate, and Robust LiDAR-Inertial Odometry.
</p>

<p align="center">
  <a href="https://arxiv.org/abs/2606.26010"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2606.26010-b31b1b?logo=arxiv&logoColor=white"></a>
  <a href="https://tumftm.github.io/FAR-LIO/"><img alt="Project page" src="https://img.shields.io/badge/project-page-1f6feb"></a>
  <a href="https://github.com/TUMFTM/FAR-LIO/actions/workflows/docker.yml"><img alt="Docker build" src="https://github.com/TUMFTM/FAR-LIO/actions/workflows/docker.yml/badge.svg"></a>
  <a href="https://github.com/TUMFTM/FAR-LIO/actions/workflows/docs.yml"><img alt="Docs build" src="https://github.com/TUMFTM/FAR-LIO/actions/workflows/docs.yml/badge.svg"></a>
  <br />
  <img alt="Docker" src="https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white">
  <img alt="ROS 2 Jazzy" src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white">
  <img alt="C++20" src="https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus&logoColor=white">
</p>

## Installation

Pull the pre-built CUDA image from the GitHub Container Registry:

```bash
docker pull ghcr.io/tumftm/far-lio:latest
```

Or build it yourself:

```bash
git clone --recursive https://github.com/TUMFTM/FAR-LIO.git
cd FAR-LIO
docker build -f docker/Dockerfile -t ghcr.io/tumftm/far-lio:latest .
```

See the [installation guide](https://tumftm.github.io/FAR-LIO/installation.html) for prerequisites and options.

## Usage

Configure the environment and launch the stack with Docker Compose:

```bash
cp .env.example .env                                  # set BAG_PATH, BAG_NAME, ROS_DOMAIN_ID, ...
docker compose --profile far-lio up                   # FAR-LIO nodes (needs a GPU)
docker compose --profile bag up                       # replay a ROS 2 bag
docker compose --profile far-lio --profile bag up     # both together
```

See the [usage guide](https://tumftm.github.io/FAR-LIO/usage.html) for the full configuration reference.

## Racetrack Deployment (A2RL)

An excerpt of FAR-LIO running on an autonomous race car on a racetrack as part of the
[Abu Dhabi Autonomous Racing League (A2RL)](https://a2rl.io/).

<table align="center">
  <tr>
    <td align="center" width="50%">
      <img height="240" alt="A2RL onboard footage" src="https://github.com/user-attachments/assets/76415cfc-4ba3-4187-afee-672ece0ad899" />
      <br /><sub>Onboard camera footage</sub>
    </td>
    <td align="center" width="50%">
      <img height="240" alt="A2RL FAR-LIO ego view" src="https://github.com/user-attachments/assets/6504064d-b9d0-4411-96c4-ed91ed6b8f72" />
      <br /><sub>FAR-LIO's odometry</sub>
    </td>
  </tr>
</table>

See the [documentation](https://tumftm.github.io/FAR-LIO/) for the architecture and further
application domains.

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

[Marcel Weinmann](mailto:marcel.weinmann@tum.de)  
[Maximilian Leitenstern](mailto:maxi.leitenstern@tum.de)  
Institute of Automotive Technology, School of Engineering and Design, Technical University of Munich, 85748 Garching, Germany

### Acknowledgements

We thank Patrick Haft and Tobias Lasser (NVIDIA Corporation) for their assistance during the CUDA development.