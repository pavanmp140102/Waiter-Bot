# M8: Docker and CI

**Goal:** the project builds, tests, and runs in a container, so it works on machines other than the development one, and a clean build proves the dependency declarations are complete.

**Depends on:** M0 for the first image, M5 to finish.

## Sub-tasks

### First image (right after M0)

- [x] `docker/Dockerfile`, multi-stage: a base that is `ros:jazzy-ros-base` plus everything `rosdep` installs from the manifests (which brings Gazebo Harmonic and Nav2), a stage that runs `rosdep install --from-paths src --ignore-src -y`, a stage that runs `colcon build`.
- [x] `.dockerignore` covering `build/`, `install/`, `log/`, `.git/`.
- [x] `docker/compose.yaml` with a `test` service (S, T, TN) to start.

### Finish (after M5)

- [ ] `test-e2e` service for N and E in headless Gazebo.
- [ ] `sim` service for the demo, with software rendering as the universal option, and a variant for a host GPU.
- [ ] `dev` service with the source mounted.
- [ ] `docker/run.sh` that picks a graphics mode and falls back with a message (DK5).
- [ ] `.github/workflows/ci.yaml`: build the image, run `test` on every push and pull request, run `test-e2e` on pushes to `main` or on demand.
- [ ] Document the commands in [../setup.md](../setup.md).

## Files created

```
docker/{Dockerfile,compose.yaml,run.sh}
.dockerignore
.github/workflows/ci.yaml
```

## Gate

DK1 to DK5. The first image gates only DK1 and DK2; the finish step gates the rest.

## Notes

- Install every dependency through `rosdep` from the manifests. A package that merely happens to be on the development machine does not count.
- No absolute host paths in the Dockerfile, Compose file, launch files or configs (S5).
- Set the ROS domain and `GZ_IP` inside the services, not in the project launch files.
- Run one simulator per container and one container at a time on one host.

## Result

First image done (after M0), measured at that point. The second half lands after M5, so this section is extended then.

Measured:

- DK1: `docker compose -f docker/compose.yaml build test` builds from a clean checkout in about 2 minutes once the base layers are cached. The only dependency source is `rosdep` reading the `package.xml` files, so a missing key in a manifest would fail this step. The image is 5.1 GB, most of it Gazebo Harmonic and Nav2 pulled in by the manifests.
- DK2: `docker compose -f docker/compose.yaml run --rm test` runs the 35 tests of the seven packages in about 6 s, with 0 failures.

Differences from the plan:

- No `dev` stage yet. The Dockerfile has a small `manifests` helper stage (it keeps only the `package.xml` files, and no empty folders, so the dependency layer is cached), then `base` and `runtime`. A `dev` stage is added when it is needed; until then the host build is the development loop.
- The image builds with plain `colcon build`, not `--symlink-install`, so it tests a real install while the host uses symlinks for development.
- `APT::Install-Recommends` is 0 to keep the image small. If the simulator or the Qt dashboard later needs a recommended package, it goes in the manifest, not the Dockerfile. Checked again when `sim` is added.
- `GZ_IP=127.0.0.1` is set in the compose environment, as the notes below ask, for the simulator services that come later.
- The base image is `ros:jazzy-ros-base`, pinned by digest, so every machine builds from the same layers.
- `docs/` is excluded from the build context in `.dockerignore`, since the image never needs it.

