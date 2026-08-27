# Third-Party Licenses

VAMP-MR itself is released under the Apache License, Version 2.0 (see `LICENSE`).
It bundles and/or derives from the third-party software listed below. Each
component is used under its own license, reproduced or referenced here.

---

## 1. VAMP (Vector-Accelerated Motion Planning)

- Upstream: https://github.com/KavrakiLab/vamp
- License: Apache License, Version 2.0
- Usage in VAMP-MR: shipped as the `vamp/` git submodule (branch `vamp-mr`), a
  **modified** version that adds multi-robot composite collision checking on top
  of VAMP's per-robot SIMD kernels. The changes are additive (a new
  `collision/multi_robot.hh` layer plus supporting bindings) and are described in
  `docs/upstream/vamp-multirobot-proposal.md`.
- Full license text: `vamp/LICENSE.txt`. VAMP's own third-party licenses
  (SIMDxorshift, nanobind, pdqsort, nigh) are under `vamp/licenses/`.

---

## 2. APEX-MR

- Upstream: https://github.com/intelligent-control-lab/APEX-MR
- License: MIT License
- Usage in VAMP-MR: portions of `mr_planner_lego/` (the LEGO assembly
  manipulation code under
  `mr_planner_lego/.../applications/lego/`, including `Lego.hpp`, `Lego.cpp`,
  and the `lego/Utils/*` headers) are **derived from** APEX-MR and adapted to be
  ROS-free for integration with the `mr_planner_core` planning engine. The
  affected files carry a per-file attribution header.

```
MIT License

Copyright (c) 2025 Intelligent Control Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
