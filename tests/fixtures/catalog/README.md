# Catalog URDF fixtures

These four small URDF files are unchanged bytes from [Workr Robot Asset Library](https://github.com/work-r-labs/robots/tree/ab6aa489855256de07ba46c7b45887fdbc42b607), revision `ab6aa489855256de07ba46c7b45887fdbc42b607`. No STL meshes are included in the fixtures. They exercise the actual URDF structure, visual offsets, metre/millimetre scales, and fixed branches in offline parser/FK tests; they do not establish controller or motion-planner compatibility.

| Fixture | Source | SHA-256 |
| --- | --- | --- |
| fanuc-crx10ia-l.urdf | [Pinned source](https://raw.githubusercontent.com/work-r-labs/robots/ab6aa489855256de07ba46c7b45887fdbc42b607/library/Fanuc/CRX_10iA_L_v1/CRX_10iA_L.urdf) | `fd6ecfdd3cd1d09161c48a75d1436aa57817c2017bca4a3827395b7bb94ad3ef` |
| fanuc-r2000ib-165f.urdf | [Pinned source](https://raw.githubusercontent.com/work-r-labs/robots/ab6aa489855256de07ba46c7b45887fdbc42b607/library/Fanuc/R_2000iB_165F_v1/R_2000iB_165F.urdf) | `fd9f4d2611dff9d0734d127b1105442cc5d398a977282777a462abc896fa1ebb` |
| yaskawa-gp7.urdf | [Pinned source](https://raw.githubusercontent.com/work-r-labs/robots/ab6aa489855256de07ba46c7b45887fdbc42b607/library/Yaskawa/GP7_v1/GP7.urdf) | `fc29b984a7f9ed2ae725c081ea1141a8861dd5798c1e7594320473af543061e9` |
| yaskawa-gp110.urdf | [Pinned source](https://raw.githubusercontent.com/work-r-labs/robots/ab6aa489855256de07ba46c7b45887fdbc42b607/library/Yaskawa/GP110_v1/GP110.urdf) | `a892a6f0ef45f4586738069fe5df6e9aa4a9dd13dac18de476aba38320e5abe5` |

Source notices and the distinction between Workr MIT and original Motoman BSD-3-Clause declarations are preserved in [model-notices](../../../vendor/work-r-labs/model-notices/README.md). FANUC source packages retain the original Toshinori Kitamura MIT notice. The original Motoman package declarations and standard BSD-3-Clause terms template are retained without inventing copyright holders.

Regenerate these fixtures, notices and the catalog together with `node scripts/generate-model-catalog.mjs`; verify without modifying them with `--check`.
