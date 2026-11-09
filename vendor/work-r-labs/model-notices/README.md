# Model source notices

The downloadable catalog is pinned to Workr Robot Asset Library revision
[`ab6aa489855256de07ba46c7b45887fdbc42b607`](https://github.com/work-r-labs/robots/tree/ab6aa489855256de07ba46c7b45887fdbc42b607).
These are community-distributed assets. Availability of a CAD model or URDF does
not establish manufacturer certification, correct controller conventions, or
supported motion-program export.

The repository-level [MIT notice](../LICENSE) names `2025 work_r`. Original
package notices also apply; the repository-level notice is not presented as a
replacement for them. The catalog downloads each model's original notices along
with its mesh and URDF files. Added FANUC and Yaskawa meshes are downloaded from
their pinned upstream URLs into the application's cache and are not bundled in
this repository's public assets.

## ABB and FANUC source packages

[source-model-MIT.txt](source-model-MIT.txt) is an unchanged copy of the MIT
notice naming `2018 Toshinori Kitamura`. The following source-package notices
are byte-for-byte identical (SHA-256
`25843f9143b800e5a2f3657e6e5fa2ec1856e489848abdd7a2a0da3265bd4866`):

- `sources/ABB/IRB1100_4kg-475-STD/IRB1100_4kg-475-STD_description/LICENSE`
- `sources/ABB/IRB1100_4kg-580-STD/IRB1100_4kg-580-STD_description/LICENSE`
- `sources/Fanuc/CRX-Series/CRX-10iA_L_v01_description/LICENSE`
- `sources/Fanuc/R2000-Series/R-2000iB165F_description/LICENSE`

Their exact pinned URLs and SHA-256/Git blob hashes are recorded in
[catalog.json](../../../src/catalog/catalog.json).

## Yaskawa / ROS-Industrial Motoman source packages

[yaskawa-gp7-package.xml](yaskawa-gp7-package.xml) and
[yaskawa-gp110-package.xml](yaskawa-gp110-package.xml) preserve the original
package metadata without modification. Both explicitly declare
`BSD-3-Clause`, refer to `https://github.com/ros-industrial/motoman`, and identify
their authors and maintainers. The GP7 package names Eric Marcil as author;
the GP110 package names Christian Juelg as author. Author fields are not
rewritten into unverified copyright claims.

The corresponding `sources/Yaskawa/GP Series/motoman_*_support` directories do
not supply a standalone BSD license/copyright notice. The reviewed upstream
Motoman tree likewise has no standalone license file. We preserve the notices
actually supplied by these packages and identify the models' license as
**declared BSD-3-Clause**, rather than claiming they are MIT-only.

[BSD-3-Clause-template.txt](BSD-3-Clause-template.txt) supplies the full standard
BSD three-clause terms and disclaimer as an unchanged copy of the
[SPDX template](https://github.com/spdx/license-list-data/blob/31846481de7ea17d92f2f46fad76abe58b2e9bb0/text/BSD-3-Clause.txt).
Its literal `<year>` and `<owner>` fields are upstream template placeholders;
they are not attribution to the robot manufacturer, package authors, or Workr.
No missing copyright holder or year has been invented. This reference text
does not replace the original package declarations or resolve information the
original distribution did not provide.

All fourteen Yaskawa catalog STL files match an original Motoman visual mesh
in the same pinned source tree byte for byte, including filenames that Workr
renamed. [motoman-mesh-provenance.json](motoman-mesh-provenance.json) records each
mapping, byte count, Git blob SHA-1 and SHA-256. The generator checks those
matches against the original source-tree entries. Standard URDF units apply:
these Yaskawa meshes are link-local metres, unlike the millimetre meshes and
visual-origin offsets used by the catalog's ABB/FANUC packages.

## Reproducing the catalog

`node scripts/generate-model-catalog.mjs` downloads only the reviewed files,
checks their original Git blob SHA-1 and sizes, validates binary STL vertices,
and writes exact SHA-256 digests into the catalog. New meshes remain in a
temporary directory, not in the repository. `--cache-dir DIRECTORY` chooses
that directory. `--check` verifies the generated catalog, raw notices,
provenance and URDF fixtures without changing them; `--offline` requires a
previously populated source cache.

On Node versions that support environment proxy handling, a network requiring
`HTTPS_PROXY` can use `NODE_USE_ENV_PROXY=1`. The generator does not read or
print proxy credentials. Preserve raw notice and fixture bytes when checking
hashes, including their original whitespace and line endings.
