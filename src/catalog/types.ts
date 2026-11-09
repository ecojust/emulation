/** Immutable, reviewed model assets. Downloading a model does not enable motion solving. */
export interface RobotCatalog {
  schemaVersion: 1;
  source: {
    repository: string;
    revision: string;
  };
  models: RobotCatalogModel[];
}

export interface RobotCatalogModel {
  id: string;
  brand: 'ABB' | 'FANUC' | 'Yaskawa';
  name: string;
  /** Present only when the app has an independently validated motion model. */
  bundledMotionModelId?: string;
  /** Relative to the model's installed asset directory, using forward slashes. */
  urdfPath: string;
  files: RobotCatalogFile[];
  licenses: RobotCatalogLicense[];
  /** Sum of all files, including the URDF and unmodified source notices. */
  downloadBytes: number;
}

export interface RobotCatalogFile {
  /** Safe relative path inside this model's own cache directory. */
  path: string;
  bytes: number;
  sha256: string;
  /** SHA-1 of `blob <byte length>\0` followed by the original file bytes. */
  gitBlobSha1: string;
  /** Immutable HTTPS URL pinned to the source revision. */
  url: string;
}

export interface RobotCatalogLicense {
  name: string;
  spdx: 'MIT' | 'BSD-3-Clause';
  /** Relative path to the original notice included in files. */
  path: string;
  url: string;
}
