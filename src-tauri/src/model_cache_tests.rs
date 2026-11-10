use super::*;
use std::{
    collections::HashMap,
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc,
    },
};

struct TemporaryDirectory(PathBuf);
impl TemporaryDirectory {
    fn new() -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "emulation-model-cache-{}-{}-{unique}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir(&path).unwrap();
        Self(path)
    }
}
impl Drop for TemporaryDirectory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn fixture() -> (Catalog, HashMap<String, Bytes>) {
    let files = HashMap::from([
        (
            "robot.urdf".to_owned(),
            Bytes::from_static(b"<robot name=\"fixture\"/>"),
        ),
        (
            "meshes/base.stl".to_owned(),
            Bytes::from(vec![42_u8; BUFFER_BYTES * 2 + 5]),
        ),
    ]);
    let revision = "ab6aa489855256de07ba46c7b45887fdbc42b607";
    let mut paths: Vec<_> = files.keys().cloned().collect();
    paths.sort();
    let entries: Vec<_> = paths.into_iter().map(|path| {
        let bytes = &files[&path];
        CatalogFile { path: path.clone(), bytes: bytes.len() as u64, sha256: digest(bytes), url: format!("https://raw.githubusercontent.com/work-r-labs/robots/{revision}/fixture/{path}") }
    }).collect();
    let catalog = Catalog {
        schema_version: 1,
        source: CatalogSource {
            repository: "https://github.com/work-r-labs/robots".into(),
            revision: revision.into(),
        },
        models: vec![CatalogModel {
            id: "fixture-model".into(),
            urdf_path: "robot.urdf".into(),
            download_bytes: entries.iter().map(|file| file.bytes).sum(),
            files: entries,
        }],
    };
    (catalog, files)
}
struct FakeFetcher {
    files: HashMap<String, Bytes>,
    calls: AtomicUsize,
    forbid: bool,
    stall_after: Option<usize>,
    sent: Arc<Notify>,
    content_length: Option<u64>,
}
impl FakeFetcher {
    fn new(files: HashMap<String, Bytes>) -> Self {
        Self {
            files,
            calls: AtomicUsize::new(0),
            forbid: false,
            stall_after: None,
            sent: Arc::new(Notify::new()),
            content_length: None,
        }
    }
    fn forbidden() -> Self {
        Self {
            forbid: true,
            ..Self::new(HashMap::new())
        }
    }
}
struct FakeStream {
    bytes: Bytes,
    count: usize,
    stall_after: Option<usize>,
    sent: Arc<Notify>,
    content_length: Option<u64>,
}
impl DownloadStream for FakeStream {
    fn content_length(&self) -> Option<u64> {
        self.content_length
    }
    async fn next_chunk(&mut self) -> Result<Option<Bytes>, String> {
        if self.stall_after.is_some_and(|limit| self.count >= limit) {
            std::future::pending::<()>().await;
        }
        if self.bytes.is_empty() {
            return Ok(None);
        }
        self.count += 1;
        let chunk = self.bytes.split_to(self.bytes.len().min(4096));
        self.sent.notify_one();
        Ok(Some(chunk))
    }
}
impl Fetcher for FakeFetcher {
    type Stream = FakeStream;
    async fn open(&self, file: &CatalogFile) -> Result<FakeStream, String> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        if self.forbid {
            return Err("network forbidden in this test".into());
        }
        let bytes = self
            .files
            .get(&file.path)
            .ok_or("missing fake response")?
            .clone();
        Ok(FakeStream {
            bytes,
            count: 0,
            stall_after: self.stall_after,
            sent: self.sent.clone(),
            content_length: self.content_length,
        })
    }
}
fn quiet(_: ModelDownloadProgress) -> Result<(), String> {
    Ok(())
}
fn no_partials(manager: &ModelCache) {
    let partial = manager.root.join(".partial");
    if partial.exists() {
        assert_eq!(std::fs::read_dir(partial).unwrap().count(), 0);
    }
}

#[test]
fn inaccessible_cache_is_stored_as_an_error_without_preventing_application_startup() {
    let root = TemporaryDirectory::new();
    let file = root.0.join("not-a-directory");
    std::fs::write(&file, b"existing file").unwrap();
    let state = ModelCacheState::new(Ok(file));
    assert!(state.cache().is_err());
    let unresolved = ModelCacheState::new(Err("app data unavailable".into()));
    assert_eq!(unresolved.cache().err().unwrap(), "app data unavailable");
}

#[tokio::test]
async fn embedded_catalog_is_pinned_and_initial_inventory_does_not_download() {
    let root = TemporaryDirectory::new();
    let manager = ModelCache::new(root.0.clone()).unwrap();
    assert_eq!(manager.catalog.models.len(), 6);
    let statuses = manager.list().await.unwrap();
    assert_eq!(statuses.len(), 6);
    assert!(statuses.iter().all(|status| !status.cached));
}

#[test]
fn rejects_unpinned_or_escaping_catalog_entries() {
    let (catalog, _) = fixture();
    let invalid_urls = [
        "http://raw.githubusercontent.com/work-r-labs/robots/ab6aa489855256de07ba46c7b45887fdbc42b607/file",
        "https://127.0.0.1/work-r-labs/robots/ab6aa489855256de07ba46c7b45887fdbc42b607/file",
        "https://raw.githubusercontent.com/work-r-labs/robots/master/file",
        "https://raw.githubusercontent.com/untrusted/repo/ab6aa489855256de07ba46c7b45887fdbc42b607/file",
        "https://raw.githubusercontent.com/work-r-labs/robots/ab6aa489855256de07ba46c7b45887fdbc42b607/file?redirect=1",
    ];
    for url in invalid_urls {
        let mut changed = catalog.clone();
        changed.models[0].files[0].url = url.into();
        assert!(validate_catalog(&changed).is_err(), "{url}");
    }
    for path in [
        "../escape.stl",
        "/absolute.stl",
        "C:\\escape.stl",
        "meshes/../escape.stl",
        "meshes//file.stl",
        MARKER,
    ] {
        let mut changed = catalog.clone();
        changed.models[0].files[0].path = path.into();
        assert!(validate_catalog(&changed).is_err(), "{path}");
    }
    let mut changed = catalog.clone();
    changed.models[0].files[0].bytes = MAX_FILE_BYTES + 1;
    assert!(validate_catalog(&changed).is_err());
    let mut changed = catalog.clone();
    changed.models[0].download_bytes += 1;
    assert!(validate_catalog(&changed).is_err());
    let mut changed = catalog;
    changed.models[0].id = "..".into();
    assert!(validate_catalog(&changed).is_err());
}

#[tokio::test]
async fn downloads_exact_bytes_and_a_new_manager_reuses_verified_cache_with_network_forbidden() {
    let root = TemporaryDirectory::new();
    let (catalog, files) = fixture();
    let manager = ModelCache::with_catalog(root.0.clone(), catalog.clone()).unwrap();
    let fetcher = FakeFetcher::new(files.clone());
    let events = Mutex::new(Vec::new());
    let first = manager
        .ensure_with(
            "first",
            "fixture-model",
            |event| {
                events.lock().unwrap().push(event);
                Ok(())
            },
            &fetcher,
        )
        .await
        .unwrap();
    assert!(!first.cache_hit);
    assert_eq!(fetcher.calls.load(Ordering::Relaxed), 2);
    assert_eq!(first.files.len(), 2);
    for file in &first.files {
        assert!(Path::new(&file.local_path).starts_with(&manager.root));
        assert_eq!(
            std::fs::read(&file.local_path).unwrap(),
            files[&file.logical_path]
        );
    }
    let events = events.lock().unwrap();
    assert_eq!(events.first().unwrap().phase, "checking");
    assert_eq!(events.last().unwrap().phase, "ready");
    assert!(events
        .iter()
        .all(|event| event.downloaded_bytes <= event.total_bytes));
    drop(events);
    drop(manager);
    let fresh = ModelCache::with_catalog(root.0.clone(), catalog).unwrap();
    let forbidden = FakeFetcher::forbidden();
    let second = fresh
        .ensure_with("offline", "fixture-model", quiet, &forbidden)
        .await
        .unwrap();
    assert!(second.cache_hit);
    assert_eq!(forbidden.calls.load(Ordering::Relaxed), 0);
    assert!(fresh.list().await.unwrap()[0].cached);
    no_partials(&fresh);
}

#[tokio::test]
async fn same_size_corruption_is_not_reported_cached_and_is_repaired_only_after_a_verified_replacement(
) {
    let root = TemporaryDirectory::new();
    let (catalog, files) = fixture();
    let manager = ModelCache::with_catalog(root.0.clone(), catalog).unwrap();
    let result = manager
        .ensure_with(
            "first",
            "fixture-model",
            quiet,
            &FakeFetcher::new(files.clone()),
        )
        .await
        .unwrap();
    let damaged = &result.files[0];
    let corrupt = vec![3_u8; damaged.bytes as usize];
    std::fs::write(&damaged.local_path, &corrupt).unwrap();
    assert!(!manager.list().await.unwrap()[0].cached);
    let forbidden = FakeFetcher::forbidden();
    assert!(manager
        .ensure_with("offline-damaged", "fixture-model", quiet, &forbidden)
        .await
        .is_err());
    assert_eq!(std::fs::read(&damaged.local_path).unwrap(), corrupt);
    no_partials(&manager);
    let repaired = manager
        .ensure_with("repair", "fixture-model", quiet, &FakeFetcher::new(files))
        .await
        .unwrap();
    assert!(!repaired.cache_hit);
    assert!(manager.list().await.unwrap()[0].cached);
    no_partials(&manager);
}

#[tokio::test]
async fn incomplete_excess_bad_hash_and_wrong_content_length_never_publish_a_cache() {
    for variant in 0..4 {
        let root = TemporaryDirectory::new();
        let (catalog, mut files) = fixture();
        let first = catalog.models[0].files[0].path.clone();
        let bytes = files[&first].clone();
        match variant {
            0 => {
                files.insert(first.clone(), bytes.slice(..bytes.len() - 1));
            }
            1 => {
                let mut excess = bytes.to_vec();
                excess.push(0);
                files.insert(first.clone(), excess.into());
            }
            2 => {
                files.insert(first.clone(), vec![7_u8; bytes.len()].into());
            }
            _ => (),
        }
        let manager = ModelCache::with_catalog(root.0.clone(), catalog).unwrap();
        let mut fetcher = FakeFetcher::new(files);
        if variant == 3 {
            fetcher.content_length = Some(1);
        }
        assert!(manager
            .ensure_with("bad-download", "fixture-model", quiet, &fetcher)
            .await
            .is_err());
        assert!(!manager.list().await.unwrap()[0].cached);
        assert!(!manager.directory(&manager.catalog.models[0]).exists());
        no_partials(&manager);
    }
}

#[tokio::test]
async fn cancellation_before_registration_and_unknown_model_do_not_fetch() {
    let root = TemporaryDirectory::new();
    let (catalog, _) = fixture();
    let manager = ModelCache::with_catalog(root.0.clone(), catalog).unwrap();
    let forbidden = FakeFetcher::forbidden();
    manager.cancel("already-cancelled");
    assert_eq!(
        manager
            .ensure_with("already-cancelled", "fixture-model", quiet, &forbidden)
            .await
            .unwrap_err(),
        CANCELLED
    );
    assert!(manager
        .ensure_with("unknown", "../../outside", quiet, &forbidden)
        .await
        .is_err());
    assert_eq!(forbidden.calls.load(Ordering::Relaxed), 0);
    for index in 0..200 {
        manager.cancel(&format!("cancel-{index}"));
    }
    assert_eq!(manager.queue.lock().unwrap().cancelled_ids.len(), 64);
}

#[tokio::test]
async fn cancelling_a_stalled_stream_removes_partial_files_and_never_reports_ready() {
    let root = TemporaryDirectory::new();
    let (catalog, files) = fixture();
    let manager = Arc::new(ModelCache::with_catalog(root.0.clone(), catalog).unwrap());
    let fetcher = Arc::new(FakeFetcher {
        stall_after: Some(1),
        ..FakeFetcher::new(files)
    });
    let worker_manager = manager.clone();
    let worker_fetcher = fetcher.clone();
    let phases = Arc::new(Mutex::new(Vec::new()));
    let worker_phases = phases.clone();
    let task = tokio::spawn(async move {
        worker_manager
            .ensure_with(
                "cancel-during-body",
                "fixture-model",
                |event| {
                    worker_phases.lock().unwrap().push(event.phase);
                    Ok(())
                },
                worker_fetcher.as_ref(),
            )
            .await
    });
    tokio::time::timeout(Duration::from_secs(2), fetcher.sent.notified())
        .await
        .unwrap();
    manager.cancel("cancel-during-body");
    let result = tokio::time::timeout(Duration::from_secs(2), task)
        .await
        .unwrap()
        .unwrap();
    assert_eq!(result.unwrap_err(), CANCELLED);
    assert!(!phases.lock().unwrap().contains(&"ready"));
    assert!(!manager.list().await.unwrap()[0].cached);
    no_partials(&manager);
}

#[tokio::test]
async fn a_new_request_cancels_the_previous_transfer_then_reuses_the_single_active_slot() {
    let root = TemporaryDirectory::new();
    let (catalog, files) = fixture();
    let manager = Arc::new(ModelCache::with_catalog(root.0.clone(), catalog).unwrap());
    let stalled = Arc::new(FakeFetcher {
        stall_after: Some(1),
        ..FakeFetcher::new(files.clone())
    });
    let worker_manager = manager.clone();
    let worker_fetcher = stalled.clone();
    let old = tokio::spawn(async move {
        worker_manager
            .ensure_with("old", "fixture-model", quiet, worker_fetcher.as_ref())
            .await
    });
    tokio::time::timeout(Duration::from_secs(2), stalled.sent.notified())
        .await
        .unwrap();
    let new = tokio::time::timeout(
        Duration::from_secs(2),
        manager.ensure_with("new", "fixture-model", quiet, &FakeFetcher::new(files)),
    )
    .await
    .unwrap()
    .unwrap();
    assert!(!new.cache_hit);
    assert_eq!(old.await.unwrap().unwrap_err(), CANCELLED);
    assert!(manager.queue.lock().unwrap().active.is_none());
    assert!(manager.queue.lock().unwrap().pending.is_none());
    no_partials(&manager);
}

#[tokio::test]
async fn pending_replacement_is_bounded_and_only_the_latest_request_can_become_active() {
    let root = TemporaryDirectory::new();
    let (catalog, _) = fixture();
    let manager = ModelCache::with_catalog(root.0.clone(), catalog).unwrap();
    let (first, mut first_cancel) = manager.reserve("first").unwrap();
    manager.acquire("first", &mut first_cancel).await.unwrap();
    let (_second, second_cancel) = manager.reserve("second").unwrap();
    let (_third, mut third_cancel) = manager.reserve("third").unwrap();
    assert!(*first_cancel.borrow());
    assert!(*second_cancel.borrow());
    assert!(!*third_cancel.borrow());
    assert_eq!(
        manager.queue.lock().unwrap().pending.as_ref().unwrap().id,
        "third"
    );
    drop(first);
    manager.acquire("third", &mut third_cancel).await.unwrap();
    assert_eq!(
        manager.queue.lock().unwrap().active.as_ref().unwrap().id,
        "third"
    );
}

#[cfg(unix)]
#[tokio::test]
async fn cache_symlinks_cannot_redirect_a_download_or_authorized_result() {
    let root = TemporaryDirectory::new();
    let outside = TemporaryDirectory::new();
    std::fs::write(outside.0.join("sentinel"), b"unchanged").unwrap();
    let (catalog, files) = fixture();
    let manager = ModelCache::with_catalog(root.0.clone(), catalog).unwrap();
    let parent = manager.root.join(&manager.catalog.source.revision);
    std::fs::create_dir(&parent).unwrap();
    std::os::unix::fs::symlink(&outside.0, parent.join("fixture-model")).unwrap();
    assert!(!manager.list().await.unwrap()[0].cached);
    assert!(manager
        .ensure_with("symlink", "fixture-model", quiet, &FakeFetcher::new(files))
        .await
        .is_err());
    assert_eq!(
        std::fs::read(outside.0.join("sentinel")).unwrap(),
        b"unchanged"
    );
    assert_eq!(std::fs::read_dir(&outside.0).unwrap().count(), 1);
    no_partials(&manager);
}

#[test]
fn collision_source_reads_only_pinned_cache_bytes_and_honors_cancellation() {
    use std::sync::atomic::AtomicBool;
    let temporary = TemporaryDirectory::new();
    let (catalog, files) = fixture();
    let relative = PathBuf::from("verified-model");
    let directory = temporary.0.join(&relative);
    std::fs::create_dir_all(directory.join("meshes")).unwrap();
    for (path, bytes) in &files {
        std::fs::write(directory.join(path), bytes).unwrap();
    }
    let handle = CollisionMeshSource {
        root: temporary.0.clone(),
        relative,
        files: catalog.models[0].files.clone(),
    };
    let file = handle
        .files
        .iter()
        .find(|file| file.path.ends_with(".stl"))
        .unwrap();
    let source = emulation_core::generic_collision::MeshSource {
        path: file.path.clone(),
        bytes: file.bytes as usize,
        sha256: file.sha256.clone(),
    };
    let cancel = AtomicBool::new(false);
    assert_eq!(handle.read(&source, &cancel).unwrap(), files[&source.path]);
    let mut forged = source.clone();
    forged.sha256 = "0".repeat(64);
    assert!(handle.read(&forged, &cancel).is_err());
    cancel.store(true, Ordering::Relaxed);
    assert!(handle.read(&source, &cancel).is_err());
    cancel.store(false, Ordering::Relaxed);
    let mut corrupt = files[&source.path].to_vec();
    corrupt[0] ^= 1;
    std::fs::write(directory.join(&source.path), corrupt).unwrap();
    assert!(handle
        .read(&source, &cancel)
        .unwrap_err()
        .contains("校验失败"));
}

#[test]
fn collision_source_cannot_escape_through_cached_symlinks() {
    #[cfg(unix)]
    {
        use std::sync::atomic::AtomicBool;
        let temporary = TemporaryDirectory::new();
        let (catalog, files) = fixture();
        let relative = PathBuf::from("model");
        let directory = temporary.0.join(&relative);
        std::fs::create_dir_all(directory.join("meshes")).unwrap();
        let file = catalog.models[0]
            .files
            .iter()
            .find(|file| file.path.ends_with(".stl"))
            .unwrap();
        let external = temporary.0.join("outside.stl");
        std::fs::write(&external, &files[&file.path]).unwrap();
        std::os::unix::fs::symlink(external, directory.join(&file.path)).unwrap();
        let source = emulation_core::generic_collision::MeshSource {
            path: file.path.clone(),
            bytes: file.bytes as usize,
            sha256: file.sha256.clone(),
        };
        let handle = CollisionMeshSource {
            root: temporary.0.clone(),
            relative,
            files: catalog.models[0].files.clone(),
        };
        assert!(handle.read(&source, &AtomicBool::new(false)).is_err());
    }
}
