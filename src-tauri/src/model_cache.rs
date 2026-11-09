//! Pinned catalog assets only: bounded downloads, verified offline reuse and atomic publication.
//! Model files are data. This module never interprets URDF, archives, scripts or arbitrary URLs.
use bytes::Bytes;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{HashSet, VecDeque},
    future::Future,
    path::{Component, Path, PathBuf},
    sync::Mutex,
    time::{Duration, Instant},
};
use tokio::{
    fs,
    io::{AsyncReadExt, AsyncWriteExt},
    sync::{watch, Notify},
};

const CATALOG: &str = include_str!("../../src/catalog/catalog.json");
const CANCELLED: &str = "模型下载已取消";
const MARKER: &str = ".complete.json";
const MAX_FILES: usize = 256;
const MAX_FILE_BYTES: u64 = 128 * 1024 * 1024;
const MAX_MODEL_BYTES: u64 = 512 * 1024 * 1024;
const BUFFER_BYTES: usize = 64 * 1024;
const DOWNLOAD_TIMEOUT: Duration = Duration::from_secs(300);
const TIMEOUT_MESSAGE: &str = "模型下载超时，请检查网络后重试";

#[derive(Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Catalog {
    schema_version: u32,
    source: CatalogSource,
    models: Vec<CatalogModel>,
}
#[derive(Clone, Deserialize)]
struct CatalogSource {
    repository: String,
    revision: String,
}
#[derive(Clone, Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
struct CatalogModel {
    id: String,
    urdf_path: String,
    files: Vec<CatalogFile>,
    download_bytes: u64,
}
#[derive(Clone, Deserialize, Serialize)]
struct CatalogFile {
    path: String,
    bytes: u64,
    sha256: String,
    url: String,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ModelCacheStatus {
    pub model_id: String,
    pub revision: String,
    /// True only after every expected file passed its length and SHA-256 checks.
    pub cached: bool,
}
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct CachedModelFile {
    pub logical_path: String,
    pub local_path: String,
    pub bytes: u64,
    pub sha256: String,
}
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct CachedModel {
    pub model_id: String,
    pub revision: String,
    pub cache_hit: bool,
    pub files: Vec<CachedModelFile>,
}
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ModelDownloadProgress {
    pub request_id: String,
    pub model_id: String,
    pub phase: &'static str,
    pub downloaded_bytes: u64,
    pub total_bytes: u64,
    pub file: Option<String>,
}
#[derive(Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
struct CompleteMarker {
    model_id: String,
    revision: String,
    manifest_sha256: String,
}

fn digest(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn valid_hex(value: &str, length: usize) -> bool {
    value.len() == length && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}
fn safe_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 100
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.".contains(&byte))
        && value != "."
        && value != ".."
}
fn safe_relative_path(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 240
        && !value.contains(['\\', ':', '\0'])
        && !value.eq_ignore_ascii_case(MARKER)
        && value
            .split('/')
            .all(|part| !part.is_empty() && part != "." && part != "..")
        && Path::new(value)
            .components()
            .all(|component| matches!(component, Component::Normal(_)))
}
fn validate_catalog(catalog: &Catalog) -> Result<(), String> {
    if catalog.schema_version != 1
        || catalog.source.repository != "https://github.com/work-r-labs/robots"
        || !valid_hex(&catalog.source.revision, 40)
        || catalog.models.is_empty()
        || catalog.models.len() > 256
    {
        return Err("机器人模型目录版本或来源无效".into());
    }
    let mut ids = HashSet::new();
    for model in &catalog.models {
        if !safe_id(&model.id)
            || !ids.insert(&model.id)
            || model.files.is_empty()
            || model.files.len() > MAX_FILES
        {
            return Err("机器人模型目录的型号或文件数量无效".into());
        }
        let mut paths = HashSet::new();
        let mut bytes = 0_u64;
        for file in &model.files {
            if !safe_relative_path(&file.path)
                || !paths.insert(file.path.to_ascii_lowercase())
                || file.bytes == 0
                || file.bytes > MAX_FILE_BYTES
                || !valid_hex(&file.sha256, 64)
            {
                return Err("机器人模型目录的文件路径、大小或摘要无效".into());
            }
            let url = reqwest::Url::parse(&file.url).map_err(|_| "模型下载地址无效")?;
            let parts: Vec<_> = url.path_segments().ok_or("模型下载路径无效")?.collect();
            let pinned_repository = parts.len() >= 4
                && valid_hex(parts[2], 40)
                && ((parts[0] == "work-r-labs"
                    && parts[1] == "robots"
                    && parts[2] == catalog.source.revision)
                    || (parts[0] == "spdx" && parts[1] == "license-list-data"));
            if url.scheme() != "https"
                || url.host_str() != Some("raw.githubusercontent.com")
                || !url.username().is_empty()
                || url.password().is_some()
                || url.port().is_some()
                || url.query().is_some()
                || url.fragment().is_some()
                || !pinned_repository
            {
                return Err("模型下载地址不在固定版本目录的允许来源中".into());
            }
            bytes = bytes.checked_add(file.bytes).ok_or("模型文件大小溢出")?;
        }
        if bytes != model.download_bytes
            || bytes > MAX_MODEL_BYTES
            || !model.files.iter().any(|file| file.path == model.urdf_path)
        {
            return Err("机器人模型目录的总大小或描述文件无效".into());
        }
    }
    Ok(())
}

struct Job {
    id: String,
    cancel: watch::Sender<bool>,
}
#[derive(Default)]
struct Queue {
    active: Option<Job>,
    pending: Option<Job>,
    cancelled_ids: VecDeque<String>,
}
pub struct ModelCache {
    root: PathBuf,
    catalog: Catalog,
    client: reqwest::Client,
    queue: Mutex<Queue>,
    changed: Notify,
    listing: tokio::sync::Mutex<()>,
}
/// A model-cache setup error must not prevent bundled robot motion from starting.
pub struct ModelCacheState(Result<ModelCache, String>);
impl ModelCacheState {
    pub fn new(app_data: Result<PathBuf, String>) -> Self {
        Self(app_data.and_then(ModelCache::new))
    }
    pub fn cache(&self) -> Result<&ModelCache, String> {
        self.0.as_ref().map_err(Clone::clone)
    }
}
struct RequestGuard<'a> {
    manager: &'a ModelCache,
    id: &'a str,
}
impl Drop for RequestGuard<'_> {
    fn drop(&mut self) {
        if let Ok(mut queue) = self.manager.queue.lock() {
            if queue.active.as_ref().is_some_and(|job| job.id == self.id) {
                queue.active = None;
            }
            if queue.pending.as_ref().is_some_and(|job| job.id == self.id) {
                queue.pending = None;
            }
        }
        self.manager.changed.notify_one();
    }
}
struct StagingDirectory {
    path: PathBuf,
    published: bool,
}
impl Drop for StagingDirectory {
    fn drop(&mut self) {
        if !self.published {
            // Normal cancellation awaits all file I/O before reaching this guard.
            // A process crash may leave a partial directory; it has no final marker/location.
            let _ = std::fs::remove_dir_all(&self.path);
        }
    }
}
fn check_cancel(cancel: &watch::Receiver<bool>) -> Result<(), String> {
    if *cancel.borrow() {
        Err(CANCELLED.into())
    } else {
        Ok(())
    }
}
async fn cancellation(cancel: &mut watch::Receiver<bool>) {
    while !*cancel.borrow() {
        if cancel.changed().await.is_err() {
            break;
        }
    }
}
async fn cancellable<T, F>(cancel: &mut watch::Receiver<bool>, future: F) -> Result<T, String>
where
    F: Future<Output = Result<T, String>>,
{
    check_cancel(cancel)?;
    tokio::select! {
        biased;
        _ = cancellation(cancel) => Err(CANCELLED.into()),
        result = future => result,
    }
}
async fn network<T, F>(
    cancel: &mut watch::Receiver<bool>,
    deadline: Instant,
    future: F,
) -> Result<T, String>
where
    F: Future<Output = Result<T, String>>,
{
    let remaining = deadline
        .checked_duration_since(Instant::now())
        .ok_or(TIMEOUT_MESSAGE)?;
    cancellable(cancel, async {
        tokio::time::timeout(remaining, future)
            .await
            .map_err(|_| TIMEOUT_MESSAGE.to_owned())?
    })
    .await
}

struct Progress<'a, P> {
    request_id: &'a str,
    model: &'a CatalogModel,
    callback: &'a P,
    last: Instant,
}
impl<P: Fn(ModelDownloadProgress) -> Result<(), String>> Progress<'_, P> {
    fn report(
        &mut self,
        phase: &'static str,
        downloaded_bytes: u64,
        file: Option<&str>,
        force: bool,
    ) -> Result<(), String> {
        if !force && self.last.elapsed() < Duration::from_millis(100) {
            return Ok(());
        }
        self.last = Instant::now();
        (self.callback)(ModelDownloadProgress {
            request_id: self.request_id.into(),
            model_id: self.model.id.clone(),
            phase,
            downloaded_bytes,
            total_bytes: self.model.download_bytes,
            file: file.map(str::to_owned),
        })
    }
}

trait DownloadStream: Send {
    fn content_length(&self) -> Option<u64>;
    fn next_chunk(&mut self) -> impl Future<Output = Result<Option<Bytes>, String>> + Send;
}
trait Fetcher: Sync {
    type Stream: DownloadStream;
    fn open(&self, file: &CatalogFile)
        -> impl Future<Output = Result<Self::Stream, String>> + Send;
}
struct HttpFetcher<'a>(&'a reqwest::Client);
struct HttpStream(reqwest::Response);
impl DownloadStream for HttpStream {
    fn content_length(&self) -> Option<u64> {
        self.0.content_length()
    }
    async fn next_chunk(&mut self) -> Result<Option<Bytes>, String> {
        self.0
            .chunk()
            .await
            .map_err(|error| format!("模型下载中断：{error}"))
    }
}
impl Fetcher for HttpFetcher<'_> {
    type Stream = HttpStream;
    async fn open(&self, file: &CatalogFile) -> Result<HttpStream, String> {
        let response = self
            .0
            .get(&file.url)
            .send()
            .await
            .map_err(|error| format!("无法下载模型文件 {}：{error}", file.path))?;
        if !response.status().is_success() {
            return Err(format!(
                "模型下载失败（HTTP {}）：{}",
                response.status().as_u16(),
                file.path
            ));
        }
        Ok(HttpStream(response))
    }
}

/// Ensure each existing component is a real directory, never a cached symlink.
fn secure_directory(root: &Path, relative: &Path, create: bool) -> Result<PathBuf, String> {
    let mut path = root.to_path_buf();
    for component in relative.components() {
        let Component::Normal(part) = component else {
            return Err("模型缓存目录无效".into());
        };
        path.push(part);
        if create {
            match std::fs::create_dir(&path) {
                Ok(()) => (),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => (),
                Err(_) => return Err("无法创建模型缓存目录，请检查磁盘空间和权限".into()),
            }
        }
        let metadata = std::fs::symlink_metadata(&path).map_err(|_| "模型缓存目录不存在")?;
        if metadata.file_type().is_symlink() || !metadata.is_dir() {
            return Err("模型缓存目录不是普通目录".into());
        }
    }
    Ok(path)
}
fn checked_file(root: &Path, relative: &str) -> Result<PathBuf, String> {
    if !safe_relative_path(relative) && relative != MARKER {
        return Err("模型缓存文件路径无效".into());
    }
    let relative = Path::new(relative);
    secure_directory(root, relative.parent().unwrap_or(Path::new("")), false)?;
    let path = root.join(relative);
    let metadata = std::fs::symlink_metadata(&path).map_err(|_| "模型缓存文件不存在")?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err("模型缓存文件不是普通文件".into());
    }
    let canonical = std::fs::canonicalize(&path).map_err(|_| "模型缓存文件路径无法解析")?;
    if !canonical.starts_with(root) {
        return Err("模型缓存文件超出允许目录".into());
    }
    Ok(canonical)
}

impl ModelCache {
    /// `app_data` comes from Tauri's path resolver, never an IPC argument.
    pub fn new(app_data: PathBuf) -> Result<Self, String> {
        let catalog: Catalog =
            serde_json::from_str(CATALOG).map_err(|_| "机器人模型目录无法读取")?;
        Self::with_catalog(app_data, catalog)
    }
    fn with_catalog(app_data: PathBuf, catalog: Catalog) -> Result<Self, String> {
        validate_catalog(&catalog)?;
        std::fs::create_dir_all(&app_data).map_err(|_| "无法创建应用模型存储目录")?;
        let app_data = std::fs::canonicalize(app_data).map_err(|_| "无法解析应用模型存储目录")?;
        let root = secure_directory(&app_data, Path::new("model-cache"), true)?;
        // reqwest 0.13's default provider is aws-lc; explicitly use the smaller ring provider.
        let _ = rustls::crypto::ring::default_provider().install_default();
        let client = reqwest::Client::builder()
            .https_only(true)
            .redirect(reqwest::redirect::Policy::none())
            .connect_timeout(Duration::from_secs(15))
            .read_timeout(Duration::from_secs(30))
            .timeout(DOWNLOAD_TIMEOUT)
            .user_agent(concat!("Emulation/", env!("CARGO_PKG_VERSION")))
            .build()
            .map_err(|_| "无法初始化模型下载服务")?;
        Ok(Self {
            root,
            catalog,
            client,
            queue: Mutex::new(Queue::default()),
            changed: Notify::new(),
            listing: tokio::sync::Mutex::new(()),
        })
    }
    fn model(&self, model_id: &str) -> Result<&CatalogModel, String> {
        self.catalog
            .models
            .iter()
            .find(|model| model.id == model_id)
            .ok_or_else(|| "机器人型号不在已验证的模型目录中".into())
    }
    fn directory(&self, model: &CatalogModel) -> PathBuf {
        self.root
            .join(&self.catalog.source.revision)
            .join(&model.id)
    }
    fn marker(&self, model: &CatalogModel) -> Result<CompleteMarker, String> {
        Ok(CompleteMarker {
            model_id: model.id.clone(),
            revision: self.catalog.source.revision.clone(),
            manifest_sha256: digest(&serde_json::to_vec(model).map_err(|_| "模型目录无法序列化")?),
        })
    }
    fn reserve<'a>(
        &'a self,
        request_id: &'a str,
    ) -> Result<(RequestGuard<'a>, watch::Receiver<bool>), String> {
        if !safe_id(request_id) {
            return Err("模型下载请求标识无效".into());
        }
        let mut queue = self.queue.lock().map_err(|_| "模型下载队列异常")?;
        if queue.cancelled_ids.iter().any(|known| known == request_id) {
            return Err(CANCELLED.into());
        }
        if queue
            .active
            .as_ref()
            .is_some_and(|job| job.id == request_id)
            || queue
                .pending
                .as_ref()
                .is_some_and(|job| job.id == request_id)
        {
            return Err("模型下载请求标识重复".into());
        }
        if let Some(active) = &queue.active {
            active.cancel.send_replace(true);
        }
        if let Some(pending) = queue.pending.take() {
            pending.cancel.send_replace(true);
        }
        let (sender, receiver) = watch::channel(false);
        queue.pending = Some(Job {
            id: request_id.into(),
            cancel: sender,
        });
        Ok((
            RequestGuard {
                manager: self,
                id: request_id,
            },
            receiver,
        ))
    }
    async fn acquire(
        &self,
        request_id: &str,
        cancel: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        loop {
            check_cancel(cancel)?;
            {
                let mut queue = self.queue.lock().map_err(|_| "模型下载队列异常")?;
                if !queue
                    .pending
                    .as_ref()
                    .is_some_and(|job| job.id == request_id)
                {
                    return Err(CANCELLED.into());
                }
                if queue.active.is_none() {
                    queue.active = queue.pending.take();
                    return Ok(());
                }
            }
            cancellable(cancel, async {
                self.changed.notified().await;
                Ok(())
            })
            .await?;
        }
    }
    pub fn cancel(&self, request_id: &str) {
        if !safe_id(request_id) {
            return;
        }
        if let Ok(mut queue) = self.queue.lock() {
            for job in [&queue.active, &queue.pending].into_iter().flatten() {
                if job.id == request_id {
                    job.cancel.send_replace(true);
                }
            }
            if !queue.cancelled_ids.iter().any(|known| known == request_id) {
                queue.cancelled_ids.push_back(request_id.into());
                if queue.cancelled_ids.len() > 64 {
                    queue.cancelled_ids.pop_front();
                }
            }
        }
    }
    pub fn is_cancelled(&self, request_id: &str) -> bool {
        self.queue
            .lock()
            .map(|queue| queue.cancelled_ids.iter().any(|known| known == request_id))
            .unwrap_or(true)
    }
    async fn verify_cache(
        &self,
        model: &CatalogModel,
        cancel: &watch::Receiver<bool>,
    ) -> Result<bool, String> {
        check_cancel(cancel)?;
        let relative = Path::new(&self.catalog.source.revision).join(&model.id);
        let Ok(directory) = secure_directory(&self.root, &relative, false) else {
            return Ok(false);
        };
        let Ok(marker_path) = checked_file(&directory, MARKER) else {
            return Ok(false);
        };
        if fs::metadata(&marker_path)
            .await
            .map_err(|_| "无法读取模型缓存标识")?
            .len()
            > 4096
        {
            return Ok(false);
        }
        let marker = fs::read(marker_path)
            .await
            .map_err(|_| "无法读取模型缓存标识")?;
        let Ok(marker) = serde_json::from_slice::<CompleteMarker>(&marker) else {
            return Ok(false);
        };
        let expected = self.marker(model)?;
        if marker.model_id != expected.model_id
            || marker.revision != expected.revision
            || marker.manifest_sha256 != expected.manifest_sha256
        {
            return Ok(false);
        }
        let mut buffer = vec![0_u8; BUFFER_BYTES];
        for file in &model.files {
            check_cancel(cancel)?;
            let Ok(path) = checked_file(&directory, &file.path) else {
                return Ok(false);
            };
            let mut handle = fs::File::open(path)
                .await
                .map_err(|_| "无法读取已下载的模型文件")?;
            if handle
                .metadata()
                .await
                .map_err(|_| "无法读取模型文件大小")?
                .len()
                != file.bytes
            {
                return Ok(false);
            }
            let mut hash = Sha256::new();
            let mut received = 0_u64;
            loop {
                check_cancel(cancel)?;
                let count = handle
                    .read(&mut buffer)
                    .await
                    .map_err(|_| "模型缓存读取失败")?;
                if count == 0 {
                    break;
                }
                received += count as u64;
                if received > file.bytes {
                    return Ok(false);
                }
                hash.update(&buffer[..count]);
            }
            if received != file.bytes || format!("{:x}", hash.finalize()) != file.sha256 {
                return Ok(false);
            }
        }
        check_cancel(cancel)?;
        Ok(true)
    }
    /// A single bounded background inventory pass; never downloads or trusts marker flags alone.
    pub async fn list(&self) -> Result<Vec<ModelCacheStatus>, String> {
        let _listing = self
            .listing
            .try_lock()
            .map_err(|_| "模型缓存正在检查，请稍后重试")?;
        let (_sender, cancel) = watch::channel(false);
        let mut statuses = Vec::with_capacity(self.catalog.models.len());
        for model in &self.catalog.models {
            statuses.push(ModelCacheStatus {
                model_id: model.id.clone(),
                revision: self.catalog.source.revision.clone(),
                cached: self.verify_cache(model, &cancel).await?,
            });
        }
        Ok(statuses)
    }
    pub async fn ensure<P>(
        &self,
        request_id: &str,
        model_id: &str,
        progress: P,
    ) -> Result<CachedModel, String>
    where
        P: Fn(ModelDownloadProgress) -> Result<(), String> + Send + Sync,
    {
        self.ensure_with(request_id, model_id, progress, &HttpFetcher(&self.client))
            .await
    }
    async fn ensure_with<P, F>(
        &self,
        request_id: &str,
        model_id: &str,
        callback: P,
        fetcher: &F,
    ) -> Result<CachedModel, String>
    where
        P: Fn(ModelDownloadProgress) -> Result<(), String> + Send + Sync,
        F: Fetcher,
    {
        let model = self.model(model_id)?;
        let (_guard, mut cancel) = self.reserve(request_id)?;
        self.acquire(request_id, &mut cancel).await?;
        let mut progress = Progress {
            request_id,
            model,
            callback: &callback,
            last: Instant::now(),
        };
        progress.report("checking", 0, None, true)?;
        let cache_hit = self.verify_cache(model, &cancel).await?;
        if !cache_hit {
            self.download(model, request_id, &mut cancel, &mut progress, fetcher)
                .await?;
        }
        check_cancel(&cancel)?;
        let directory = self.directory(model);
        let files = model
            .files
            .iter()
            .map(|file| {
                let path = checked_file(&directory, &file.path)?;
                Ok(CachedModelFile {
                    logical_path: file.path.clone(),
                    local_path: path
                        .to_str()
                        .ok_or("模型缓存路径无法转换为有效文本")?
                        .into(),
                    bytes: file.bytes,
                    sha256: file.sha256.clone(),
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        check_cancel(&cancel)?;
        progress.report("ready", model.download_bytes, None, true)?;
        check_cancel(&cancel)?;
        Ok(CachedModel {
            model_id: model.id.clone(),
            revision: self.catalog.source.revision.clone(),
            cache_hit,
            files,
        })
    }
    async fn download<P, F>(
        &self,
        model: &CatalogModel,
        request_id: &str,
        cancel: &mut watch::Receiver<bool>,
        progress: &mut Progress<'_, P>,
        fetcher: &F,
    ) -> Result<(), String>
    where
        P: Fn(ModelDownloadProgress) -> Result<(), String> + Send + Sync,
        F: Fetcher,
    {
        check_cancel(cancel)?;
        let partial_root = secure_directory(&self.root, Path::new(".partial"), true)?;
        let staging_path = partial_root.join(request_id);
        fs::create_dir(&staging_path)
            .await
            .map_err(|_| "无法创建本次模型下载的临时目录")?;
        let mut staging = StagingDirectory {
            path: staging_path,
            published: false,
        };
        let deadline = Instant::now() + DOWNLOAD_TIMEOUT;
        let mut completed_bytes = 0_u64;
        for file in &model.files {
            check_cancel(cancel)?;
            progress.report("downloading", completed_bytes, Some(&file.path), true)?;
            let mut stream = network(cancel, deadline, fetcher.open(file)).await?;
            if stream
                .content_length()
                .is_some_and(|length| length != file.bytes)
            {
                return Err(format!("模型文件大小与目录不符：{}", file.path));
            }
            let relative = Path::new(&file.path);
            secure_directory(
                &staging.path,
                relative.parent().unwrap_or(Path::new("")),
                true,
            )?;
            let mut handle = fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(staging.path.join(relative))
                .await
                .map_err(|_| "无法写入模型缓存，请检查磁盘空间和权限")?;
            let mut received = 0_u64;
            let mut hash = Sha256::new();
            while let Some(chunk) = network(cancel, deadline, stream.next_chunk()).await? {
                if chunk.len() as u64 > file.bytes - received {
                    return Err(format!("模型文件超过目录规定的大小：{}", file.path));
                }
                handle
                    .write_all(&chunk)
                    .await
                    .map_err(|_| "无法写入模型文件，请检查磁盘空间")?;
                hash.update(&chunk);
                received += chunk.len() as u64;
                progress.report(
                    "downloading",
                    completed_bytes + received,
                    Some(&file.path),
                    false,
                )?;
            }
            check_cancel(cancel)?;
            if received != file.bytes || format!("{:x}", hash.finalize()) != file.sha256 {
                return Err(format!("模型文件不完整或 SHA-256 校验失败：{}", file.path));
            }
            handle.sync_all().await.map_err(|_| "无法保存模型文件")?;
            drop(handle);
            completed_bytes += received;
        }
        check_cancel(cancel)?;
        if Instant::now() >= deadline {
            return Err(TIMEOUT_MESSAGE.into());
        }
        progress.report("preparing", completed_bytes, None, true)?;
        let marker =
            serde_json::to_vec(&self.marker(model)?).map_err(|_| "无法创建模型缓存标识")?;
        let mut handle = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(staging.path.join(MARKER))
            .await
            .map_err(|_| "无法写入模型缓存标识")?;
        handle
            .write_all(&marker)
            .await
            .map_err(|_| "无法写入模型缓存标识")?;
        handle
            .sync_all()
            .await
            .map_err(|_| "无法保存模型缓存标识")?;
        drop(handle);
        check_cancel(cancel)?;
        if Instant::now() >= deadline {
            return Err(TIMEOUT_MESSAGE.into());
        }
        let revision_root =
            secure_directory(&self.root, Path::new(&self.catalog.source.revision), true)?;
        let final_directory = revision_root.join(&model.id);
        if let Ok(metadata) = fs::symlink_metadata(&final_directory).await {
            if metadata.file_type().is_symlink() || !metadata.is_dir() {
                return Err("模型缓存目标不是普通目录".into());
            }
            // A previously damaged cache is removed only once its replacement is fully verified.
            fs::remove_dir_all(&final_directory)
                .await
                .map_err(|_| "无法替换损坏的模型缓存")?;
        }
        check_cancel(cancel)?;
        fs::rename(&staging.path, &final_directory)
            .await
            .map_err(|_| "无法完成模型缓存，请检查目录权限")?;
        staging.published = true;
        Ok(())
    }
}

#[cfg(feature = "desktop")]
#[tauri::command]
pub async fn list_model_cache(
    state: tauri::State<'_, ModelCacheState>,
) -> Result<Vec<ModelCacheStatus>, String> {
    state.cache()?.list().await
}
#[cfg(feature = "desktop")]
#[tauri::command]
pub async fn ensure_model(
    app: tauri::AppHandle,
    state: tauri::State<'_, ModelCacheState>,
    request_id: String,
    model_id: String,
    on_progress: tauri::ipc::Channel<ModelDownloadProgress>,
) -> Result<CachedModel, String> {
    use tauri::Manager;
    let cache = state.cache()?;
    let model = cache
        .ensure(&request_id, &model_id, |event| {
            on_progress.send(event).map_err(|_| CANCELLED.to_string())
        })
        .await?;
    if cache.is_cancelled(&request_id) {
        return Err(CANCELLED.into());
    }
    // No wildcard directory access: only complete, verified catalog files become readable.
    for file in &model.files {
        app.asset_protocol_scope()
            .allow_file(&file.local_path)
            .map_err(|_| "无法授权读取已验证模型文件")?;
    }
    Ok(model)
}
#[cfg(feature = "desktop")]
#[tauri::command]
pub fn cancel_model_download(state: tauri::State<'_, ModelCacheState>, request_id: String) {
    if let Ok(cache) = state.cache() {
        cache.cancel(&request_id);
    }
}

#[cfg(test)]
#[path = "model_cache_tests.rs"]
mod tests;
