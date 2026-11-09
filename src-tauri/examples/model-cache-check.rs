//! Explicit real-network diagnostic. Unit tests use a forbidden-network fake instead.
use emulation_desktop::model_cache::ModelCache;
use std::path::PathBuf;

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let root = args
        .next()
        .ok_or("usage: model-cache-check <temporary-cache-root> <catalog-model-id>")?;
    let model_id = args.next().ok_or("missing catalog model ID")?;
    if args.next().is_some() {
        return Err("unexpected extra arguments".into());
    }
    let manager = ModelCache::new(PathBuf::from(root))?;
    let result = manager
        .ensure(
            &format!("check-{}", std::process::id()),
            &model_id,
            |event| {
                eprintln!(
                    "{} {}/{} {}",
                    event.phase,
                    event.downloaded_bytes,
                    event.total_bytes,
                    event.file.as_deref().unwrap_or("")
                );
                Ok(())
            },
        )
        .await?;
    println!(
        "{}",
        serde_json::json!({ "modelId": result.model_id, "revision": result.revision, "cacheHit": result.cache_hit, "verifiedFiles": result.files.len(), "bytes": result.files.iter().map(|file| file.bytes).sum::<u64>() })
    );
    Ok(())
}
