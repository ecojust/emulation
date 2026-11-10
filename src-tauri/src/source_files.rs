//! Offline source text I/O. This does not compile or certify controller programs.
use serde::{Deserialize, Serialize};
use std::{
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
    time::{SystemTime, UNIX_EPOCH},
};

pub const MAX_SOURCE_BYTES: usize = 1024 * 1024;
pub const MAX_IMPORT_BYTES: usize = 4 * MAX_SOURCE_BYTES;
pub const MAX_SOURCE_FILES: usize = 32;
static TEMP_ID: AtomicU64 = AtomicU64::new(0);
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SourceTextFile {
    pub name: String,
    pub text: String,
}
#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SaveSourceRequest {
    pub language_id: String,
    pub file_name: String,
    pub text: String,
}
#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct SavedSource {
    pub path: String,
    pub file_name: String,
}

pub fn extensions(language: &str) -> Result<&'static [&'static str], String> {
    match language {
        "rapid" => Ok(&["modx", "sysx", "mod", "sys"]),
        "fanuc-ls" => Ok(&["ls"]),
        "inform" => Ok(&["jbi"]),
        "krl" => Ok(&["src", "dat"]),
        _ => Err("未知机器人程序语言".into()),
    }
}
pub fn validate_name(language: &str, name: &str) -> Result<(), String> {
    let allowed = extensions(language)?;
    // Match ECMAScript String.trim, used by the UI's shared file-name validator.
    // Rust's Unicode White_Space differs for U+0085 and U+FEFF.
    let javascript_whitespace = |c: char| matches!(c, '\u{9}'..='\u{d}' | '\u{20}' | '\u{a0}' | '\u{1680}' | '\u{2000}'..='\u{200a}' | '\u{2028}' | '\u{2029}' | '\u{202f}' | '\u{205f}' | '\u{3000}' | '\u{feff}');
    if name.is_empty()
        || name.len() > 255
        || name.encode_utf16().count() > 128
        || name.trim_matches(javascript_whitespace) != name
        || name
            .chars()
            .any(|c| c <= '\u{1f}' || c == '\u{7f}' || "/\\:*?\"<>|".contains(c))
        || name.ends_with(['.', ' '])
        || name == "."
        || name == ".."
    {
        return Err("源文件名必须是有效的本地文件名，不能包含路径或控制字符".into());
    }
    let stem = name.split('.').next().unwrap_or("").to_ascii_uppercase();
    if [
        "COM¹", "COM²", "COM³", "LPT¹", "LPT²", "LPT³", "CON", "PRN", "AUX", "NUL", "COM1", "COM2",
        "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9", "LPT1", "LPT2", "LPT3", "LPT4",
        "LPT5", "LPT6", "LPT7", "LPT8", "LPT9",
    ]
    .contains(&stem.as_str())
    {
        return Err("源文件名是操作系统保留名称".into());
    }
    if name.rfind('.').unwrap_or(0) == 0 {
        return Err("源文件需要非空文件名和扩展名".into());
    }
    let extension = Path::new(name)
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("");
    if !allowed
        .iter()
        .any(|value| extension.eq_ignore_ascii_case(value))
    {
        return Err(format!("此语言的源文件扩展名应为 {}", allowed.join(" / ")));
    }
    Ok(())
}
pub fn validate_text(text: &str) -> Result<(), String> {
    if text.len() > MAX_SOURCE_BYTES {
        return Err("源文件超过 1 MiB 限制".into());
    }
    if text.contains('\0') {
        return Err("文件包含二进制或UTF-16数据；请转换为UTF-8源文件".into());
    }
    Ok(())
}
pub fn read_source(path: &Path, language: &str) -> Result<SourceTextFile, String> {
    let name = path
        .file_name()
        .and_then(|s| s.to_str())
        .ok_or("文件名不是有效Unicode")?
        .to_owned();
    validate_name(language, &name)?;
    let input = File::open(path).map_err(|_| "无法读取选中的源文件")?;
    if !input
        .metadata()
        .map_err(|_| "无法读取源文件信息")?
        .is_file()
    {
        return Err("请选择普通源文件".into());
    }
    let mut bytes = Vec::new();
    input
        .take((MAX_SOURCE_BYTES + 1) as u64)
        .read_to_end(&mut bytes)
        .map_err(|_| "读取源文件失败")?;
    if bytes.len() > MAX_SOURCE_BYTES {
        return Err("源文件超过 1 MiB 限制".into());
    }
    let text = String::from_utf8(bytes)
        .map_err(|_| "源文件不是UTF-8；请先转换编码，避免内容被错误替换")?;
    validate_text(&text)?;
    Ok(SourceTextFile { name, text })
}
/// A failed write leaves the selected original file intact. Temporary files
/// live beside the destination so the final rename stays on one filesystem.
pub fn write_source(path: &Path, language: &str, text: &str) -> Result<(), String> {
    let name = path
        .file_name()
        .and_then(|s| s.to_str())
        .ok_or("文件名不是有效Unicode")?;
    validate_name(language, name)?;
    validate_text(text)?;
    let parent = path.parent().ok_or("保存路径无效")?;
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| "无法创建临时源文件")?
        .as_nanos();
    let temp = parent.join(format!(
        ".emulation-source-{}-{}-{}.tmp",
        std::process::id(),
        stamp,
        TEMP_ID.fetch_add(1, Ordering::Relaxed)
    ));
    let result = (|| {
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temp)
            .map_err(|_| "无法创建源文件，请检查目录权限")?;
        file.write_all(text.as_bytes())
            .and_then(|_| file.sync_all())
            .map_err(|_| "源文件写入失败，原文件未替换")?;
        drop(file);
        fs::rename(&temp, path).map_err(|_| "无法替换源文件，原文件保留")?;
        Ok(())
    })();
    if result.is_err() {
        let _ = fs::remove_file(&temp);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    fn directory() -> std::path::PathBuf {
        let root = std::env::temp_dir().join(format!(
            "emulation-source-test-{}-{}",
            std::process::id(),
            TEMP_ID.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&root).unwrap();
        root
    }
    #[test]
    fn validates_language_names_without_claiming_controller_validity() {
        for (language, name) in [
            ("rapid", "程序.MOD"),
            ("fanuc-ls", "TEST.LS"),
            ("inform", "JOB.jbi"),
            ("krl", "cell.dat"),
        ] {
            validate_name(language, name).unwrap();
        }
        for name in [
            "../x.mod",
            "dir\\x.mod",
            "CON.mod",
            "data.mod ",
            "x\0.mod",
            "x.ls",
        ] {
            assert!(validate_name("rapid", name).is_err(), "{name:?}");
        }
        assert!(extensions("karel").is_err());
    }
    #[test]
    fn portable_name_boundaries_match_the_javascript_draft_validator() {
        for name in [
            format!("{}.mod", "a".repeat(124)),
            format!("{}.mod", "中".repeat(83)),
            "\u{85}allowed.mod".into(),
            "COM10.mod".into(),
        ] {
            validate_name("rapid", &name).unwrap();
        }
        for name in [
            format!("{}.mod", "a".repeat(125)),
            format!("{}.mod", "中".repeat(84)),
            "\u{feff}hidden.mod".into(),
            "\u{a0}space.mod".into(),
            "COM¹.mod".into(),
            "lpt².sys".into(),
            ".mod".into(),
            "missing-extension".into(),
        ] {
            assert!(validate_name("rapid", &name).is_err(), "{name:?}");
        }
        let exact_limit = format!("{}x", "中".repeat(MAX_SOURCE_BYTES / 3));
        assert_eq!(exact_limit.len(), MAX_SOURCE_BYTES);
        validate_text(&exact_limit).unwrap();
        assert!(validate_text(&(exact_limit + "x")).is_err());
    }
    #[test]
    fn source_round_trip_preserves_utf8_bom_crlf_and_unknown_syntax() {
        let root = directory();
        let path = root.join("demo.mod");
        let source = "\u{feff}MODULE Demo\r\n! 中文 😀\n  VendorOption \\Future;\r\nENDMODULE";
        write_source(&path, "rapid", source).unwrap();
        assert_eq!(fs::read(&path).unwrap(), source.as_bytes());
        assert_eq!(read_source(&path, "rapid").unwrap().text, source);
        write_source(&path, "rapid", "new").unwrap();
        assert_eq!(fs::read_to_string(&path).unwrap(), "new");
        assert_eq!(fs::read_dir(&root).unwrap().count(), 1);
        fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn modern_rapid_extensions_are_plain_utf8_and_keep_imported_bytes() {
        let root = directory();
        for name in ["module.modx", "System.SYSX", "legacy.mod", "legacy.sys"] {
            for prefix in ["", "\u{feff}"] {
                let text = format!("{prefix}MODULE Test\r\n! 中文\r\nENDMODULE\r\n");
                let path = root.join(name);
                write_source(&path, "rapid", &text).unwrap();
                assert_eq!(fs::read(&path).unwrap(), text.as_bytes());
                let loaded = read_source(&path, "rapid").unwrap();
                assert_eq!(loaded.name, name);
                assert_eq!(loaded.text, text);
            }
        }
        for language in ["fanuc-ls", "inform", "krl"] {
            assert!(validate_name(language, "module.modx").is_err());
        }
        assert!(validate_name("rapid", "module.modxx").is_err());
        fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn rejects_binary_oversize_and_failed_writes_without_replacing_original() {
        let root = directory();
        let path = root.join("demo.mod");
        fs::write(&path, b"original").unwrap();
        assert!(write_source(&path, "rapid", &"x".repeat(MAX_SOURCE_BYTES + 1)).is_err());
        assert_eq!(fs::read(&path).unwrap(), b"original");
        fs::write(&path, [0xff, 0xfe, 0, 0]).unwrap();
        assert!(read_source(&path, "rapid").is_err());
        fs::write(&path, vec![b'x'; MAX_SOURCE_BYTES + 1]).unwrap();
        assert!(read_source(&path, "rapid").is_err());
        assert!(write_source(&root.join("missing").join("x.mod"), "rapid", "text").is_err());
        fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn rejected_destination_never_appends_an_unconfirmed_extension() {
        let root = directory();
        let chosen = root.join("demo");
        let similar = root.join("demo.mod");
        fs::write(&chosen, b"chosen original").unwrap();
        fs::write(&similar, b"unconfirmed original").unwrap();
        assert!(write_source(&chosen, "rapid", "replacement").is_err());
        assert_eq!(fs::read(&chosen).unwrap(), b"chosen original");
        assert_eq!(fs::read(&similar).unwrap(), b"unconfirmed original");
        assert_eq!(fs::read_dir(&root).unwrap().count(), 2);
        fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn failed_final_rename_retains_destination_and_cleans_the_completed_temporary_file() {
        let root = directory();
        let destination = root.join("existing.mod");
        fs::create_dir(&destination).unwrap();
        fs::write(destination.join("sentinel"), b"keep me").unwrap();
        assert!(read_source(&destination, "rapid").is_err());
        assert!(write_source(&destination, "rapid", "complete replacement").is_err());
        assert_eq!(fs::read(destination.join("sentinel")).unwrap(), b"keep me");
        assert_eq!(fs::read_dir(&root).unwrap().count(), 1);
        fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn simultaneous_readers_only_observe_whole_old_or_new_source_files() {
        use std::sync::{atomic::AtomicBool, Arc, Barrier};
        let root = directory();
        let path = root.join("atomic.mod");
        let old = "a".repeat(64 * 1024);
        let new = "b".repeat(64 * 1024);
        write_source(&path, "rapid", &old).unwrap();
        let barrier = Arc::new(Barrier::new(2));
        let done = Arc::new(AtomicBool::new(false));
        let writer_path = path.clone();
        let writer_barrier = barrier.clone();
        let writer_done = done.clone();
        let writer_old = old.clone();
        let writer_new = new.clone();
        let writer = std::thread::spawn(move || {
            writer_barrier.wait();
            for index in 0..8 {
                write_source(
                    &writer_path,
                    "rapid",
                    if index % 2 == 0 {
                        &writer_new
                    } else {
                        &writer_old
                    },
                )
                .unwrap();
                std::thread::yield_now();
            }
            writer_done.store(true, Ordering::Release);
        });
        barrier.wait();
        while !done.load(Ordering::Acquire) && !writer.is_finished() {
            let bytes = fs::read(&path).unwrap();
            assert!(bytes == old.as_bytes() || bytes == new.as_bytes());
        }
        writer.join().unwrap();
        assert_eq!(fs::read_dir(&root).unwrap().count(), 1);
        fs::remove_dir_all(root).unwrap();
    }
}
