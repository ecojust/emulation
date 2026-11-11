#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use emulation_core::{
    collision::{CollisionRequest, CollisionResult},
    endpoint_inspection::EndpointInspectionRequest,
    generic_collision::GenericCollisionRequest,
    generic_endpoint_inspection::{GenericEndpointRequest, GenericIkSolution},
    generic_planner::{GenericPlanRequest, GenericPlanResult},
    generic_pose_planner::{GenericPosePlanRequest, GenericPosePlanResult},
    grinding::{GrindingRequest, GrindingResult},
    program::ProgramExportRequest,
    surface::{SurfaceRequest, SurfaceResult},
    types::{IkSolution, PlanRequest, PlanResult},
};
use emulation_desktop::{
    engine::{Engine, Reply, Work},
    model_cache,
    pendant::{Mode as PendantMode, PendantServer, PendantSession, PendantState},
};
use std::sync::Arc;
use tauri::{Emitter, Manager, State, WebviewUrl, WebviewWindowBuilder};
use tauri_plugin_clipboard_manager::ClipboardExt;
use tauri_plugin_dialog::DialogExt;

struct UiSmoke {
    enabled: bool,
}

// Only the local workspace owns the LAN service. The phone endpoint exposes
// typed simulation inputs, never Tauri commands, file access or robot devices.
fn pendant_window(window: &tauri::WebviewWindow) -> Result<(), String> {
    if window.label() == "main" {
        Ok(())
    } else {
        Err("示教器只能从本机工作区启动".into())
    }
}

#[tauri::command]
async fn start_pendant(
    app: tauri::AppHandle,
    window: tauri::WebviewWindow,
    server: State<'_, Arc<PendantServer>>,
) -> Result<PendantSession, String> {
    pendant_window(&window)?;
    server
        .inner()
        .start(Arc::new(move |input| {
            let _ = app.emit_to("main", "pendant-input", input);
        }))
        .await
}
#[tauri::command]
async fn stop_pendant(
    window: tauri::WebviewWindow,
    server: State<'_, Arc<PendantServer>>,
) -> Result<(), String> {
    pendant_window(&window)?;
    server.stop().await;
    Ok(())
}
#[tauri::command]
fn pendant_status(
    window: tauri::WebviewWindow,
    server: State<'_, Arc<PendantServer>>,
) -> Result<Option<PendantSession>, String> {
    pendant_window(&window)?;
    Ok(server.status())
}
#[tauri::command]
fn publish_pendant_state(
    window: tauri::WebviewWindow,
    server: State<'_, Arc<PendantServer>>,
    session_id: String,
    state: PendantState,
) -> Result<(), String> {
    pendant_window(&window)?;
    server.publish_state(&session_id, state)
}
#[tauri::command]
async fn pendant_jog(
    window: tauri::WebviewWindow,
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: emulation_core::pendant_jog::JogRequest,
) -> Result<emulation_core::pendant_jog::JogResult, String> {
    pendant_window(&window)?;
    match engine
        .submit(request_id, Work::Jog(request))?
        .await
        .map_err(|_| "示教求解未完成")??
    {
        Reply::Jog(result) => Ok(result),
        _ => Err("原生示教返回类型错误".into()),
    }
}
#[tauri::command]
async fn smoke_pendant_drive(
    window: tauri::WebviewWindow,
    smoke: State<'_, UiSmoke>,
    server: State<'_, Arc<PendantServer>>,
    mode: PendantMode,
) -> Result<(), String> {
    pendant_window(&window)?;
    if !smoke.enabled {
        return Err("UI smoke mode is not enabled".into());
    }
    server.smoke_drive(mode).await
}

#[tauri::command]
async fn open_robot_sources(
    app: tauri::AppHandle,
    language_id: String,
) -> Result<Option<Vec<emulation_desktop::source_files::SourceTextFile>>, String> {
    use emulation_desktop::source_files::*;
    let suffixes = extensions(&language_id)?;
    tauri::async_runtime::spawn_blocking(move || {
        let Some(paths) = app
            .dialog()
            .file()
            .add_filter("Robot source", suffixes)
            .blocking_pick_files()
        else {
            return Ok(None);
        };
        if paths.len() > MAX_SOURCE_FILES {
            return Err("一次最多导入32个源文件".into());
        }
        let mut files = Vec::new();
        let mut total = 0;
        for selected in paths {
            let path = selected.into_path().map_err(|_| "请选择本机源文件")?;
            let file = read_source(&path, &language_id)?;
            total += file.text.len();
            if total > MAX_IMPORT_BYTES {
                return Err("导入内容超过4 MiB限制".into());
            }
            files.push(file);
        }
        Ok(Some(files))
    })
    .await
    .map_err(|_| "源文件选择未能完成")?
}

#[tauri::command]
async fn save_robot_source(
    app: tauri::AppHandle,
    request: emulation_desktop::source_files::SaveSourceRequest,
) -> Result<Option<emulation_desktop::source_files::SavedSource>, String> {
    use emulation_desktop::source_files::*;
    validate_name(&request.language_id, &request.file_name)?;
    validate_text(&request.text)?;
    let suffixes = extensions(&request.language_id)?;
    tauri::async_runtime::spawn_blocking(move || {
        let Some(selected) = app
            .dialog()
            .file()
            .add_filter("Robot source", suffixes)
            .set_file_name(&request.file_name)
            .blocking_save_file()
        else {
            return Ok(None);
        };
        let path = selected.into_path().map_err(|_| "请选择本机保存路径")?;
        if path.extension().is_none() {
            return Err("请在保存对话框中保留源文件扩展名，避免改变已确认的目标路径".into());
        }
        write_source(&path, &request.language_id, &request.text)?;
        Ok(Some(SavedSource {
            file_name: path
                .file_name()
                .and_then(|s| s.to_str())
                .ok_or("文件名无效")?
                .to_owned(),
            path: path.to_string_lossy().into_owned(),
        }))
    })
    .await
    .map_err(|_| "源文件保存未能完成")?
}

#[tauri::command]
async fn preview_program(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: ProgramExportRequest,
) -> Result<emulation_core::program::ProgramFile, String> {
    match engine
        .submit(request_id, Work::Export(request))?
        .await
        .map_err(|_| "模块生成未完成")??
    {
        Reply::Export(file) => Ok(file),
        _ => Err("原生模块返回类型错误".into()),
    }
}

#[tauri::command]
fn ui_smoke_report(
    app: tauri::AppHandle,
    state: State<'_, UiSmoke>,
    success: bool,
    detail: String,
) -> Result<(), String> {
    if !state.enabled {
        return Err("UI smoke mode is not enabled".into());
    }
    println!(
        "Native UI smoke: {} — {}",
        if success { "passed" } else { "failed" },
        detail.chars().take(16000).collect::<String>()
    );
    app.exit(if success { 0 } else { 1 });
    Ok(())
}

#[tauri::command]
async fn solve_motion(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: PlanRequest,
) -> Result<PlanResult, String> {
    let reply = engine
        .submit(request_id, Work::Plan(request))?
        .await
        .map_err(|_| "原生运动计算未完成")??;
    match reply {
        Reply::Plan(result) => Ok(result),
        _ => Err("原生计算返回了错误的结果类型".into()),
    }
}

#[tauri::command]
async fn plan_grinding_path(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: GrindingRequest,
) -> Result<GrindingResult, String> {
    match engine
        .submit(request_id, Work::Grinding(request))?
        .await
        .map_err(|_| "路径规划未完成")??
    {
        Reply::Grinding(result) => Ok(result),
        _ => Err("原生路径规划返回类型错误".into()),
    }
}

#[tauri::command]
async fn generate_surface_path(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: SurfaceRequest,
) -> Result<SurfaceResult, String> {
    match engine
        .submit(request_id, Work::Surface(request))?
        .await
        .map_err(|_| "表面路径生成未完成")??
    {
        Reply::Surface(result) => Ok(result),
        _ => Err("原生表面路径返回类型错误".into()),
    }
}

#[tauri::command]
async fn solve_generic_motion(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: GenericPlanRequest,
) -> Result<GenericPlanResult, String> {
    match engine
        .submit(request_id, Work::GenericPlan(request))?
        .await
        .map_err(|_| "原生 URDF 运动计算未完成")??
    {
        Reply::GenericPlan(result) => Ok(result),
        _ => Err("原生 URDF 计算返回类型错误".into()),
    }
}

#[tauri::command]
async fn solve_generic_pose_motion(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: GenericPosePlanRequest,
) -> Result<GenericPosePlanResult, String> {
    match engine
        .submit(request_id, Work::GenericPosePlan(request))?
        .await
        .map_err(|_| "原生 URDF 位姿运动计算未完成")??
    {
        Reply::GenericPosePlan(result) => Ok(result),
        _ => Err("原生 URDF 位姿计算返回类型错误".into()),
    }
}

#[tauri::command]
async fn inspect_generic_endpoint(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: GenericEndpointRequest,
) -> Result<Vec<GenericIkSolution>, String> {
    match engine
        .submit(request_id, Work::InspectGenericEndpoint(request))?
        .await
        .map_err(|_| "原生 URDF 端点逆解检查未完成")??
    {
        Reply::InspectGenericEndpoint(result) => Ok(result),
        _ => Err("原生 URDF 端点检查返回类型错误".into()),
    }
}

#[tauri::command]
async fn check_generic_collisions(
    engine: State<'_, Arc<Engine>>,
    cache: State<'_, model_cache::ModelCacheState>,
    request_id: String,
    request: GenericCollisionRequest,
) -> Result<Vec<CollisionResult>, String> {
    // Resolve only trusted catalog metadata here; cancellable disk reads,
    // verification, BVH construction and checks run on the bounded CPU worker.
    let source = cache.cache()?.collision_source(&request.model_id)?;
    match engine
        .submit(request_id, Work::GenericCollision { request, source })?
        .await
        .map_err(|_| "原生 URDF 网格干涉检查未完成")??
    {
        Reply::GenericCollision(result) => Ok(result),
        _ => Err("原生 URDF 网格检查返回类型错误".into()),
    }
}

#[tauri::command]
async fn check_collisions(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: CollisionRequest,
) -> Result<Vec<CollisionResult>, String> {
    match engine
        .submit(request_id, Work::Collision(request))?
        .await
        .map_err(|_| "端点干涉检查未完成")??
    {
        Reply::Collision(results) => Ok(results),
        _ => Err("原生干涉检查返回类型错误".into()),
    }
}

#[tauri::command]
async fn inspect_endpoint(
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: EndpointInspectionRequest,
) -> Result<Vec<IkSolution>, String> {
    match engine
        .submit(request_id, Work::InspectEndpoint(request))?
        .await
        .map_err(|_| "端点逆解检查未完成")??
    {
        Reply::InspectEndpoint(solutions) => Ok(solutions),
        _ => Err("原生端点检查返回类型错误".into()),
    }
}

#[tauri::command]
fn cancel_motion(engine: State<'_, Arc<Engine>>, request_id: String) {
    engine.cancel(&request_id);
}

#[tauri::command]
fn copy_rapid(app: tauri::AppHandle, text: String) -> Result<(), String> {
    if text.len() > 1_000_000 {
        return Err("复制内容过长".into());
    }
    app.clipboard()
        .write_text(text)
        .map_err(|_| "无法写入剪贴板，请选中指令手动复制".into())
}

#[tauri::command]
async fn save_program(
    app: tauri::AppHandle,
    window: tauri::WebviewWindow,
    engine: State<'_, Arc<Engine>>,
    request_id: String,
    request: ProgramExportRequest,
) -> Result<Option<String>, String> {
    let reply = engine
        .submit(request_id.clone(), Work::Export(request))?
        .await
        .map_err(|_| "模块生成未完成")??;
    let file = match reply {
        Reply::Export(file) => file,
        _ => return Err("模块生成返回了错误的结果类型".into()),
    };
    if engine.is_cancelled(&request_id) {
        return Err("模块导出已取消".into());
    }
    let engine = engine.inner().clone();
    tauri::async_runtime::spawn_blocking(move || {
        let selected = app
            .dialog()
            .file()
            .set_parent(&window)
            .set_title("保存 ABB RAPID 模块")
            .set_file_name(&file.file_name)
            .add_filter("ABB RAPID 模块", &["mod"])
            .blocking_save_file();
        let Some(selected) = selected else {
            return Ok(None);
        };
        if engine.is_cancelled(&request_id) {
            return Err("参数已变化，模块导出已取消".into());
        }
        let mut path = selected.into_path().map_err(|_| "请选择本机文件路径")?;
        if path.extension().is_none() {
            path.set_extension("mod");
        }
        if !path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("mod"))
        {
            return Err("ABB RAPID 模块必须保存为 .mod 文件".into());
        }
        std::fs::write(&path, file.content.as_bytes())
            .map_err(|_| "无法保存模块，请检查目标目录权限和可用空间")?;
        Ok(Some(path.to_string_lossy().into_owned()))
    })
    .await
    .map_err(|_| "无法完成模块保存")?
}

fn smoke_test() -> Result<(), String> {
    use emulation_core::{
        kinematics::forward_kinematics, model::get_robot_model, planner::plan_move_l,
        program::export_program, types::Target,
    };
    for id in ["irb1100-4-0.58", "irb1100-4-0.475"] {
        let model = get_robot_model(id).ok_or("missing robot model")?;
        let start = forward_kinematics(&model, &model.ready_joints)?;
        let request = PlanRequest {
            model_id: id.into(),
            start_joints: model.ready_joints,
            target: Target {
                position: [start.pose.position[0], 30.0, start.pose.position[2]],
                orientation: Some(start.pose.orientation),
            },
            speed_mm_per_second: 100.0,
        };
        let PlanResult::Success(plan) = plan_move_l(&request) else {
            return Err("native planner rejected smoke target".into());
        };
        if plan.path.len() < 7 || (plan.selected.pose.position[1] - 30.0).abs() > 0.001 {
            return Err("native planner returned an invalid smoke path".into());
        }
        let export_request: ProgramExportRequest = serde_json::from_value(serde_json::json!({ "programName": "EmulationSmoke", "controller": "abb_rapid", "motion": request })).map_err(|e| e.to_string())?;
        let file = export_program(&export_request).map_err(|e| e.to_string())?;
        if !file.content.contains("MODULE EmulationSmoke")
            || !file.content.contains("ENDMODULE")
            || !file.content.contains("MoveL")
            || !file.content.contains("CJointT")
        {
            return Err("native module export smoke failed".into());
        }
    }
    println!(
        "Emulation {} native solver and ABB module smoke passed ({})",
        env!("CARGO_PKG_VERSION"),
        std::env::consts::ARCH
    );
    Ok(())
}

fn main() {
    if std::env::args().any(|arg| arg == "--smoke-test") {
        match smoke_test() {
            Ok(()) => return,
            Err(error) => {
                eprintln!("Native smoke failed: {error}");
                std::process::exit(1);
            }
        }
    }
    let ui_smoke = std::env::args().any(|arg| arg == "--ui-smoke-test");
    if ui_smoke {
        std::thread::spawn(|| {
            std::thread::sleep(std::time::Duration::from_secs(100));
            eprintln!("Native UI smoke timed out");
            std::process::exit(1);
        });
    }
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_clipboard_manager::init())
        .manage(Arc::new(Engine::new()))
        .manage(Arc::new(PendantServer::new()))
        .manage(UiSmoke { enabled: ui_smoke })
        .invoke_handler(tauri::generate_handler![
            start_pendant,
            stop_pendant,
            pendant_status,
            publish_pendant_state,
            pendant_jog,
            smoke_pendant_drive,
            solve_motion,
            open_robot_sources,
            save_robot_source,
            solve_generic_motion,
            solve_generic_pose_motion,
            inspect_generic_endpoint,
            check_generic_collisions,
            plan_grinding_path,
            generate_surface_path,
            check_collisions,
            inspect_endpoint,
            cancel_motion,
            save_program,
            copy_rapid,
            preview_program,
            ui_smoke_report,
            model_cache::list_model_cache,
            model_cache::ensure_model,
            model_cache::cancel_model_download
        ])
        .setup(move |app| {
            app.manage(model_cache::ModelCacheState::new(
                app.path().app_data_dir().map_err(|error| error.to_string()),
            ));
            let mut window =
                WebviewWindowBuilder::new(app, "main", WebviewUrl::App("index.html".into()))
                    // Keep scene-tree HTML5 drops in the WebView; file imports use dialogs.
                    .disable_drag_drop_handler()
                    .title("Emulation · 机器人运动工作台")
                    .inner_size(1440.0, 1000.0)
                    .min_inner_size(1000.0, 720.0)
                    .on_navigation(|url| {
                        matches!(
                            url.host_str(),
                            Some("tauri.localhost") | Some("localhost") | Some("127.0.0.1")
                        )
                    })
                    .on_new_window(|_, _| tauri::webview::NewWindowResponse::Deny);
            if ui_smoke {
                window = window.initialization_script(include_str!("ui-smoke-init.js"));
            }
            window.build()?;
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("Unable to initialize Emulation desktop");
}
