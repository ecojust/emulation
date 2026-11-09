use emulation_core::kinematics::forward_kinematics;
use emulation_core::model::get_robot_model;
use emulation_core::planner::plan_move_l;
use emulation_core::program::{
    controller_capabilities, export_program, export_program_bundle, export_program_with_cancel,
    postprocessor_for, validate_program_name, ControllerTarget, ProgramExportRequest, RobotBrand,
};
use emulation_core::{PlanRequest, PlanResult, Target};
use std::sync::atomic::AtomicBool;

fn request(model_id: &str) -> ProgramExportRequest {
    let model = get_robot_model(model_id).unwrap();
    let initial = forward_kinematics(&model, &model.ready_joints)
        .unwrap()
        .pose;
    ProgramExportRequest {
        program_name: "EmulationMotion".into(),
        controller: ControllerTarget::AbbRapid,
        motion: PlanRequest {
            model_id: model_id.into(),
            start_joints: model.ready_joints,
            target: Target {
                position: [initial.position[0] - 15., 30., initial.position[2] - 10.],
                orientation: Some(initial.orientation),
            },
            speed_mm_per_second: 123.,
        },
    }
}

fn declared_array(content: &str, name: &str) -> serde_json::Value {
    let line = content
        .lines()
        .find(|line| line.contains(&format!(" {name} := ")))
        .unwrap();
    let literal = line
        .split_once(":=")
        .unwrap()
        .1
        .trim()
        .trim_end_matches(';');
    serde_json::from_str(literal).unwrap()
}

#[test]
fn abb_modules_preserve_verified_pose_configuration_speed_and_unwrapped_start() {
    for model_id in ["irb1100-4-0.58", "irb1100-4-0.475"] {
        let mut input = request(model_id);
        input.motion.start_joints[5] = 360.;
        let PlanResult::Success(plan) = plan_move_l(&input.motion) else {
            panic!("fixture must be reachable")
        };
        let module = export_program(&input).unwrap();
        assert_eq!(module.file_name, "EmulationMotion.mod");
        assert_eq!(module.controller, ControllerTarget::AbbRapid);
        assert_eq!(module.model_id, model_id);
        assert!(module.content.starts_with("MODULE EmulationMotion\r\n"));
        assert!(module.content.ends_with("    ENDPROC\r\nENDMODULE\r\n"));
        assert!(module.content.is_ascii());
        assert_eq!(module.content.matches("PROC main()").count(), 1);
        assert_eq!(module.content.matches("        MoveL ").count(), 1);
        let target = declared_array(&module.content, "emulationTarget");
        for axis in 0..3 {
            assert!(
                (target[0][axis].as_f64().unwrap() - plan.selected.pose.position[axis]).abs()
                    <= 0.000501
            );
        }
        for axis in 0..4 {
            assert!(
                (target[1][axis].as_f64().unwrap() - plan.selected.pose.orientation[axis]).abs()
                    <= 0.000000000501
            );
            assert_eq!(
                target[2][axis].as_i64().unwrap(),
                i64::from(plan.selected.configuration[axis])
            );
        }
        assert_eq!(target[3], serde_json::json!([9e9, 9e9, 9e9, 9e9, 9e9, 9e9]));
        assert_eq!(
            declared_array(&module.content, "emulationSpeed"),
            serde_json::json!([123, 500, 5000, 1000])
        );
        assert_eq!(
            declared_array(&module.content, "emulationStart")[0][5]
                .as_f64()
                .unwrap(),
            360.
        );
        assert!(!module.content.contains("v123"));
    }
}

#[test]
fn module_checks_each_starting_axis_before_any_motion_and_never_adds_a_positioning_move() {
    let module = export_program(&request("irb1100-4-0.58")).unwrap();
    let text = module.content;
    let read = text.find("emulationActual := CJointT();").unwrap();
    let conf = text.find("ConfL \\On;").unwrap();
    let movement = text
        .find("MoveL emulationTarget,emulationSpeed,fine,tool0\\WObj:=wobj0;")
        .unwrap();
    assert!(read < conf && conf < movement);
    assert!(!text.contains("MoveAbsJ"));
    assert!(!text.contains("ConfL \\Off"));
    for axis in 1..=6 {
        let condition = format!(
            "IF Abs(emulationActual.robax.rax_{axis}-emulationStart.robax.rax_{axis}) > 0.01 THEN"
        );
        let start = text.find(&condition).unwrap();
        let end = start + text[start..].find("ENDIF").unwrap();
        assert!(start > read && end < movement);
        assert!(text[start..end].contains("RETURN;"));
        assert!(text[start..end].contains("TPWrite"));
    }
}

#[test]
fn exports_replan_and_reject_unreachable_singular_invalid_or_cancelled_requests() {
    let mut input = request("irb1100-4-0.58");
    input.motion.target.position = [2000., 0., 500.];
    assert_eq!(export_program(&input).unwrap_err().code, "plan_rejected");
    input = request("irb1100-4-0.58");
    input.motion.start_joints = [0.; 6];
    assert_eq!(export_program(&input).unwrap_err().code, "plan_rejected");
    input = request("irb1100-4-0.58");
    input.motion.target.orientation = Some([0.; 4]);
    assert_eq!(export_program(&input).unwrap_err().code, "plan_rejected");
    input = request("irb1100-4-0.58");
    input.motion.speed_mm_per_second = f64::NAN;
    assert_eq!(export_program(&input).unwrap_err().code, "plan_rejected");
    input = request("irb1100-4-0.58");
    assert_eq!(
        export_program_with_cancel(&input, &AtomicBool::new(true))
            .unwrap_err()
            .code,
        "cancelled"
    );
}

#[test]
fn endpoint_inverse_solution_alone_does_not_authorize_a_module() {
    let mut input = request("irb1100-4-0.58");
    let model = get_robot_model(&input.motion.model_id).unwrap();
    let endpoint = forward_kinematics(&model, &[180., 20., -30., 0., 40., 0.])
        .unwrap()
        .pose;
    input.motion.target = Target {
        position: endpoint.position,
        orientation: Some(endpoint.orientation),
    };
    assert_eq!(export_program(&input).unwrap_err().code, "plan_rejected");
}

#[test]
fn unsupported_brand_or_model_is_never_lowered_to_renamed_abb_syntax() {
    for controller in [
        ControllerTarget::FanucLs,
        ControllerTarget::KukaKrl,
        ControllerTarget::YaskawaInform,
    ] {
        let mut input = request("irb1100-4-0.58");
        input.controller = controller;
        assert_eq!(
            export_program(&input).unwrap_err().code,
            "unsupported_controller"
        );
        assert!(postprocessor_for(controller).is_err());
    }
    let mut input = request("irb1100-4-0.58");
    input.motion.model_id = "future-non-abb-model".into();
    assert_eq!(
        export_program(&input).unwrap_err().code,
        "unsupported_robot_model"
    );
}

#[test]
fn capabilities_distinguish_adapter_interfaces_from_implemented_export() {
    let capabilities = controller_capabilities();
    assert_eq!(capabilities.len(), 4);
    assert_eq!(
        capabilities
            .iter()
            .filter(|capability| capability.export_supported)
            .count(),
        1
    );
    let abb = capabilities
        .iter()
        .find(|capability| capability.brand == RobotBrand::Abb)
        .unwrap();
    assert_eq!(abb.file_extensions, vec!["mod"]);
    assert_eq!(abb.supported_model_ids.len(), 2);
    for capability in capabilities
        .iter()
        .filter(|capability| !capability.export_supported)
    {
        assert!(capability.supported_model_ids.is_empty());
        assert!(!capability.note.is_empty());
    }
}

#[test]
fn names_cannot_inject_rapid_or_unsafe_platform_filenames() {
    for name in [
        "",
        "1Motion",
        "_Motion",
        "../Motion",
        "A/B",
        "A\\B",
        "A.mod",
        "程序",
        "A\nENDMODULE",
        "MODULE",
        "MAIN",
        "MoVeL",
        "tool0",
        "emulationTarget",
        "CoN",
        "lPt1",
        "Abcdefghijklmnopqrstuvwxyz1234567",
    ] {
        assert!(
            validate_program_name(name).is_err(),
            "accepted invalid name {name:?}"
        );
    }
    for name in [
        "A",
        "EmulationMotion",
        "Robot_Path_02",
        "Abcdefghijklmnopqrstuvwxyz123456",
    ] {
        validate_program_name(name).unwrap();
    }
}

#[test]
fn request_contract_is_strict_and_does_not_accept_a_client_provided_program_body() {
    let input = request("irb1100-4-0.58");
    let mut json = serde_json::to_value(&input).unwrap();
    assert_eq!(json["controller"], "abb_rapid");
    assert_eq!(json["motion"]["modelId"], "irb1100-4-0.58");
    assert_eq!(
        serde_json::from_value::<ProgramExportRequest>(json.clone()).unwrap(),
        input
    );
    json["content"] = serde_json::json!("MODULE Injected\nENDMODULE");
    assert!(serde_json::from_value::<ProgramExportRequest>(json).is_err());
}

#[test]
fn generic_postprocessing_returns_a_bundle_while_abb_single_file_api_remains_compatible() {
    let input = request("irb1100-4-0.58");
    let bundle = export_program_bundle(&input).unwrap();
    assert_eq!(bundle.controller, ControllerTarget::AbbRapid);
    assert_eq!(bundle.model_id, input.motion.model_id);
    assert_eq!(bundle.files.len(), 1);
    assert_eq!(bundle.files[0], export_program(&input).unwrap());
    assert_eq!(bundle.files[0].warnings, bundle.warnings);
    let json = serde_json::to_value(bundle).unwrap();
    assert_eq!(json["files"][0]["fileName"], "EmulationMotion.mod");
    assert_eq!(json["controller"], "abb_rapid");
}
