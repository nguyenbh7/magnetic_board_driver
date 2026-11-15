//! Nov12 research-only frozen-pose magnetic-field residual/Jacobian audit.
 //!
 //! Called ONLY through a dedicated Rust test with explicit local private inputs.
 //! The original-pose scorer uses exact finite-cylinder residual() and does not
 //! invoke any optimizer. A SEPARATE optional board-subset ablation test DOES
 //! fit diagnostic poses, but never replaces any frozen result.
 //! The 6DOF poses, PRE magnet strengths, board offsets, geometry and masks are
 //! loaded from the frozen original experimental outputs, never recalibrated.
 //! Output remains private and no live-app configuration changes.

use super::*;
use serde::Deserialize;
use std::path::Path;

#[derive(Deserialize)]
struct FrozenFrame {
    board_id:u16, frame_id:u32, base_time_us:u64, sensor_mask:u16,
    offsets_us:[u16;16], fields_ut:[[f64;3];16],
}

#[derive(Deserialize)]
struct FrozenMask { board_id:u16, valid_mask:u16 }

#[derive(Deserialize)]
struct FrozenReplay {
    format:String, frames:Vec<FrozenFrame>, endpoint_masks:Vec<FrozenMask>,
    calibration_background_raw_mt:Option<[[[f64;3];16];3]>,
}

#[derive(Deserialize)]
struct FrozenPose {
    t_s:f64, pose_mm_rad:[f64;6], field_rmse_mt:f64, fitted:bool,
}

#[derive(Deserialize)]
struct FrozenTrack {
    source_replay_format:String,
    initial_magnet_centroid_pre_lps_mm:[f64;3],
    poses:Vec<FrozenPose>,
}

fn frozen_gram(
    profile:&RigidProfile, observations:&[Observation], pose:[f64;6],
) -> [[f64;6];6] {
    let base=residual(profile,observations,pose).expect("Finite frozen model residual");
    let mut j=Vec::<Vec<f64>>::new();
    for col in 0..6 {
        let h=if col<3 {0.02} else {0.0002};
        let mut test=pose;
        test[col]=if pose[col]+h<=profile.upper_bounds[col] {
            pose[col]+h
        } else if pose[col]-h>=profile.lower_bounds[col] {
            pose[col]-h
        } else {
            panic!("Frozen dynamic Jacobian coordinate outside model bounds");
        };
        let step=test[col]-pose[col];
        let physical_scale=if col<3 {1.0} else {0.01};
        let new=residual(profile,observations,test)
            .expect("Finite frozen dynamic shifted residual");
        j.push(new.iter().zip(&base).map(|(a,b)|
            (a-b)/step*physical_scale).collect());
    }
    std::array::from_fn(|i|std::array::from_fn(|k|
        j[i].iter().zip(&j[k]).map(|(a,b)|a*b).sum::<f64>()
            /base.len() as f64))
}

/// Gradient of the ORIGINAL total magnetic least-squares objective at one
/// frozen six-DOF pose, partitioned by original receiver. This performs NO fit.
/// J and residual share the exact finite-cylinder model, masking and LPS axes.
/// Each board's contribution is normalized by the SAME total field component
/// count, making the three vectors additive. Parameter scale: [mm, mm, mm,
/// 0.01 rad, 0.01 rad, 0.01 rad]. The quantity is J^T r / N in
/// mT^2 per scaled parameter unit; NOT displacement error or a fitted offset.
fn frozen_board_tangent_pressure(
    profile:&RigidProfile, observations:&[Observation], pose:[f64;6],
)->serde_json::Value {
    let base=residual(profile,observations,pose).expect("Finite frozen residual");
    assert_eq!(base.len(),3*observations.len());
    let mut columns=Vec::<Vec<f64>>::with_capacity(6);
    for k in 0..6 {
        let h=if k<3 {0.02} else {0.0002};
        let mut shifted=pose;
        shifted[k]=if pose[k]+h<=profile.upper_bounds[k] {
            pose[k]+h
        } else if pose[k]-h>=profile.lower_bounds[k] {
            pose[k]-h
        } else {
            panic!("Frozen tangent derivative coordinate outside bounds");
        };
        let step=shifted[k]-pose[k];
        let scaled=if k<3 {1.0} else {0.01};
        let shifted_r=residual(profile,observations,shifted)
            .expect("Finite frozen perturbed residual");
        columns.push(shifted_r.iter().zip(&base).map(|(a,b)|
            (a-b)/step*scaled).collect());
    }
    let mut gradient=[[0.0_f64;6];3];
    let mut components=[0_usize;3];
    for (i,obs) in observations.iter().enumerate() {
        let ids=profile.boards.iter()
            .filter(|b|b.sensors_mm.contains(&obs.sensor_mm))
            .map(|b|b.raw_board_id as usize).collect::<Vec<_>>();
        assert_eq!(ids.len(),1,"Original observation belongs to exactly one board");
        let b=ids[0];
        components[b]+=3;
        for a in 0..3 {
            let j=3*i+a;
            for k in 0..6 {
                gradient[b][k]+=columns[k][j]*base[j];
            }
        }
    }
    assert!(components.iter().all(|n|*n>=24));
    let denom=base.len() as f64;
    let board_json=(0..3).map(|b|serde_json::json!({
        "raw_board_id":b,
        "field_component_count":components[b],
        "Jt_residual_over_GLOBAL_component_count_scaled_6":
            gradient[b].map(|value|value/denom),
    })).collect::<Vec<_>>();
    serde_json::json!({
        "boards":board_json,
        "global_field_component_count":base.len(),
        "parameter_units":["mm","mm","mm","0.01rad","0.01rad","0.01rad"],
        "units":"mT^2 per scaled parameter unit",
        "NO_refit_NO_offset_change":true,
        "preselected_single_snapshot_not_window_statistics":true,
    })
}

fn raw_frame(src:&FrozenFrame)->BoardFrame {
    let mut frame=BoardFrame::default();
    frame.board_id=src.board_id;
    frame.frame_id=src.frame_id;
    frame.base_time_us=src.base_time_us;
    frame.sensor_mask=src.sensor_mask;
    for i in 0..16 {
        frame.samples[i].bx_ut=src.fields_ut[i][0] as f32;
        frame.samples[i].by_ut=src.fields_ut[i][1] as f32;
        frame.samples[i].bz_ut=src.fields_ut[i][2] as f32;
        frame.samples[i].time_offset_us=src.offsets_us[i];
    }
    frame
}

fn scored_frozen(
    profile:&RigidProfile, observations:&[Observation], pose:[f64;6],
) -> serde_json::Value {
    let r=residual(profile,observations,pose).expect("Frozen residual failed");
    assert_eq!(r.len(),3*observations.len());
    let mut squared=[[0.0_f64;3];3];
    let mut signed=[[0.0_f64;3];3];
    let mut counted=[0_usize;3];
    let mut sensor_squared=[[[0.0_f64;3];16];3];
    let mut sensor_n=[[0_usize;16];3];
    for (i,obs) in observations.iter().enumerate() {
        let mut match_site=None;
        for board in &profile.boards {
            for sensor in 0..16 {
                if board.sensors_mm[sensor]==obs.sensor_mm {
                    assert!(match_site.is_none(),
                        "Observed sensor coordinates ambiguous between boards");
                    match_site=Some((board.raw_board_id as usize,sensor));
                }
            }
        }
        let (board,sensor)=match_site.expect("Observed sensor missing from frozen CT profile");
        counted[board]+=1;
        sensor_n[board][sensor]+=1;
        for axis in 0..3 {
            let value=r[3*i+axis];
            squared[board][axis]+=value*value;
            signed[board][axis]+=value;
            sensor_squared[board][sensor][axis]+=value*value;
        }
    }
    let mut boards=Vec::new();
    for b in 0..3 {
        assert!(counted[b]>=8,"Insufficient observations on a frozen receiver");
        let count=counted[b] as f64;
        boards.push(serde_json::json!({
            "board_id":b,
            "sensor_count":counted[b],
            "rmse_mt":(squared[b].iter().sum::<f64>()/(3.0*count)).sqrt(),
            "LPS_axis_rmse_mt":std::array::from_fn::<_,3,_>(|a|
                (squared[b][a]/count).sqrt()),
            "LPS_axis_model_minus_observed_mean_mt":std::array::from_fn::<_,3,_>(|a|
                signed[b][a]/count),
            "per_firmware_sensor_rmse_mt":(0..16).map(|s| {
                if sensor_n[b][s]>0 {
                    Some((sensor_squared[b][s].iter().sum::<f64>()
                        /(3*sensor_n[b][s]) as f64).sqrt())
                } else {None}
            }).collect::<Vec<_>>(),
        }));
    }
    serde_json::json!({
        "global_rmse_mt":(r.iter().map(|v|v*v).sum::<f64>()
            /r.len() as f64).sqrt(),
        "observation_count":observations.len(),
        "boards":boards,
    })
}

#[test]
fn experimental_gk_nov12_frozen_pose_board_audit() {
    let Ok(replay_path)=std::env::var("GK_NOV12_FROZEN_REPLAY_IN") else {
        // Normal cargo tests remain data-independent and side-effect free.
        return;
    };
    let pose_path=std::env::var("GK_NOV12_FROZEN_POSES_IN")
        .expect("Set frozen poses path");
    let summary_path=std::env::var("GK_NOV12_FROZEN_SUMMARY_IN")
        .expect("Set frozen PRE calibration/endpoint summary path");
    let ct_path=std::env::var("GK_NOV12_FROZEN_PRE_CT")
        .expect("Set original PRE CT detector JSON");
    let out=std::env::var("GK_NOV12_FROZEN_OUT")
        .expect("Set new PRIVATE diagnostic output path");
    assert!(!Path::new(&out).exists(),"Refuse overwrite of frozen audit output");
    let replay:FrozenReplay=serde_json::from_str(
        &std::fs::read_to_string(&replay_path).expect("Read original private replay"))
        .expect("Parse original private replay");
    let frozen:FrozenTrack=serde_json::from_str(
        &std::fs::read_to_string(&pose_path).expect("Read original saved poses"))
        .expect("Parse frozen poses");
    let summary:serde_json::Value=serde_json::from_str(
        &std::fs::read_to_string(&summary_path).expect("Read original PRE calibration"))
        .expect("Parse PRE calibration summary");
    assert_eq!(replay.format,"gk_nov06_private_motion_replay_v1");
    assert_eq!(frozen.source_replay_format,replay.format);
    assert!(replay.calibration_background_raw_mt.is_none(),
            "Expected original no-saved-background research arm");
    assert_eq!(summary["stationary_calibration_QC_pass"],true);
    assert_eq!(summary["fit_rejected"],0);
    assert_eq!(summary["full_motion_accepted_poses"].as_u64().unwrap() as usize,
               frozen.poses.len());
    assert_eq!(summary["PRE_magnetic_axis_variant"],"original_CT_unsigned_axes");
    assert_eq!(replay.endpoint_masks.len(),3);
    assert!(frozen.poses.len()>100);
    let geom=crate::cbct_geometry::CbctGeometry::from_detector_json(
        &std::fs::read_to_string(&ct_path).expect("Read original private PRE CT"),
        "Nov12 PRE CT frozen audit".into(),"research only".into()
    ).expect("Original PRE CT geometry");
    let mut profile=geom.experimental_candidate_profile_with_reviewed_cad()
        .expect("Original frozen CT-to-firmware geometry unavailable");
    let strengths:[f64;3]=serde_json::from_value(
        summary["signed_remanences_mt"].clone()).expect("Frozen remanences");
    let offsets:[[f64;3];3]=serde_json::from_value(
        summary["pre_fitted_constant_board_offsets_lps_mt"].clone())
        .expect("Frozen PRE board offsets");
    assert!(profile.origin_mm.iter().zip(frozen.initial_magnet_centroid_pre_lps_mm)
        .all(|(a,b)|(*a-b).abs()<1e-6),
        "PRE origin differs from frozen motion trajectory");
    for (magnet,value) in profile.magnets.iter_mut().zip(strengths) {
        magnet.remanence_mt=value;
    }
    for board in &mut profile.boards {
        board.background_mt=offsets[board.raw_board_id as usize];
    }
    let mut masks=[0_u16;3];
    for mask in &replay.endpoint_masks {
        assert!((mask.board_id as usize)<3);
        masks[mask.board_id as usize]=mask.valid_mask;
    }
    assert!(masks.iter().all(|m|m.count_ones()>=8));
    let mut live=RigidLiveFit::default();
    live.set_experimental_gk_profile(profile.clone(),None);
    live.set_presence(BoardPresence {board_mask:0b111,sensor_masks:masks});

    let requests:[f64;5]=if pose_path.contains("B_to_C"){
        [2.,55.,62.,66.,85.]
    }else{[2.,10.,65.,95.,105.]};
    let mut samples=vec![None::<(f64,[f64;6],Vec<Observation>)>; requests.len()];
    let mut rows=Vec::new();
    let mut next=0_usize;
    for source in &replay.frames {
        let frame=raw_frame(source);
        let Some(job)=live.receive_frame(&frame,source.frame_id%10==0) else {
            continue;
        };
        // This test evaluates the ORIGINAL saved 6DOF pose and NEVER calls
        // job.run(), optimize_pose(), or magnetic calibration.
        live.busy=false;
        assert!(next<frozen.poses.len(),"Replay yielded extra fit-attempt frame");
        let pose=&frozen.poses[next];
        let time=job.time_us as f64/1e6;
        assert!(pose.fitted,"Frozen file includes unaccepted trajectory");
        assert!((time-pose.t_s).abs()<1e-5,
                "Replay fit sampling/timestamps changed since frozen result");
        let score=scored_frozen(&profile,&job.observations,pose.pose_mm_rad);
        let computed=score["global_rmse_mt"].as_f64().unwrap();
        assert!((computed-pose.field_rmse_mt).abs()<2e-5,
                "Exact Rust forward residual no longer reproduces original accepted pose RMSE");
        for (i,requested) in requests.iter().enumerate() {
            if (time-requested).abs()<0.2
                && samples[i].as_ref().is_none_or(|old|
                    (old.0-requested).abs()>(time-requested).abs()) {
                samples[i]=Some((time,pose.pose_mm_rad,job.observations.clone()));
            }
        }
        rows.push(serde_json::json!({
            "t_s":time,
            "saved_global_rmse_mt":pose.field_rmse_mt,
            "reconstructed_global_rmse_mt":computed,
            "observation_count":score["observation_count"],
            "boards":score["boards"],
        }));
        next+=1;
    }
    assert_eq!(next,frozen.poses.len(),
               "Replay failed to reconstruct every accepted original pose");
    let local_jacobians=requests.iter().zip(samples.into_iter()).map(|(req,opt)| {
        let (time,pose,obs)=opt.expect("Missing predeclared local Jacobian sample");
        serde_json::json!({
            "requested_time_s":req,
            "actual_time_s":time,
            "observation_count":obs.len(),
            "JtJ_over_field_component_count_scaled_6x6":
                frozen_gram(&profile,&obs,pose),
            "per_board_frozen_pose_tangent_pressure":
                frozen_board_tangent_pressure(&profile,&obs,pose),
            "parameter_order":["dL_mm","dP_mm","dS_mm",
                "rx_scaled_0p01rad","ry_scaled_0p01rad","rz_scaled_0p01rad"],
            "rotation_scale_rad_per_unit":0.01,
            "local_tangent_only_not_global_identifiability":true,
        })
    }).collect::<Vec<_>>();
    let report=serde_json::json!({
        "format":"gk_nov12_frozen_dynamic_field_residual_audit_v1",
        "research_only":true,
        "no_refit_no_recalibration_no_CT_POST_or_HDMM":true,
        "exact_frozen_forward_global_rmse_parity":true,
        "candidate_gain7_firmware_settings_unverified":true,
        "ct_geometry_approved":false,
        "sample_count":rows.len(),
        "source_replay_private":replay_path,
        "source_saved_pose_private":pose_path,
        "source_frozen_summary_private":summary_path,
        "frames":rows,
        "local_frozen_Jacobian":local_jacobians,
    });
    if let Some(parent)=Path::new(&out).parent() {
        std::fs::create_dir_all(parent).expect("Create private audit output directory");
    }
    std::fs::write(&out,serde_json::to_string_pretty(&report).unwrap()+"\n")
        .expect("Write new private frozen-pose diagnostic");
    println!("FROZEN Nov12 board audit: {} original poses; no refit -> {}",
             next,out);
}


#[test]
fn frozen_board_scorer_reproduces_global_field_cost_on_synthetic_measurements() {
    // This test intentionally loads NO Nov12 patient or acquisition data.
    let profile=RigidProfile::a1_reference().expect("Bundled synthetic/reference profile");
    let mut obs=Vec::new();
    for board in &profile.boards {
        for sensor in &board.sensors_mm {
            obs.push(Observation {sensor_mm:*sensor,measured_mt:[0.0;3]});
        }
    }
    let pose=profile.initial_pose;
    let exact=residual(&profile,&obs,pose).expect("Reference model residual");
    let expected=(exact.iter().map(|v|v*v).sum::<f64>()
        /exact.len() as f64).sqrt();
    let scored=scored_frozen(&profile,&obs,pose);
    assert!((scored["global_rmse_mt"].as_f64().unwrap()-expected).abs()<1e-12);
    let b=scored["boards"].as_array().unwrap();
    assert_eq!(b.len(),3);
    assert!(b.iter().all(|x|x["sensor_count"].as_u64()==Some(16)));
    let pressure=frozen_board_tangent_pressure(&profile,&obs,pose);
    assert_eq!(pressure["NO_refit_NO_offset_change"],true);
    assert_eq!(pressure["global_field_component_count"].as_u64(),Some(144));
    let board_grad=pressure["boards"].as_array().unwrap();
    assert_eq!(board_grad.len(),3);
    assert!(board_grad.iter().all(|x|x["field_component_count"].as_u64()==Some(48)));
    for item in board_grad {
        let g=item["Jt_residual_over_GLOBAL_component_count_scaled_6"]
            .as_array().unwrap();
        assert_eq!(g.len(),6);
        assert!(g.iter().all(|x|x.as_f64().unwrap().is_finite()));
    }
    let gram=frozen_gram(&profile,&obs,pose);
    for i in 0..6 {
        assert!(gram[i][i].is_finite() && gram[i][i]>=0.0);
        for j in 0..6 {
            assert!((gram[i][j]-gram[j][i]).abs()<1e-10);
        }
    }
}


/// Independent diagnostic ONLY: re-optimize selected ORIGINAL measurement
/// snapshots on predeclared board subsets. No calibration, CT truth or HDMM
/// enters an objective; no original trajectory is overwritten or modified.
/// Unlike experimental_gk_nov12_frozen_pose_board_audit, this TEST DOES
/// perform new subset pose fits and must not be called a frozen-pose scorer.
#[test]
fn experimental_gk_nov12_predeclared_board_subset_ablation() {
    let Ok(replay_path)=std::env::var("GK_NOV12_FROZEN_REPLAY_IN") else {
        return; // normal cargo test requires no private acquisition data
    };
    let pose_path=std::env::var("GK_NOV12_FROZEN_POSES_IN")
        .expect("Frozen subset audit needs original poses");
    let summary_path=std::env::var("GK_NOV12_FROZEN_SUMMARY_IN")
        .expect("Frozen subset audit needs original summary");
    let ct_path=std::env::var("GK_NOV12_FROZEN_PRE_CT")
        .expect("Frozen subset audit needs original PRE CT");
    let dest=std::env::var("GK_NOV12_FROZEN_ABLATION_OUT")
        .expect("Set NEW private receiver ablation output file");
    assert!(!Path::new(&dest).exists(),"Refuse overwriting private ablation output");

    let replay:FrozenReplay=serde_json::from_str(
        &std::fs::read_to_string(&replay_path).expect("Read original replay"))
        .expect("Parse original replay");
    let frozen:FrozenTrack=serde_json::from_str(
        &std::fs::read_to_string(&pose_path).expect("Read original full-motion poses"))
        .expect("Parse original full-motion poses");
    let summary:serde_json::Value=serde_json::from_str(
        &std::fs::read_to_string(&summary_path).expect("Read original summary"))
        .expect("Parse original summary");
    assert_eq!(replay.format,"gk_nov06_private_motion_replay_v1");
    assert_eq!(frozen.source_replay_format,replay.format);
    assert!(replay.calibration_background_raw_mt.is_none());
    assert_eq!(summary["stationary_calibration_QC_pass"],true);
    assert_eq!(summary["fit_rejected"],0);
    assert_eq!(summary["PRE_magnetic_axis_variant"],"original_CT_unsigned_axes");
    assert_eq!(summary["full_motion_accepted_poses"].as_u64().unwrap() as usize,
        frozen.poses.len());
    let geom=crate::cbct_geometry::CbctGeometry::from_detector_json(
        &std::fs::read_to_string(&ct_path).expect("Read original PRE CT detector"),
        "Nov12 subset audit PRE CT".into(),"research only".into()
    ).expect("Parse original PRE CT geometry");
    let mut profile=geom.experimental_candidate_profile_with_reviewed_cad()
        .expect("No original CT-firmware model for subset audit");
    assert!(profile.origin_mm.iter()
        .zip(frozen.initial_magnet_centroid_pre_lps_mm)
        .all(|(a,b)|(*a-b).abs()<1e-6));
    let strengths:[f64;3]=serde_json::from_value(
        summary["signed_remanences_mt"].clone()).expect("Frozen magnet strengths");
    let offsets:[[f64;3];3]=serde_json::from_value(
        summary["pre_fitted_constant_board_offsets_lps_mt"].clone())
        .expect("Frozen per-board offsets");
    for (magnet,value) in profile.magnets.iter_mut().zip(strengths) {
        magnet.remanence_mt=value;
    }
    for board in &mut profile.boards {
        board.background_mt=offsets[board.raw_board_id as usize];
    }
    let mut masks=[0_u16;3];
    assert_eq!(replay.endpoint_masks.len(),3);
    for mask in &replay.endpoint_masks {
        assert!((mask.board_id as usize)<3);
        masks[mask.board_id as usize]=mask.valid_mask;
    }
    assert!(masks.iter().all(|m|m.count_ones()>=8));

    let mut live=RigidLiveFit::default();
    live.set_experimental_gk_profile(profile.clone(),None);
    live.set_presence(BoardPresence {board_mask:0b111,sensor_masks:masks});

    // Same fixed times for all subjects in each interval; never selected
    // for a better HDMM fit or smaller boardwise residual.
    let requests:Vec<f64>=if pose_path.contains("B_to_C"){
        vec![2.0,55.0,62.0,66.0,85.0]
    }else{vec![2.0,10.0,65.0,95.0,104.9]};
    let mut samples=vec![None::<(f64,[f64;6],Vec<Observation>)>;requests.len()];
    let mut next=0_usize;
    for source in &replay.frames {
        let frame=raw_frame(source);
        let Some(job)=live.receive_frame(&frame,source.frame_id%10==0) else {
            continue;
        };
        live.busy=false; // retain ORIGINAL measured snapshots; never fit here
        assert!(next<frozen.poses.len(),"More replay attempts than saved poses");
        let saved=&frozen.poses[next];
        let time=job.time_us as f64/1e6;
        assert!(saved.fitted && (saved.t_s-time).abs()<1e-5,
                "Original replay timestamps/acceptance differ");
        for (i,target) in requests.iter().enumerate(){
            if (time-target).abs()<0.2 && samples[i].as_ref().is_none_or(|old|
                (old.0-target).abs()>(time-target).abs()){
                samples[i]=Some((time,saved.pose_mm_rad,job.observations.clone()));
            }
        }
        next+=1;
    }
    assert_eq!(next,frozen.poses.len(),"Original replay had unexpected gaps");

    fn select_receiver_observations(
        profile:&RigidProfile,all:&[Observation], selected:&[usize],
    )->Vec<Observation>{
        let sites=profile.boards.iter()
            .filter(|b|selected.contains(&(b.raw_board_id as usize)))
            .flat_map(|b|b.sensors_mm.iter().copied())
            .collect::<Vec<_>>();
        all.iter().filter(|o|sites.contains(&o.sensor_mm))
            .cloned().collect::<Vec<_>>()
    }
    fn pose_delta(
        profile:&RigidProfile, reference:[f64;6], candidate:[f64;6],
        point:[f64;3],
    )->(f64,f64,f64) {
        let t=sub(
            [candidate[0],candidate[1],candidate[2]],
            [reference[0],reference[1],reference[2]]);
        let rot=mul_matrix(
            rodrigues([candidate[3],candidate[4],candidate[5]]),
            transpose(rodrigues([reference[3],reference[4],reference[5]])));
        let angle=((rot[0][0]+rot[1][1]+rot[2][2]-1.0)/2.0)
            .clamp(-1.,1.).acos()*RAD_TO_DEG;
        let p0=rigid_point_at_pose(profile,reference,point);
        let p1=rigid_point_at_pose(profile,candidate,point);
        let dp=sub(p1,p0);
        (dot(t,t).sqrt(),angle,dot(dp,dp).sqrt())
    }
    let marker=frozen.initial_magnet_centroid_pre_lps_mm; // Only magnet centroid; NOT optical HDMM.
    let variants:[(&str,&[usize]);4]=[
        ("all_0_1_2",&[0,1,2]),
        ("side_1_2",&[1,2]),
        ("superior_left_0_1",&[0,1]),
        ("superior_right_0_2",&[0,2]),
    ];
    let mut output=Vec::<serde_json::Value>::new();
    for (requested,sample) in requests.iter().zip(samples.into_iter()) {
        let (time,reference,obs)=sample
            .expect("Predeclared original Hall measurement snapshot unavailable");
        let scored=scored_frozen(&profile,&obs,reference);
        let saved_rmse=frozen.poses.iter().find(|p|
            (p.t_s-time).abs()<1e-5).unwrap().field_rmse_mt;
        assert!((scored["global_rmse_mt"].as_f64().unwrap()-saved_rmse).abs()<2e-5,
            "Frozen full-three-board forward score does not reproduce original fit");
        let original_all=score_per_receiver(&profile,&obs,reference);
        let mut arms=Vec::new();
        for &(name,ids) in &variants {
            let train=select_receiver_observations(&profile,&obs,ids);
            assert!(train.len()>=8*ids.len(),"Original receiver subset unexpectedly incomplete");
            let mut attempts=Vec::new();
            for (label,seed) in [
                ("original_frozen_pose",reference),
                ("original_profile_initial_pose",profile.initial_pose)
            ] {
                let answer=optimize_pose(&profile,&train,seed);
                let row=match answer {
                    Some(fit)=>{
                        let (centroid_sep,angle,projected_sep)=
                            pose_delta(&profile,reference,fit.pose,marker);
                        serde_json::json!({
                            "seed":label,
                            "returned":true,
                            "original_gate_pass_on_TRAIN":
                                fit.rmse_mt<=profile.max_rmse_mt,
                            "train_rmse_mt":fit.rmse_mt,
                            "all_board_rmse_mt":(
                                residual(&profile,&obs,fit.pose)
                                    .expect("Full-board score of subset fit")
                                    .iter().map(|x|x*x).sum::<f64>()
                                    /(3*obs.len()) as f64).sqrt(),
                            "all_board_receiver_RMSE_mt":
                                score_per_receiver(&profile,&obs,fit.pose),
                            "centroid_pose_difference_vs_original_mm":centroid_sep,
                            "rotation_difference_vs_original_deg":angle,
                            "centroid_point_difference_vs_original_mm":projected_sep,
                            "pose_mm_rad":fit.pose,
                        })
                    }
                    None=>serde_json::json!({
                        "seed":label,"returned":false,
                    }),
                };
                attempts.push(row);
            }
            arms.push(serde_json::json!({
                "subset":name,
                "raw_board_ids":ids,
                "original_measurement_count":train.len(),
                "attempts":attempts,
            }));
        }
        output.push(serde_json::json!({
            "requested_time_s":requested,
            "actual_time_s":time,
            "original_saved_pose_mm_rad":reference,
            "original_saved_global_rmse_mt":saved_rmse,
            "original_all_board_receiver_RMSE_mt":original_all,
            "ablation_arms":arms,
        }));
    }
    let result=serde_json::json!({
        "format":"gk_nov12_test_only_fixed_calibration_receiver_subset_pose_ablation_v1",
        "research_only":true,
        "new_diagnostic_pose_fits_performed":true,
        "original_frozen_trajectory_modified":false,
        "PRE_frozen_strengths_offsets_CT_geometry_masks":true,
        "CT_POST_and_HDMM_not_used_in_objective":true,
        "original_hall_three_board_fit_not_replaced":true,
        "comparison_point_kind":"magnet_centroid_NOT_optical_sticker",
        "original_frozen_pose_path_PRIVATE":pose_path,
        "requested_times_prespecified_s":requests,
        "snapshots":output,
        "finite_seeds_and_subsets_cannot_prove_true_pose_or_global_uniqueness":true,
    });
    if let Some(parent)=Path::new(&dest).parent(){
        std::fs::create_dir_all(parent).expect("Create separate private subset report");
    }
    std::fs::write(&dest,serde_json::to_string_pretty(&result).unwrap()+"\n")
        .expect("Write private subset ablation without modifying source data");
    println!("Nov12 subset ablations: {} fixed snapshots -> {}",
             result["snapshots"].as_array().unwrap().len(),dest);
}

/// Independent per-board RMSE at an arbitrary candidate pose, on ALL original
/// receiver observations, including those held out from subset optimization.
fn score_per_receiver(
    profile:&RigidProfile,obs:&[Observation],pose:[f64;6],
)->[f64;3]{
    let r=residual(profile,obs,pose).expect("Subset score forward model");
    let mut sq=[0.0_f64;3];
    let mut n=[0_usize;3];
    for (i,o) in obs.iter().enumerate() {
        let b=profile.boards.iter().find(|b|
            b.sensors_mm.contains(&o.sensor_mm))
            .expect("A scored observation has original receiver")
            .raw_board_id as usize;
        n[b]+=3;
        for k in 0..3 {sq[b]+=r[3*i+k]*r[3*i+k];}
    }
    std::array::from_fn(|b|{
        if n[b]>0 {(sq[b]/n[b] as f64).sqrt()} else {f64::NAN}
    })
}

#[test]
fn subset_receiver_score_matches_original_all_board_score_synthetic(){
    let profile=RigidProfile::a1_reference().expect("Reference profile");
    let obs=profile.boards.iter().flat_map(|b|b.sensors_mm.iter().map(|p|
        Observation{sensor_mm:*p,measured_mt:[0.0;3]})).collect::<Vec<_>>();
    let board=score_per_receiver(&profile,&obs,profile.initial_pose);
    let global=residual(&profile,&obs,profile.initial_pose).unwrap();
    let by_board=board.iter().map(|v|v*v).sum::<f64>()/3.0;
    let direct=global.iter().map(|v|v*v).sum::<f64>()/global.len() as f64;
    assert!((by_board-direct).abs()<1e-10);
    assert!(board.iter().all(|v|v.is_finite()));
}
