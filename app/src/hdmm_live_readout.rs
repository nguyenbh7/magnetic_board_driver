//! Prominent Hall-predicted HDMM sticker readout, with explicit freshness/QC.
//! This is never a measurement from the optical HDMM device.

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct HdmmLiveReadout {
    pub value: String,
    pub caption: &'static str,
    pub invalid: bool,
    pub held: bool,
}

/// Never let an old accepted pose appear to be a fresh measurement after a
/// rejected fit, stopped stream, or latched raw Hall input fault.
pub(crate) fn hdmm_live_readout(
    point_ready: bool,
    zero_ready: bool,
    last_accepted_mm: Option<f64>,
    input_invalid: bool,
    receiving: bool,
    replay_loaded: bool,
    fit_status: &str,
) -> HdmmLiveReadout {
    if input_invalid {
        return HdmmLiveReadout {
            value: "INVALID".into(),
            caption: "Hall input QC failed. No current HDMM-point prediction; old plot is historical.",
            invalid: true,
            held: true,
        };
    }
    if !point_ready {
        return HdmmLiveReadout {
            value: "— mm".into(),
            caption: "Load a reviewed sticker from this CT, inspect its position, then set the HDMM point.",
            invalid: false,
            held: true,
        };
    }
    if !zero_ready {
        return HdmmLiveReadout {
            value: "— mm".into(),
            caption: "Sticker selected; wait for an accepted Hall fit and set the common stationary zero.",
            invalid: false,
            held: true,
        };
    }
    let Some(mm) = last_accepted_mm.filter(|n| n.is_finite() && *n >= 0.0) else {
        return HdmmLiveReadout {
            value: "— mm".into(),
            caption: "Awaiting a new accepted Hall rigid-body pose at the reviewed sticker location.",
            invalid: false,
            held: true,
        };
    };
    let current = (receiving || replay_loaded) && fit_status == "Tracking";
    HdmmLiveReadout {
        value: format!("{mm:.3} mm"),
        caption: if !current {
            "HOLD — last accepted Hall prediction, NOT current tracking. Check stream and fit status."
        } else if replay_loaded {
            "REPLAY — Hall-derived sticker prediction from recorded data, NOT optical HDMM."
        } else {
            "LIVE Hall-predicted sticker displacement, NOT optical HDMM. Compare at the same zero."
        },
        invalid: false,
        held: !current,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn live_prediction_is_numeric_and_clearly_not_optically_measured() {
        let r = hdmm_live_readout(true,true,Some(0.85142),false,true,false,"Tracking");
        assert_eq!(r.value, "0.851 mm");
        assert!(r.caption.contains("LIVE Hall-predicted"));
        assert!(r.caption.contains("NOT optical HDMM"));
        assert!(!r.invalid && !r.held);
    }

    #[test]
    fn missing_reviewed_point_and_zero_never_show_old_magnitude() {
        let none = hdmm_live_readout(false,true,Some(4.2),false,true,false,"Tracking");
        assert_eq!(none.value,"— mm");
        let nozero = hdmm_live_readout(true,false,Some(4.2),false,true,false,"Tracking");
        assert_eq!(nozero.value,"— mm");
        assert!(nozero.caption.contains("zero"));
    }

    #[test]
    fn rejected_fit_and_stopped_stream_mark_old_number_held() {
        for (receiving,status) in [(true,"Fit rejected: 0.5 mT"),(false,"Field stream stopped")] {
            let r=hdmm_live_readout(true,true,Some(2.1),false,receiving,false,status);
            assert_eq!(r.value,"2.100 mm");
            assert!(r.held && r.caption.starts_with("HOLD"));
        }
    }

    #[test]
    fn latched_sensor_fault_hides_stale_prediction() {
        let r=hdmm_live_readout(true,true,Some(5.25),true,true,false,"Tracking");
        assert_eq!(r.value,"INVALID");
        assert!(r.invalid && r.held);
        assert!(!r.caption.contains("LIVE"));
    }

    #[test]
    fn replay_is_labeled_and_invalid_numbers_are_never_shown() {
        let replay=hdmm_live_readout(true,true,Some(1.25),false,false,true,"Tracking");
        assert!(replay.caption.starts_with("REPLAY"));
        assert!(!replay.held);
        for mm in [f64::NAN,f64::INFINITY,-1.0] {
            let r=hdmm_live_readout(true,true,Some(mm),false,true,false,"Tracking");
            assert_eq!(r.value,"— mm");
        }
    }
}
