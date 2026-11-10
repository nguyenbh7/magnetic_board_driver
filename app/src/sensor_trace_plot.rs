use iced::widget::canvas;
use iced::{mouse, Color, Element, Length, Pixels, Point, Rectangle, Renderer, Theme};

use crate::sensor_trace::SensorTracePoint;

const PLOT_HEIGHT: f32 = 220.0;

/// Axis-specific ADVISORY limits, in mT. These are NOT independently verified
/// analog saturation thresholds or per-chip register readbacks. The caller
/// derives them from the same nominal MLX range model as the range banner.
#[derive(Debug, Clone, Copy)]
pub struct TraceRangeGuides {
    pub advisory_mt: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct SensorTracePlot {
    board_id: u16,
    sensor_index: Option<u8>,
    points: Vec<SensorTracePoint>,
    limits: TraceRangeGuides,
}

pub fn sensor_trace_plot<Message: 'static>(
    board_id: u16,
    sensor_index: Option<u8>,
    points: Vec<SensorTracePoint>,
    limits: TraceRangeGuides,
) -> Element<'static, Message> {
    canvas(SensorTracePlot {
        board_id,
        sensor_index,
        points,
        limits,
    })
    .width(Length::Fill)
    .height(Length::Fixed(PLOT_HEIGHT))
    .into()
}

impl<Message> canvas::Program<Message> for SensorTracePlot {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let mut frame = canvas::Frame::new(renderer, bounds.size());

        let width = bounds.width;
        let height = bounds.height;

        if width < 180.0 || height < 120.0 {
            return vec![frame.into_geometry()];
        }

        let plot_left = 90.0;
        let plot_right = width - 20.0;
        let plot_top = 22.0;
        let plot_bottom = height - 42.0;

        let plot_width = (plot_right - plot_left).max(1.0);
        let plot_height = (plot_bottom - plot_top).max(1.0);

        let axis_color = Color::from_rgb8(120, 120, 120);
        let grid_color = Color::from_rgb8(220, 220, 220);
        let label_color = Color::from_rgb8(80, 80, 80);

        let bx_color = Color::from_rgb8(180, 60, 60);
        let by_color = Color::from_rgb8(60, 150, 80);
        let bz_color = Color::from_rgb8(55, 95, 180);

        draw_label(
            &mut frame,
            "Magnetic field (mT)",
            Point::new(plot_left, 3.0),
            11.0,
            label_color,
        );

        draw_label(
            &mut frame,
            "Time (s)",
            Point::new(plot_left + 0.5 * plot_width - 20.0, height - 18.0),
            11.0,
            label_color,
        );

        if self.points.len() < 2 {
            let sensor_text = self
                .sensor_index
                .map(|sensor| format!("sensor {}", sensor))
                .unwrap_or_else(|| "no sensor selected".to_string());

            draw_label(
                &mut frame,
                format!(
                    "Board {} {}: waiting for trace history",
                    self.board_id, sensor_text
                ),
                Point::new(plot_left + 12.0, plot_top + 24.0),
                12.0,
                label_color,
            );

            return vec![frame.into_geometry()];
        }

        let x_min = self.points.first().map(|p| p.time_s).unwrap_or(0.0);
        let mut x_max = self.points.last().map(|p| p.time_s).unwrap_or(1.0);

        if x_max <= x_min {
            x_max = x_min + 1.0;
        }

        // Keep the threshold guides inside the visible axis rather than
        // silently off-screen when a low-amplitude trace is displayed.
        let (mut y_min, mut y_max) =
            field_bounds_with_guides_mt(&self.points, self.limits.advisory_mt);

        if y_min > 0.0 {
            y_min = 0.0;
        }

        if y_max < 0.0 {
            y_max = 0.0;
        }

        let y_range = (y_max - y_min).abs().max(0.001);
        y_min -= 0.10 * y_range;
        y_max += 0.10 * y_range;

        let map_x = |time_s: f64| {
            plot_left + (((time_s - x_min) / (x_max - x_min)) as f32 * plot_width)
        };

        let map_y = |field_mt: f64| {
            plot_bottom - (((field_mt - y_min) / (y_max - y_min)) as f32 * plot_height)
        };

        for i in 0..=4 {
            let frac = i as f32 / 4.0;

            let x = plot_left + frac * plot_width;
            let t = x_min + (x_max - x_min) * frac as f64;

            frame.stroke(
                &canvas::Path::line(Point::new(x, plot_top), Point::new(x, plot_bottom)),
                canvas::Stroke::default()
                    .with_color(grid_color)
                    .with_width(0.5),
            );

            draw_label(
                &mut frame,
                format!("{:.1}", t),
                Point::new(x - 12.0, plot_bottom + 7.0),
                10.0,
                label_color,
            );

            let y = plot_bottom - frac * plot_height;
            let field_mt = y_min + (y_max - y_min) * frac as f64;

            frame.stroke(
                &canvas::Path::line(Point::new(plot_left, y), Point::new(plot_right, y)),
                canvas::Stroke::default()
                    .with_color(grid_color)
                    .with_width(0.5),
            );

            draw_label(
                &mut frame,
                format!("{:.3}", field_mt),
                Point::new(plot_left - 66.0, y - 7.0),
                10.0,
                label_color,
            );
        }

        let zero_y = map_y(0.0);
        if zero_y >= plot_top && zero_y <= plot_bottom {
            frame.stroke(
                &canvas::Path::line(
                    Point::new(plot_left, zero_y),
                    Point::new(plot_right, zero_y),
                ),
                canvas::Stroke::default()
                    .with_color(axis_color)
                    .with_width(0.8),
            );
        }

        frame.stroke(
            &canvas::Path::line(
                Point::new(plot_left, plot_bottom),
                Point::new(plot_right, plot_bottom),
            ),
            canvas::Stroke::default()
                .with_color(axis_color)
                .with_width(1.0),
        );

        frame.stroke(
            &canvas::Path::line(
                Point::new(plot_left, plot_top),
                Point::new(plot_left, plot_bottom),
            ),
            canvas::Stroke::default()
                .with_color(axis_color)
                .with_width(1.0),
        );

        // A dotted + and - guide for each component's advisory boundary.
        // Bx and By usually have IDENTICAL limits: draw the common guide
        // once to avoid hiding one color on top of the other.
        let axis_limits = self.limits.advisory_mt;
        for (axis, color) in [(0, bx_color), (1, by_color), (2, bz_color)] {
            let limit = axis_limits[axis];
            if !limit.is_finite() || limit <= 0.0 { continue; }
            if axis == 1 && (limit - axis_limits[0]).abs() < 1.0e-6 {
                continue;
            }
            for signed_limit in [-limit, limit] {
                let y = map_y(signed_limit);
                if y >= plot_top && y <= plot_bottom {
                    draw_dotted_guide(
                        &mut frame, plot_left, plot_right, y, color,
                    );
                }
            }
        }

        draw_series(&mut frame, &self.points, |p| p.bx_mt, map_x, map_y, bx_color);
        draw_series(&mut frame, &self.points, |p| p.by_mt, map_x, map_y, by_color);
        draw_series(&mut frame, &self.points, |p| p.bz_mt, map_x, map_y, bz_color);

        draw_legend(&mut frame, plot_right - 130.0, plot_top + 4.0, bx_color, "Bx");
        draw_legend(&mut frame, plot_right - 86.0, plot_top + 4.0, by_color, "By");
        draw_legend(&mut frame, plot_right - 42.0, plot_top + 4.0, bz_color, "Bz");

        if let Some(last) = self.points.last() {
            draw_label(
                &mut frame,
                format!(
                    "latest: Bx={:.4}, By={:.4}, Bz={:.4} mT",
                    last.bx_mt,
                    last.by_mt,
                    last.bz_mt,
                ),
                Point::new(plot_left + 8.0, plot_top + 4.0),
                11.0,
                label_color,
            );
        }

        vec![frame.into_geometry()]
    }
}

/// Expand measured trace bounds to include the two signed guide locations
/// for every valid axis-specific advisory threshold.
fn field_bounds_with_guides_mt(
    points: &[SensorTracePoint], guides: [f64; 3],
) -> (f64, f64) {
    let (mut lo, mut hi) = field_bounds_mt(points);
    for limit in guides {
        if limit.is_finite() && limit > 0.0 {
            lo = lo.min(-limit);
            hi = hi.max(limit);
        }
    }
    (lo, hi)
}

fn draw_dotted_guide(
    frame: &mut canvas::Frame, left: f32, right: f32, y: f32, color: Color,
) {
    // One canvas path with separated short segments avoids a large number
    // of individual draw calls when all three board plots update live.
    let mut segments = canvas::path::Builder::new();
    let mut x = left;
    while x < right {
        segments.move_to(Point::new(x, y));
        segments.line_to(Point::new((x + 2.0).min(right), y));
        x += 6.0;
    }
    frame.stroke(
        &segments.build(),
        canvas::Stroke::default().with_color(color).with_width(1.2),
    );
}

fn field_bounds_mt(points: &[SensorTracePoint]) -> (f64, f64) {
    let mut min_value = f64::INFINITY;
    let mut max_value = f64::NEG_INFINITY;

    for point in points {
        for value_mt in [point.bx_mt, point.by_mt, point.bz_mt] {
            min_value = min_value.min(value_mt);
            max_value = max_value.max(value_mt);
        }
    }

    if !min_value.is_finite() || !max_value.is_finite() {
        (-0.001, 0.001)
    } else if (max_value - min_value).abs() < 1.0e-9 {
        (min_value - 0.001, max_value + 0.001)
    } else {
        (min_value, max_value)
    }
}

fn draw_series(
    frame: &mut canvas::Frame,
    points: &[SensorTracePoint],
    accessor: fn(&SensorTracePoint) -> f64,
    map_x: impl Fn(f64) -> f32,
    map_y: impl Fn(f64) -> f32,
    color: Color,
) {
    let mut builder = canvas::path::Builder::new();

    for (index, point) in points.iter().enumerate() {
        let p = Point::new(map_x(point.time_s), map_y(accessor(point)));

        if index == 0 {
            builder.move_to(p);
        } else {
            builder.line_to(p);
        }
    }

    let path = builder.build();

    frame.stroke(
        &path,
        canvas::Stroke::default()
            .with_color(color)
            .with_width(1.8),
    );
}

fn draw_legend(
    frame: &mut canvas::Frame,
    x: f32,
    y: f32,
    color: Color,
    label: &'static str,
) {
    frame.stroke(
        &canvas::Path::line(Point::new(x, y + 6.0), Point::new(x + 18.0, y + 6.0)),
        canvas::Stroke::default()
            .with_color(color)
            .with_width(2.0),
    );

    draw_label(
        frame,
        label,
        Point::new(x + 22.0, y),
        11.0,
        Color::from_rgb8(80, 80, 80),
    );
}

fn draw_label(
    frame: &mut canvas::Frame,
    content: impl Into<String>,
    position: Point,
    size: f32,
    color: Color,
) {
    frame.fill_text(canvas::Text {
        content: content.into(),
        position,
        color,
        size: Pixels(size),
        ..canvas::Text::default()
    });
}
#[cfg(test)]
mod trace_limit_plot_tests {
    use super::*;

    #[test]
    fn dotted_guides_use_independent_signed_xy_and_z_levels() {
        let points = [
            SensorTracePoint {time_s: 0.0, bx_mt: 0.4, by_mt: -0.2, bz_mt: 0.6},
            SensorTracePoint {time_s: 1.0, bx_mt: 0.5, by_mt: -0.1, bz_mt: 0.7},
        ];
        let (lo, hi) = field_bounds_with_guides_mt(
            &points, [4.9152, 4.9152, 7.929856],
        );
        assert_eq!(lo, -7.929856);
        assert_eq!(hi, 7.929856);
        let (wide_lo, wide_hi) = field_bounds_with_guides_mt(
            &points, [25.0, 25.0, 25.0],
        );
        assert_eq!((wide_lo, wide_hi), (-25.0, 25.0));
    }

    #[test]
    fn invalid_limits_cannot_distort_raw_trace_bounds() {
        let points = [
            SensorTracePoint {time_s: 0.0, bx_mt: -0.25, by_mt: 0.0, bz_mt: 0.1},
            SensorTracePoint {time_s: 1.0, bx_mt: 0.4, by_mt: 0.0, bz_mt: 0.3},
        ];
        assert_eq!(
            field_bounds_with_guides_mt(&points, [f64::NAN, 0.0, -10.0]),
            (-0.25, 0.4),
        );
    }
}
