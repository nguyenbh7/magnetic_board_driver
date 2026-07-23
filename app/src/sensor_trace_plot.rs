use iced::widget::canvas;
use iced::{mouse, Color, Element, Length, Pixels, Point, Rectangle, Renderer, Theme};

use crate::sensor_trace::SensorTracePoint;

const PLOT_HEIGHT: f32 = 220.0;

#[derive(Debug, Clone)]
pub struct SensorTracePlot {
    board_id: u16,
    sensor_index: Option<u8>,
    points: Vec<SensorTracePoint>,
}

pub fn sensor_trace_plot<Message: 'static>(
    board_id: u16,
    sensor_index: Option<u8>,
    points: Vec<SensorTracePoint>,
) -> Element<'static, Message> {
    canvas(SensorTracePlot {
        board_id,
        sensor_index,
        points,
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
            "Magnetic field (uT)",
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

        let (mut y_min, mut y_max) = field_bounds(&self.points);

        if y_min > 0.0 {
            y_min = 0.0;
        }

        if y_max < 0.0 {
            y_max = 0.0;
        }

        let y_range = (y_max - y_min).abs().max(1.0);
        y_min -= 0.10 * y_range;
        y_max += 0.10 * y_range;

        let map_x = |time_s: f64| {
            plot_left + (((time_s - x_min) / (x_max - x_min)) as f32 * plot_width)
        };

        let map_y = |field_ut: f64| {
            plot_bottom - (((field_ut - y_min) / (y_max - y_min)) as f32 * plot_height)
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
            let field_ut = y_min + (y_max - y_min) * frac as f64;

            frame.stroke(
                &canvas::Path::line(Point::new(plot_left, y), Point::new(plot_right, y)),
                canvas::Stroke::default()
                    .with_color(grid_color)
                    .with_width(0.5),
            );

            draw_label(
                &mut frame,
                format!("{:.0}", field_ut),
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

        draw_series(&mut frame, &self.points, |p| p.bx_ut, map_x, map_y, bx_color);
        draw_series(&mut frame, &self.points, |p| p.by_ut, map_x, map_y, by_color);
        draw_series(&mut frame, &self.points, |p| p.bz_ut, map_x, map_y, bz_color);

        draw_legend(&mut frame, plot_right - 130.0, plot_top + 4.0, bx_color, "Bx");
        draw_legend(&mut frame, plot_right - 86.0, plot_top + 4.0, by_color, "By");
        draw_legend(&mut frame, plot_right - 42.0, plot_top + 4.0, bz_color, "Bz");

        if let Some(last) = self.points.last() {
            draw_label(
                &mut frame,
                format!(
                    "latest: Bx={:.1}, By={:.1}, Bz={:.1} uT",
                    last.bx_ut, last.by_ut, last.bz_ut
                ),
                Point::new(plot_left + 8.0, plot_top + 4.0),
                11.0,
                label_color,
            );
        }

        vec![frame.into_geometry()]
    }
}

fn field_bounds(points: &[SensorTracePoint]) -> (f64, f64) {
    let mut min_value = f64::INFINITY;
    let mut max_value = f64::NEG_INFINITY;

    for point in points {
        for value in [point.bx_ut, point.by_ut, point.bz_ut] {
            min_value = min_value.min(value);
            max_value = max_value.max(value);
        }
    }

    if !min_value.is_finite() || !max_value.is_finite() {
        (-1.0, 1.0)
    } else if (max_value - min_value).abs() < 1.0e-9 {
        (min_value - 1.0, max_value + 1.0)
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