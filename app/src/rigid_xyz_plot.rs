//! Signed XYZ displacement graph for the three-board rigid 6-DOF fit.
use iced::widget::canvas;
use iced::{mouse, Color, Element, Length, Pixels, Point, Rectangle, Renderer, Theme};

use crate::rigid_fit::RigidXyzPoint;

const PLOT_HEIGHT: f32 = 260.0;

#[derive(Debug, Clone)]
struct RigidXyzPlot {
    points: Vec<RigidXyzPoint>,
}

pub fn rigid_xyz_plot<Message: 'static>(points: Vec<RigidXyzPoint>) -> Element<'static, Message> {
    canvas(RigidXyzPlot { points })
        .width(Length::Fill)
        .height(Length::Fixed(PLOT_HEIGHT))
        .into()
}

impl<Message> canvas::Program<Message> for RigidXyzPlot {
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
        if width < 160.0 || height < 140.0 {
            return vec![frame.into_geometry()];
        }

        let plot_left = 70.0;
        let plot_right = width - 16.0;
        let plot_top = 42.0;
        let plot_bottom = height - 34.0;
        let plot_width = (plot_right - plot_left).max(1.0);
        let plot_height = (plot_bottom - plot_top).max(1.0);
        let axis = Color::from_rgb8(120, 120, 120);
        let grid = Color::from_rgb8(205, 205, 205);
        let labels = Color::from_rgb8(75, 75, 75);
        let colors = [
            Color::from_rgb8(210, 65, 65),
            Color::from_rgb8(45, 135, 70),
            Color::from_rgb8(45, 95, 205),
        ];

        write(&mut frame, "Signed XYZ displacement (mm)",
              Point::new(plot_left, 2.0), 13.0, labels);
        for (index, label) in ["X", "Y", "Z"].iter().enumerate() {
            let pos = Point::new(plot_right - 102.0 + 34.0 * index as f32, 21.0);
            write(&mut frame, *label, pos, 12.0, colors[index]);
        }

        let span = self.points.iter().flat_map(|p|
            [p.dx_mm.abs(), p.dy_mm.abs(), p.dz_mm.abs()]
        ).fold(0.0_f64, f64::max);
        let range = nice_limit((span * 1.12).max(1.0));
        let t_min = self.points.first().map(|p| p.time_s).unwrap_or(0.0);
        let mut t_max = self.points.last().map(|p| p.time_s).unwrap_or(t_min + 1.0);
        if t_max <= t_min { t_max = t_min + 1.0; }

        let to_x = |t: f64| {
            plot_left + (((t - t_min) / (t_max - t_min)) as f32 * plot_width)
        };
        let to_y = |y: f64| {
            plot_top + (((range - y) / (2.0 * range)) as f32 * plot_height)
        };

        for i in 0..=4 {
            let fraction = i as f64 / 4.0;
            let displacement = range * (1.0 - 2.0 * fraction);
            let y = to_y(displacement);
            let color = if i == 2 { axis } else { grid };
            frame.stroke(
                &canvas::Path::line(Point::new(plot_left, y), Point::new(plot_right, y)),
                canvas::Stroke::default().with_color(color)
                    .with_width(if i == 2 { 1.3 } else { 0.5 }),
            );
            write(&mut frame, format!("{displacement:.1}"),
                  Point::new(plot_left - 47.0, y - 7.0), 10.0, labels);

            let t = t_min + (t_max - t_min) * fraction;
            let x = to_x(t);
            frame.stroke(
                &canvas::Path::line(Point::new(x, plot_top), Point::new(x, plot_bottom)),
                canvas::Stroke::default().with_color(grid).with_width(0.5),
            );
            write(&mut frame, format!("{t:.1}"),
                  Point::new(x - 14.0, plot_bottom + 7.0), 10.0, labels);
        }

        frame.stroke(
            &canvas::Path::line(Point::new(plot_left, plot_top),
                                Point::new(plot_left, plot_bottom)),
            canvas::Stroke::default().with_color(axis).with_width(1.0),
        );

        if self.points.len() < 2 {
            write(&mut frame, "Waiting for two accepted rigid poses",
                  Point::new(plot_left + 10.0, plot_top + 24.0), 12.0, labels);
            return vec![frame.into_geometry()];
        }

        for (axis_index, color) in colors.iter().enumerate() {
            let mut builder = canvas::path::Builder::new();
            for (index, point) in self.points.iter().enumerate() {
                let y = match axis_index {
                    0 => point.dx_mm,
                    1 => point.dy_mm,
                    _ => point.dz_mm,
                };
                let p = Point::new(to_x(point.time_s), to_y(y));
                if index == 0 { builder.move_to(p); } else { builder.line_to(p); }
            }
            frame.stroke(
                &builder.build(),
                canvas::Stroke::default().with_color(*color).with_width(2.0),
            );
        }

        write(&mut frame, "Elapsed time (s)",
              Point::new(plot_left + plot_width * 0.5 - 40.0, height - 13.0),
              11.0, labels);
        vec![frame.into_geometry()]
    }
}

fn write(frame: &mut canvas::Frame, content: impl Into<String>, at: Point,
         size: f32, color: Color) {
    frame.fill_text(canvas::Text {
        content: content.into(),
        position: at,
        color,
        size: Pixels(size),
        ..canvas::Text::default()
    });
}

fn nice_limit(value: f64) -> f64 {
    if !value.is_finite() || value <= 0.0 { return 1.0; }
    let unit = 10.0_f64.powf(value.log10().floor());
    let normalized = value / unit;
    unit * if normalized <= 1.0 { 1.0 }
           else if normalized <= 2.0 { 2.0 }
           else if normalized <= 5.0 { 5.0 }
           else { 10.0 }
}
