use iced::widget::canvas;
use iced::{Color, Element, Length, Pixels, Point, Rectangle, Renderer, Theme, mouse};

use crate::live_fit::DisplacementPoint;

const PLOT_HEIGHT: f32 = 180.0;

#[derive(Debug, Clone)]
pub struct DisplacementPlot {
    board_id: u16,
    points: Vec<DisplacementPoint>,
}

pub fn displacement_plot<Message: 'static>(
    board_id: u16,
    points: Vec<DisplacementPoint>,
) -> Element<'static, Message> {
    canvas(DisplacementPlot { board_id, points })
        .width(Length::Fill)
        .height(Length::Fixed(PLOT_HEIGHT))
        .into()
}

impl<Message> canvas::Program<Message> for DisplacementPlot {
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

        if width < 160.0 || height < 120.0 {
            return vec![frame.into_geometry()];
        }

        let plot_left = 86.0;
        let plot_right = width - 18.0;
        let plot_top = 18.0;
        let plot_bottom = height - 42.0;

        let plot_width = (plot_right - plot_left).max(1.0);
        let plot_height = (plot_bottom - plot_top).max(1.0);

        let axis_color = Color::from_rgb8(120, 120, 120);
        let grid_color = Color::from_rgb8(210, 210, 210);
        let label_color = Color::from_rgb8(80, 80, 80);
        let line_color = Color::from_rgb8(40, 95, 180);

        draw_label(
            &mut frame,
            "Displacement (mm)",
            Point::new(plot_left, 2.0),
            11.0,
            label_color,
        );

        draw_label(
            &mut frame,
            "Time (s)",
            Point::new(plot_left + 0.5 * plot_width - 22.0, height - 18.0),
            11.0,
            label_color,
        );

        let x_axis = canvas::Path::line(
            Point::new(plot_left, plot_bottom),
            Point::new(plot_right, plot_bottom),
        );
        let y_axis = canvas::Path::line(
            Point::new(plot_left, plot_top),
            Point::new(plot_left, plot_bottom),
        );

        frame.stroke(
            &x_axis,
            canvas::Stroke::default()
                .with_color(axis_color)
                .with_width(1.0),
        );
        frame.stroke(
            &y_axis,
            canvas::Stroke::default()
                .with_color(axis_color)
                .with_width(1.0),
        );

        if self.points.len() < 2 {
            draw_label(
                &mut frame,
                "Waiting for displacement history",
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

        let y_min = 0.0;
        let raw_y_max = self
            .points
            .iter()
            .map(|p| p.displacement_mm)
            .fold(0.0_f64, f64::max);

        let y_max = nice_axis_max((raw_y_max * 1.10).max(1.0));

        let map_x =
            |time_s: f64| plot_left + (((time_s - x_min) / (x_max - x_min)) as f32 * plot_width);

        let map_y = |displacement_mm: f64| {
            plot_bottom - (((displacement_mm - y_min) / (y_max - y_min)) as f32 * plot_height)
        };

        for i in 0..=4 {
            let frac = i as f32 / 4.0;

            let x = plot_left + frac * plot_width;
            let t = x_min + (x_max - x_min) * frac as f64;

            let vertical_grid =
                canvas::Path::line(Point::new(x, plot_top), Point::new(x, plot_bottom));

            frame.stroke(
                &vertical_grid,
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
            let displacement = y_min + (y_max - y_min) * frac as f64;

            let horizontal_grid =
                canvas::Path::line(Point::new(plot_left, y), Point::new(plot_right, y));

            frame.stroke(
                &horizontal_grid,
                canvas::Stroke::default()
                    .with_color(grid_color)
                    .with_width(0.5),
            );

            draw_label(
                &mut frame,
                format!("{:.2}", displacement),
                Point::new(plot_left - 58.0, y - 7.0),
                10.0,
                label_color,
            );
        }

        let mut builder = canvas::path::Builder::new();

        for (index, point) in self.points.iter().enumerate() {
            let p = Point::new(map_x(point.time_s), map_y(point.displacement_mm));

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
                .with_color(line_color)
                .with_width(2.0),
        );

        if let Some(last) = self.points.last() {
            let last_point = canvas::Path::circle(
                Point::new(map_x(last.time_s), map_y(last.displacement_mm)),
                3.0,
            );

            frame.fill(&last_point, line_color);

            draw_label(
                &mut frame,
                format!("{:.2} mm @ {:.1} s", last.displacement_mm, last.time_s),
                Point::new(plot_right - 112.0, plot_top + 4.0),
                11.0,
                label_color,
            );
        }

        vec![frame.into_geometry()]
    }
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

fn nice_axis_max(value: f64) -> f64 {
    if value <= 0.0 || !value.is_finite() {
        return 1.0;
    }

    let exponent = value.log10().floor();
    let base = 10.0_f64.powf(exponent);
    let normalized = value / base;

    let nice = if normalized <= 1.0 {
        1.0
    } else if normalized <= 2.0 {
        2.0
    } else if normalized <= 5.0 {
        5.0
    } else {
        10.0
    };

    nice * base
}
