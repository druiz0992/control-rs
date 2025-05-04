use control_rs::plotter;
use plotters::prelude::*;
use std::f64::consts::PI;

fn plotting_cost(x: &[f64; 2]) -> f64 {
    let q = [2.0, -3.0];
    let q_matrix = [[1.65539, 2.89376], [2.89376, 6.51521]];
    let x1 = x[0];
    let x2 = x[1];

    let quad = 0.5
        * (x1 * (q_matrix[0][0] * x1 + q_matrix[0][1] * x2)
            + x2 * (q_matrix[1][0] * x1 + q_matrix[1][1] * x2));
    let linear = q[0] * x1 + q[1] * x2;
    let nonlinear = (-1.3 * x1 + 0.3 * x2.powi(2)).exp();
    quad + linear + nonlinear
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("/tmp/output.png", (800, 800)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("Cost Function", ("sans-serif", 30))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(-0.6..0.0, 0.0..0.6)?;

    chart.configure_mesh().x_desc("X₁").y_desc("X₂").draw()?;

    // Create grid and compute min/max of the cost function
    let step = 0.01;
    let x_range: Vec<f64> = (-60..=0).map(|i| i as f64 * step).collect();
    let y_range: Vec<f64> = (0..=60).map(|i| i as f64 * step).collect();

    let mut cost_values = vec![];
    for &x1 in &x_range {
        for &x2 in &y_range {
            cost_values.push(plotting_cost(&[x1, x2]));
        }
    }

    let z_min = cost_values.iter().cloned().reduce(f64::min).unwrap_or(0.0);
    let z_max = cost_values.iter().cloned().reduce(f64::max).unwrap_or(1.0);

    // Plot filled contour using color mapping
    for &x1 in &x_range {
        for &x2 in &y_range {
            let z = plotting_cost(&[x1, x2]);
            let norm = (z - z_min) / (z_max - z_min + 1e-12); // Normalize to [0, 1]
            let color = HSLColor(240.0 / 360.0 - norm * 240.0 / 360.0, 1.0, 0.5);
            chart.draw_series(std::iter::once(Rectangle::new(
                [(x1, x2), (x1 + step, x2 + step)],
                color.filled(),
            )))?;
        }
    }

    // Draw circular constraint
    let circle_points: Vec<(f64, f64)> = (0..200)
        .map(|i| {
            let theta = i as f64 * 2.0 * PI / 200.0;
            (0.5 * theta.cos(), 0.5 * theta.sin())
        })
        .collect();

    chart
        .draw_series(LineSeries::new(circle_points, &BLACK))?
        .label("constraint")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], &BLACK));

    chart.configure_series_labels().draw()?;
    plotter::display("/tmp/output.png").unwrap();

    Ok(())
}
