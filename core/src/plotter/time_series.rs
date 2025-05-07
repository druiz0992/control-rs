use crate::numeric_services::symbolic::{
    ExprRegistry, ExprScalar, ExprVector, SymbolicExpr, SymbolicFunction, TryIntoEvalResult,
};
use crate::physics::traits::State;
use crate::physics::{Energy, ModelError};
use nalgebra::DVector;
use plotters::prelude::*;
use std::sync::Arc;

/// Generic plot for states over time
pub fn plot_states<S: State>(
    times: &[f64],
    states: &[S],
    filename: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let (min_value, max_value) = states
        .iter()
        .flat_map(|s| s.as_vec())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), val| {
            (min.min(val), max.max(val))
        });

    let mut chart = ChartBuilder::on(&root)
        .caption("State trajectory", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(times[0]..times[times.len() - 1], min_value..max_value)?;

    chart.configure_mesh().draw()?;

    let num_states = states[0].as_vec().len();
    let labels = <S as State>::labels();
    for i in 0..num_states {
        let series: Vec<(f64, f64)> = times
            .iter()
            .zip(states.iter())
            .map(|(&t, s)| (t, s.as_vec()[i]))
            .collect();

        let label = labels.get(i).cloned().ok_or("Label index out of bounds")?;

        chart
            .draw_series(LineSeries::new(series, &Palette99::pick(i)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], Palette99::pick(i)));
    }

    chart
        .configure_series_labels()
        .background_style(WHITE)
        .draw()?;

    Ok(())
}

/// Plot total, kinetic, and potential energy over time
pub fn plot_energy(
    times: &[f64],
    energies: &[Energy],
    filename: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let (_min_value, max_value) = energies
        .iter()
        .map(|s| s.total())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), val| {
            (min.min(val), max.max(val))
        });
    let mut chart = ChartBuilder::on(&root)
        .caption("Energy Over Time", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(
            times[0]..times[times.len() - 1],
            0.0..max_value + max_value * 0.2,
        )?;

    chart.configure_mesh().draw()?;

    let mut plot_series = |extract: fn(&Energy) -> f64, color, label| {
        let series: Vec<(f64, f64)> = times
            .iter()
            .zip(energies.iter())
            .map(|(&t, e)| (t, extract(e)))
            .collect();
        chart
            .draw_series(LineSeries::new(series, color))?
            .label(label)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], color));
        Ok::<_, Box<dyn std::error::Error>>(())
    };

    plot_series(|e| e.get_kinetic(), &RED, "Kinetic")?;
    plot_series(|e| e.get_potential(), &BLUE, "Potential")?;
    plot_series(|e| e.total(), &GREEN, "Total")?;

    chart
        .configure_series_labels()
        .background_style(WHITE)
        .draw()?;

    Ok(())
}

pub fn plot_minimization(
    cost_function_expr: &ExprScalar,
    history: &[Vec<f64>],
    eq_constraints_expr: Option<ExprVector>,
    ineq_constraints_expr: Option<ExprVector>,
    unknown_expr: &ExprVector,
    filename: &str,
    registry: &Arc<ExprRegistry>,
) -> Result<(), ModelError> {
    let root = BitMapBackend::new(filename, (800, 800)).into_drawing_area();
    root.fill(&WHITE)
        .map_err(|e| ModelError::Other(e.to_string()))?;

    let mut chart = ChartBuilder::on(&root)
        .caption("Cost Function", ("sans-serif", 30))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(-0.6..0.0, 0.0..0.6)
        .map_err(|e| ModelError::Other(e.to_string()))?;

    chart
        .configure_mesh()
        .x_desc("X₁")
        .y_desc("X₂")
        .draw()
        .map_err(|e| ModelError::Other(e.to_string()))?;

    // Create grid and compute min/max of the cost function
    let step = 0.01;
    let x_range: Vec<f64> = (-60..=0).map(|i| i as f64 * step).collect();
    let y_range: Vec<f64> = (0..=60).map(|i| i as f64 * step).collect();

    let mut cost_matrix = Vec::new();
    let cost_fn = SymbolicFunction::new(cost_function_expr.to_fn(registry)?, unknown_expr);

    for &x1 in &x_range {
        let mut row = Vec::new();
        for &x2 in &y_range {
            cost_fn
                .eval(&[x1, x2])
                .try_into_eval_result()
                .map(|val: f64| row.push(val))?;
        }
        cost_matrix.push(row);
    }

    let z_min = cost_matrix
        .iter()
        .flat_map(|row| row.iter())
        .cloned()
        .reduce(f64::min)
        .unwrap_or(0.0);

    let z_max = cost_matrix
        .iter()
        .flat_map(|row| row.iter())
        .cloned()
        .reduce(f64::max)
        .unwrap_or(1.0);

    // Plot filled contour using color mapping
    for (i, &x1) in x_range.iter().enumerate() {
        for (j, &x2) in y_range.iter().enumerate() {
            let z = cost_matrix[i][j];
            let norm = (z - z_min) / (z_max - z_min + 1e-12); // Normalize to [0, 1]
            let color = HSLColor(240.0 / 360.0 - norm * 240.0 / 360.0, 1.0, 0.5);
            chart
                .draw_series(std::iter::once(Rectangle::new(
                    [(x1, x2), (x1 + step, x2 + step)],
                    color.filled(),
                )))
                .map_err(|e| ModelError::Unexpected(format!("Drawing error: {:?}", e)))?;
        }
    }

    if let Some(eq_expr) = eq_constraints_expr {
        let constraint_points =
            build_constraints(&eq_expr, unknown_expr, registry, (-60, 60, step))?;

        for (i, points) in constraint_points.iter().enumerate() {
            chart
                .draw_series(
                    points
                        .iter()
                        .map(|&(x, y)| Circle::new((x, y), 1, BLACK.filled())),
                )
                .map_err(|e| ModelError::Other(e.to_string()))?
                .label(format!("eq constraint {}", i + 1))
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], BLACK));
        }
    }

    if let Some(ineq_expr) = ineq_constraints_expr {
        let constraint_points =
            build_constraints(&ineq_expr, unknown_expr, registry, (-60, 60, step))?;
        for (i, points) in constraint_points.iter().enumerate() {
            chart
                .draw_series(points.iter().map(|&(x, y)| {
                    Polygon::new(
                        vec![
                            (x, y),
                            (x + 0.005, y + 0.01), // Top-right point of triangle
                            (x - 0.005, y + 0.01), // Top-left point of triangle
                        ],
                        BLACK, // Fill color for the triangle
                    )
                }))
                .map_err(|e| ModelError::Other(e.to_string()))?
                .label(format!("ineq constraint {}", i + 1))
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], BLACK));
        }
    }

    // Overlay the minimization history (scatter plot or line plot)
    let history_points: Vec<(f64, f64)> =
        history.iter().map(|point| (point[0], point[1])).collect();

    // Plot the minimization path as a thicker line
    let line_style = ShapeStyle {
        color: RED.to_rgba(),
        filled: true,
        stroke_width: 3, // Increase stroke width for a thicker line
    };
    // Plot the minimization path as a line (or scatter points)
    chart
        .draw_series(LineSeries::new(history_points.clone(), line_style))
        .map_err(|e| ModelError::Other(e.to_string()))?
        .label("Minimization History")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], RED));

    // Initial and final points
    if let Some(first) = history.first() {
        chart
            .draw_series(std::iter::once(Circle::new(
                (first[0], first[1]),
                5,
                BLUE.filled(), // Initial guess = blue 'o'
            )))
            .map_err(|e| ModelError::Other(e.to_string()))?
            .label("Initial Guess (o)")
            .legend(|(x, y)| Circle::new((x, y), 5, BLUE.filled()));
    }

    if let Some(last) = history.last() {
        chart
            .draw_series(std::iter::once(PathElement::new(
                vec![
                    (last[0] - 0.01, last[1] - 0.01),
                    (last[0] + 0.01, last[1] + 0.01),
                    (last[0] - 0.01, last[1] + 0.01),
                    (last[0] + 0.01, last[1] - 0.01),
                ],
                ShapeStyle {
                    color: BLACK.to_rgba(),
                    filled: false,
                    stroke_width: 2,
                },
            )))
            .map_err(|e| ModelError::Other(e.to_string()))?
            .label("Final Result (x)")
            .legend(|(x, y)| {
                PathElement::new(
                    vec![
                        (x - 3, y - 3),
                        (x + 3, y + 3),
                        (x - 3, y + 3),
                        (x + 3, y - 3),
                    ],
                    ShapeStyle::from(&BLACK).stroke_width(2),
                )
            });
    }

    chart
        .configure_series_labels()
        .draw()
        .map_err(|e| ModelError::Other(e.to_string()))?;

    Ok(())
}

fn build_constraints(
    constraint_expr: &ExprVector,
    unknown_expr: &ExprVector,
    registry: &Arc<ExprRegistry>,
    range: (i32, i32, f64),
) -> Result<Vec<Vec<(f64, f64)>>, ModelError> {
    let constraint_fn = SymbolicFunction::new(
        constraint_expr // Assume first constraint for now
            .to_fn(registry)?,
        unknown_expr,
    );
    let n_constraints = constraint_expr.len();
    let mut constraint_points: Vec<Vec<(f64, f64)>> = vec![Vec::new(); n_constraints];
    let (min_range, max_range, step) = range;
    let x_range: Vec<f64> = (min_range..=0).map(|i| i as f64 * step).collect();
    let y_range: Vec<f64> = (0..=max_range).map(|i| i as f64 * step).collect();
    for &x1 in &x_range {
        for &x2 in &y_range {
            let res: DVector<f64> = constraint_fn.eval(&[x1, x2]).try_into_eval_result()?;
            res.iter()
                .zip(constraint_points.iter_mut())
                .filter(|(val, _)| val.abs() < 0.005)
                .for_each(|(_, points)| points.push((x1, x2)));
        }
    }
    Ok(constraint_points)
}
